#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QLabel>
#include <QTimer>
#include <QMessageBox>
#include <QTabWidget>
#include <QWidget>
#include <QPainter>
#include <QVector>
#include <QComboBox>
#include <QPushButton>
#include <QTextEdit>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QCheckBox>
#include <QScrollBar>
#include <QWheelEvent>
#include <QLineEdit>
#include <QSpinBox>
#include <QSlider>
#include <cstdint>
#include <cmath>

// ============================================================
//  数据结构
// ============================================================

// 激光雷达单点数据
struct LidarPoint {
    float    angle;       // 角度  (度, 0~360)
    uint16_t distance;    // 距离  (mm)
    uint8_t  confidence;  // 置信度 (0~255)
};

// 四向障碍物距离（与 STM32 侧 ObstacleDistance_t 相同含义）
struct ObstacleDistance {
    uint16_t front = 9999;
    uint16_t right = 9999;
    uint16_t back  = 9999;
    uint16_t left  = 9999;
};

// ============================================================
//  点云可视化 Widget（极坐标散点图）
//  - 滚轮缩放
//  - 颜色编码距离：绿→黄→橙→红
// ============================================================
class PointCloudWidget : public QWidget {
    Q_OBJECT
public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    void updateData(const QVector<LidarPoint>& points, const ObstacleDistance& obs);
    void setThresholds(int t1, int t2, int t3);

protected:
    void paintEvent(QPaintEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    QVector<LidarPoint> m_points;
    ObstacleDistance    m_obstacle;
    float               m_scale = 0.08f;
    int                 m_threshold1 = 800;
    int                 m_threshold2 = 400;
    int                 m_threshold3 = 200;

    QColor distToColor(uint16_t dist) const;
};

// ============================================================
//  障碍物方向 Widget（俯视示意图 + 四向数字）
// ============================================================
class ObstacleWidget : public QWidget {
    Q_OBJECT
public:
    explicit ObstacleWidget(QWidget *parent = nullptr);
    void updateObstacle(const ObstacleDistance& obs);
    void setThresholds(int t1, int t2, int t3);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    ObstacleDistance m_obs;
    int              m_threshold1 = 800;
    int              m_threshold2 = 400;
    int              m_threshold3 = 200;
    QColor distToColor(uint16_t dist) const;
    void drawDirectionBar(QPainter& p, int cx, int cy, int dir,
                          uint16_t dist, const QString& label);
};

// ============================================================
//  主窗口
// ============================================================
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onBtnOpenClicked();
    void onBtnCloseClicked();
    void onBtnRefreshClicked();
    void onReadyRead();
    void onTimerUpdate();
    void onBtnSendThresholdClicked();
    void onBtnResetThresholdClicked();
    void onThreshold1Changed(int value);
    void onThreshold2Changed(int value);
    void onThreshold3Changed(int value);
    void onResponseTimeout();  // 响应超时处理

private:
    // --- UI 构建 ---
    void buildUI();
    void buildCommTab(QWidget* tab);
    void buildCloudTab(QWidget* tab);
    void buildObstacleTab(QWidget* tab);
    void buildThresholdTab(QWidget* tab);
    void scanPorts();

    // --- 数据解析 ---
    void processBuffer();          // 从 m_rxBuf 中搜索并解析帧
    void parseFrame(const uint8_t* f);  // 解析一帧（47字节）
    void updateObstacle();         // 从点云计算四向障碍物距离
    uint8_t calcCRC8(const uint8_t* data, int len) const;

    // LD14P 帧格式常量
    static constexpr uint8_t FRAME_HEADER = 0x54;
    static constexpr uint8_t FRAME_VERLEN = 0x2C;
    static constexpr int     FRAME_LEN    = 47;
    static const uint8_t     CRC_TABLE[256];

    // --- 灯带阈值通信协议常量 ---
    static constexpr uint8_t THRESHOLD_FRAME_HEADER = 0xAA;
    static constexpr uint8_t THRESHOLD_CMD_SET      = 0x10;
    static constexpr uint8_t THRESHOLD_FRAME_LEN     = 14;

    // --- 串口 ---
    QSerialPort *m_serial;
    QByteArray   m_rxBuf;       // 未处理的接收缓冲（帧组装用）
    qint64       m_rxCount = 0; // 总接收字节数

    // --- 阈值发送状态 ---
    bool m_waitingForResponse = false;  // 是否在等待下位机响应
    QTimer *m_responseTimer = nullptr;  // 响应超时定时器
    static constexpr int RESPONSE_TIMEOUT_MS = 500;  // 超时时间500ms

    // --- 点云数据 ---
    QVector<LidarPoint> m_scanPoints;    // 当前圈点云（720点槽位）
    int  m_ptIdx      = 0;              // 下一个写入位置
    int  m_frameCount = 0;              // 已解析帧数
    bool m_newScanReady = false;        // 新一圈就绪标志（供 Timer 刷新 UI）

    // --- 四向障碍物 ---
    ObstacleDistance m_obstacle;

    // --- UI 控件 ---
    QComboBox   *m_cbxPort;
    QPushButton *m_btnOpen, *m_btnClose, *m_btnRefresh;

    // Tab1: 通讯助手
    QCheckBox   *m_chkHex, *m_chkParsed;
    QTextEdit   *m_textReceive;
    QPushButton *m_btnClear;
    QPushButton *m_btnPause;
    bool         m_paused = false;

    // Tab2: 点云视图
    PointCloudWidget *m_cloudWidget;
    QLabel *m_labScanInfo;
    QLabel *m_labLegend1, *m_labLegend2, *m_labLegend3, *m_labLegend4;

    // Tab3: 障碍物
    ObstacleWidget *m_obsWidget;
    QLabel *m_labFront, *m_labRight, *m_labBack, *m_labLeft;

    // Tab4: 灯带阈值设置
    QSpinBox *m_spinThreshold1;
    QSpinBox *m_spinThreshold2;
    QSpinBox *m_spinThreshold3;
    QSlider *m_sliderThreshold1;
    QSlider *m_sliderThreshold2;
    QSlider *m_sliderThreshold3;
    QSpinBox *m_spinLedCount;
    QSlider *m_sliderLedCount;
    QPushButton *m_btnSendThreshold;
    QPushButton *m_btnResetThreshold;
    QLabel *m_labThresholdStatus;

    // 状态栏
    QLabel *m_labStatus, *m_labRX;
    QTimer *m_timer;

protected:
    void paintEvent(QPaintEvent *event) override;
};

#endif // MAINWINDOW_H
