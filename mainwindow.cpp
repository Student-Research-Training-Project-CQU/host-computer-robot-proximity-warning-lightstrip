#include "mainwindow.h"
#include <QPainterPath>
#include <algorithm>
#include <QStatusBar>
// ============================================================
//  CRC8 查找表（来自 LD14P 开发手册，与 STM32 侧完全一致）
// ============================================================
const uint8_t MainWindow::CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
    0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
    0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
    0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
    0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
    0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
    0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
    0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
    0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
    0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

// ============================================================
//  PointCloudWidget
// ============================================================
PointCloudWidget::PointCloudWidget(QWidget *parent) : QWidget(parent) {
    setMinimumSize(420, 420);
}

void PointCloudWidget::updateData(const QVector<LidarPoint>& pts, const ObstacleDistance& obs) {
    m_points  = pts;
    m_obstacle = obs;
    update();  // 触发重绘
}

void PointCloudWidget::wheelEvent(QWheelEvent *event) {
    // 滚轮上/下 → 放大/缩小
    if (event->angleDelta().y() > 0)
        m_scale *= 1.12f;
    else
        m_scale /= 1.12f;
    m_scale = qBound(0.015f, m_scale, 0.5f);
    update();
}

// 距离→颜色映射：绿(安全) → 黄 → 橙 → 红(危险)
QColor PointCloudWidget::distToColor(uint16_t dist) const {
    if (dist == 0 || dist > 6000) return QColor(80, 80, 80);
    if (dist > 800)  return QColor(40, 200, 80);
    if (dist > 400)  return QColor(220, 200, 0);
    if (dist > 200)  return QColor(255, 120, 0);
    return QColor(230, 30, 30);
}

void PointCloudWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const int cx = width()  / 2;
    const int cy = height() / 2;

    // 背景
    p.fillRect(rect(), QColor(18, 18, 28));

    // 绘制同心圆（量程环），每 500mm 一圈
    p.setPen(QPen(QColor(50, 55, 80), 0.5, Qt::DashLine));
    for (int r_mm = 500; r_mm <= 6000; r_mm += 500) {
        int r_px = static_cast<int>(r_mm * m_scale);
        p.drawEllipse(cx - r_px, cy - r_px, 2 * r_px, 2 * r_px);
        // 标注距离
        p.setPen(QColor(90, 100, 130));
        p.setFont(QFont("Arial", 8));
        p.drawText(cx + r_px + 3, cy - 2,
                   QString("%1m").arg(r_mm / 1000.0, 0, 'f', 1));
        p.setPen(QPen(QColor(50, 55, 80), 0.5, Qt::DashLine));
    }

    // 十字线
    p.setPen(QPen(QColor(60, 65, 100), 0.5));
    p.drawLine(cx, 0, cx, height());
    p.drawLine(0, cy, width(), cy);

    // 方向标签
    // LD14P 坐标系：0°=正前方，顺时针增大
    // 屏幕绘制：0°→上方，90°→右，180°→下，270°→左
    p.setPen(QColor(160, 170, 220));
    p.setFont(QFont("Arial", 9, QFont::Bold));
    p.drawText(cx - 8, 16,              "前(0°)");
    p.drawText(width() - 46, cy + 5,    "右(90°)");
    p.drawText(cx - 16, height() - 6,   "后(180°)");
    p.drawText(6,        cy + 5,         "左(270°)");

    // 绘制点云
    p.setPen(Qt::NoPen);
    for (const auto& pt : m_points) {
        if (pt.distance == 0 || pt.confidence < 20) continue;
        if (pt.distance > 6000) continue;

        // 极坐标→直角坐标
        // 0°=上(前)，顺时针 → screen_x = cx + d*sin(θ), screen_y = cy - d*cos(θ)
        double rad = qDegreesToRadians((double)pt.angle);
        int sx = cx + static_cast<int>(pt.distance * m_scale * std::sin(rad));
        int sy = cy - static_cast<int>(pt.distance * m_scale * std::cos(rad));

        if (sx < 0 || sx >= width() || sy < 0 || sy >= height()) continue;

        p.setBrush(distToColor(pt.distance));
        p.drawEllipse(sx - 2, sy - 2, 4, 4);
    }

    // 机器人标志（中心蓝点 + 前向三角）
    p.setBrush(QColor(60, 140, 255));
    p.setPen(QPen(QColor(120, 180, 255), 0.5));
    p.drawEllipse(cx - 6, cy - 6, 12, 12);
    // 前向小三角（指向上）
    QPolygonF tri;
    tri << QPointF(cx, cy - 16) << QPointF(cx - 5, cy - 6) << QPointF(cx + 5, cy - 6);
    p.setBrush(QColor(60, 140, 255));
    p.drawPolygon(tri);
}

// ============================================================
//  ObstacleWidget
// ============================================================
ObstacleWidget::ObstacleWidget(QWidget *parent) : QWidget(parent) {
    setMinimumSize(320, 320);
}

void ObstacleWidget::updateObstacle(const ObstacleDistance& obs) {
    m_obs = obs;
    update();
}

QColor ObstacleWidget::distToColor(uint16_t dist) const {
    if (dist > 800)  return QColor(40, 190, 80);
    if (dist > 400)  return QColor(220, 190, 0);
    if (dist > 200)  return QColor(255, 110, 0);
    return QColor(220, 30, 30);
}

// dir: 0=前(上) 1=右 2=后(下) 3=左
void ObstacleWidget::drawDirectionBar(QPainter& p, int cx, int cy,
                                       int dir, uint16_t dist, const QString& label) {
    const int BAR_W = 70;
    const int BAR_H = 36;
    const int GAP   = 52;  // 距离机器人中心的距离

    int bx = cx, by = cy;
    switch (dir) {
    case 0: bx = cx - BAR_W/2; by = cy - GAP - BAR_H; break;  // 前(上)
    case 1: bx = cx + GAP;     by = cy - BAR_H/2;     break;  // 右
    case 2: bx = cx - BAR_W/2; by = cy + GAP;         break;  // 后(下)
    case 3: bx = cx - GAP - BAR_W; by = cy - BAR_H/2; break;  // 左
    }

    QColor c = distToColor(dist);
    p.setPen(QPen(c.darker(140), 1.5));
    p.setBrush(c.darker(200));
    p.drawRoundedRect(bx, by, BAR_W, BAR_H, 6, 6);

    p.setPen(c.lighter(150));
    p.setFont(QFont("Arial", 9, QFont::Bold));
    p.drawText(bx + 4, by + 14, label);

    p.setPen(c.lighter(120));
    p.setFont(QFont("Courier", 9));
    QString dstr = (dist >= 9000) ? QString("  ---") : QString("%1mm").arg(dist);
    p.drawText(bx + 4, by + 28, dstr);

    // 连接线（bar → 机器人框）
    p.setPen(QPen(c.darker(120), 1, Qt::DotLine));
    int bxc = bx + BAR_W/2, byc = by + BAR_H/2;
    int rxc = cx, ryc = cy;
    switch (dir) {
    case 0: p.drawLine(bxc, by + BAR_H, bxc, cy - 24); break;
    case 1: p.drawLine(bx,  byc,        cx + 24, ryc);  break;
    case 2: p.drawLine(bxc, by,          bxc, cy + 24);  break;
    case 3: p.drawLine(bx + BAR_W, byc, cx - 24, ryc);  break;
    }
    Q_UNUSED(rxc); Q_UNUSED(ryc);
}

void ObstacleWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    p.fillRect(rect(), QColor(20, 22, 32));

    const int cx = width()  / 2;
    const int cy = height() / 2;

    // 四个方向
    drawDirectionBar(p, cx, cy, 0, m_obs.front, "前");
    drawDirectionBar(p, cx, cy, 1, m_obs.right, "右");
    drawDirectionBar(p, cx, cy, 2, m_obs.back,  "后");
    drawDirectionBar(p, cx, cy, 3, m_obs.left,  "左");

    // 机器人本体（中心矩形）
    const int RW = 44, RH = 44;
    p.setPen(QPen(QColor(80, 130, 220), 1.5));
    p.setBrush(QColor(30, 50, 100));
    p.drawRoundedRect(cx - RW/2, cy - RH/2, RW, RH, 6, 6);
    // 前向标记
    QPolygonF tri;
    tri << QPointF(cx, cy - RH/2 - 8)
        << QPointF(cx - 6, cy - RH/2)
        << QPointF(cx + 6, cy - RH/2);
    p.setBrush(QColor(80, 140, 255));
    p.setPen(Qt::NoPen);
    p.drawPolygon(tri);
    // 文字
    p.setPen(QColor(160, 185, 240));
    p.setFont(QFont("Arial", 8));
    p.drawText(cx - 14, cy + 5, "Robot");
}

// ============================================================
//  MainWindow
// ============================================================
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    m_serial = new QSerialPort(this);
    m_timer  = new QTimer(this);
    m_scanPoints.resize(720);  // 预分配720点（≈一圈）

    buildUI();
    scanPorts();

    connect(m_serial, &QSerialPort::readyRead,  this, &MainWindow::onReadyRead);
    connect(m_timer,  &QTimer::timeout,          this, &MainWindow::onTimerUpdate);
    m_timer->start(100);  // 10Hz 刷新 UI
}

MainWindow::~MainWindow() {}

// ----- 构建 UI -----
void MainWindow::buildUI() {
    setWindowTitle("集成式预警灯带系统上位机");
    resize(1000, 720);

    QWidget *central = new QWidget(this);
    setCentralWidget(central);
    QVBoxLayout *root = new QVBoxLayout(central);
    root->setSpacing(6);
    root->setContentsMargins(8, 8, 8, 6);

    // ── 串口控制栏 ──────────────────────────────────────────────
    QGroupBox *serialGrp = new QGroupBox("串口配置（波特率固定 230400，8N1）", central);
    QHBoxLayout *sLay = new QHBoxLayout(serialGrp);
    sLay->setSpacing(8);

    sLay->addWidget(new QLabel("串口:"));
    m_cbxPort = new QComboBox();
    m_cbxPort->setMinimumWidth(110);
    sLay->addWidget(m_cbxPort);

    m_btnRefresh = new QPushButton("刷新");
    m_btnOpen    = new QPushButton("打开串口");
    m_btnClose   = new QPushButton("关闭串口");
    m_btnClose->setEnabled(false);
    m_btnRefresh->setFixedWidth(60);
    m_btnOpen->setFixedWidth(90);
    m_btnClose->setFixedWidth(90);

    sLay->addWidget(m_btnRefresh);
    sLay->addWidget(m_btnOpen);
    sLay->addWidget(m_btnClose);
    sLay->addStretch();

    connect(m_btnRefresh, &QPushButton::clicked, this, &MainWindow::onBtnRefreshClicked);
    connect(m_btnOpen,    &QPushButton::clicked, this, &MainWindow::onBtnOpenClicked);
    connect(m_btnClose,   &QPushButton::clicked, this, &MainWindow::onBtnCloseClicked);

    root->addWidget(serialGrp);

    // ── Tab ─────────────────────────────────────────────────────
    QTabWidget *tabs = new QTabWidget(central);

    QWidget *tab1 = new QWidget(); buildCommTab(tab1);
    QWidget *tab2 = new QWidget(); buildCloudTab(tab2);
    QWidget *tab3 = new QWidget(); buildObstacleTab(tab3);

    tabs->addTab(tab1, "📡  通讯助手");
    tabs->addTab(tab2, "🗺  点云视图");
    tabs->addTab(tab3, "⚠  障碍物信息");

    root->addWidget(tabs, 1);

    // ── 状态栏 ────────────────────────────────────────────────
    m_labStatus = new QLabel("  ❌ 未连接雷达");
    m_labRX     = new QLabel("  RX: 0 Bytes");
    statusBar()->addWidget(m_labStatus);
    statusBar()->addWidget(m_labRX);
}

void MainWindow::buildCommTab(QWidget* tab) {
    QVBoxLayout *lay = new QVBoxLayout(tab);
    lay->setSpacing(6);

    // 控制行
    QHBoxLayout *ctrlLay = new QHBoxLayout();
    m_chkHex    = new QCheckBox("原始HEX显示");
    m_chkParsed = new QCheckBox("显示解析结果");
    m_chkParsed->setChecked(true);
    m_btnClear  = new QPushButton("清除");
    m_btnClear->setFixedWidth(60);
    m_btnPause  = new QPushButton("⏸ 暂停");   // 新增
    m_btnPause->setFixedWidth(80);              // 新增
    m_btnPause->setCheckable(true);             // 新增

    ctrlLay->addWidget(m_chkHex);
    ctrlLay->addWidget(m_chkParsed);
    ctrlLay->addStretch();
    ctrlLay->addWidget(m_btnPause);   // 新增
    ctrlLay->addWidget(m_btnClear);

    // 新增：按钮点击逻辑
    connect(m_btnPause, &QPushButton::toggled, this, [this](bool paused){
        m_paused = paused;
        m_btnPause->setText(paused ? "▶ 恢复" : "⏸ 暂停");
        m_btnPause->setStyleSheet(paused ?
                                      "background:#c0392b; color:white; border-radius:4px;" :
                                      "");
    });

    lay->addLayout(ctrlLay);

    // 说明标签
    QLabel *hint = new QLabel(
        "说明: 选\"原始HEX显示\"查看原始字节流；"
        "选\"显示解析结果\"查看每帧的角度/距离/置信度；"
        "两者可同时勾选。每帧 47 字节，帧头 0x54 0x2C。");
    hint->setWordWrap(true);
    hint->setStyleSheet("color: gray; font-size: 11px;");
    lay->addWidget(hint);

    m_textReceive = new QTextEdit();
    m_textReceive->setReadOnly(true);
    m_textReceive->setFont(QFont("Courier New", 9));
    m_textReceive->setStyleSheet(
        "background:#1e1e1e; color:#d4d4d4; border-radius:4px;");
    lay->addWidget(m_textReceive, 1);

    connect(m_btnClear, &QPushButton::clicked, m_textReceive, &QTextEdit::clear);
    connect(m_chkHex, &QCheckBox::toggled, [this](bool){ m_textReceive->clear(); });
}

void MainWindow::buildCloudTab(QWidget* tab) {
    QVBoxLayout *lay = new QVBoxLayout(tab);

    m_labScanInfo = new QLabel("等待雷达数据...（鼠标滚轮缩放）");
    m_labScanInfo->setStyleSheet("color: gray;");
    lay->addWidget(m_labScanInfo);

    m_cloudWidget = new PointCloudWidget();
    lay->addWidget(m_cloudWidget, 1);

    // 颜色图例
    QHBoxLayout *legend = new QHBoxLayout();
    auto mkLeg = [](const QString& color, const QString& text) {
        QLabel *l = new QLabel(text);
        l->setStyleSheet(QString("background:%1; color:white; "
                                  "padding:2px 8px; border-radius:3px; font-size:11px;").arg(color));
        return l;
    };
    legend->addWidget(new QLabel("距离颜色:"));
    legend->addWidget(mkLeg("#28be50", "> 800mm  安全"));
    legend->addWidget(mkLeg("#dcc800", "400~800mm  注意"));
    legend->addWidget(mkLeg("#ff7800", "200~400mm  警告"));
    legend->addWidget(mkLeg("#e61e1e", "< 200mm  危险"));
    legend->addStretch();
    lay->addLayout(legend);
}

void MainWindow::buildObstacleTab(QWidget* tab) {
    QHBoxLayout *lay = new QHBoxLayout(tab);

    m_obsWidget = new ObstacleWidget();
    lay->addWidget(m_obsWidget, 2);

    // 右侧数字面板
    QGroupBox *numGrp = new QGroupBox("四向障碍物距离（mm）");
    QGridLayout *grid = new QGridLayout(numGrp);
    grid->setSpacing(10);

    auto mkDistLabel = [](const QString& text) {
        QLabel *l = new QLabel(text);
        l->setAlignment(Qt::AlignCenter);
        l->setFont(QFont("Arial", 15, QFont::Bold));
        l->setMinimumSize(120, 55);
        l->setStyleSheet(
            "border: 2px solid #555; border-radius: 8px; "
            "background: #1a1a2a; color: #88ff88; padding: 4px;");
        return l;
    };

    m_labFront = mkDistLabel("前\n---");
    m_labRight = mkDistLabel("右\n---");
    m_labBack  = mkDistLabel("后\n---");
    m_labLeft  = mkDistLabel("左\n---");

    //  布局成十字形
    grid->addWidget(m_labFront, 0, 1, Qt::AlignCenter);
    grid->addWidget(m_labLeft,  1, 0, Qt::AlignCenter);
    grid->addWidget(new QLabel("🤖", nullptr), 1, 1, Qt::AlignCenter);
    grid->addWidget(m_labRight, 1, 2, Qt::AlignCenter);
    grid->addWidget(m_labBack,  2, 1, Qt::AlignCenter);

    QLabel *note = new QLabel(
        "颜色含义：\n"
        "● 绿色  > 800mm（安全）\n"
        "● 黄色  400~800mm（注意）\n"
        "● 橙色  200~400mm（警告）\n"
        "● 红色  < 200mm（危险）\n\n"
        "与 STM32 侧 obstacle_detect.c\n"
        "保持相同扇区划分（±50°）。");
    note->setStyleSheet("color: gray; font-size: 11px; padding: 8px;");
    grid->addWidget(note, 3, 0, 1, 3);

    lay->addWidget(numGrp, 1);
}

// ----- 串口操作 -----
void MainWindow::scanPorts() {
    m_cbxPort->clear();
    const auto ports = QSerialPortInfo::availablePorts();
    for (const auto& info : ports)
        m_cbxPort->addItem(info.portName());
    if (m_cbxPort->count() == 0)
        m_cbxPort->addItem("（未发现串口）");
}

void MainWindow::onBtnRefreshClicked() { scanPorts(); }

void MainWindow::onBtnOpenClicked() {
    const QString portName = m_cbxPort->currentText();
    if (portName.isEmpty() || portName.startsWith("（")) {
        QMessageBox::warning(this, "提示", "请先刷新并选择串口！");
        return;
    }

    m_serial->setPortName(portName);
    m_serial->setBaudRate(230400);
    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setStopBits(QSerialPort::OneStop);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serial->open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, "串口错误",
            "无法打开串口: " + m_serial->errorString() +
            "\n请确认设备已连接且驱动正常。");
        return;
    }

    m_labStatus->setText("  ✅ 已连接 — " + portName);
    m_btnOpen->setEnabled(false);
    m_btnClose->setEnabled(true);
    m_cbxPort->setEnabled(false);
    m_rxBuf.clear();
    m_ptIdx = 0;
    m_frameCount = 0;
    m_rxCount = 0;
}

void MainWindow::onBtnCloseClicked() {
    m_serial->close();
    m_labStatus->setText("  ❌ 未连接雷达");
    m_btnOpen->setEnabled(true);
    m_btnClose->setEnabled(false);
    m_cbxPort->setEnabled(true);
}

// ----- 数据接收 -----
void MainWindow::onReadyRead() {
    const QByteArray data = m_serial->readAll();
    m_rxCount += data.size();

    if (!m_paused && m_chkHex->isChecked()) {
        const QString hex = data.toHex(' ').toUpper();
        // 超过上限删掉前半部分，保持滚动
        if (m_textReceive->document()->blockCount() > 3000) {
            QTextCursor cursor = m_textReceive->textCursor();
            cursor.movePosition(QTextCursor::Start);
            cursor.movePosition(QTextCursor::Down,
                                QTextCursor::KeepAnchor, 1500);
            cursor.removeSelectedText();
        }
        m_textReceive->insertPlainText(hex + " ");
    }

    m_rxBuf.append(data);
    processBuffer();
}

// ----- 帧搜索与解析 -----
void MainWindow::processBuffer() {
    while (m_rxBuf.size() >= FRAME_LEN) {
        // 1. 搜索帧头 0x54 0x2C
        int hdrPos = -1;
        for (int i = 0; i <= m_rxBuf.size() - 2; ++i) {
            if ((uint8_t)m_rxBuf[i]   == FRAME_HEADER &&
                (uint8_t)m_rxBuf[i+1] == FRAME_VERLEN) {
                hdrPos = i;
                break;
            }
        }

        if (hdrPos < 0) {
            // 没找到帧头，保留最后1字节（可能是帧头的高位）
            if (m_rxBuf.size() > 1)
                m_rxBuf.remove(0, m_rxBuf.size() - 1);
            return;
        }

        // 丢弃帧头之前的"垃圾字节"
        if (hdrPos > 0)
            m_rxBuf.remove(0, hdrPos);

        // 2. 数据不够一帧 → 等待
        if (m_rxBuf.size() < FRAME_LEN)
            return;

        // 3. CRC 校验（对前 46 字节做校验，结果比对第 47 字节）
        const auto* raw = reinterpret_cast<const uint8_t*>(m_rxBuf.constData());
        uint8_t calcCrc = calcCRC8(raw, FRAME_LEN - 1);
        uint8_t rxCrc   = raw[FRAME_LEN - 1];

        if (calcCrc != rxCrc) {
            // CRC 不对 → 跳过一字节重新搜索
            m_rxBuf.remove(0, 1);
            continue;
        }

        // 4. 有效帧 → 解析
        parseFrame(raw);
        m_rxBuf.remove(0, FRAME_LEN);
        m_frameCount++;
    }
}

void MainWindow::parseFrame(const uint8_t* f) {
    /*  LD14P 帧字段布局（字节偏移）：
     *  [0]     0x54  帧头
     *  [1]     0x2C  版本+长度
     *  [2-3]   speed      转速（小端，度/秒）
     *  [4-5]   start_angle 起始角（小端，0.01°）
     *  [6-8]   point[0]   距离低位 | 距离高位 | 置信度
     *  [9-11]  point[1]   ...
     *  ...
     *  [42-43] end_angle  结束角（小端，0.01°）
     *  [44-45] timestamp  时间戳（小端，ms）
     *  [46]    crc8
     */
    uint16_t speed      = (uint16_t)f[2] | ((uint16_t)f[3] << 8);
    float    startAngle = ((uint16_t)f[4] | ((uint16_t)f[5] << 8)) / 100.0f;
    float    endAngle   = ((uint16_t)f[42]| ((uint16_t)f[43]<< 8)) / 100.0f;
    uint16_t ts         = (uint16_t)f[44] | ((uint16_t)f[45]<< 8);

    // 处理跨零（如起始350°，结束10°）
    if (startAngle > endAngle)
        endAngle += 360.0f;

    // 解析 12 个测量点，线性插值角度
    for (int j = 0; j < 12; ++j) {
        int off = 6 + j * 3;
        uint16_t dist = (uint16_t)f[off] | ((uint16_t)f[off+1] << 8);
        uint8_t  conf = f[off + 2];

        float angle = startAngle + (endAngle - startAngle) / 11.0f * j;
        if (angle >= 360.0f) angle -= 360.0f;

        // 循环写入720点槽位
        if (m_ptIdx < m_scanPoints.size())
            m_scanPoints[m_ptIdx] = { angle, dist, conf };
        m_ptIdx++;
    }

    // 解析结果输出到通讯助手（仅在勾选且未开HEX模式时）
    if (!m_paused && m_chkParsed->isChecked() && !m_chkHex->isChecked()) {
        // 超过上限删掉前半部分
        if (m_textReceive->document()->blockCount() > 2000) {
            QTextCursor cursor = m_textReceive->textCursor();
            cursor.movePosition(QTextCursor::Start);
            cursor.movePosition(QTextCursor::Down,
                                QTextCursor::KeepAnchor, 1000);
            cursor.removeSelectedText();
        }

        QString line = QString(
                           "[#%1] 速度:%2°/s  起始角:%3°  结束角:%4°  时间:%5ms\n")
                           .arg(m_frameCount, 5)
                           .arg(speed)
                           .arg(startAngle, 6, 'f', 2)
                           .arg(endAngle > 360.0f ? endAngle - 360.0f : endAngle, 6, 'f', 2)
                           .arg(ts);
        // 显示第0、5、11点的数据
        for (int j : {0, 5, 11}) {
            int off = 6 + j * 3;
            uint16_t d = (uint16_t)f[off] | ((uint16_t)f[off+1] << 8);
            uint8_t  c = f[off + 2];
            float    a = startAngle + (endAngle - startAngle) / 11.0f * j;
            if (a >= 360.0f) a -= 360.0f;
            line += QString("       点%1: %2°  %3mm  置信=%4\n")
                        .arg(j).arg(a, 6, 'f', 2).arg(d).arg(c);
        }
        line += "       ...\n";
        m_textReceive->insertPlainText(line);
        // 自动滚到底部
        m_textReceive->verticalScrollBar()->setValue(
            m_textReceive->verticalScrollBar()->maximum());
    }

    // 一圈（720点≈60帧）完成 → 计算障碍物，通知 UI 刷新
    if (m_ptIdx >= 720) {
        m_ptIdx = 0;
        updateObstacle();
        m_newScanReady = true;
    }

    Q_UNUSED(ts);
}

// ----- 障碍物距离计算（与 STM32 obstacle_detect.c 逻辑一致）-----
void MainWindow::updateObstacle() {
    uint16_t fMin = 9999, rMin = 9999, bMin = 9999, lMin = 9999;
    int      fCnt = 0,    rCnt = 0,    bCnt = 0,    lCnt = 0;

    for (const auto& pt : m_scanPoints) {
        if (pt.distance == 0 || pt.confidence < 40) continue;

        float    a = pt.angle;
        uint16_t d = pt.distance;

        // 前：围绕 0°（±50°）
        if (a <= 50.0f || a >= 310.0f)
            { if (d < fMin) fMin = d; fCnt++; }
        // 右：围绕 90°
        else if (a >= 40.0f && a <= 140.0f)
            { if (d < rMin) rMin = d; rCnt++; }
        // 后：围绕 180°
        else if (a >= 130.0f && a <= 230.0f)
            { if (d < bMin) bMin = d; bCnt++; }
        // 左：围绕 270°
        else if (a >= 220.0f && a <= 320.0f)
            { if (d < lMin) lMin = d; lCnt++; }
    }

    // 有效点数达到3个才更新（同 STM32 侧的 min count 判断）
    if (fCnt >= 3) m_obstacle.front = fMin;
    if (rCnt >= 3) m_obstacle.right = rMin;
    if (bCnt >= 3) m_obstacle.back  = bMin;
    if (lCnt >= 3) m_obstacle.left  = lMin;
}

// ----- 定时刷新 UI（10Hz）-----
void MainWindow::onTimerUpdate() {
    // 状态栏更新（每次都刷）
    m_labRX->setText(QString("  RX: %1 Bytes | 帧数: %2")
                     .arg(m_rxCount).arg(m_frameCount));

    if (!m_newScanReady) return;
    m_newScanReady = false;

    // ── 刷新点云视图 ──
    m_cloudWidget->updateData(m_scanPoints, m_obstacle);
    int validPts = 0;
    for (const auto& pt : m_scanPoints)
        if (pt.distance > 0 && pt.confidence >= 20) validPts++;
    m_labScanInfo->setText(
        QString("帧数: %1  有效点: %2/720  【滚轮缩放】")
        .arg(m_frameCount).arg(validPts));

    // ── 刷新障碍物视图 ──
    m_obsWidget->updateObstacle(m_obstacle);

    auto distStr = [](uint16_t d) -> QString {
        return d >= 9000 ? "---" : QString::number(d) + " mm";
    };
    auto distStyle = [](uint16_t d) -> QString {
        QString base = "border: 2px solid #555; border-radius: 8px; "
                       "background: #1a1a2a; padding: 4px; "
                       "font-size: 14pt; font-weight: bold;";
        if (d > 800)  return base + "color: #44ee66;";
        if (d > 400)  return base + "color: #ddcc00;";
        if (d > 200)  return base + "color: #ff8800;";
        return             base + "color: #ee2222;";
    };

    m_labFront->setText("前\n" + distStr(m_obstacle.front));
    m_labFront->setStyleSheet(distStyle(m_obstacle.front));
    m_labRight->setText("右\n" + distStr(m_obstacle.right));
    m_labRight->setStyleSheet(distStyle(m_obstacle.right));
    m_labBack->setText("后\n"  + distStr(m_obstacle.back));
    m_labBack->setStyleSheet(distStyle(m_obstacle.back));
    m_labLeft->setText("左\n"  + distStr(m_obstacle.left));
    m_labLeft->setStyleSheet(distStyle(m_obstacle.left));
}

// ----- CRC8 -----
uint8_t MainWindow::calcCRC8(const uint8_t* data, int len) const {
    uint8_t crc = 0;
    for (int i = 0; i < len; ++i)
        crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
    return crc;
}
