#include "dialog.h"
#include "ui_dialog.h"
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QFileInfo>
#include <QCoreApplication>
#include <QDateTime>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/QCamera>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QOrbitCameraController>
#include <QVBoxLayout>
#include <QQuaternion>
#include <QTabWidget>
#include <QLabel>
#include <QLineEdit>
#include <QAbstractButton>
#include <QCheckBox>
#include <QFont>
#include <QFontMetrics>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QSettings>
#include <QTimer>
#include <QDebug>

namespace {

bool looksLikeFirmwareLine(const QString &line)
{
    static const QStringList markers = {
        QStringLiteral("Welcome to"),
        QStringLiteral("Drone disarmed"),
        QStringLiteral("Drone armed"),
        QStringLiteral("Chip:"),
        QStringLiteral("Invalid command"),
        QStringLiteral("pitchTelMask"),
        QStringLiteral("Controllers reset"),
        QStringLiteral("Free heap:"),
        QStringLiteral("Temperature:"),
    };
    for (const QString &marker : markers) {
        if (line.contains(marker))
            return true;
    }
    if (line.contains(QStringLiteral(" = ")))
        return true;
    if (line.startsWith(QStringLiteral("TAKELOG_")))
        return true;
    return false;
}

bool looksLikeFirmwareTelemetry(const QString &line, int logMode)
{
    if (logMode == 0)
        return false;
    const QStringList parts = line.split(QRegExp(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
    if (parts.isEmpty() || parts.size() > 3)
        return false;
    for (const QString &part : parts) {
        bool ok = false;
        part.toInt(&ok);
        if (!ok)
            return false;
    }
    return true;
}

} // namespace

static constexpr char kSerialPort[] = "/dev/ttyUSB0";
static constexpr char kSettingsOrg[] = "UAV_Lon_ESP8266";
static constexpr char kSettingsApp[] = "Qt_Flix";

static constexpr float kMilliRadToRad = 1e-3f;
static constexpr float kRadToDeg = float(180.0 / 3.14159265358979323846);
static constexpr double kDegToMilliRad = 1000.0 * 3.14159265358979323846 / 180.0;

static void applyChoiceStyle(QAbstractButton *btn)
{
    const bool on = btn->isChecked();
    QFont f = btn->font();
    f.setWeight(on ? QFont::Bold : QFont::Normal);
    btn->setFont(f);
    btn->setStyleSheet(on ? QStringLiteral("color: #1565C0;") : QString());
}

static void wireChoiceStyle(QAbstractButton *btn)
{
    applyChoiceStyle(btn);
    QObject::connect(btn, &QAbstractButton::toggled, btn, [btn](bool) {
        applyChoiceStyle(btn);
    });
}

/// 0 = replot every telemetry sample (max noise detail); >0 ms to limit CPU (e.g. 16).
static constexpr qint64 kTelemetryRedrawIntervalMs = 0;

static QString resolveUavStlPath()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int i = 0; i < 14; ++i) {
        const QString candidate = dir.absoluteFilePath(QStringLiteral("flixESPDrone/uav.stl"));
        if (QFile::exists(candidate))
            return QFileInfo(candidate).absoluteFilePath();
        if (!dir.cdUp())
            break;
    }
    const QStringList legacy = {
        QStringLiteral("/home/haipham/Documents/UAV_Projects/project_quad/UAV_Lon_ESP8266/flixESPDrone/uav.stl"),
        QStringLiteral("/home/haipham/Documents/project_quad/UAV_Lon_ESP8266/flixESPDrone/uav.stl"),
    };
    for (const QString &p : legacy) {
        if (QFile::exists(p))
            return QFileInfo(p).absoluteFilePath();
    }
    return QString();
}

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
    , serialPort(new QSerialPort(this))
{
    ui->setupUi(this);
    setupRuntimeLayout();
    {
        QFont sectionFont = ui->groupBoxPitchSource->font();
        sectionFont.setBold(true);
        ui->groupBoxPitchSource->setFont(sectionFont);
        ui->groupBoxManual->setFont(sectionFont);
        ui->groupBoxLPF->setFont(sectionFont);
    }
    for (QRadioButton *rb : findChildren<QRadioButton *>())
        wireChoiceStyle(rb);
    for (QCheckBox *cb : findChildren<QCheckBox *>())
        wireChoiceStyle(cb);

    QObject::connect(ui->checkBoxLogTelPitchAng, &QCheckBox::toggled,
                     this, &Dialog::onPitchTelChannelToggled);
    QObject::connect(ui->checkBoxLogTelPitchRate, &QCheckBox::toggled,
                     this, &Dialog::onPitchTelChannelToggled);
    QObject::connect(ui->checkBoxLogTelPitchCtrl, &QCheckBox::toggled,
                     this, &Dialog::onPitchTelChannelToggled);
    setupPitchTelemetryPlot();
    setPlotLegendCheckboxesVisible(false);
    setPitchTelCheckboxesVisible(true);
    setupParamFieldWiring();

    connect(ui->rbQuaternion, &QRadioButton::toggled, this, [this](bool checked) {
        if (!checked) return;
        ui->rbCompFilt->setEnabled(false);
        ui->rbKalmanFilt->setEnabled(false);
        ui->rbUseLPF->setEnabled(true);
        ui->rbNoLPF->setEnabled(true);
        ui->rbUseLPF->setToolTip(QString());
        ui->rbNoLPF->setToolTip(QString());
        sendCommand("p QuatEn 1");
        if (ui->rbNoLPF->isChecked()) {
            sendCommand("p LPF_En 0");
        } else {
            QSignalBlocker b(ui->rbUseLPF);
            ui->rbUseLPF->setChecked(true);
            sendCommand("p LPF_En 1");
        }
        sendCommand("save");
    });
    connect(ui->rbNoPitchQuat, &QRadioButton::toggled, this, [this](bool checked) {
        if (!checked) return;
        const QString useLpfNote = tr("Lọc acc/rates + đạo hàm PID-D luôn BẬT khi dùng "
                                       "Complementary hoặc Kalman (LPF_En=2/3 đã >= 0.5),"
                                       " không thể chọn riêng ở chế độ No Quaternion.");
        ui->rbUseLPF->setEnabled(false);
        ui->rbUseLPF->setToolTip(useLpfNote);
        ui->rbNoLPF->setEnabled(true);
        ui->rbNoLPF->setToolTip(QString());
        ui->rbCompFilt->setEnabled(true);
        ui->rbKalmanFilt->setEnabled(true);
        sendCommand("p QuatEn 0");
        if (ui->rbNoLPF->isChecked()) {
            sendCommand("p LPF_En 0");
        } else if (ui->rbKalmanFilt->isChecked()) {
            sendCommand("p LPF_En 3");
        } else {
            QSignalBlocker b(ui->rbCompFilt);
            ui->rbCompFilt->setChecked(true);
            sendCommand("p LPF_En 2");
        }
        sendCommand("save");
    });

    connect(ui->sliderManualFront, &QSlider::valueChanged, this, [this](int val) {
        ui->progressManualFront->setValue(val);
        sendCommand(QString("mfront %1").arg(val));
    });
    connect(ui->sliderManualRear, &QSlider::valueChanged, this, [this](int val) {
        ui->progressManualRear->setValue(val);
        sendCommand(QString("mrear %1").arg(val));
    });

    connect(ui->rbUseLPF, &QRadioButton::toggled, this, [this](bool checked) {
        if (checked) {
            sendCommand("p LPF_En 1");
            sendCommand("save");
        }
    });
    connect(ui->rbNoLPF, &QRadioButton::toggled, this, [this](bool checked) {
        if (checked) {
            sendCommand("p LPF_En 0");
            sendCommand("save");
        }
    });
    connect(ui->rbCompFilt, &QRadioButton::toggled, this, [this](bool checked) {
        if (!checked || !ui->rbCompFilt->isEnabled()) return;
        sendCommand("p LPF_En 2");
        sendCommand("save");
    });
    connect(ui->rbKalmanFilt, &QRadioButton::toggled, this, [this](bool checked) {
        if (!checked || !ui->rbKalmanFilt->isEnabled()) return;
        sendCommand("p LPF_En 3");
        sendCommand("save");
    });

    ui->rbCompFilt->setEnabled(false);
    ui->rbKalmanFilt->setEnabled(false);

    setupControlTabs();
    setupCtrlAlgOutsideTabs();
    currentLogMode = 0;
    initUav3D(ui->uav3dHost);

    // Thrust: 0..1000
    ui->verticalSliderThrust->setMinimum(0);
    ui->verticalSliderThrust->setMaximum(1000);
    ui->verticalSliderThrust->setValue(0);
    ui->labelThrustPercent->setText(QStringLiteral("0%"));

    isArmed = false;
    updateArmUi();
    setupSerialLink();

    sendCommand(QStringLiteral("disarm"));
    applyDefaultPitchOnlyChannels();
    sendAllParams();
    syncPitchSourceToFirmware();
    startLiveLog();
}

void Dialog::setupRuntimeLayout()
{
    constexpr int kDisableW = 141;
    constexpr int kDisableH = 31;
    constexpr int kPitchW = 92;
    constexpr int kPitchH = 23;
    constexpr int kGap = 12;
    const int rowX = 50 + qRound(1.0 / 2.54 * logicalDpiY());
    const int rowY = 250 + qRound(1.5 / 2.54 * logicalDpiY());
    ui->btnDisableAll->setGeometry(rowX, rowY, kDisableW, kDisableH);
    ui->checkBoxPitch->setGeometry(rowX + kDisableW + kGap,
                                    rowY + (kDisableH - kPitchH) / 2,
                                    kPitchW, kPitchH);

    const int shiftXPx = qRound(2.5 / 2.54 * logicalDpiY());
    const int shiftYPx = qRound(2.0 / 2.54 * logicalDpiY());
    constexpr int kBaseX = 10;
    constexpr int kBaseY = 170;
    constexpr int kEditW = 80;
    constexpr int kEditH = 25;
    constexpr int kLabelH = 22;

    const QString pitchLabel = QStringLiteral("Желаемый угол");
    ui->label_19->setText(pitchLabel);
    const QFontMetrics fm(ui->label_19->font());
    const int labelW = fm.horizontalAdvance(pitchLabel) + 6;
    const int rowX2 = kBaseX + shiftXPx;
    const int rowY2 = kBaseY + shiftYPx;
    ui->label_19->setGeometry(rowX2, rowY2 + (kEditH - kLabelH) / 2, labelW, kLabelH);
    ui->lineEdit_setPitch->setGeometry(rowX2 + labelW + kGap, rowY2, kEditW, kEditH);

    const QRect dumpBtn = ui->btnSaveLog->geometry();
    const int dumpShiftY = qRound(1.0 / 2.54 * logicalDpiY());
    ui->btnSaveLog->setGeometry(dumpBtn.x(), dumpBtn.y() + dumpShiftY,
                                dumpBtn.width(), dumpBtn.height());
}

void Dialog::setupParamFieldWiring()
{
    struct FieldWire {
        QLineEdit *edit;
        const char *param;
        int precision;
        bool saveAfter;
    };
    const FieldWire fields[] = {
        {ui->lineEdit_Kpf, "RKPF", 6, false},
        {ui->lineEdit_Kpv, "RKPV", 6, false},
        {ui->lineEdit_Kdv, "RKDV", 6, false},
        {ui->lineEdit_RTv, "RTV", 6, false},
        {ui->lineEdit_RTmu, "RTmu", 6, false},
        {ui->lineEdit_RTi, "RTI", 6, false},
        {ui->lineEdit_RTe, "RTE", 6, false},
        {ui->lineEdit_RTm, "RTM", 6, false},
        {ui->lineEdit_PKPF, "PKPF", 6, false},
        {ui->lineEdit_PKPV, "PKPV", 6, false},
        {ui->lineEdit_PKDV, "PKDV", 6, false},
        {ui->lineEdit_PTV, "PTV", 6, false},
        {ui->lineEdit_PTMU, "PTmu", 6, false},
        {ui->lineEdit_PTI, "PTI", 6, false},
        {ui->lineEdit_PTE, "PTE", 6, false},
        {ui->lineEdit_PTM, "PTM", 6, false},
        {ui->lineEdit_Kal_roll, "rKlmf", 4, false},
        {ui->lineEdit_Kal_pitch, "pKlmf", 4, false},
        {ui->lineEdit_Kal_gyroroll, "rKlmv", 4, false},
        {ui->lineEdit_Kal_gyropitch, "pKlmv", 4, false},
        {ui->lineEdit_CFAlpha, "CFAlpha", 4, true},
        {ui->lineEdit_LPFAlpha, "LPFAlp", 4, true},
    };
    for (const FieldWire &row : fields) {
        QLineEdit *edit = row.edit;
        const QString param = QString::fromLatin1(row.param);
        const int precision = row.precision;
        const bool saveAfter = row.saveAfter;
        connect(edit, &QLineEdit::returnPressed, this, [this, edit, param, precision, saveAfter]() {
            sendParamFromField(edit, param, precision, saveAfter);
        });
    }
    connect(ui->lineEdit_setPitch, &QLineEdit::returnPressed, this, [this]() {
        bool ok = false;
        const double deg = ui->lineEdit_setPitch->text().toDouble(&ok);
        if (!ok)
            return;
        sendCommand(QStringLiteral("p SetPt %1").arg(qRound(deg * kDegToMilliRad)));
    });
}

void Dialog::setupSerialLink()
{
    m_linkProbeTimer = new QTimer(this);
    m_linkProbeTimer->setInterval(2000);
    QObject::connect(m_linkProbeTimer, &QTimer::timeout, this, &Dialog::probeUavLink);
    QObject::connect(serialPort, &QSerialPort::readyRead,
                     this, &Dialog::handleSerialData);
    QObject::connect(serialPort, &QSerialPort::errorOccurred, this,
                     [this](QSerialPort::SerialPortError err) {
        if (err != QSerialPort::NoError) {
            if (err == QSerialPort::ResourceError)
                m_uavLinkAlive = false;
            updateSerialUi();
        }
    });

    openSerial();
    m_linkProbeTimer->start();
    probeUavLink();
}

void Dialog::sendParamFromField(QLineEdit *field, const QString &paramName,
                                int precision, bool saveAfter)
{
    if (!field)
        return;
    bool ok = false;
    const double value = field->text().toDouble(&ok);
    if (!ok)
        return;
    sendCommand(QStringLiteral("p %1 %2").arg(paramName).arg(value, 0, 'f', precision));
    if (saveAfter)
        sendCommand(QStringLiteral("save"));
}

void Dialog::sendParamIfValid(QLineEdit *field, const QString &paramName)
{
    if (!field || field->text().trimmed().isEmpty())
        return;
    bool ok = false;
    const double value = field->text().toDouble(&ok);
    if (!ok)
        return;
    sendCommand(QStringLiteral("p %1 %2").arg(paramName).arg(value, 0, 'g', 10));
}

Dialog::~Dialog()
{
    if (serialPort->isOpen())
        serialPort->close();
    delete ui;
}

void Dialog::openSerial()
{
    // ESP8266/ESP32 port name — check with `ls /dev/ttyUSB*`
    serialPort->setPortName(QString::fromLatin1(kSerialPort));
    serialPort->setBaudRate(QSerialPort::Baud115200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort->open(QIODevice::ReadWrite)) {
        qDebug() << "Failed to open" << serialPort->portName()
                 << ":" << serialPort->errorString();
        m_uavLinkAlive = false;
    } else {
        qDebug() << "Opened" << serialPort->portName();
        m_uavLinkAlive = false;
    }
    updateSerialUi();
}

void Dialog::setUavLinkAlive(bool alive)
{
    if (m_uavLinkAlive == alive)
        return;
    m_uavLinkAlive = alive;
    updateSerialUi();
}

void Dialog::noteFirmwareRx(const QString &line)
{
    Q_UNUSED(line);
    m_lastFirmwareRxMs = QDateTime::currentMSecsSinceEpoch();
    setUavLinkAlive(true);
}

void Dialog::probeUavLink()
{
    if (!serialPort->isOpen()) {
        setUavLinkAlive(false);
        return;
    }

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    constexpr qint64 kLinkTimeoutMs = 5000;
    if (m_uavLinkAlive && m_lastFirmwareRxMs > 0 &&
        now - m_lastFirmwareRxMs > kLinkTimeoutMs) {
        setUavLinkAlive(false);
    }

    sendCommand(QStringLiteral("sys"));
}

void Dialog::updateSerialUi()
{
    const bool connected = serialPort->isOpen() && m_uavLinkAlive;
    if (connected) {
        ui->labelSerialState->setText(QStringLiteral("Connected"));
        ui->labelSerialState->setStyleSheet(
            QStringLiteral("background-color: #27AE60; color: white; font-weight: bold;"));
    } else {
        ui->labelSerialState->setText(QStringLiteral("Not connected"));
        ui->labelSerialState->setStyleSheet(
            QStringLiteral("background-color: #C0392B; color: white; font-weight: bold;"));
    }
}

void Dialog::applyDefaultPitchOnlyChannels()
{
    ui->checkBoxPitch->blockSignals(true);
    ui->checkBoxPitch->setChecked(true);
    ui->checkBoxPitch->blockSignals(false);

    exitManualMode();

    if (!serialPort->isOpen())
        return;
    // Disable roll/yaw torque first, then enable pitch (matches bench default; pitch-on resets controllers).
    sendCommand("dscnl 1 1");
    sendCommand("dscnl 3 1");
    sendCommand("dscnl 2 0");
}

void Dialog::exitManualMode()
{
    if (!m_manualMode) return;
    m_manualMode = false;
    ui->sliderManualFront->blockSignals(true);
    ui->sliderManualRear->blockSignals(true);
    ui->sliderManualFront->setValue(0);
    ui->sliderManualRear->setValue(0);
    ui->sliderManualFront->blockSignals(false);
    ui->sliderManualRear->blockSignals(false);
    ui->progressManualFront->setValue(0);
    ui->progressManualRear->setValue(0);
    sendCommand("mfront 0");
    sendCommand("mrear 0");
    ui->labelManualMode->setText("Manual OFF");
}

void Dialog::syncPitchSourceToFirmware()
{
    if (!serialPort->isOpen())
        return;
    if (ui->rbQuaternion->isChecked()) {
        sendCommand(QStringLiteral("p QuatEn 1"));
        sendCommand(ui->rbUseLPF->isChecked()
                        ? QStringLiteral("p LPF_En 1")
                        : QStringLiteral("p LPF_En 0"));
    } else {
        sendCommand(QStringLiteral("p QuatEn 0"));
        if (ui->rbNoLPF->isChecked())
            sendCommand(QStringLiteral("p LPF_En 0"));
        else if (ui->rbKalmanFilt->isChecked())
            sendCommand(QStringLiteral("p LPF_En 3"));
        else
            sendCommand(QStringLiteral("p LPF_En 2"));
    }
}

void Dialog::sendPdPiParams()
{
    sendParamIfValid(ui->lineEdit_Kpf,          QStringLiteral("RKPF"));
    sendParamIfValid(ui->lineEdit_Kpv,          QStringLiteral("RKPV"));
    sendParamIfValid(ui->lineEdit_Kdv,          QStringLiteral("RKDV"));
    sendParamIfValid(ui->lineEdit_RTv,          QStringLiteral("RTV"));
    sendParamIfValid(ui->lineEdit_RTmu,         QStringLiteral("RTmu"));
    sendParamIfValid(ui->lineEdit_RTi,          QStringLiteral("RTI"));
    sendParamIfValid(ui->lineEdit_RTe,          QStringLiteral("RTE"));
    sendParamIfValid(ui->lineEdit_RTm,          QStringLiteral("RTM"));
    sendParamIfValid(ui->lineEdit_PKPF,         QStringLiteral("PKPF"));
    sendParamIfValid(ui->lineEdit_PKPV,         QStringLiteral("PKPV"));
    sendParamIfValid(ui->lineEdit_PKDV,         QStringLiteral("PKDV"));
    sendParamIfValid(ui->lineEdit_PTV,          QStringLiteral("PTV"));
    sendParamIfValid(ui->lineEdit_PTMU,         QStringLiteral("PTmu"));
    sendParamIfValid(ui->lineEdit_PTI,          QStringLiteral("PTI"));
    sendParamIfValid(ui->lineEdit_PTE,          QStringLiteral("PTE"));
    sendParamIfValid(ui->lineEdit_PTM,          QStringLiteral("PTM"));
    sendParamIfValid(ui->lineEdit_Kal_roll,     QStringLiteral("rKlmf"));
    sendParamIfValid(ui->lineEdit_Kal_pitch,    QStringLiteral("pKlmf"));
    sendParamIfValid(ui->lineEdit_Kal_gyroroll, QStringLiteral("rKlmv"));
    sendParamIfValid(ui->lineEdit_Kal_gyropitch, QStringLiteral("pKlmv"));
}

void Dialog::sendAllParams()
{
    sendPdPiParams();

    sendParamIfValid(m_pidPitchP,     QStringLiteral("PITCH_P"));
    sendParamIfValid(m_pidPitchI,     QStringLiteral("PITCH_I"));
    sendParamIfValid(m_pidPitchD,     QStringLiteral("PITCH_D"));
    sendParamIfValid(m_pidPitchRateP, QStringLiteral("PRateP"));
    sendParamIfValid(m_pidPitchRateI, QStringLiteral("PRateI"));
    sendParamIfValid(m_pidPitchRateD, QStringLiteral("PRateD"));
    sendParamIfValid(ui->lineEdit_LPFAlpha, QStringLiteral("LPFAlp"));
    sendParamIfValid(ui->lineEdit_CFAlpha,  QStringLiteral("CFAlpha"));
    sendParamIfValid(ui->lineEdit_KQang,    QStringLiteral("KQang"));
    sendParamIfValid(ui->lineEdit_KRmeas,   QStringLiteral("KRmeas"));
    sendParamIfValid(m_smcLambda, QStringLiteral("SMCLm"));
    sendParamIfValid(m_smcK,       QStringLiteral("SMCK"));
    sendParamIfValid(m_smcPhi,     QStringLiteral("SMCPhi"));
    sendParamIfValid(m_smcKrate,   QStringLiteral("SMCKrt"));
    sendParamIfValid(m_smcKi,      QStringLiteral("SMCKi"));
    sendParamIfValid(m_smcIMax,    QStringLiteral("SMCImax"));
    sendParamIfValid(m_smcVMax,    QStringLiteral("SMCVmax"));
    sendParamIfValid(m_smcLpfAlpha, QStringLiteral("SMCLpfa"));
    sendParamIfValid(m_smcLpfTau,  QStringLiteral("SMCLpft"));

    sendCommand(QStringLiteral("p CtrlAlg %1").arg(m_ctrlAlgMode));
}

void Dialog::resetControllerDefaults()
{
    auto setLe = [](QLineEdit *le, const QString &text) {
        if (le)
            le->setText(text);
    };

    // PD-PI (bench defaults from dialog.ui)
    setLe(ui->lineEdit_Kal_roll,      QStringLiteral("0.05"));
    setLe(ui->lineEdit_Kal_gyroroll,  QStringLiteral("0.05"));
    setLe(ui->lineEdit_Kal_pitch,     QStringLiteral("0.1"));
    setLe(ui->lineEdit_Kal_gyropitch, QStringLiteral("0.1"));
    setLe(ui->lineEdit_Kpf,           QStringLiteral("1.3"));
    setLe(ui->lineEdit_Kpv,           QStringLiteral("0.2"));
    setLe(ui->lineEdit_Kdv,           QStringLiteral("0.015"));
    setLe(ui->lineEdit_RTv,           QStringLiteral("0.01"));
    setLe(ui->lineEdit_RTmu,          QStringLiteral("0.02"));
    setLe(ui->lineEdit_RTi,           QStringLiteral("0.25"));
    setLe(ui->lineEdit_RTe,           QStringLiteral("0.3"));
    setLe(ui->lineEdit_RTm,           QStringLiteral("0.05"));
    setLe(ui->lineEdit_PKPF,          QStringLiteral("0.8"));
    setLe(ui->lineEdit_PKPV,          QStringLiteral("0.05"));
    setLe(ui->lineEdit_PKDV,          QStringLiteral("0.01"));
    setLe(ui->lineEdit_PTV,           QStringLiteral("0.01"));
    setLe(ui->lineEdit_PTMU,          QStringLiteral("0.02"));
    setLe(ui->lineEdit_PTI,           QStringLiteral("0.2"));
    setLe(ui->lineEdit_PTE,           QStringLiteral("0.015"));
    setLe(ui->lineEdit_PTM,           QStringLiteral("0.05"));

    // Classic PID pitch cascade
    setLe(m_pidPitchP,     QStringLiteral("2.5"));
    setLe(m_pidPitchI,     QStringLiteral("0.0"));
    setLe(m_pidPitchD,     QStringLiteral("0.0"));
    setLe(m_pidPitchRateP, QStringLiteral("0.18"));
    setLe(m_pidPitchRateI, QStringLiteral("0.025"));
    setLe(m_pidPitchRateD, QStringLiteral("0.005"));

    // SMC
    setLe(m_smcLambda,   QStringLiteral("2.0"));
    setLe(m_smcK,        QStringLiteral("0.07"));
    setLe(m_smcPhi,      QStringLiteral("0.8"));
    setLe(m_smcKrate,    QStringLiteral("1.0"));
    setLe(m_smcKi,       QStringLiteral("0.2"));
    setLe(m_smcIMax,     QStringLiteral("0.5"));
    setLe(m_smcVMax,     QStringLiteral("15.0"));
    setLe(m_smcLpfAlpha, QStringLiteral("0.1"));
    setLe(m_smcLpfTau,   QStringLiteral("0.0"));

    QSettings settings(QStringLiteral("UAV_Lon_ESP8266"), QStringLiteral("Qt_Flix"));
    settings.setValue(QStringLiteral("smcLambda"),   2.0);
    settings.setValue(QStringLiteral("smcK"),          0.07);
    settings.setValue(QStringLiteral("smcPhi"),      0.8);
    settings.setValue(QStringLiteral("smcKrate"),    1.0);
    settings.setValue(QStringLiteral("smcKi"),       0.2);
    settings.setValue(QStringLiteral("smcIMax"),     0.5);
    settings.setValue(QStringLiteral("smcVMax"),     15.0);
    settings.setValue(QStringLiteral("smcLpfAlpha"), 0.1);
    settings.setValue(QStringLiteral("smcLpfTau"),   0.0);

    sendAllParams();
    sendCommand(QStringLiteral("save"));

    ui->textEditLog->append(QStringLiteral("Set Default: controller coefficients restored."));
}

void Dialog::setupControlTabs()
{
    m_ctrlTabs = new QTabWidget(this);
    constexpr int kGetParamsColumnLeft = 420;  // textEditLog / Get parameters — do not overlap
    constexpr int kPanelMargin = 10;
    constexpr int kPanelBaseY = 282;
    const int yShift = qRound(3.0 / 2.54 * logicalDpiY());
    const QRect tabGeo(kPanelMargin, kPanelBaseY + yShift,
                       kGetParamsColumnLeft - kPanelMargin * 2, 450);
    m_ctrlTabs->setGeometry(tabGeo);
    m_ctrlTabs->raise();
    m_ctrlTabs->tabBar()->setExpanding(true);

    m_tabPdPi = new QWidget;
    m_tabPid = new QWidget;
    m_ctrlTabs->addTab(m_tabPdPi, QStringLiteral("PD-PI"));
    m_ctrlTabs->addTab(m_tabPid, QStringLiteral("Classic PID"));
    m_tabSmc = new QWidget;
    m_ctrlTabs->addTab(m_tabSmc, QStringLiteral("SMC"));

    QFont tabFont = m_ctrlTabs->tabBar()->font();
    tabFont.setBold(true);
    m_ctrlTabs->tabBar()->setFont(tabFont);

    const int tabBarH = qMax(28, m_ctrlTabs->tabBar()->sizeHint().height());
    const QPoint originInDialog(tabGeo.x(), tabGeo.y() + tabBarH - 30);

    auto centerTabContents = [&](QWidget *tab) {
        const auto kids = tab->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly);
        if (kids.isEmpty())
            return;
        int minX = kids.first()->x();
        int maxX = 0;
        for (QWidget *w : kids) {
            minX = qMin(minX, w->x());
            maxX = qMax(maxX, w->x() + w->width());
        }
        const int xOffset = (tabGeo.width() - (maxX - minX)) / 2 - minX;
        for (QWidget *w : kids)
            w->move(w->x() + xOffset, w->y());
    };

    auto reparentPdPi = [&](QWidget *w) {
        if (!w)
            return;
        const QPoint p = w->pos();
        w->setParent(m_tabPdPi);
        w->move(p - originInDialog);
    };

    const QList<QWidget *> pdpiWidgets = {
        ui->label, ui->lineEdit_Kpf, ui->label_2, ui->lineEdit_Kpv, ui->label_3, ui->lineEdit_Kdv,
        ui->label_8, ui->lineEdit_RTv, ui->label_9, ui->lineEdit_RTmu,
        ui->label_7, ui->lineEdit_RTe, ui->label_5, ui->lineEdit_RTi, ui->label_6, ui->lineEdit_RTm,
        ui->label_10, ui->lineEdit_PKPF, ui->label_17, ui->lineEdit_PKPV, ui->label_14, ui->lineEdit_PKDV,
        ui->label_11, ui->lineEdit_PTV, ui->label_12, ui->lineEdit_PTMU,
        ui->label_13, ui->lineEdit_PTI, ui->label_15, ui->lineEdit_PTE, ui->label_16, ui->lineEdit_PTM,
        ui->label_20, ui->lineEdit_Kal_roll, ui->label_21, ui->lineEdit_Kal_pitch,
        ui->label_23, ui->lineEdit_Kal_gyroroll, ui->label_22, ui->lineEdit_Kal_gyropitch,
    };
    for (QWidget *w : pdpiWidgets) {
        w->move(w->x(), w->y() + yShift);
        reparentPdPi(w);
    }
    centerTabContents(m_tabPdPi);

    auto addPidRow = [&](int rowY, const QString &title, QLineEdit **outEdit, const QString &defVal) {
        auto *lb = new QLabel(title, m_tabPid);
        lb->move(10, rowY);
        *outEdit = new QLineEdit(m_tabPid);
        (*outEdit)->setGeometry(100, rowY - 2, qMin(200, tabGeo.width() - 120), 26);
        (*outEdit)->setText(defVal);
    };

    auto *hint = new QLabel(
        QStringLiteral("(Bench pitch: 6 fields — pitch angle + ω_pitch; tune roll PID via `p` commands)"),
        m_tabPid);
    hint->move(10, 8);
    hint->setStyleSheet(QStringLiteral("color: gray;"));
    hint->setWordWrap(true);
    hint->setGeometry(10, 8, tabGeo.width() - 20, 32);

    addPidRow(44, QStringLiteral("Pitch θ Kp"), &m_pidPitchP, QStringLiteral("2.5"));
    addPidRow(76, QStringLiteral("Pitch θ Ki"), &m_pidPitchI, QStringLiteral("0.0"));
    addPidRow(108, QStringLiteral("Pitch θ Kd"), &m_pidPitchD, QStringLiteral("0.0"));
    addPidRow(154, QStringLiteral("Pitch ω Kp"), &m_pidPitchRateP, QStringLiteral("0.18"));
    addPidRow(186, QStringLiteral("Pitch ω Ki"), &m_pidPitchRateI, QStringLiteral("0.025"));
    addPidRow(218, QStringLiteral("Pitch ω Kd"), &m_pidPitchRateD, QStringLiteral("0.005"));
    centerTabContents(m_tabPid);

    auto *smcHint = new QLabel(
        QStringLiteral("v_sp=λe (±Vmax), s=v_sp−K_rate·ω\n"
                       "u=Umax·tanh(K·s/Φ)+Ki∫e → LPF"),
        m_tabSmc);
    smcHint->setGeometry(10, 8, tabGeo.width() - 20, 40);
    smcHint->setStyleSheet(QStringLiteral("color: gray;"));
    smcHint->setWordWrap(true);

    QSettings settings(QStringLiteral("UAV_Lon_ESP8266"), QStringLiteral("Qt_Flix"));

    auto addSmcRow = [&](int rowY, const QString &title, QLineEdit **outEdit, const QString &defVal) {
        auto *lb = new QLabel(title, m_tabSmc);
        lb->move(10, rowY);
        *outEdit = new QLineEdit(m_tabSmc);
        (*outEdit)->setGeometry(130, rowY - 2, qMin(150, tabGeo.width() - 150), 26);
        (*outEdit)->setText(defVal);
    };
    addSmcRow(48,  QStringLiteral("λ (SMCLm)"),      &m_smcLambda, settings.value(QStringLiteral("smcLambda"), 2.0).toString());
    addSmcRow(80,  QStringLiteral("K (SMCK)"),       &m_smcK,      settings.value(QStringLiteral("smcK"), 0.07).toString());
    addSmcRow(112, QStringLiteral("Φ (SMCPhi)"),     &m_smcPhi,    settings.value(QStringLiteral("smcPhi"), 0.8).toString());
    addSmcRow(144, QStringLiteral("K_rate (SMCKrt)"), &m_smcKrate, settings.value(QStringLiteral("smcKrate"), 1.0).toString());
    addSmcRow(176, QStringLiteral("Ki (SMCKi)"),      &m_smcKi,     settings.value(QStringLiteral("smcKi"), 0.2).toString());
    addSmcRow(208, QStringLiteral("I_max (SMCImax)"), &m_smcIMax,  settings.value(QStringLiteral("smcIMax"), 0.5).toString());
    addSmcRow(240, QStringLiteral("V_max (SMCVmax)"), &m_smcVMax,  settings.value(QStringLiteral("smcVMax"), 15.0).toString());
    addSmcRow(272, QStringLiteral("LPF α (SMCLpfa)"), &m_smcLpfAlpha, settings.value(QStringLiteral("smcLpfAlpha"), 0.1).toString());
    addSmcRow(304, QStringLiteral("LPF τ (SMCLpft)"), &m_smcLpfTau, settings.value(QStringLiteral("smcLpfTau"), 0.0).toString());
    centerTabContents(m_tabSmc);

    auto wireSmcField = [this](QLineEdit *le, const QString &paramName, const char *settingsKey) {
        QObject::connect(le, &QLineEdit::returnPressed, this, [this, le, paramName, settingsKey]() {
            bool ok = false;
            const double v = le->text().toDouble(&ok);
            if (!ok)
                return;
            sendCommand(QStringLiteral("p %1 %2").arg(paramName).arg(v, 0, 'g', 10));
            sendCommand(QStringLiteral("save"));
            QSettings(QStringLiteral("UAV_Lon_ESP8266"), QStringLiteral("Qt_Flix"))
                .setValue(QString::fromLatin1(settingsKey), v);
        });
    };

    auto wirePidField = [this](QLineEdit *le, const QString &paramName) {
        QObject::connect(le, &QLineEdit::returnPressed, this, [this, le, paramName]() {
            bool ok = false;
            const double v = le->text().toDouble(&ok);
            if (!ok)
                return;
            sendCommand(QStringLiteral("p %1 %2").arg(paramName).arg(v, 0, 'g', 10));
        });
    };
    wirePidField(m_pidPitchP, QStringLiteral("PITCH_P"));
    wirePidField(m_pidPitchI, QStringLiteral("PITCH_I"));
    wirePidField(m_pidPitchD, QStringLiteral("PITCH_D"));
    wirePidField(m_pidPitchRateP, QStringLiteral("PRateP"));
    wirePidField(m_pidPitchRateI, QStringLiteral("PRateI"));
    wirePidField(m_pidPitchRateD, QStringLiteral("PRateD"));
    wireSmcField(m_smcK,      QStringLiteral("SMCK"),    "smcK");
    wireSmcField(m_smcLambda, QStringLiteral("SMCLm"),   "smcLambda");
    wireSmcField(m_smcPhi,    QStringLiteral("SMCPhi"),  "smcPhi");
    wireSmcField(m_smcKrate,  QStringLiteral("SMCKrt"),  "smcKrate");
    wireSmcField(m_smcKi,     QStringLiteral("SMCKi"),   "smcKi");
    wireSmcField(m_smcIMax,   QStringLiteral("SMCImax"), "smcIMax");
    wireSmcField(m_smcVMax,   QStringLiteral("SMCVmax"), "smcVMax");
    wireSmcField(m_smcLpfAlpha, QStringLiteral("SMCLpfa"), "smcLpfAlpha");
    wireSmcField(m_smcLpfTau, QStringLiteral("SMCLpft"), "smcLpfTau");

    wirePidField(ui->lineEdit_KQang,  QStringLiteral("KQang"));
    wirePidField(ui->lineEdit_KRmeas, QStringLiteral("KRmeas"));

    QObject::connect(m_ctrlTabs, &QTabWidget::currentChanged, this, [this](int index) {
        if (index >= 0 && index <= 2 && index != m_ctrlAlgMode)
            setCtrlAlgMode(index, true);
    });

    m_btnSetDefault = new QPushButton(QStringLiteral("Set Default"), this);
    const int moveUpPx = qRound(1.0 / 2.54 * logicalDpiY());  // 1 cm above tab bottom
    const int btnY = tabGeo.y() + tabGeo.height() - 42 - moveUpPx;
    m_btnSetDefault->setGeometry(tabGeo.x() + 10, btnY, tabGeo.width() - 20, 34);
    QFont setDefaultFont = m_btnSetDefault->font();
    setDefaultFont.setBold(true);
    m_btnSetDefault->setFont(setDefaultFont);
    m_btnSetDefault->setToolTip(
        QStringLiteral("Restore safe bench controller gains (PD-PI / PID / SMC)"));
    m_btnSetDefault->setAutoDefault(false);
    m_btnSetDefault->setDefault(false);
    m_btnSetDefault->setFocusPolicy(Qt::NoFocus);
    m_btnSetDefault->raise();
    QObject::connect(m_btnSetDefault, &QPushButton::clicked, this, [this]() {
        resetControllerDefaults();
    });
}

void Dialog::setupCtrlAlgOutsideTabs()
{
    constexpr int modeLeft = 10;
    constexpr int kModeBaseTop = 740;
    const int yShift = qRound(3.0 / 2.54 * logicalDpiY());
    const int modeTop = kModeBaseTop + yShift;
    auto *algLb = new QLabel(QStringLiteral("Control mode"), this);
    algLb->setGeometry(modeLeft, modeTop, 95, 28);

    m_btnCtrlPdPi = new QPushButton(QStringLiteral("PD-PI"), this);
    m_btnCtrlPid = new QPushButton(QStringLiteral("PID"), this);
    m_btnCtrlPdPi->setGeometry(modeLeft + 100, modeTop, 55, 28);
    m_btnCtrlPid->setGeometry(modeLeft + 160, modeTop, 50, 28);
    m_btnCtrlPdPi->setCheckable(true);
    m_btnCtrlPid->setCheckable(true);
    m_btnCtrlPdPi->setAutoDefault(false);
    m_btnCtrlPid->setAutoDefault(false);
    m_btnCtrlPdPi->setDefault(false);
    m_btnCtrlPid->setDefault(false);
    m_btnCtrlPdPi->setFocusPolicy(Qt::NoFocus);
    m_btnCtrlPid->setFocusPolicy(Qt::NoFocus);
    m_btnCtrlPdPi->setToolTip(QStringLiteral("Use PD-PI KrenCtrl: p CtrlAlg 0"));
    m_btnCtrlPid->setToolTip(QStringLiteral("Use classical cascade PID: p CtrlAlg 1"));

    QSettings settings(QStringLiteral("UAV_Lon_ESP8266"), QStringLiteral("Qt_Flix"));
    m_ctrlAlgMode = settings.value(QStringLiteral("ctrlAlgMode"), 0).toInt();
    if (m_ctrlAlgMode != 0 && m_ctrlAlgMode != 2)
        m_ctrlAlgMode = 1;

    QObject::connect(m_btnCtrlPdPi, &QPushButton::clicked, this, [this]() {
        setCtrlAlgMode(0, true);
    });
    QObject::connect(m_btnCtrlPid, &QPushButton::clicked, this, [this]() {
        setCtrlAlgMode(1, true);
    });

    m_btnCtrlSmc = new QPushButton(QStringLiteral("SMC"), this);
    m_btnCtrlSmc->setGeometry(modeLeft + 215, modeTop, 50, 28);
    m_btnCtrlSmc->setCheckable(true);
    m_btnCtrlSmc->setAutoDefault(false);
    m_btnCtrlSmc->setDefault(false);
    m_btnCtrlSmc->setFocusPolicy(Qt::NoFocus);
    m_btnCtrlSmc->setToolTip(QStringLiteral("Sliding Mode Controller: p CtrlAlg 2"));

    QObject::connect(m_btnCtrlSmc, &QPushButton::clicked, this, [this]() {
        setCtrlAlgMode(2, true);
    });

    // All 3 mode buttons created — refresh UI state
    setCtrlAlgMode(m_ctrlAlgMode, false);

    algLb->raise();
    m_btnCtrlPdPi->raise();
    m_btnCtrlPid->raise();
    m_btnCtrlSmc->raise();
}

void Dialog::setCtrlAlgMode(int mode, bool sendToFirmware)
{
    m_ctrlAlgMode = (mode == 0) ? 0 : (mode == 2) ? 2 : 1;

    if (m_btnCtrlPdPi && m_btnCtrlPid && m_btnCtrlSmc) {
        m_btnCtrlPdPi->blockSignals(true);
        m_btnCtrlPid->blockSignals(true);
        m_btnCtrlSmc->blockSignals(true);
        m_btnCtrlPdPi->setChecked(m_ctrlAlgMode == 0);
        m_btnCtrlPid->setChecked(m_ctrlAlgMode == 1);
        m_btnCtrlSmc->setChecked(m_ctrlAlgMode == 2);
        m_btnCtrlPdPi->blockSignals(false);
        m_btnCtrlPid->blockSignals(false);
        m_btnCtrlSmc->blockSignals(false);

        const QString active = QStringLiteral(
            "background-color: #2d7dd2; color: white; font-weight: bold;");
        const QString inactive = QStringLiteral("");
        m_btnCtrlPdPi->setStyleSheet(m_ctrlAlgMode == 0 ? active : inactive);
        m_btnCtrlPid->setStyleSheet(m_ctrlAlgMode  == 1 ? active : inactive);
        m_btnCtrlSmc->setStyleSheet(m_ctrlAlgMode  == 2 ? active : inactive);
    }

    if (m_ctrlTabs) {
        m_ctrlTabs->blockSignals(true);
        m_ctrlTabs->setCurrentIndex(m_ctrlAlgMode);
        m_ctrlTabs->blockSignals(false);
    }

    QSettings settings(QStringLiteral("UAV_Lon_ESP8266"), QStringLiteral("Qt_Flix"));
    settings.setValue(QStringLiteral("ctrlAlgMode"), m_ctrlAlgMode);

    if (sendToFirmware) {
        sendCommand(QStringLiteral("p CtrlAlg %1").arg(m_ctrlAlgMode));
        sendCommand(QStringLiteral("save"));
    }
}

void Dialog::setPlotLegendCheckboxesVisible(bool visible)
{
    ui->checkBoxPlotRoll->setVisible(visible);
    ui->checkBoxPlotKlmRoll->setVisible(visible);
    ui->checkBoxPlotGyro->setVisible(visible);
    ui->checkBoxPlotKlmGyro->setVisible(visible);
    ui->checkBoxPlotTorque->setVisible(visible);
}

void Dialog::initUav3D(QWidget *host)
{
    m_view3d = new Qt3DExtras::Qt3DWindow();
    m_view3d->defaultFrameGraph()->setClearColor(QColor(245,245,245));

    m_viewContainer = QWidget::createWindowContainer(m_view3d, host);
    m_viewContainer->setMinimumSize(QSize(300, 300));

    if (!host->layout())
        host->setLayout(new QVBoxLayout());
    host->layout()->setContentsMargins(0,0,0,0);
    host->layout()->addWidget(m_viewContainer);

    m_root3d = new Qt3DCore::QEntity();

    // Camera
    auto *cam = m_view3d->camera();
    cam->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.01f, 1000.0f);
    cam->setPosition(QVector3D(0, 0.3f, 1.0f));
    cam->setViewCenter(QVector3D(0, 0, 0));

    auto *camCtrl = new Qt3DExtras::QOrbitCameraController(m_root3d);
    camCtrl->setCamera(cam);

    // UAV entity
    auto *uavEntity = new Qt3DCore::QEntity(m_root3d);

    auto *mesh = new Qt3DRender::QMesh();
    const QString stlPath = resolveUavStlPath();
    if (stlPath.isEmpty()) {
        qWarning() << "uav.stl not found — place file in flixESPDrone/uav.stl (or run Qt_Flix from UAV_Lon_ESP8266 tree)";
    } else {
        mesh->setSource(QUrl::fromLocalFile(stlPath));
        qDebug() << "STL UAV:" << stlPath;
        QObject::connect(mesh, &Qt3DRender::QMesh::statusChanged,
                         mesh, [stlPath](Qt3DRender::QMesh::Status st) {
                             if (st == Qt3DRender::QMesh::Error)
                                 qWarning() << "STL load error:" << stlPath;
                         });
    }

    m_uavTf = new Qt3DCore::QTransform();
    m_uavTf->setScale(0.005f); // adjust for STL size
    m_modelOffset = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90.0f);

    auto *mat = new Qt3DExtras::QPhongMaterial();

    uavEntity->addComponent(mesh);
    uavEntity->addComponent(m_uavTf);
    uavEntity->addComponent(mat);

    m_view3d->setRootEntity(m_root3d);
}
void Dialog::setUavAttitudeDeg(float rollDeg, float pitchDeg)
{
    if (!m_uavTf) return;

    // After STL centering (m_modelOffset): roll ~ Y mesh, pitch ~ X mesh — flip sign via m_rollSign / m_pitchSign if inverted
    const QVector3D rollAxis = m_modelOffset.rotatedVector(QVector3D(0, 1, 0)).normalized();
    const QVector3D pitchAxis = m_modelOffset.rotatedVector(QVector3D(1, 0, 0)).normalized();

    const QQuaternion qPitch = QQuaternion::fromAxisAndAngle(pitchAxis, m_pitchSign * pitchDeg);
    const QQuaternion qRoll = QQuaternion::fromAxisAndAngle(rollAxis, m_rollSign * rollDeg);

    m_uavTf->setRotation(qRoll * qPitch * m_modelOffset);
}

void Dialog::setupPitchTelemetryPlot()
{
    ui->customPlot->clearGraphs();

    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setName(QStringLiteral("pitch θ (°)"));
    ui->customPlot->graph(0)->setPen(QPen(Qt::blue));

    ui->customPlot->addGraph();
    ui->customPlot->graph(1)->setName(QStringLiteral("pitch ω (°/s)"));
    ui->customPlot->graph(1)->setPen(QPen(Qt::red));

    ui->customPlot->addGraph();
    ui->customPlot->graph(2)->setName(QStringLiteral("Torque pitch"));
    ui->customPlot->graph(2)->setPen(QPen(Qt::darkGreen));

    ui->customPlot->xAxis->setLabel(QStringLiteral("samples"));
    ui->customPlot->yAxis->setLabel(QStringLiteral("value"));
    ui->customPlot->legend->setVisible(true);
    ui->customPlot->setInteraction(QCP::iRangeDrag, true);
    ui->customPlot->setInteraction(QCP::iRangeZoom, true);

    updatePitchTelPlotVisibility();
    ui->customPlot->replot();
}

void Dialog::addLogSample2(double pitchDeg, double pitchOmegaDegS, int torqueY1000)
{
    logIndex2 += 1.0;

    logTime2.append(logIndex2);
    logM2_pitchThetaRad.append(pitchDeg);
    logM2_pitchOmegaRadS.append(pitchOmegaDegS);
    logM2_torqueY1000.append(torqueY1000);

    const int maxPoints = 1000;
    if (logTime2.size() > maxPoints) {
        const int extra = logTime2.size() - maxPoints;
        logTime2.remove(0, extra);
        logM2_pitchThetaRad.remove(0, extra);
        logM2_pitchOmegaRadS.remove(0, extra);
        logM2_torqueY1000.remove(0, extra);
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if (nowMs - m_lastTelemetryRedrawMs < kTelemetryRedrawIntervalMs)
        return;
    m_lastTelemetryRedrawMs = nowMs;

    if (ui->customPlot->graphCount() >= 3) {
        ui->customPlot->graph(0)->setData(logTime2, logM2_pitchThetaRad);
        ui->customPlot->graph(1)->setData(logTime2, logM2_pitchOmegaRadS);
        ui->customPlot->graph(2)->setData(logTime2, logM2_torqueY1000);
    }

    updatePitchTelPlotVisibility();

    if (!logTime2.isEmpty()) {
        ui->customPlot->xAxis->setRange(logTime2.last(), 200, Qt::AlignRight);
    }
    ui->customPlot->yAxis->rescale(true);
    ui->customPlot->replot();

    setUavAttitudeDeg(m_pitchDegFiltered, 0.f);
}
bool Dialog::isTakeLogDataLine(const QString &line)
{
    const QStringList p = line.split(',');
    if (p.size() != 3)
        return false;
    bool ok1, ok2, ok3;
    p[0].toInt(&ok1);
    p[1].toInt(&ok2);
    p[2].toInt(&ok3);
    return ok1 && ok2 && ok3;
}

void Dialog::on_btnSaveLog_clicked()
{
    // Send takelog → ESP32 dumps circular buffer at max rate over Serial.
    // Qt receives TAKELOG_START … TAKELOG_END then saves CSV in saveTakeLog().
    if (m_receivingTakeLog)
        return;  // already receiving, do not resend
    if (!serialPort->isOpen()) {
        ui->textEditLog->append(QStringLiteral("Dump Log: serial port not open"));
        return;
    }
    m_receivingTakeLog = true;
    m_takeLogStarted   = true;  // accept CSV lines immediately (UART may lead TAKELOG_START)
    m_takeLogExpected  = -1;
    m_takeLogRawLines.clear();
    ui->btnSaveLog->setEnabled(false);
    ui->btnSaveLog->setText("Receiving...");
    sendCommand("takelog");
}

void Dialog::saveTakeLog()
{
    QString baseDir = QDir::homePath() + "/Documents/UAV_Projects/project_quad/UAV_Lon_ESP8266/log";
    QDir().mkpath(baseDir);
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString fileName = baseDir + "/takelog_" + timestamp + ".csv";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "Failed to open file:" << fileName << file.errorString();
        ui->btnSaveLog->setEnabled(true);
        ui->btnSaveLog->setText("Dump Log");
        return;
    }

    QTextStream out(&file);
    out << "pitch_deg,gyro_dps,torque\n";
    static constexpr double kPitchScale  = 1e-4 * 57.29577951;  // ×10000 rad → deg
    static constexpr double kGyroScale   = 0.1;                  // decideg/s → deg/s
    static constexpr double kTorqueScale = 1e-4;                 // ×10000 normalized
    for (const QString &line : m_takeLogRawLines) {
        const QStringList p = line.split(',');
        if (p.size() == 3) {
            bool ok1, ok2, ok3;
            const int iv0 = p[0].toInt(&ok1);
            const int iv1 = p[1].toInt(&ok2);
            const int iv2 = p[2].toInt(&ok3);
            if (ok1 && ok2 && ok3) {
                out << QString::number(iv0 * kPitchScale,  'f', 3) << ','
                    << QString::number(iv1 * kGyroScale,   'f', 1) << ','
                    << QString::number(iv2 * kTorqueScale, 'f', 5) << '\n';
                continue;
            }
        }
        out << line << '\n';   // fallback: keep unparsed line as-is
    }
    file.close();

    const int sampleCount = m_takeLogRawLines.size();
    QString msg = QString("Dump Log saved: %1  (%2 samples)").arg(fileName).arg(sampleCount);
    if (sampleCount == 0) {
        msg += QStringLiteral(
            "\n  Buffer was empty. Wait ~6 s after connect/arm (live plot moving), "
            "then Dump Log. Reflash ESP32 if problem persists.");
    } else if (m_takeLogExpected > 0 && sampleCount < m_takeLogExpected) {
        msg += QString("\n  Warning: expected %1 samples, received %2")
                   .arg(m_takeLogExpected)
                   .arg(sampleCount);
    }
    ui->textEditLog->append(msg);
    qDebug() << msg;

    ui->btnSaveLog->setEnabled(true);
    ui->btnSaveLog->setText("Dump Log");
}


void Dialog::updateArmUi()
{
    if (isArmed) {
        // ARM button red, DISARM gray
        ui->push_arm->setStyleSheet("background-color: red; color: white; font-weight: bold;");
        ui->push_disarm->setStyleSheet("");

        ui->labelArmState->setText("ARMED");
        ui->labelArmState->setStyleSheet("background-color: red; color:white; font-weight:bold;");
    } else {
        ui->push_arm->setStyleSheet("");
        ui->push_disarm->setStyleSheet("background-color: green; color: white; font-weight: bold;");

        ui->labelArmState->setText("STOPPED");
        ui->labelArmState->setStyleSheet("background-color: green; color:white; font-weight:bold;");
    }
}

void Dialog::resetTakeLogReceiveState()
{
    m_receivingTakeLog = false;
    m_takeLogStarted   = false;
    m_takeLogExpected  = -1;
    m_takeLogRawLines.clear();
    ui->btnSaveLog->setEnabled(true);
    ui->btnSaveLog->setText(QStringLiteral("Dump Log"));
}

void Dialog::handleLogLine(const QString &line)
{
    if (looksLikeFirmwareLine(line) || looksLikeFirmwareTelemetry(line, currentLogMode))
        noteFirmwareRx(line);

    // ---- high-rate dump from takelog command ----
    if (m_receivingTakeLog) {
        if (line.startsWith(QStringLiteral("TAKELOG_START"))) {
            m_takeLogStarted = true;
            m_takeLogRawLines.clear();
            const QString rest = line.mid(QStringLiteral("TAKELOG_START").size()).trimmed();
            if (!rest.isEmpty()) {
                bool ok = false;
                const int n = rest.toInt(&ok);
                if (ok)
                    m_takeLogExpected = n;
            }
            return;
        }
        if (line == QStringLiteral("TAKELOG_END")) {
            m_receivingTakeLog = false;
            m_takeLogStarted   = false;
            saveTakeLog();              // must run before clear — lines live in m_takeLogRawLines
            resetTakeLogReceiveState();
            return;
        }
        if (m_takeLogStarted && isTakeLogDataLine(line)) {
            m_takeLogRawLines.append(line);
            return;
        }
        // Waiting for TAKELOG_START — do not block live telemetry.
    }

    QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);

    // col1: pitch milli-rad → degrees; col2: gyro deci-deg/s → deg/s; col3: torque×1000
    if (currentLogMode == 1) {
        const unsigned mask = pitchTelMaskFromUi();
        unsigned need = 0;
        for (unsigned t = mask; t != 0u; t >>= 1u)
            need += (t & 1u);
        if (need > 0 && static_cast<unsigned>(parts.size()) == need) {
            bool ok = true;
            int idx = 0;
            if (mask & 1u) {
                const int v = parts[idx++].toInt(&ok);
                if (!ok)
                    goto after_telemetry;
                m_lastTelPitchMilliRad = v;
                // milli-rad → rad → degrees
                const float pitchDeg = float(v) * kMilliRadToRad * kRadToDeg;
                m_pitchDegFiltered += m_attAlpha * (pitchDeg - m_pitchDegFiltered);
            }
            if (mask & 2u) {
                const int v = parts[idx++].toInt(&ok);
                if (!ok)
                    goto after_telemetry;
                m_lastTelPitchOmegaMilli = v;  // deci-deg/s
            }
            if (mask & 4u) {
                const int v = parts[idx++].toInt(&ok);
                if (!ok)
                    goto after_telemetry;
                m_lastTelUy = v;
            }
            // col1 → degrees; col2 deci-deg/s → deg/s
            addLogSample2(double(m_lastTelPitchMilliRad) * double(kMilliRadToRad) * double(kRadToDeg),
                          double(m_lastTelPitchOmegaMilli) / 10.0,
                          m_lastTelUy);
            return;
        }
    }
after_telemetry:

    // remaining: params, text...
    if (capturingParams) {
        if (line.contains(" = ")) {
            ui->textEditLog->append(line);
            return;
        } else {
            capturingParams = false;
        }
    }
}

void Dialog::sendCommand(const QString &cmd)
{
    if (!serialPort->isOpen()) {
        qDebug() << "Serial not open, cannot send:" << cmd;
        return;
    }

    QString line = cmd;
    if (!line.endsWith('\n'))
        line.append('\n');

    QByteArray data = line.toUtf8();
    serialPort->write(data);

    qDebug() << ">> sent:" << line.trimmed();
}


void Dialog::on_push_arm_clicked()
{
    if (ui->verticalSliderThrust->value() > 0) {
        qDebug() << "Cannot arm: stick not at safe position";
        ui->verticalSliderThrust->setValue(0);
        sendCommand("mtr 4 0");
        return;
    }
    sendCommand("arm");
    isArmed = true;
    updateArmUi();
    startLiveLog();
}

void Dialog::on_push_disarm_clicked()
{
    sendCommand("disarm");
    isArmed = false;
    updateArmUi();

    // always zero thrust on disarm
    ui->verticalSliderThrust->setValue(0);
    sendCommand("mtr 4 0");
    sendCommand("p SetRl 0");
    sendCommand("p SetPt 0");
    applyDefaultPitchOnlyChannels();
}

void Dialog::on_verticalSliderThrust_valueChanged(int value)
{
    ui->labelThrustPercent->setText(QStringLiteral("%1%").arg(value / 10));
    QString cmd = QString("mtr 4 %1").arg(value);
    sendCommand(cmd);
}

void Dialog::handleSerialData()
{
    static QString buffer;

    buffer += QString::fromUtf8(serialPort->readAll());

    int idx;
    while ((idx = buffer.indexOf('\n')) != -1) {
        QString line = buffer.left(idx);
        buffer.remove(0, idx + 1);

        line.remove('\r');
        line = line.trimmed();
        if (line.isEmpty())
            continue;
        handleLogLine(line);
    }
}

// ===== CHECKBOX ENABLE/DISABLE CONTROL CHANNELS (dscnl) =====

void Dialog::on_checkBoxPitch_toggled(bool checked)
{
    if (checked) {
        // enable pitch channel = dscnl 2 0
        sendCommand("dscnl 2 0");
        exitManualMode();
    } else {
        // disable pitch channel = dscnl 2 1
        sendCommand("dscnl 2 1");
    }
}

void Dialog::on_btnDisableAll_clicked()
{
    sendCommand("dscnl 4 1");
    ui->checkBoxPitch->blockSignals(true);
    ui->checkBoxPitch->setChecked(false);
    ui->checkBoxPitch->blockSignals(false);

    m_manualMode = true;
    ui->labelManualMode->setText("Manual mode selected");
}

void Dialog::on_save_para_clicked()
{
    sendCommand("save");
}

void Dialog::on_btnGetParams_clicked()
{
    ui->textEditLog->clear();
    capturingParams = true;
    sendCommand("p");
}

// ===== LOG ON/OFF =====

void Dialog::startLiveLog()
{
    if (currentLogMode == 1)
        return;

    resetTakeLogReceiveState();
    currentLogMode = 1;

    logTime2.clear();
    logM2_pitchThetaRad.clear();
    logM2_pitchOmegaRadS.clear();
    logM2_torqueY1000.clear();
    logIndex2 = 0.0;
    m_lastTelPitchMilliRad = 0;
    m_lastTelPitchOmegaMilli = 0;
    m_lastTelUy = 0;
    m_lastTelemetryRedrawMs = 0;

    setupPitchTelemetryPlot();
    setPlotLegendCheckboxesVisible(false);
    setPitchTelCheckboxesVisible(true);
    syncPitchTelMaskToFirmware();
    if (serialPort->isOpen())
        sendCommand("log 1");
}

void Dialog::on_btnLogOn_clicked()
{
    startLiveLog();
}

void Dialog::on_btnLogOff_clicked()
{
    resetTakeLogReceiveState();
    sendCommand("log 0");
    currentLogMode = 0;
    setupPitchTelemetryPlot();
    setPlotLegendCheckboxesVisible(false);
    setPitchTelCheckboxesVisible(true);
}

unsigned Dialog::pitchTelMaskFromUi() const
{
    unsigned m = 0;
    if (ui->checkBoxLogTelPitchAng->isChecked())
        m |= 1u;
    if (ui->checkBoxLogTelPitchRate->isChecked())
        m |= 2u;
    if (ui->checkBoxLogTelPitchCtrl->isChecked())
        m |= 4u;
    return m ? m : 7u;
}

void Dialog::setPitchTelCheckboxesVisible(bool visible)
{
    ui->checkBoxLogTelPitchAng->setVisible(visible);
    ui->checkBoxLogTelPitchRate->setVisible(visible);
    ui->checkBoxLogTelPitchCtrl->setVisible(visible);
}

void Dialog::syncPitchTelMaskToFirmware()
{
    sendCommand(QStringLiteral("logtel %1").arg(pitchTelMaskFromUi()));
}

void Dialog::updatePitchTelPlotVisibility()
{
    if (ui->customPlot->graphCount() < 3 || currentLogMode != 1)
        return;
    ui->customPlot->graph(0)->setVisible(ui->checkBoxLogTelPitchAng->isChecked());
    ui->customPlot->graph(1)->setVisible(ui->checkBoxLogTelPitchRate->isChecked());
    ui->customPlot->graph(2)->setVisible(ui->checkBoxLogTelPitchCtrl->isChecked());
}

void Dialog::onPitchTelChannelToggled()
{
    if (!ui->checkBoxLogTelPitchAng->isChecked()
        && !ui->checkBoxLogTelPitchRate->isChecked()
        && !ui->checkBoxLogTelPitchCtrl->isChecked()) {
        ui->checkBoxLogTelPitchAng->blockSignals(true);
        ui->checkBoxLogTelPitchAng->setChecked(true);
        ui->checkBoxLogTelPitchAng->blockSignals(false);
    }
    updatePitchTelPlotVisibility();
    ui->customPlot->replot();
    if (serialPort->isOpen() && currentLogMode == 1)
        syncPitchTelMaskToFirmware();
}
