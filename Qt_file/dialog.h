#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QSerialPort>
#include <QTimer>
#include "qcustomplot.h"
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QForwardRenderer>

QT_BEGIN_NAMESPACE
namespace Ui { class Dialog; }
QT_END_NAMESPACE

class QTabWidget;
class QLineEdit;
class QPushButton;

/// Наземная станция Qt для стенда pitch (1 степень свободы).
/// Связь: ПК → USB → ESP8266 → UART → ESP32; текстовые CLI-команды @ 115200.
class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();

private slots:
    void on_verticalSliderThrust_valueChanged(int value);
    void handleSerialData();
    void on_push_arm_clicked();
    void on_push_disarm_clicked();
    void on_checkBoxPitch_toggled(bool checked);
    void on_btnDisableAll_clicked();
    void on_save_para_clicked();
    void on_btnGetParams_clicked();
    void on_btnLogOn_clicked();
    void on_btnLogOff_clicked();
    void on_btnSaveLog_clicked();
    void onPitchTelChannelToggled();

private:
    Ui::Dialog *ui;
    QSerialPort *serialPort;

    // --- Живой график телеметрии pitch (режим log 1) ---
    QVector<double> logTime2;
    QVector<double> logM2_pitchThetaRad;   // фактически градусы (историческое имя)
    QVector<double> logM2_pitchOmegaRadS;  // град/с
    QVector<double> logM2_torqueY1000;
    double logIndex2 = 0.0;
    int currentLogMode = 0;  // 0 = выкл., 1 = поток pitch θ/ω/момент
    bool capturingParams = false;  // режим вывода Flash через «Get parameters»

    // Последние значения телеметрии (для частичной маски каналов)
    int m_lastTelPitchMilliRad = 0;
    int m_lastTelPitchOmegaMilli = 0;
    int m_lastTelUy = 0;

    bool isArmed = false;
    bool m_manualMode = false;       // ручной режим (Disable → Front/Rear)
    bool m_uavLinkAlive = false;     // ESP32 отвечает (не только открыт USB)
    qint64 m_lastFirmwareRxMs = 0;
    QTimer *m_linkProbeTimer = nullptr;  // опрос связи командой sys

    // --- Приём буфера TakeLog с борта (команда takelog) ---
    bool m_receivingTakeLog = false;
    bool m_takeLogStarted = false;
    int m_takeLogExpected = -1;
    QStringList m_takeLogRawLines;

    // --- 3D-вид квадрокоптера (uav.stl) ---
    Qt3DExtras::Qt3DWindow *m_view3d = nullptr;
    QWidget *m_viewContainer = nullptr;
    Qt3DCore::QEntity *m_root3d = nullptr;
    Qt3DCore::QTransform *m_uavTf = nullptr;
    QQuaternion m_modelOffset = QQuaternion();
    float m_pitchDegFiltered = 0.0f;
    float m_attAlpha = 0.25f;   // сглаживание угла для 3D
    float m_rollSign = -1.0f;   // знак оси roll в модели STL
    float m_pitchSign = 1.0f;   // знак оси pitch в модели STL
    qint64 m_lastTelemetryRedrawMs = 0;

    // --- Вкладки регуляторов (создаются в runtime) ---
    QTabWidget *m_ctrlTabs = nullptr;
    QWidget *m_tabPdPi = nullptr;
    QWidget *m_tabPid = nullptr;
    QWidget *m_tabSmc = nullptr;
    QPushButton *m_btnCtrlPdPi = nullptr;
    QPushButton *m_btnCtrlPid = nullptr;
    QPushButton *m_btnCtrlSmc = nullptr;
    QPushButton *m_btnSetDefault = nullptr;
    int m_ctrlAlgMode = 0;  // 0 = PD-PI, 1 = каскадный PID, 2 = SMC
    QLineEdit *m_pidPitchP = nullptr;
    QLineEdit *m_pidPitchI = nullptr;
    QLineEdit *m_pidPitchD = nullptr;
    QLineEdit *m_pidPitchRateP = nullptr;
    QLineEdit *m_pidPitchRateI = nullptr;
    QLineEdit *m_pidPitchRateD = nullptr;
    QLineEdit *m_smcLambda = nullptr;
    QLineEdit *m_smcK = nullptr;
    QLineEdit *m_smcPhi = nullptr;
    QLineEdit *m_smcKrate = nullptr;
    QLineEdit *m_smcKi = nullptr;
    QLineEdit *m_smcIMax = nullptr;
    QLineEdit *m_smcVMax = nullptr;
    QLineEdit *m_smcLpfAlpha = nullptr;
    QLineEdit *m_smcLpfTau = nullptr;

    // Инициализация UI и связей
    void setupRuntimeLayout();       // смещение виджетов (DPI, стенд)
    void setupParamFieldWiring();    // Enter → отправка параметра на борт
    void setupSerialLink();          // порт, таймер probe, readyRead
    void setupControlTabs();         // PD-PI / Classic PID / SMC
    void setupCtrlAlgOutsideTabs();  // кнопки Control mode под вкладками
    void setupPitchTelemetryPlot();  // QCustomPlot: θ, ω, момент

    // Связь и безопасность
    void updateArmUi();
    void updateSerialUi();
    void setUavLinkAlive(bool alive);
    void noteFirmwareRx(const QString &line);
    void probeUavLink();
    void openSerial();
    void applyDefaultPitchOnlyChannels();  // dscnl: только pitch
    void exitManualMode();

    // Параметры → ESP32 Flash (через CLI p / save)
    void sendParamFromField(QLineEdit *field, const QString &paramName,
                            int precision, bool saveAfter);
    void sendParamIfValid(QLineEdit *field, const QString &paramName);
    void sendPdPiParams();
    void sendAllParams();
    void resetControllerDefaults();
    void syncPitchSourceToFirmware();  // QuatEn + LPF_En по радиокнопкам
    void sendCommand(const QString &cmd);
    void setCtrlAlgMode(int mode, bool sendToFirmware);

    // Телеметрия и графики
    void startLiveLog();
    void setPlotLegendCheckboxesVisible(bool visible);
    void setPitchTelCheckboxesVisible(bool visible);
    void syncPitchTelMaskToFirmware();
    unsigned pitchTelMaskFromUi() const;
    void updatePitchTelPlotVisibility();
    void addLogSample2(double pitchDeg, double pitchOmegaDegS, int torqueY1000);
    void handleLogLine(const QString &line);
    void saveTakeLog();
    void resetTakeLogReceiveState();
    static bool isTakeLogDataLine(const QString &line);

    // 3D-модель
    void initUav3D(QWidget *host);
    void setUavAttitudeDeg(float rollDeg, float pitchDeg);
};

#endif // DIALOG_H
