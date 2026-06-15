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

/// Qt ground station for bench pitch control (serial CLI to ESP32 via USB).
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

    // Live pitch telemetry plot (log mode 1)
    QVector<double> logTime2;
    QVector<double> logM2_pitchThetaRad;   // values are degrees (legacy name)
    QVector<double> logM2_pitchOmegaRadS;  // values are deg/s
    QVector<double> logM2_torqueY1000;
    double logIndex2 = 0.0;
    int currentLogMode = 0;  // 0 = off, 1 = live pitch telemetry
    bool capturingParams = false;

    int m_lastTelPitchMilliRad = 0;
    int m_lastTelPitchOmegaMilli = 0;
    int m_lastTelUy = 0;

    bool isArmed = false;
    bool m_manualMode = false;
    bool m_uavLinkAlive = false;
    qint64 m_lastFirmwareRxMs = 0;
    QTimer *m_linkProbeTimer = nullptr;

    // TakeLog CSV dump from firmware buffer
    bool m_receivingTakeLog = false;
    bool m_takeLogStarted = false;
    int m_takeLogExpected = -1;
    QStringList m_takeLogRawLines;

    // 3D attitude view
    Qt3DExtras::Qt3DWindow *m_view3d = nullptr;
    QWidget *m_viewContainer = nullptr;
    Qt3DCore::QEntity *m_root3d = nullptr;
    Qt3DCore::QTransform *m_uavTf = nullptr;
    QQuaternion m_modelOffset = QQuaternion();
    float m_pitchDegFiltered = 0.0f;
    float m_attAlpha = 0.25f;
    float m_rollSign = -1.0f;
    float m_pitchSign = 1.0f;
    qint64 m_lastTelemetryRedrawMs = 0;

    // Control tabs (PD-PI / PID / SMC) built at runtime
    QTabWidget *m_ctrlTabs = nullptr;
    QWidget *m_tabPdPi = nullptr;
    QWidget *m_tabPid = nullptr;
    QWidget *m_tabSmc = nullptr;
    QPushButton *m_btnCtrlPdPi = nullptr;
    QPushButton *m_btnCtrlPid = nullptr;
    QPushButton *m_btnCtrlSmc = nullptr;
    QPushButton *m_btnSetDefault = nullptr;
    int m_ctrlAlgMode = 0;  // 0 = PD-PI, 1 = PID, 2 = SMC
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

    void setupRuntimeLayout();
    void setupParamFieldWiring();
    void setupSerialLink();
    void setupControlTabs();
    void setupCtrlAlgOutsideTabs();
    void setupPitchTelemetryPlot();

    void updateArmUi();
    void updateSerialUi();
    void setUavLinkAlive(bool alive);
    void noteFirmwareRx(const QString &line);
    void probeUavLink();
    void openSerial();
    void applyDefaultPitchOnlyChannels();
    void exitManualMode();
    void sendParamFromField(QLineEdit *field, const QString &paramName,
                            int precision, bool saveAfter);
    void sendParamIfValid(QLineEdit *field, const QString &paramName);
    void sendPdPiParams();
    void sendAllParams();
    void resetControllerDefaults();
    void startLiveLog();
    void syncPitchSourceToFirmware();
    void sendCommand(const QString &cmd);
    void setCtrlAlgMode(int mode, bool sendToFirmware);
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
    void initUav3D(QWidget *host);
    void setUavAttitudeDeg(float rollDeg, float pitchDeg);
};

#endif // DIALOG_H
