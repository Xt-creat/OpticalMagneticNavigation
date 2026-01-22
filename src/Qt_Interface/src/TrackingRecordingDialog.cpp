#include "TrackingRecordingDialog.h"
#include "ui_TrackingRecordingDialog.h"
#include <iomanip>
#include <sstream>
#include <QDir>
#include <QDebug>

TrackingRecordingDialog::TrackingRecordingDialog(const QString& savePath, QWidget *parent)
    : QDialog(parent), ui(new Ui::TrackingRecording), m_savePath(savePath)
{
    ui->setupUi(this);

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &TrackingRecordingDialog::onUpdateTime);

    connect(ui->m_startBtn, &QPushButton::clicked, this, &TrackingRecordingDialog::onStartRecording);
    connect(ui->m_stopBtn, &QPushButton::clicked, this, &TrackingRecordingDialog::onStopRecording);
}

TrackingRecordingDialog::~TrackingRecordingDialog()
{
    onStopRecording();
    delete ui;
}

void TrackingRecordingDialog::onStartRecording()
{
    if (m_savePath.isEmpty()) {
        ui->m_statusLabel->setText(QString::fromLocal8Bit("错误: 未指定保存路径！"));
        return;
    }

    m_recordCount++;
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString opticalPath = m_savePath + QString("/Optical_Data_%1_%2.csv").arg(m_recordCount).arg(timestamp);
    QString magneticPath = m_savePath + QString("/Magnetic_Data_%1_%2.csv").arg(m_recordCount).arg(timestamp);

    m_opticalFile.open(opticalPath.toStdString(), std::ios::out);
    m_magneticFile.open(magneticPath.toStdString(), std::ios::out);

    if (m_opticalFile.is_open() && m_magneticFile.is_open()) {
        m_headerWrittenO = false; // 重置标志位
        m_headerWrittenM = false;

        m_isRecording = true;
        m_recordStartTime = QDateTime::currentDateTime();
        ui->m_startBtn->setEnabled(false);
        ui->m_stopBtn->setEnabled(true);
        m_timer->start(1000);
        ui->m_statusLabel->setText(QString::fromLocal8Bit("状态: 正在记录 (第 %1 次)").arg(m_recordCount));
        ui->m_timeLabel->setText("00:00:00");
    } else {
        ui->m_statusLabel->setText(QString::fromLocal8Bit("错误: 无法创建文件！"));
    }
}

void TrackingRecordingDialog::onStopRecording()
{
    if (m_isRecording) {
        m_isRecording = false;
        m_timer->stop();
        if (m_opticalFile.is_open()) {
            m_opticalFile.flush();
            m_opticalFile.close();
        }
        if (m_magneticFile.is_open()) {
            m_magneticFile.flush();
            m_magneticFile.close();
        }
        
        ui->m_startBtn->setEnabled(true);
        ui->m_stopBtn->setEnabled(false);
        ui->m_statusLabel->setText(QString::fromLocal8Bit("状态: 已停止，数据已保存"));
    }
}

void TrackingRecordingDialog::onUpdateTime()
{
    qint64 secs = m_recordStartTime.secsTo(QDateTime::currentDateTime());
    int h = secs / 3600;
    int m = (secs % 3600) / 60;
    int s = secs % 60;
    ui->m_timeLabel->setText(QString("%1:%2:%3")
        .arg(h, 2, 10, QChar('0'))
        .arg(m, 2, 10, QChar('0'))
        .arg(s, 2, 10, QChar('0')));
}

void TrackingRecordingDialog::addOData(const std::vector<ToolData>& tools)
{
    if (m_isRecording && m_opticalFile.is_open()) {
        if (!m_headerWrittenO && !tools.empty()) {
            writeHeader(m_opticalFile, tools);
            m_headerWrittenO = true;
        }
        writeToolDataToCSV(m_opticalFile, tools);
    }
}

void TrackingRecordingDialog::addMData(const std::vector<ToolData>& tools)
{
    if (m_isRecording && m_magneticFile.is_open()) {
        if (!m_headerWrittenM && !tools.empty()) {
            writeHeader(m_magneticFile, tools);
            m_headerWrittenM = true;
        }
        writeToolDataToCSV(m_magneticFile, tools);
    }
}

void TrackingRecordingDialog::writeHeader(std::ofstream& file, const std::vector<ToolData>& tools) {
    file << "ToolCount";
    for (size_t t = 0; t < tools.size(); t++) {
        file << ",ToolInfo,Frame#,PortHandle,Face#,timespec_s,timespec_ns,"
            "TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers";
        for (int m = 0; m < (int)tools[t].markers.size(); m++) {
            file << ",Marker" << m << ".Status,Tx,Ty,Tz";
        }
    }
    file << std::endl;
}

void TrackingRecordingDialog::writeToolDataToCSV(std::ofstream& file, const std::vector<ToolData>& tools)
{
    if (tools.empty()) return;

    // 每行以工具数量开头
    file << tools.size();

    // 顺序写入每个工具的数据
    for (size_t i = 0; i < tools.size(); ++i) {
        file << "," << toolDataToCSVRow(tools[i]);
    }
    file << "\n"; 
}

std::string TrackingRecordingDialog::toolDataToCSVRow(const ToolData& toolData)
{
    std::stringstream ss;
    ss << std::setprecision(toolData.PRECISION);
    
    ss << toolData.toolInfo << ","
       << toolData.frameNumber << ","
       << "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ","
       << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ","
       << toolData.timespec_s << "," << toolData.timespec_ns << ",";
    
    if (toolData.transform.isMissing()) {
        ss << "Missing,,,,,,,,"; 
    } else {
        ss << "Enabled," // 使用与 Vega.csv 一致的状态词
           << toolData.transform.q0 << "," << toolData.transform.qx << ","
           << toolData.transform.qy << "," << toolData.transform.qz << ","
           << toolData.transform.tx << "," << toolData.transform.ty << ","
           << toolData.transform.tz << "," << toolData.transform.error;
    }

    // 写入标志点数据
    ss << "," << toolData.markers.size();
    for (int i = 0; i < (int)toolData.markers.size(); i++) {
        const auto& marker = toolData.markers[i];
        if (marker.status == 0x01 || marker.status == 0x02) { // Missing 或 OutOfVolume
            ss << ",Missing,,,";
        } else {
            ss << ",OK," << marker.x << "," << marker.y << "," << marker.z;
        }
    }

    return ss.str();
}

