#ifndef TRACKINGRECORDINGDIALOG_H
#define TRACKINGRECORDINGDIALOG_H

#include <QDialog>
#include <QTimer>
#include <QDateTime>
#include <fstream>
#include "ToolData.h"

namespace Ui {
class TrackingRecording;
}

class TrackingRecordingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TrackingRecordingDialog(const QString& savePath, QWidget *parent = nullptr);
    ~TrackingRecordingDialog();

    void addOData(const std::vector<ToolData>& tools);
    void addMData(const std::vector<ToolData>& tools);

private slots:
    void onStartRecording();
    void onStopRecording();
    void onUpdateTime();

private:
    void writeHeader(std::ofstream& file, const std::vector<ToolData>& tools); 
    void writeToolDataToCSV(std::ofstream& file, const std::vector<ToolData>& tools);
    std::string toolDataToCSVRow(const ToolData& toolData);
    void tryWriteSynchronizedRows();
    uint32_t extractFrameNumber(const std::vector<ToolData>& tools) const;

    Ui::TrackingRecording *ui;
    QString m_savePath;
    bool m_isRecording = false;
    int m_recordCount = 0;
    QDateTime m_recordStartTime;
    
    std::ofstream m_opticalFile;
    std::ofstream m_magneticFile;
    bool m_headerWrittenO = false; 
    bool m_headerWrittenM = false; 
    QTimer* m_timer;

    std::vector<ToolData> m_pendingOData;
    std::vector<ToolData> m_pendingMData;
    bool m_hasPendingOData = false;
    bool m_hasPendingMData = false;
};

#endif // TRACKINGRECORDINGDIALOG_H
