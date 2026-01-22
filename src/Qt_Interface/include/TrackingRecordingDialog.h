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

    // 用于接收来自主界面的实时数据
    void addOData(const std::vector<ToolData>& tools);
    void addMData(const std::vector<ToolData>& tools);

private slots:
    void onStartRecording();
    void onStopRecording();
    void onUpdateTime();

private:
    void writeHeader(std::ofstream& file, const std::vector<ToolData>& tools); // 新增：写表头
    void writeToolDataToCSV(std::ofstream& file, const std::vector<ToolData>& tools);
    std::string toolDataToCSVRow(const ToolData& toolData);

    Ui::TrackingRecording *ui;
    QString m_savePath;
    bool m_isRecording = false;
    int m_recordCount = 0;
    QDateTime m_recordStartTime;
    
    std::ofstream m_opticalFile;
    std::ofstream m_magneticFile;
    bool m_headerWrittenO = false; // 新增：光学表头标志
    bool m_headerWrittenM = false; // 新增：电磁表头标志
    QTimer* m_timer;
};

#endif // TRACKINGRECORDINGDIALOG_H

