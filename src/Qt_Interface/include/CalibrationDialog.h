#ifndef CALIBRATIONDIALOG_H
#define CALIBRATIONDIALOG_H

#include <QDialog>
#include "ToolData.h"
#include <fstream>
#include <opencv2/opencv.hpp>

namespace Ui {
    class Calibration;
}

class CalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    QString m_savePath;

    explicit CalibrationDialog(const QString& savePath, int sampleFreqHz = 10, QWidget *parent = nullptr);
    ~CalibrationDialog();
    void setSampleFrequencyHz(int sampleFreqHz);
    
    std::string toolDataToCSV(const ToolData& toolData);

    void saveMatToTxt(const cv::Mat& mat, const std::string& filename);

    void addOData(const std::vector<ToolData>& tools);
    void addMData(const std::vector<ToolData>& tools);

private slots:
    void Calibrationcalculate();     
    void StaticCalibrateCollection(); 
    void onStartButtonClicked();      

private:
    Ui::Calibration *ui;  

    int currentGroupIndex = 0;      
    int totalGroups = 30;           
    const int numberOfLines = 10;   
    bool headerWritten = false;     
    bool m_isCollecting = false;    
    int m_sampleFrequencyHz = 10;   
    int m_sampleIntervalMs = 100;   

    std::vector<ToolData> m_lastOData; 
    std::vector<ToolData> m_lastMData;

    std::ofstream csvFile1, csvFile2;

};

#endif // CALIBRATIONDIALOG_H
