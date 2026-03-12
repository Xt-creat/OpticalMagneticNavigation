#include "CalibrationDialog.h"
#include "ui_CalibrationDialog.h"
#include <QApplication>
#include <QTimer>
#include <QMessageBox>
#include <QDebug>
#include <CombinedApi.h>
#include <OMCalibrate.h>
#include <fstream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <iomanip>

extern CombinedApi O_capi;
extern CombinedApi M_capi;

CalibrationDialog::CalibrationDialog(const QString& savePath, int sampleFreqHz, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Calibration),
    m_savePath(savePath)
{
    ui->setupUi(this);
    setSampleFrequencyHz(sampleFreqHz);

    if (m_savePath.isEmpty()) {
        ui->label_2->setText(QString::fromUtf8("未选择路径"));
        ui->label->setText(QString::fromUtf8("错误: 请先在主界面选择数据保存路径！"));
        ui->startButton->setEnabled(false);
    }
    else {
        ui->label_2->setText(QString::fromUtf8("等待开始..."));
    }
    
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &CalibrationDialog::reject);
    connect(ui->startButton, &QPushButton::clicked, this, &CalibrationDialog::onStartButtonClicked);
    connect(ui->recordButton, &QPushButton::clicked, this, &CalibrationDialog::StaticCalibrateCollection);
    connect(ui->calculateButton, &QPushButton::clicked, this, &CalibrationDialog::Calibrationcalculate);
}

void CalibrationDialog::setSampleFrequencyHz(int sampleFreqHz)
{
    m_sampleFrequencyHz = std::max(1, sampleFreqHz);
    m_sampleIntervalMs = std::max(1, 1000 / m_sampleFrequencyHz);
}

void CalibrationDialog::onStartButtonClicked()
{
    totalGroups = ui->groupSpinBox->value();
    currentGroupIndex = 0;

    if (csvFile1.is_open()) csvFile1.close();
    if (csvFile2.is_open()) csvFile2.close();

    csvFile1.open((m_savePath + "/Vega.csv").toStdString(), std::ios::out);
    csvFile2.open((m_savePath + "/Aurora.csv").toStdString(), std::ios::out);

    if (!csvFile1.is_open() || !csvFile2.is_open()) {
        ui->label->setText(QString::fromUtf8("错误: 无法创建CSV文件，请检查路径权限"));
        return;
    }

    csvFile1 << "#Tools\n";
    csvFile2 << "#Tools\n";
    headerWritten = false;

    ui->groupSpinBox->setEnabled(false);
    ui->iterSpinBox->setEnabled(false);
    ui->startButton->setEnabled(false);
    ui->recordButton->setEnabled(true);
    
    ui->label_2->setText(QString::fromUtf8("当前进度: 0 / %1 组").arg(totalGroups));
    ui->label->setText(QString::fromUtf8("请保持工具静止，点击“单组采集”"));
}

CalibrationDialog::~CalibrationDialog()
{
    if (csvFile1.is_open()) csvFile1.close();
    if (csvFile2.is_open()) csvFile2.close();
    delete ui;
}

void CalibrationDialog::addOData(const std::vector<ToolData>& tools) {
    m_lastOData = tools;
}

void CalibrationDialog::addMData(const std::vector<ToolData>& tools) {
    m_lastMData = tools;
}

void CalibrationDialog::StaticCalibrateCollection()
{
    const int linesPerGroup = 10;  
    int linesWritten = 0;
    int maxWaitCycles = 200; 
    int waitCycles = 0;

    ui->recordButton->setEnabled(false);
    ui->label->setText(QString::fromUtf8("正在采集第 %1 组数据 (共 %2 组)...").arg(currentGroupIndex + 1).arg(totalGroups));
    
    m_lastOData.clear();
    m_lastMData.clear();

    std::vector<std::string> groupLines1;
    std::vector<std::string> groupLines2;

    while (linesWritten < linesPerGroup && waitCycles < maxWaitCycles)
    {
        QApplication::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(m_sampleIntervalMs));

        std::vector<ToolData> enabledTools1 = m_lastOData;
        std::vector<ToolData> enabledTools2 = m_lastMData;

        bool allValid = (enabledTools1.size() >= 2) && (!enabledTools2.empty());
        
        if (allValid) {
            if (enabledTools1[0].transform.isMissing() || 
                enabledTools1[1].transform.isMissing() || 
                enabledTools2[0].transform.isMissing()) {
                allValid = false;
            }
        }

        if (!allValid) {
            waitCycles++;
            continue; 
        }

        double emError = enabledTools2[0].transform.error;
        if (emError > 0.2) {
            ui->label->setText(QString::fromUtf8("<font color='red'><b>采集异常：</b>电磁工具误差过大 (%1 > 0.2)！<br/>请确保工具可见且保持静止，点击“单组采集”重试第 %2 组。</font>")
                .arg(emError)
                .arg(currentGroupIndex + 1));
            ui->recordButton->setEnabled(true);
            return;
        }

        if (!headerWritten) {
            csvFile1 << "ToolCount";
            for (int t = 0; t < (int)enabledTools1.size(); t++) {
                csvFile1 << ",ToolInfo,Frame#,PortHandle,Face#,timespec_s,timespec_ns,"
                    "TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers";
                for (int m = 0; m < (int)enabledTools1[t].markers.size(); m++) {
                    csvFile1 << ",Marker" << m << ".Status,Tx,Ty,Tz";
                }
            }
            csvFile1 << std::endl;

            csvFile2 << "ToolCount";
            for (int t = 0; t < (int)enabledTools2.size(); t++) {
                csvFile2 << ",ToolInfo,Frame#,PortHandle,Face#,timespec_s,timespec_ns,"
                    "TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers";
                for (int m = 0; m < (int)enabledTools2[t].markers.size(); m++) {
                    csvFile2 << ",Marker" << m << ".Status,Tx,Ty,Tz";
                }
            }
            csvFile2 << std::endl;
            headerWritten = true;
        }

        std::stringstream ss1, ss2;
        ss1 << enabledTools1.size();
        for (int t = 0; t < (int)enabledTools1.size(); t++) {
            ss1 << "," << enabledTools1[t].toolInfo << "," << toolDataToCSV(enabledTools1[t]);
        }
        groupLines1.push_back(ss1.str());

        ss2 << enabledTools2.size();
        for (int t = 0; t < (int)enabledTools2.size(); t++) {
            ss2 << "," << enabledTools2[t].toolInfo << "," << toolDataToCSV(enabledTools2[t]);
        }
        groupLines2.push_back(ss2.str());

        linesWritten++;
        waitCycles = 0; 
        m_lastOData.clear();
        m_lastMData.clear();
    }

    if (waitCycles >= maxWaitCycles) {
        ui->label->setText(QString::fromUtf8("<font color='red'>采集失败: 未收到有效数据。请确保主界面跟踪正在运行且工具可见。</font>"));
        ui->recordButton->setEnabled(true);
        return;
    }

    for (const auto& line : groupLines1) csvFile1 << line << std::endl;
    for (const auto& line : groupLines2) csvFile2 << line << std::endl;

    csvFile1.flush();
    csvFile2.flush();

    currentGroupIndex++;
    ui->label_2->setText(QString::fromUtf8("当前进度: %1 / %2 组").arg(currentGroupIndex).arg(totalGroups));

    if (currentGroupIndex >= totalGroups) {
        ui->label->setText(QString::fromUtf8("<b>数据采集全部完成！</b><br/>请点击“执行计算”生成标定矩阵"));
        ui->recordButton->setEnabled(false);
        ui->calculateButton->setEnabled(true);
    }
    else {
        ui->label->setText(QString::fromUtf8("第 %1 组采集成功！请移动工具到下一位置，然后点击“单组采集”").arg(currentGroupIndex));
        ui->recordButton->setEnabled(true);
    }
}

std::string CalibrationDialog::toolDataToCSV(const ToolData& toolData)
{
    std::stringstream stream;
    stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
    stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
        << "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
    stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";
    stream << toolData.timespec_s << "," << toolData.timespec_ns << ",";

    if (toolData.transform.isMissing())
    {
        stream << "Missing,,,,,,,,";
    }
    else
    {
        stream << TransformStatus::toString(toolData.transform.getErrorCode()) << ","
            << toolData.transform.q0 << "," << toolData.transform.qx << "," << toolData.transform.qy << "," << toolData.transform.qz << ","
            << toolData.transform.tx << "," << toolData.transform.ty << "," << toolData.transform.tz << "," << toolData.transform.error;
    }

    stream << "," << toolData.markers.size();
    for (int i = 0; i < (int)toolData.markers.size(); i++)
    {
        stream << "," << MarkerStatus::toString(toolData.markers[i].status);
        if (toolData.markers[i].status == MarkerStatus::Missing)
        {
            stream << ",,,";
        }
        else
        {
            stream << "," << toolData.markers[i].x << "," << toolData.markers[i].y << "," << toolData.markers[i].z;
        }
    }
    return stream.str();
}

void CalibrationDialog::Calibrationcalculate() {
    OMCalibrate OMC;

    ui->label->setText(QString::fromUtf8("正在计算标定矩阵..."));
    ui->calculateButton->setEnabled(false);
    QApplication::processEvents();

    OMC.LoadData(m_savePath.toStdString(), totalGroups);

    int maxIterations = ui->iterSpinBox->value();
    std::vector<CalibrationError> iterationErrors;
    std::vector<cv::Mat> m12ems_history;
    std::vector<cv::Mat> em2m2_history;
    
    std::vector<int> activeGroupIds;
    for (int i = 0; i < (int)OMC.tool1_data.size(); ++i) activeGroupIds.push_back(i + 1);
    std::vector<std::vector<int>> iterationGroupIdsHistory;
    std::vector<int> removedIds;

    for (int iter = 0; iter <= maxIterations; ++iter) {
        iterationGroupIdsHistory.push_back(activeGroupIds);

        OMC.HandeyeCalibrate3();

        if (OMC.tool1_2EMsensor.empty() || OMC.EM_2tool2.empty()) {
            ui->label->setText(QString::fromUtf8("<font color='red'>计算失败: 数据不足或解算异常</font>"));
            ui->calculateButton->setEnabled(true);
            return;
        }

        CalibrationError currentError = OMC.EvaluateCalibration();
        iterationErrors.push_back(currentError);
        m12ems_history.push_back(OMC.tool1_2EMsensor.clone());
        em2m2_history.push_back(OMC.EM_2tool2.clone());

        if (iter < maxIterations && currentError.translationErrors.size() > 3) {
            auto maxIt = std::max_element(currentError.translationErrors.begin(), currentError.translationErrors.end());
            int maxIdx = std::distance(currentError.translationErrors.begin(), maxIt);
            
            removedIds.push_back(activeGroupIds[maxIdx]);
            activeGroupIds.erase(activeGroupIds.begin() + maxIdx);
            OMC.RemoveGroup(maxIdx);
        } else {
            break; 
        }
    }

    QString resultPath = m_savePath + "/CalibrationResult.txt";
    std::ofstream file(resultPath.toStdString());
    if (file.is_open()) {
        file << "==========================================================\n";
        file << "            Iterative Calibration Detailed Report\n";
        file << "==========================================================\n\n";

        file << "Summary:\n";
        file << "Initial Avg Translation Error: " << iterationErrors.front().avgTranslationError << " mm\n";
        file << "Final Avg Translation Error:   " << iterationErrors.back().avgTranslationError << " mm\n";
        file << "Total Iterations Executed:     " << iterationErrors.size() - 1 << "\n";
        file << "Removed Groups (Original IDs): ";
        if (removedIds.empty()) file << "None";
        else for (int id : removedIds) file << id << " ";
        file << "\n\n";

        file << "Iteration Process History:\n";
        file << "Iter | AvgTransErr(mm) | AvgRotErr(deg) | RemainingGroups\n";
        file << "--------------------------------------------------------\n";
        for (size_t i = 0; i < iterationErrors.size(); ++i) {
            file << std::setw(4) << std::left << i << " | "
                 << std::setw(15) << std::fixed << std::setprecision(6) << iterationErrors[i].avgTranslationError << " | "
                 << std::setw(14) << iterationErrors[i].avgRotationError << " | "
                 << iterationErrors[i].translationErrors.size() << "\n";
        }
        file << "\n";

        auto writeDetailedErrors = [&](int iterIdx, const std::string& title) {
            file << ">>> " << title << "\n";
            const auto& errs = iterationErrors[iterIdx];
            const auto& ids = iterationGroupIdsHistory[iterIdx];
            file << "GroupID | TranslationErr(mm) | RotationErr(deg)\n";
            file << "-----------------------------------------------\n";
            for (size_t i = 0; i < errs.translationErrors.size(); ++i) {
                file << std::setw(7) << std::left << ids[i] << " | " 
                     << std::setw(18) << std::fixed << std::setprecision(6) << errs.translationErrors[i] << " | "
                     << errs.rotationErrors[i] << "\n";
            }
            file << "\n";
        };

        writeDetailedErrors(0, "Initial Detailed Errors (Iteration 0)");
        if (iterationErrors.size() > 1) {
            writeDetailedErrors(iterationErrors.size() - 1, "Final Detailed Errors (Optimized)");
        }

        file << ">>> Final Calibration Matrices:\n\n";
        file << "tool1_2EMsensor:\n";
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) file << std::setw(12) << m12ems_history.back().at<double>(r, c) << " ";
            file << "\n";
        }
        file << "\nEM_2tool2:\n";
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) file << std::setw(12) << em2m2_history.back().at<double>(r, c) << " ";
            file << "\n";
        }
        file.close();
    }

    saveMatToTxt(OMC.tool1_2EMsensor, "tool1_2EMsensor.txt");
    saveMatToTxt(OMC.EM_2tool2, "EM_2tool2.txt");

    const CalibrationError& initial = iterationErrors.front();
    const CalibrationError& final = iterationErrors.back();
    
    QString removedStr = "None";
    if (!removedIds.empty()) {
        QStringList list;
        for(int id : removedIds) list << QString::number(id);
        removedStr = list.join(", ");
    }

    QString msg = QString::fromUtf8("标定优化完成！\n\n"
        "初始误差: %1 mm / %2 度\n"
        "最终误差: %3 mm / %4 度\n"
        "剔除组号: %5\n\n"
        "详细报告已存至: CalibrationResult.txt")
        .arg(initial.avgTranslationError, 0, 'f', 4)
        .arg(initial.avgRotationError, 0, 'f', 4)
        .arg(final.avgTranslationError, 0, 'f', 4)
        .arg(final.avgRotationError, 0, 'f', 4)
        .arg(removedStr);

    QMessageBox::information(this, QString::fromUtf8("标定优化结果"), msg);

    ui->label->setText(QString::fromUtf8("<font color='green'><b>优化计算完成！</b></font><br/>最终平均误差: %1 mm").arg(final.avgTranslationError));
    ui->calculateButton->setEnabled(true);
}

void CalibrationDialog::saveMatToTxt(const cv::Mat& mat, const std::string& filename)
{
    QString fullPath = m_savePath + "/" + QString::fromStdString(filename);
    std::ofstream file(fullPath.toStdString());
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << fullPath.toStdString() << std::endl;
        return;
    }

    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            switch (mat.type()) {
            case CV_32F:
                file << mat.at<float>(i, j);
                break;
            case CV_64F:
                file << mat.at<double>(i, j);
                break;
            default:
                std::cerr << "暂不支持的类型: " << mat.type() << std::endl;
                file.close();
                return;
            }
            if (j < mat.cols - 1) file << " ";
        }
        file << "\n";
    }

    file.close();
    std::cout << "矩阵已保存到: " << fullPath.toStdString() << std::endl;
}
