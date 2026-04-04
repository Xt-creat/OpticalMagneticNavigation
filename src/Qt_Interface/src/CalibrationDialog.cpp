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
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

//void CalibrationDialog::Calibrationcalculate() {
//    OMCalibrate OMC;
//
//    ui->label->setText(QString::fromUtf8("正在计算标定矩阵..."));
//    ui->calculateButton->setEnabled(false);
//    QApplication::processEvents();
//
//    OMC.LoadData(m_savePath.toStdString(), totalGroups);
//
//    int maxIterations = ui->iterSpinBox->value();
//    std::vector<CalibrationError> iterationErrors;
//    std::vector<cv::Mat> m12ems_history;
//    std::vector<cv::Mat> em2m2_history;
//    
//    std::vector<int> activeGroupIds;
//    for (int i = 0; i < (int)OMC.tool1_data.size(); ++i) activeGroupIds.push_back(i + 1);
//    std::vector<std::vector<int>> iterationGroupIdsHistory;
//    std::vector<int> removedIds;
//
//    for (int iter = 0; iter <= maxIterations; ++iter) {
//        iterationGroupIdsHistory.push_back(activeGroupIds);
//
//        OMC.HandeyeCalibrate3();
//
//        if (OMC.tool1_2EMsensor.empty() || OMC.EM_2tool2.empty()) {
//            ui->label->setText(QString::fromUtf8("<font color='red'>计算失败: 数据不足或解算异常</font>"));
//            ui->calculateButton->setEnabled(true);
//            return;
//        }
//
//        CalibrationError currentError = OMC.EvaluateCalibration();
//        iterationErrors.push_back(currentError);
//        m12ems_history.push_back(OMC.tool1_2EMsensor.clone());
//        em2m2_history.push_back(OMC.EM_2tool2.clone());
//
//        if (iter < maxIterations && currentError.translationErrors.size() > 3) {
//            auto maxIt = std::max_element(currentError.translationErrors.begin(), currentError.translationErrors.end());
//            int maxIdx = std::distance(currentError.translationErrors.begin(), maxIt);
//            
//            removedIds.push_back(activeGroupIds[maxIdx]);
//            activeGroupIds.erase(activeGroupIds.begin() + maxIdx);
//            OMC.RemoveGroup(maxIdx);
//        } else {
//            break; 
//        }
//    }
//
//    QString resultPath = m_savePath + "/CalibrationResult.txt";
//    std::ofstream file(resultPath.toStdString());
//    if (file.is_open()) {
//        file << "==========================================================\n";
//        file << "            Iterative Calibration Detailed Report\n";
//        file << "==========================================================\n\n";
//
//        file << "Summary:\n";
//        file << "Initial Avg Translation Error: " << iterationErrors.front().avgTranslationError << " mm\n";
//        file << "Final Avg Translation Error:   " << iterationErrors.back().avgTranslationError << " mm\n";
//        file << "Total Iterations Executed:     " << iterationErrors.size() - 1 << "\n";
//        file << "Removed Groups (Original IDs): ";
//        if (removedIds.empty()) file << "None";
//        else for (int id : removedIds) file << id << " ";
//        file << "\n\n";
//
//        file << "Iteration Process History:\n";
//        file << "Iter | AvgTransErr(mm) | AvgRotErr(deg) | RemainingGroups\n";
//        file << "--------------------------------------------------------\n";
//        for (size_t i = 0; i < iterationErrors.size(); ++i) {
//            file << std::setw(4) << std::left << i << " | "
//                 << std::setw(15) << std::fixed << std::setprecision(6) << iterationErrors[i].avgTranslationError << " | "
//                 << std::setw(14) << iterationErrors[i].avgRotationError << " | "
//                 << iterationErrors[i].translationErrors.size() << "\n";
//        }
//        file << "\n";
//
//        auto writeDetailedErrors = [&](int iterIdx, const std::string& title) {
//            file << ">>> " << title << "\n";
//            const auto& errs = iterationErrors[iterIdx];
//            const auto& ids = iterationGroupIdsHistory[iterIdx];
//            file << "GroupID | TranslationErr(mm) | RotationErr(deg)\n";
//            file << "-----------------------------------------------\n";
//            for (size_t i = 0; i < errs.translationErrors.size(); ++i) {
//                file << std::setw(7) << std::left << ids[i] << " | " 
//                     << std::setw(18) << std::fixed << std::setprecision(6) << errs.translationErrors[i] << " | "
//                     << errs.rotationErrors[i] << "\n";
//            }
//            file << "\n";
//        };
//
//        writeDetailedErrors(0, "Initial Detailed Errors (Iteration 0)");
//        if (iterationErrors.size() > 1) {
//            writeDetailedErrors(iterationErrors.size() - 1, "Final Detailed Errors (Optimized)");
//        }
//
//        file << ">>> Final Calibration Matrices:\n\n";
//        file << "tool1_2EMsensor:\n";
//        for (int r = 0; r < 4; r++) {
//            for (int c = 0; c < 4; c++) file << std::setw(12) << m12ems_history.back().at<double>(r, c) << " ";
//            file << "\n";
//        }
//        file << "\nEM_2tool2:\n";
//        for (int r = 0; r < 4; r++) {
//            for (int c = 0; c < 4; c++) file << std::setw(12) << em2m2_history.back().at<double>(r, c) << " ";
//            file << "\n";
//        }
//        file.close();
//    }
//
//    saveMatToTxt(OMC.tool1_2EMsensor, "tool1_2EMsensor.txt");
//    saveMatToTxt(OMC.EM_2tool2, "EM_2tool2.txt");
//
//    const CalibrationError& initial = iterationErrors.front();
//    const CalibrationError& final = iterationErrors.back();
//    
//    QString removedStr = "None";
//    if (!removedIds.empty()) {
//        QStringList list;
//        for(int id : removedIds) list << QString::number(id);
//        removedStr = list.join(", ");
//    }
//
//    QString msg = QString::fromUtf8("标定优化完成！\n\n"
//        "初始误差: %1 mm / %2 度\n"
//        "最终误差: %3 mm / %4 度\n"
//        "剔除组号: %5\n\n"
//        "详细报告已存至: CalibrationResult.txt")
//        .arg(initial.avgTranslationError, 0, 'f', 4)
//        .arg(initial.avgRotationError, 0, 'f', 4)
//        .arg(final.avgTranslationError, 0, 'f', 4)
//        .arg(final.avgRotationError, 0, 'f', 4)
//        .arg(removedStr);
//
//    QMessageBox::information(this, QString::fromUtf8("标定优化结果"), msg);
//
//    ui->label->setText(QString::fromUtf8("<font color='green'><b>优化计算完成！</b></font><br/>最终平均误差: %1 mm").arg(final.avgTranslationError));
//    ui->calculateButton->setEnabled(true);
//}

void CalibrationDialog::Calibrationcalculate() {
    OMCalibrate OMC;

    ui->label->setText(QString::fromUtf8("正在执行 Proposed1 优化标定 (AX=XB & SVD)..."));
    ui->calculateButton->setEnabled(false);
    QApplication::processEvents();

    OMC.LoadData(m_savePath.toStdString(), totalGroups);

    // 调用新的 Proposed1 算法，默认剔除平移误差最大的 5 组数据
    int num_to_skip = 5;
    std::vector<int> removedIds = OMC.ProposedCalibration(num_to_skip);

    // 计算最终的全系统误差
    CalibrationError finalError = OMC.EvaluateCalibration();

    // 生成 Proposed1 详细标定报告
    QString resultPath = m_savePath + "/CalibrationResult_Proposed1.txt";
    std::ofstream file(resultPath.toStdString());
    if (file.is_open()) {
        file << "==========================================================\n";
        file << "        Proposed1 Calibration Detailed Report\n";
        file << "==========================================================\n\n";

        file << "Summary:\n";
        file << "Final Avg Translation Error:   " << finalError.avgTranslationError << " mm\n";
        file << "Final Avg Rotation Error:      " << finalError.avgRotationError << " deg\n";
        file << "Removed Groups (Original IDs, top " << num_to_skip << " max errors): ";
        if (removedIds.empty()) file << "None";
        else {
            for (int id : removedIds) file << (id + 1) << " "; // +1 匹配人类直觉组号
        }
        file << "\n\n";

        file << ">>> Final Proposed1 Matrices:\n\n";
        file << "tool1_2EMsensor (Hc1 inverse - NonLinear AX=XB):\n";
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) file << std::setw(12) << OMC.tool1_2EMsensor.at<double>(r, c) << " ";
            file << "\n";
        }
        file << "\nEM_2tool2 (Hc2 - Virtual Point Cloud SVD):\n";
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) file << std::setw(12) << OMC.EM_2tool2.at<double>(r, c) << " ";
            file << "\n";
        }
        file.close();
    }

    // 保存矩阵到系统后续融合(Fusion)可用的 txt
    saveMatToTxt(OMC.tool1_2EMsensor, "tool1_2EMsensor.txt");
    saveMatToTxt(OMC.EM_2tool2, "EM_2tool2.txt");

    // UI 提示格式化
    QString removedStr = "None";
    if (!removedIds.empty()) {
        QStringList list;
        for (int id : removedIds) list << QString::number(id + 1);
        removedStr = list.join(", ");
    }

    QString msg = QString::fromUtf8("Proposed1 标定完成！\n\n"
        "最终平均平移误差: %1 mm\n"
        "最终平均旋转误差: %2 度\n"
        "自动剔除异常组号: %3\n\n"
        "详细报告已存至: CalibrationResult_Proposed1.txt")
        .arg(finalError.avgTranslationError, 0, 'f', 4)
        .arg(finalError.avgRotationError, 0, 'f', 4)
        .arg(removedStr);

    QMessageBox::information(this, QString::fromUtf8("Proposed1 标定结果"), msg);

    ui->label->setText(QString::fromUtf8("<font color='green'><b>Proposed1 计算成功！</b></font><br/>最终平均平移误差: %1 mm").arg(finalError.avgTranslationError));
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

namespace {
    // 匿名空间下的辅助工具：OpenCV 与 Eigen 的安全转换
    Eigen::Matrix4d cvMat2Eigen(const cv::Mat& mat) {
        Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
        if (mat.empty() || mat.rows < 4 || mat.cols < 4) return res;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                res(i, j) = mat.at<double>(i, j);
        return res;
    }

    cv::Mat eigen2CvMat(const Eigen::Matrix4d& mat) {
        cv::Mat res = cv::Mat::eye(4, 4, CV_64F);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                res.at<double>(i, j) = mat(i, j);
        return res;
    }

    // 旋转矩阵与轴角(李代数)互相转换，避免四元数非线性约束
    Eigen::Vector3d rot2AxisAngle(const Eigen::Matrix3d& R) {
        Eigen::AngleAxisd aa(R);
        return aa.axis() * aa.angle();
    }

    Eigen::Matrix3d axisAngle2Rot(const Eigen::Vector3d& r) {
        double angle = r.norm();
        if (angle < 1e-8) return Eigen::Matrix3d::Identity();
        return Eigen::AngleAxisd(angle, r / angle).toRotationMatrix();
    }
}

cv::Mat OMCalibrate::solveAXXB_LM(const std::vector<cv::Mat>& A_cv, const std::vector<cv::Mat>& B_cv, const cv::Mat& initial_X) {
    int n = A_cv.size();
    std::vector<Eigen::Matrix4d> A(n), B(n);
    for (int i = 0; i < n; ++i) {
        A[i] = cvMat2Eigen(A_cv[i]);
        B[i] = cvMat2Eigen(B_cv[i]);
    }

    // 提取初始值的参数 (3个旋转参数 + 3个平移参数)
    Eigen::Matrix4d X_init = cvMat2Eigen(initial_X);
    Eigen::VectorXd x(6);
    x.segment<3>(0) = rot2AxisAngle(X_init.block<3, 3>(0, 0));
    x.segment<3>(3) = X_init.block<3, 1>(0, 3);

    // 误差计算闭包 (重构 MATLAB 中的 costFunc1)
    auto calcRes = [&](const Eigen::VectorXd& params) {
        Eigen::VectorXd res(12 * n);
        Eigen::Matrix3d R = axisAngle2Rot(params.segment<3>(0));
        Eigen::Vector3d t = params.segment<3>(3);
        for (int i = 0; i < n; ++i) {
            Eigen::Matrix3d Ra = A[i].block<3, 3>(0, 0);
            Eigen::Vector3d ta = A[i].block<3, 1>(0, 3);
            Eigen::Matrix3d Rb = B[i].block<3, 3>(0, 0);
            Eigen::Vector3d tb = B[i].block<3, 1>(0, 3);

            // R_A * R_X - R_X * R_B
            Eigen::Matrix3d dR = Ra * R - R * Rb;
            // t_A + R_A * t_X - (t_X + R_X * t_B)
            Eigen::Vector3d dt = Ra * t + ta - R * tb - t;

            for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) res(i * 12 + r * 3 + c) = dR(r, c);
            res.segment<3>(i * 12 + 9) = dt;
        }
        return res;
        };

    // Levenberg-Marquardt 非线性迭代求解
    double lambda = 1e-3;
    for (int iter = 0; iter < 100; ++iter) {
        Eigen::VectorXd r0 = calcRes(x);
        Eigen::MatrixXd J(12 * n, 6);
        double eps = 1e-5;
        // 数值雅可比矩阵
        for (int j = 0; j < 6; ++j) {
            Eigen::VectorXd xp = x; xp(j) += eps;
            J.col(j) = (calcRes(xp) - r0) / eps;
        }
        Eigen::MatrixXd H = J.transpose() * J;
        H += lambda * Eigen::MatrixXd::Identity(6, 6);
        Eigen::VectorXd g = J.transpose() * r0;

        Eigen::VectorXd dx = H.ldlt().solve(-g);
        Eigen::VectorXd x_new = x + dx;

        if (calcRes(x_new).squaredNorm() < r0.squaredNorm()) {
            x = x_new;
            lambda /= 10;
            if (dx.norm() < 1e-6) break;
        }
        else {
            lambda *= 10;
        }
    }

    Eigen::Matrix4d X_opt = Eigen::Matrix4d::Identity();
    X_opt.block<3, 3>(0, 0) = axisAngle2Rot(x.segment<3>(0));
    X_opt.block<3, 1>(0, 3) = x.segment<3>(3);
    return eigen2CvMat(X_opt);
}

cv::Mat OMCalibrate::optimizeBaseToGridSVD(const std::vector<cv::Mat>& tool1_2tool2_vec,
    const std::vector<cv::Mat>& EMsensor_2Aurora_vec,
    const cv::Mat& EMsensor_2tool1) {
    // 逆矩阵: M1 -> EMS
    cv::Mat inv_EMsensor_2tool1;
    cv::invert(EMsensor_2tool1, inv_EMsensor_2tool1, cv::DECOMP_SVD);
    Eigen::Matrix4d T_M1_EMS = cvMat2Eigen(inv_EMsensor_2tool1);

    // 工具标志点局部坐标 (毫米转米) -> 等同于 MATLAB points_in_tool_mm
    Eigen::MatrixXd P_M1(4, 4);
    P_M1 << 0.0, 0.0, 0.0, 0.0,
        0.0, 0.02859, 0.0, -0.04432,
        0.0, 0.04102, 0.08800, 0.04045,
        1.0, 1.0, 1.0, 1.0;

    int n_poses = EMsensor_2Aurora_vec.size();
    Eigen::MatrixXd P_M2_all(3, 4 * n_poses);
    Eigen::MatrixXd P_EM_all(3, 4 * n_poses);

    for (int j = 0; j < n_poses; ++j) {
        Eigen::Matrix4d t12t2 = cvMat2Eigen(tool1_2tool2_vec[j]);
        Eigen::Matrix4d ems2aurora = cvMat2Eigen(EMsensor_2Aurora_vec[j]);

        Eigen::MatrixXd P_M2_hom = t12t2 * P_M1;
        Eigen::MatrixXd P_EM_hom = ems2aurora * T_M1_EMS * P_M1;

        P_M2_all.block(0, j * 4, 3, 4) = P_M2_hom.block(0, 0, 3, 4);
        P_EM_all.block(0, j * 4, 3, 4) = P_EM_hom.block(0, 0, 3, 4);
    }

    // 绝对定向 SVD 解算
    Eigen::Vector3d mean_M2 = P_M2_all.rowwise().mean();
    Eigen::Vector3d mean_EM = P_EM_all.rowwise().mean();

    Eigen::MatrixXd P_M2_centered = P_M2_all.colwise() - mean_M2;
    Eigen::MatrixXd P_EM_centered = P_EM_all.colwise() - mean_EM;

    Eigen::Matrix3d H = P_EM_centered * P_M2_centered.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R_opt = V * U.transpose();
    if (R_opt.determinant() < 0) {
        V.col(2) *= -1.0;
        R_opt = V * U.transpose();
    }

    Eigen::Vector3d t_opt = mean_M2 - R_opt * mean_EM;

    Eigen::Matrix4d EM_2tool2_opt = Eigen::Matrix4d::Identity();
    EM_2tool2_opt.block<3, 3>(0, 0) = R_opt;
    EM_2tool2_opt.block<3, 1>(0, 3) = t_opt;

    return eigen2CvMat(EM_2tool2_opt);
}

std::vector<int> OMCalibrate::ProposedCalibration(int num_to_skip) {
    std::vector<int> skipList;
    if (tool1_2Vega.size() < 10) return skipList;

    // 1. 利用 OpenCV 获取 Tsai 初始解 (替代 Lazax_HandeyeTSAI.m)
    std::vector<cv::Mat> R_gripper2base, t_gripper2base, R_target2cam, t_target2cam;
    for (size_t i = 0; i < EMsensor_2Aurora.size(); ++i) {
        R_gripper2base.push_back(EMsensor_2Aurora[i](cv::Rect(0, 0, 3, 3)));
        t_gripper2base.push_back(EMsensor_2Aurora[i](cv::Rect(3, 0, 1, 3)));
        R_target2cam.push_back(tool2_2tool1[i](cv::Rect(0, 0, 3, 3)));
        t_target2cam.push_back(tool2_2tool1[i](cv::Rect(3, 0, 1, 3)));
    }
    cv::Mat R_cam2gripper, t_cam2gripper;
    cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
        R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);

    cv::Mat tool1_2EMsensor_Tsai = createTransformationMatrix(R_cam2gripper, t_cam2gripper);
    cv::Mat EMsensor_2tool1_Tsai;
    cv::invert(tool1_2EMsensor_Tsai, EMsensor_2tool1_Tsai, cv::DECOMP_SVD);

    // 2. 粗略计算闭环矩阵用于异常值评估
    cv::Mat EM_2tool2_Tsai = cv::Mat::zeros(4, 4, CV_64F);
    for (size_t j = 0; j < tool1_2tool2.size(); ++j) {
        cv::Mat chain = tool1_2tool2[j] * EMsensor_2tool1_Tsai * Aurora_2EMsensor[j];
        EM_2tool2_Tsai += chain;
    }
    EM_2tool2_Tsai /= (double)tool1_2tool2.size();

    // 3. 计算全局重投影误差并寻找异常组 (等效 MATLAB 逻辑)
    struct FrameErr { int idx; double err; };
    std::vector<FrameErr> errors;
    for (size_t j = 0; j < tool2_2Vega.size(); ++j) {
        cv::Mat Tout = tool2_2Vega[j] * EM_2tool2_Tsai * EMsensor_2Aurora[j] * tool1_2EMsensor_Tsai;
        cv::Mat t_out = Tout(cv::Rect(3, 0, 1, 3));
        cv::Mat t_true = tool1_2Vega[j](cv::Rect(3, 0, 1, 3));
        errors.push_back({ (int)j, cv::norm(t_out - t_true) });
    }

    std::sort(errors.begin(), errors.end(), [](const FrameErr& a, const FrameErr& b) { return a.err > b.err; });
    for (int i = 0; i < num_to_skip && i < (int)errors.size(); ++i) skipList.push_back(errors[i].idx);

    std::vector<int> sorted_skipList = skipList;
    std::sort(sorted_skipList.rbegin(), sorted_skipList.rend());
    for (int idx : sorted_skipList) RemoveGroup(idx);

    // 4. 构建相对位姿数据对 A 和 B
    std::vector<cv::Mat> A_rel, B_rel;
    for (size_t k = 0; k < EMsensor_2Aurora.size() - 1; ++k) {
        cv::Mat Ak = Aurora_2EMsensor[k + 1] * EMsensor_2Aurora[k];
        cv::Mat Bk = tool2_2tool1[k + 1] * tool1_2tool2[k];
        A_rel.push_back(Ak);
        B_rel.push_back(Bk);
    }

    // 5. 执行非线性迭代 (Proposed1 核心)
    EMsensor_2tool1 = solveAXXB_LM(A_rel, B_rel, EMsensor_2tool1_Tsai);
    cv::invert(EMsensor_2tool1, tool1_2EMsensor, cv::DECOMP_SVD);

    // 6. SVD 点云解算
    EM_2tool2 = optimizeBaseToGridSVD(tool1_2tool2, EMsensor_2Aurora, EMsensor_2tool1);

    return skipList;
}
