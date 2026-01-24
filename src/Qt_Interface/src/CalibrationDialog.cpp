#include "CalibrationDialog.h"
#include "ui_CalibrationDialog.h"  // 包含自动生成的UI头文件
#include <QApplication>
#include <QTimer>
#include <QMessageBox>
#include <QDebug>
#include <CombinedApi.h>
#include <OMCalibrate.h>
#include <fstream>
#include <thread>
#include <chrono>


extern CombinedApi O_capi;
extern CombinedApi M_capi;


CalibrationDialog::CalibrationDialog(const QString& savePath, QWidget *parent) :
	QDialog(parent),
	ui(new Ui::Calibration),  // 初始化UI
	m_savePath(savePath)
{
	ui->setupUi(this);  // 设置UI

	// 检查路径有效性
	if (m_savePath.isEmpty()) {
		ui->label_2->setText(QString::fromLocal8Bit("未选择路径"));
		ui->label->setText(QString::fromLocal8Bit("错误: 请先在主界面选择数据保存路径！"));
		ui->startButton->setEnabled(false);
	}
	else {
		ui->label_2->setText(QString::fromLocal8Bit("等待开始..."));
	}
	
	connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &CalibrationDialog::reject);

	connect(ui->startButton, &QPushButton::clicked, this, &CalibrationDialog::onStartButtonClicked);

	connect(ui->recordButton, &QPushButton::clicked,
		this, &CalibrationDialog::StaticCalibrateCollection);

	connect(ui->calculateButton, &QPushButton::clicked, this, &CalibrationDialog::Calibrationcalculate);
}

void CalibrationDialog::onStartButtonClicked()
{
	totalGroups = ui->groupSpinBox->value();
	currentGroupIndex = 0;

	// 尝试打开 CSV 文件
	if (csvFile1.is_open()) csvFile1.close();
	if (csvFile2.is_open()) csvFile2.close();

	csvFile1.open((m_savePath + "/Vega.csv").toStdString(), std::ios::out);
	csvFile2.open((m_savePath + "/Aurora.csv").toStdString(), std::ios::out);

	if (!csvFile1.is_open() || !csvFile2.is_open()) {
		ui->label->setText(QString::fromLocal8Bit("错误: 无法创建CSV文件，请检查路径权限"));
		return;
	}

	// 写初始标记
	csvFile1 << "#Tools\n";
	csvFile2 << "#Tools\n";
	headerWritten = false;

	ui->groupSpinBox->setEnabled(false);
	ui->iterSpinBox->setEnabled(false);
	ui->startButton->setEnabled(false);
	ui->recordButton->setEnabled(true);
	
	ui->label_2->setText(QString::fromLocal8Bit("当前进度: 0 / %1 组").arg(totalGroups));
	ui->label->setText(QString::fromLocal8Bit("请保持工具静止，点击“单组采集”"));
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
	int maxWaitCycles = 200; // 等待数据的最大循环次数
	int waitCycles = 0;

	ui->recordButton->setEnabled(false);
	ui->label->setText(QString::fromLocal8Bit("正在采集第 %1 组数据 (共 %2 组)...").arg(currentGroupIndex + 1).arg(totalGroups));
	
	// 清理旧缓存，确保采集的是最新时刻的数据
	m_lastOData.clear();
	m_lastMData.clear();

	// 局部缓存本组的数据行，确保只有在采集完整且合格时才写入文件
	std::vector<std::string> groupLines1;
	std::vector<std::string> groupLines2;

	while (linesWritten < linesPerGroup && waitCycles < maxWaitCycles)
	{
		QApplication::processEvents();
		std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 采样频率匹配主界面

		// 直接使用由 MainWindow 推送的数据
		std::vector<ToolData> enabledTools1 = m_lastOData;
		std::vector<ToolData> enabledTools2 = m_lastMData;

		bool allValid = (enabledTools1.size() >= 3) && (!enabledTools2.empty());
		
		if (allValid) {
			if (enabledTools1[0].transform.isMissing() || 
				enabledTools1[2].transform.isMissing() || 
				enabledTools2[0].transform.isMissing()) {
				allValid = false;
			}
		}

		if (!allValid) {
			waitCycles++;
			continue; 
		}

		// --- 新增：误差验证逻辑 ---
		double emError = enabledTools2[0].transform.error;
		if (emError > 0.2) {
			ui->label->setText(QString::fromLocal8Bit("<font color='red'><b>采集异常：</b>电磁工具误差过大 (%1 > 0.2)！<br/>请确保工具可见且保持静止，点击“单组采集”重试第 %2 组。</font>")
				.arg(emError)
				.arg(currentGroupIndex + 1));
			ui->recordButton->setEnabled(true);
			return; // 终止当前组采集，文件不写入，进度不增加
		}

		// 写 CSV 表头 (全局只写一次)
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

		// 将数据行构造为字符串并存入局部缓存
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
		m_lastOData.clear(); // 消费掉当前帧，等待下一帧推送
		m_lastMData.clear();
	}

	if (waitCycles >= maxWaitCycles) {
		ui->label->setText(QString::fromLocal8Bit("<font color='red'>采集失败: 未收到有效数据。请确保主界面跟踪正在运行且工具可见。</font>"));
		ui->recordButton->setEnabled(true);
		return;
	}

	// 采集成功：将缓存的 10 行数据写入文件
	for (const auto& line : groupLines1) csvFile1 << line << std::endl;
	for (const auto& line : groupLines2) csvFile2 << line << std::endl;

	csvFile1.flush();
	csvFile2.flush();

	currentGroupIndex++;
	ui->label_2->setText(QString::fromLocal8Bit("当前进度: %1 / %2 组").arg(currentGroupIndex).arg(totalGroups));

	if (currentGroupIndex >= totalGroups) {
		ui->label->setText(QString::fromLocal8Bit("<b>数据采集全部完成！</b><br/>请点击“执行计算”生成标定矩阵"));
		ui->recordButton->setEnabled(false);
		ui->calculateButton->setEnabled(true);
	}
	else {
		ui->label->setText(QString::fromLocal8Bit("第 %1 组采集成功！请移动工具到下一位置，然后点击“单组采集”").arg(currentGroupIndex));
		ui->recordButton->setEnabled(true);
	}
}

/**
* @brief Returns a string representation of the data in CSV format.
* @details The CSV format is: "Frame#,ToolHandle,Face,TransformStatus,q0,qx,qy,qz,tx,ty,tz,error,#markers,[Marker1:status,x,y,z],[Marker2..."
*/
std::string CalibrationDialog::toolDataToCSV(const ToolData& toolData)
{
	std::stringstream stream;
	stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
	stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
		<< "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
	stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";

	// 分开输出 timespec_s 和 timespec_ns
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

	// Each marker is printed as: status,tx,ty,tz
	stream << "," << toolData.markers.size();
	for (int i = 0; i < toolData.markers.size(); i++)
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

	ui->label->setText(QString::fromLocal8Bit("正在计算标定矩阵..."));
	ui->calculateButton->setEnabled(false);
	QApplication::processEvents();

	// 1. 初始加载数据
	OMC.LoadData(m_savePath.toStdString(), totalGroups);

	int maxIterations = ui->iterSpinBox->value();
	std::vector<CalibrationError> iterationErrors;
	std::vector<cv::Mat> m12ems_history;
	std::vector<cv::Mat> em2m2_history;
	
	// 追踪原始索引
	std::vector<int> activeGroupIds;
	for (int i = 0; i < (int)OMC.M1_data.size(); ++i) activeGroupIds.push_back(i + 1);
	std::vector<std::vector<int>> iterationGroupIdsHistory; // 记录每次迭代剩下的 ID
	std::vector<int> removedIds;

	// 2. 迭代优化循环
	for (int iter = 0; iter <= maxIterations; ++iter) {
		iterationGroupIdsHistory.push_back(activeGroupIds);

		// 执行当前数据集的标定
		OMC.HandeyeCalibrate3();

		if (OMC.Marker1_2EMsensor.empty() || OMC.EM_2Marker2.empty()) {
			ui->label->setText(QString::fromLocal8Bit("<font color='red'>计算失败: 数据不足或解算异常</font>"));
			ui->calculateButton->setEnabled(true);
			return;
		}

		// 评估当前标定结果
		CalibrationError currentError = OMC.EvaluateCalibration();
		iterationErrors.push_back(currentError);
		m12ems_history.push_back(OMC.Marker1_2EMsensor.clone());
		em2m2_history.push_back(OMC.EM_2Marker2.clone());

		// 如果不是最后一次迭代，且还有足够的数据，则剔除误差最大的一组
		if (iter < maxIterations && currentError.translationErrors.size() > 3) {
			auto maxIt = std::max_element(currentError.translationErrors.begin(), currentError.translationErrors.end());
			int maxIdx = std::distance(currentError.translationErrors.begin(), maxIt);
			
			removedIds.push_back(activeGroupIds[maxIdx]); // 记录被剔除的原始 ID
			activeGroupIds.erase(activeGroupIds.begin() + maxIdx); // 从当前活跃 ID 中移除
			OMC.RemoveGroup(maxIdx);
		} else {
			break; 
		}
	}

	// 3. 保存详细结果报告 (包含迭代前后对比)
	QString resultPath = m_savePath + "/CalibrationResult.txt";
	std::ofstream file(resultPath.toStdString());
	if (file.is_open()) {
		file << "==========================================================\n";
		file << "            Iterative Calibration Detailed Report\n";
		file << "==========================================================\n\n";

		// A. 概览
		file << "Summary:\n";
		file << "Initial Avg Translation Error: " << iterationErrors.front().avgTranslationError << " mm\n";
		file << "Final Avg Translation Error:   " << iterationErrors.back().avgTranslationError << " mm\n";
		file << "Total Iterations Executed:     " << iterationErrors.size() - 1 << "\n";
		file << "Removed Groups (Original IDs): ";
		if (removedIds.empty()) file << "None";
		else for (int id : removedIds) file << id << " ";
		file << "\n\n";

		// 新增：迭代过程历史（列出中间每一步的平均误差）
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

		// B. 初始与最终的详细误差对比
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

		// C. 最终矩阵
		file << ">>> Final Calibration Matrices:\n\n";
		file << "Marker1_2EMsensor:\n";
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) file << std::setw(12) << m12ems_history.back().at<double>(r, c) << " ";
			file << "\n";
		}
		file << "\nEM_2Marker2:\n";
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) file << std::setw(12) << em2m2_history.back().at<double>(r, c) << " ";
			file << "\n";
		}
		file.close();
	}

	// 4. 更新最终矩阵文件供主界面加载
	saveMatToTxt(OMC.Marker1_2EMsensor, "Marker1_2EMsensor.txt");
	saveMatToTxt(OMC.EM_2Marker2, "EM_2Marker2.txt");

	// 5. 弹出结果反馈
	const CalibrationError& initial = iterationErrors.front();
	const CalibrationError& final = iterationErrors.back();
	
	QString removedStr = "None";
	if (!removedIds.empty()) {
		QStringList list;
		for(int id : removedIds) list << QString::number(id);
		removedStr = list.join(", ");
	}

	QString msg = QString::fromLocal8Bit("标定优化完成！\n\n"
		"初始误差: %1 mm / %2 度\n"
		"最终误差: %3 mm / %4 度\n"
		"剔除组号: %5\n\n"
		"详细报告已存至: CalibrationResult.txt")
		.arg(initial.avgTranslationError, 0, 'f', 4)
		.arg(initial.avgRotationError, 0, 'f', 4)
		.arg(final.avgTranslationError, 0, 'f', 4)
		.arg(final.avgRotationError, 0, 'f', 4)
		.arg(removedStr);

	QMessageBox::information(this, QString::fromLocal8Bit("标定优化结果"), msg);

	ui->label->setText(QString::fromLocal8Bit("<font color='green'><b>优化计算完成！</b></font><br/>最终平均误差: %1 mm").arg(final.avgTranslationError));
	ui->calculateButton->setEnabled(true);
}


void CalibrationDialog::saveMatToTxt(const cv::Mat& mat, const std::string& filename)
{
	QString fullPath = m_savePath + "/" + QString::fromStdString(filename); // 拼接完整路径
	std::ofstream file(fullPath.toStdString()); // 使用完整路径打开文件
	if (!file.is_open()) {
		std::cerr << "无法打开文件: " << fullPath.toStdString() << std::endl; // 打印完整路径
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
			if (j < mat.cols - 1) file << " "; // 列之间用空格分隔
		}
		file << "\n"; // 换行
	}

	file.close();
	std::cout << "矩阵已保存到: " << fullPath.toStdString() << std::endl; // 打印完整路径
}




