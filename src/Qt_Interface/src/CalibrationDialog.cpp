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

		// 写数据行
		csvFile1 << enabledTools1.size();
		for (int t = 0; t < (int)enabledTools1.size(); t++) {
			csvFile1 << "," << enabledTools1[t].toolInfo << "," << toolDataToCSV(enabledTools1[t]);
		}
		csvFile1 << std::endl;

		csvFile2 << enabledTools2.size();
		for (int t = 0; t < (int)enabledTools2.size(); t++) {
			csvFile2 << "," << enabledTools2[t].toolInfo << "," << toolDataToCSV(enabledTools2[t]);
		}
		csvFile2 << std::endl;

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
	//调用OMCalibrate中的方法实现采集数据的处理和初步计算标定矩阵
	OMCalibrate OMC;

	ui->label->setText(QString::fromLocal8Bit("正在计算标定矩阵..."));
	ui->calculateButton->setEnabled(false);
	QApplication::processEvents(); // 刷新UI显示

	// 传入保存路径，加载数据
	OMC.LoadData(m_savePath.toStdString(), totalGroups);

	// 执行标定计算
	OMC.HandeyeCalibrate3();
	
	// 检查标定结果是否有效
	if (OMC.Marker1_2EMsensor.empty() || OMC.EM_2Marker2.empty()) {
		ui->label->setText(QString::fromLocal8Bit("<font color='red'>计算失败: 采集数据无法解算有效矩阵，请重新采集</font>"));
		ui->calculateButton->setEnabled(true);
		return;
	}

	// 评估标定结果
	CalibrationError error = OMC.EvaluateCalibration();

	// 保存评估结果到文件
	QString resultPath = m_savePath + "/CalibrationResult.txt";
	std::ofstream file(resultPath.toStdString());
	if (file.is_open()) {
		file << "=== Calibration Matrices ===\n\n";
		file << "Marker1_2EMsensor:\n";
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) file << OMC.Marker1_2EMsensor.at<double>(i, j) << " ";
			file << "\n";
		}
		file << "\nEM_2Marker2:\n";
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) file << OMC.EM_2Marker2.at<double>(i, j) << " ";
			file << "\n";
		}
		file << "\n=== Evaluation Results ===\n\n";
		file << "Average Translation Error: " << error.avgTranslationError << " mm\n";
		file << "Average Rotation Error: " << error.avgRotationError << " degrees\n";
		file << "\nDetailed Errors per Group:\n";
		for (size_t i = 0; i < error.translationErrors.size(); i++) {
			file << "Group " << i + 1 << ": Translation Error = " << error.translationErrors[i] 
				 << " mm, Rotation Error = " << error.rotationErrors[i] << " degrees\n";
		}
		file.close();
	}

	// 同时保存原始格式的矩阵文件供主界面加载
	saveMatToTxt(OMC.Marker1_2EMsensor, "Marker1_2EMsensor.txt");
	saveMatToTxt(OMC.EM_2Marker2, "EM_2Marker2.txt");

	// 弹出结果窗口
	QString msg = QString::fromLocal8Bit("标定计算完成！\n\n平均平移误差: %1 mm\n平均旋转误差: %2 度\n\n结果已保存至: %3")
		.arg(error.avgTranslationError, 0, 'f', 4)
		.arg(error.avgRotationError, 0, 'f', 4)
		.arg(resultPath);
	QMessageBox::information(this, QString::fromLocal8Bit("标定评估结果"), msg);

	ui->label->setText(QString::fromLocal8Bit("<font color='green'><b>计算完成并已评估！</b></font><br/>结果已保存到 CalibrationResult.txt"));
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




