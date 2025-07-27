#include "CalibrationDialog.h"
#include "ui_CalibrationDialog.h"  // 包含自动生成的UI头文件
#include <QPushButton>
#include <QTimer>
#include <QMessageBox>
#include <QDebug>
#include <CombinedApi.h>
#include <OMCalibrate.h>
#include <fstream>


extern CombinedApi O_capi;
extern CombinedApi M_capi;


CalibrationDialog::CalibrationDialog(const QString& savePath, QWidget *parent) :
	QDialog(parent),
	ui(new Ui::Calibration),  // 初始化UI
	m_savePath(savePath)
{
	ui->setupUi(this);  // 设置UI

	
	// 设置窗口标题
	this->setWindowTitle("System calibration");

	// 修改标签文本
	//ui->label->setText("Preparing to start system calibration...");
	ui->label_2->setText("Waiting for collection to begin");

	// 打开 CSV 文件（只打开一次）
	csvFile1.open("Vega.csv", std::ios::out);
	csvFile2.open("Aurora.csv", std::ios::out);

	// 写 CSV 表头
	csvFile1 << "#Tools\n";
	csvFile2 << "#Tools\n";

	
	//connect(ui->startButton, &QPushButton::clicked,this, &CalibrationDialog::StaticCalibrateCollection);    //初始的采集实现
	connect(ui->buttonBox->button(QDialogButtonBox::Cancel), &QPushButton::clicked,this, &CalibrationDialog::reject);


	connect(ui->recordButton, &QPushButton::clicked,
		this, &CalibrationDialog::StaticCalibrateCollection);

	connect(ui->calculateButton, &QPushButton::clicked, this, &CalibrationDialog::Calibrationcalculate);
}

CalibrationDialog::~CalibrationDialog()
{
	if (csvFile1.is_open()) csvFile1.close();
	if (csvFile2.is_open()) csvFile2.close();
	delete ui;
}




void CalibrationDialog::StaticCalibrateCollection()
{
	const int numberOfLines = 10;  // 每组采集 10 行
	int linesWritten = 0;

	while (linesWritten < numberOfLines)
	{
		std::vector<ToolData> enabledTools2 = M_capi.getTrackingDataBX(
			TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms);
		std::vector<ToolData> enabledTools1 = O_capi.getTrackingDataBX2();

		// 如果是第 1 行，写 CSV 表头
		if (linesWritten == 0) {
			for (int t = 0; t < enabledTools1.size(); t++) {
				csvFile1 << ",ToolInfo,Frame#,PortHandle,Face#,timespec_s,timespec_ns,"
					"TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers";
				for (int m = 0; m < enabledTools1[t].markers.size(); m++) {
					csvFile1 << ",Marker" << m << ".Status,Tx,Ty,Tz";
				}
			}
			csvFile1 << std::endl;

			for (int t = 0; t < enabledTools2.size(); t++) {
				csvFile2 << ",ToolInfo,Frame#,PortHandle,Face#,timespec_s,timespec_ns,"
					"TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers";
				for (int m = 0; m < enabledTools2[t].markers.size(); m++) {
					csvFile2 << ",Marker" << m << ".Status,Tx,Ty,Tz";
				}
			}
			csvFile2 << std::endl;
		}

		// 写 Optical
		for (int t = 0; t < enabledTools1.size(); t++) {
			csvFile1 << "," << enabledTools1[t].toolInfo << ","
				<< toolDataToCSV(enabledTools1[t]);
		}
		csvFile1 << std::endl;

		// 写 Magnetic
		for (int t = 0; t < enabledTools2.size(); t++) {
			csvFile2 << "," << enabledTools2[t].toolInfo << ","
				<< toolDataToCSV(enabledTools2[t]);
		}
		csvFile2 << std::endl;

		linesWritten++;
	}
	csvFile1 << std::endl;
	csvFile2 << std::endl;

	// 1 组采集完成 → 更新组数
	currentGroupIndex++;

	// 更新 UI 显示进度
	ui->label_2->setText(QString("Current progress: %1 / %2 groups")
		.arg(currentGroupIndex)
		.arg(totalGroups));

	// 如果全部完成
	if (currentGroupIndex >= totalGroups) {
		ui->label->setText("All data collection completed!");
		ui->recordButton->setEnabled(false);

		ui->label->setText("Calculate Done !");
	}
	else {
		ui->label->setText("Collecting...");
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

	OMC.HandeyeCalibrate3();
	std::cout << "Marker1_2EMsensor" << OMC.Marker1_2EMsensor << std::endl;
	std::cout << std::endl;
	//std::cout << "EMsensor_2Marker1" << OMC.EMsensor_2Marker1 << std::endl;
	//std::cout << std::endl;
	std::cout << "EM_2Marker2" << OMC.EM_2Marker2 << std::endl;
	std::cout << std::endl;

	saveMatToTxt(OMC.Marker1_2EMsensor, "Marker1_2EMsensor.txt");
	saveMatToTxt(OMC.EM_2Marker2, "EM_2Marker2.txt");

	ui->label->setText("Calculate Done !");
}


void CalibrationDialog::saveMatToTxt(const cv::Mat& mat, const std::string& filename)
{
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "无法打开文件: " << filename << std::endl;
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
	std::cout << "矩阵已保存到: " << filename << std::endl;
}




