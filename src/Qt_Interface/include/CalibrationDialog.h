#ifndef CALIBRATIONDIALOG_H
#define CALIBRATIONDIALOG_H

#include <QDialog>
#include "ToolData.h"
#include <fstream>
#include <opencv2/opencv.hpp>

// 前向声明Ui命名空间中的类
namespace Ui {
	class Calibration;
}

class CalibrationDialog : public QDialog
{
	Q_OBJECT

public:
	QString m_savePath;

	explicit CalibrationDialog(const QString& savePath, QWidget *parent = nullptr);
	~CalibrationDialog();
	
	std::string toolDataToCSV(const ToolData& toolData);

	void saveMatToTxt(const cv::Mat& mat, const std::string& filename);

	

private slots:
	void Calibrationcalculate();     //计算手眼标定
	void StaticCalibrateCollection(); // 采集

private:
	Ui::Calibration *ui;  

	int currentGroupIndex = 0;      // 当前采集的组数
	const int totalGroups = 10;     // 总共 30 组
	const int numberOfLines = 10;   // 每组采集 10 行

	std::ofstream csvFile1, csvFile2;

};

#endif // CALIBRATIONDIALOG_H