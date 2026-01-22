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

	// 新增：接收外部推送的数据
	void addOData(const std::vector<ToolData>& tools);
	void addMData(const std::vector<ToolData>& tools);

private slots:
	void Calibrationcalculate();     //计算手眼标定
	void StaticCalibrateCollection(); // 采集
	void onStartButtonClicked();      // 新增：点击开始后的处理

private:
	Ui::Calibration *ui;  

	int currentGroupIndex = 0;      // 当前采集的组数
	int totalGroups = 30;           // 总共组数 (可调)
	const int numberOfLines = 10;   // 每组采集 10 行
	bool headerWritten = false;     // 是否已写表头
	bool m_isCollecting = false;    // 是否正在单组采集

	std::vector<ToolData> m_lastOData; // 缓存最新数据
	std::vector<ToolData> m_lastMData;

	std::ofstream csvFile1, csvFile2;

};

#endif // CALIBRATIONDIALOG_H