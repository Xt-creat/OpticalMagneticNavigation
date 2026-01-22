#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include "NDIWorker.h"
#include"DeviceReader.h"
#include "DisplayWidget.h" 
#include "TrackingRecordingDialog.h"
#include <opencv2/opencv.hpp>
#include <vtkSmartPointer.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
	bool m_connected = false;
	QString m_savePath;

	std::vector<ToolData> O_data;        //实时更新的光学和电磁跟踪数据
	std::vector<ToolData> M_data;

    MainWindow(QWidget *parent = 0);
	~MainWindow();

	void quaternionToEuler(double w, double x, double y, double z,
		double& roll, double& pitch, double& yaw);

	void loadMatFromTxt(const std::string& filename, cv::Mat& mat);
	
	cv::Mat transformToMatrix(const Transform& transform_O2);

	void rotationMatrixToQuaternion(const cv::Mat& R, double& q0, double& qx, double& qy, double& qz);
	void matrixToTransform(const cv::Mat& transformMat, double& q0, double& qx, double& qy, double& qz, double& tx, double& ty, double& tz);

private slots:
	void OnSelectBtnClicked();
	void OnStartBtnClicked();
	void updateSystemStatus(bool isConnected);

	void OnStartTracking();
	void OnStopTracking();

	void onDisplayBtnClicked();  

	void updateODataLabel(const std::vector<ToolData>& tools);
	void updateMDataLabel(const std::vector<ToolData>& tools);

	void stopTrackingThreads(); // 新增：停止跟踪线程
	void startTrackingThreads(); // 新增：启动跟踪线程

	////四元数和平移转矩阵
	//void MainWindow::PinErrorRMS();

private:
    Ui::MainWindow *ui;
	NDIWorker *m_NDIWorker;
	DeviceReader* deviceReader1;
	DeviceReader* deviceReader2;
	QThread* thread1;
	QThread* thread2;

	qint64 m_StartTimePoint;

	DisplayWidget* displayWidget = nullptr;  // 保存显示窗口指针
	TrackingRecordingDialog* m_recordingDialog = nullptr;
	class CalibrationDialog* m_calibrationDialog = nullptr; // 新增：标定对话框指针
};

#endif // MAINWINDOW_H
