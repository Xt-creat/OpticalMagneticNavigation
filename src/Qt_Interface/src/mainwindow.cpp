#include <iostream>
#include <QPushButton>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QString>
#include <QThread>
#include <QDateTime>
#include <QMessageBox>
#include <qdebug.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>



#include "mainwindow.h"
#include "NDIWorker.h"
#include "ui_mainwindow.h"
#include "CalibrationDialog.h"
//#include "mainwindow.moc"

CombinedApi O_capi = CombinedApi();  
CombinedApi M_capi = CombinedApi();

//extern CombinedApi O_capi;
//extern CombinedApi M_capi;

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow),
	m_NDIWorker(nullptr),
	deviceReader1(nullptr),
	deviceReader2(nullptr),
	thread1(nullptr),
	thread2(nullptr)
{
	ui->setupUi(this);

	ui->m_OdataLabel->setMinimumHeight(150);
	ui->m_OdataLabel->setWordWrap(true);
	ui->m_MdataLabel->setMinimumHeight(150);
	ui->m_MdataLabel->setWordWrap(true);
	//ui->m_FusionLabel->setMinimumHeight(150);
	//ui->m_FusionLabel->setWordWrap(true);


	ui->m_systemstatus->setStyleSheet("font-size: 16px;");
	ui->m_OdataLabel->setStyleSheet("font-size: 16px;");
	ui->m_MdataLabel->setStyleSheet("font-size: 16px;");
	ui->m_FusionLabel->setStyleSheet("font-size: 16px;");


	connect(ui->m_SelectSavePathBtn, &QPushButton::clicked, this, &MainWindow::OnSelectBtnClicked);     //选择数据存储路径
	m_NDIWorker = new NDIWorker;
	ui->m_LinkBtn->setCheckable(false);
	connect(ui->m_LinkBtn, &QPushButton::clicked, m_NDIWorker, &NDIWorker::LinkNDI);                     //连接设备
	connect(m_NDIWorker, &NDIWorker::linkStatusChanged, this, &MainWindow::updateSystemStatus);
	connect(ui->m_CalibrateBtn, &QPushButton::clicked, this, &MainWindow::OnStartBtnClicked);               //系统标定
	connect(ui->m_TrackingBtn, &QPushButton::clicked, this, &MainWindow::OnStartTracking);                 //开始跟踪
	connect(ui->m_StopBtn, &QPushButton::clicked, this, &MainWindow::OnStopTracking);             //停止跟踪

	connect(ui->m_DisplayBtn, &QPushButton::clicked, this, &MainWindow::onDisplayBtnClicked);     //可视化

	ui->m_DisplaySavePathLineEdit->setEnabled(false);

	ui->m_SleepSpinBox->setValue(30);
	
}

MainWindow::~MainWindow()
{
    delete ui;

	if (m_NDIWorker) delete m_NDIWorker;

	if (thread1) {
		thread1->quit();
		thread1->wait();
		delete thread1;
		thread1 = nullptr;
	}

	if (thread2) {
		thread2->quit();
		thread2->wait();
		delete thread2;
		thread2 = nullptr;
	}

	if (deviceReader1) {
		delete deviceReader1;
		deviceReader1 = nullptr;
	}
	if (deviceReader2) {
		delete deviceReader2;
		deviceReader2 = nullptr;
	}
}

void MainWindow::OnStartBtnClicked()
{
	 CalibrationDialog *dialog = new CalibrationDialog(m_savePath, this);
	 dialog->show();
}



void MainWindow::updateSystemStatus(bool isConnected) {
	if (isConnected) {
		m_connected = isConnected;
		ui->m_systemstatus->setText("Device connection successful");
		ui->m_systemstatus->setStyleSheet("color: green;"); // 可选：设置绿色文字
	}
	else {
		ui->m_systemstatus->setText("Device not connected");
		ui->m_systemstatus->setStyleSheet("color: red;");
	}

}


void MainWindow::OnStartTracking()
{
	if (!m_connected) {
		ui->m_systemstatus->setText("Device not connected");
		ui->m_systemstatus->setStyleSheet("color: red;");
		return;
	}

	// --- 关键修复：检查是否已经在跟踪中 ---
	// 如果线程已经存在且在运行，说明系统正在采集数据，只需处理对话框逻辑
	if ((thread1 && thread1->isRunning()) || (thread2 && thread2->isRunning())) {
		if (!m_recordingDialog) {
			m_recordingDialog = new TrackingRecordingDialog(m_savePath, this);
			m_recordingDialog->setAttribute(Qt::WA_DeleteOnClose);
			connect(m_recordingDialog, &QObject::destroyed, [this]() {
				m_recordingDialog = nullptr;
				});
		}
		m_recordingDialog->show();
		m_recordingDialog->raise();
		return;
	}

	// 只有在没启动时才执行启动流程，不再盲目调用 OnStopTracking()
	O_capi.startTracking();
	M_capi.startTracking();

	//  只有在可视化窗口存在的情况下才启动动画
	if (displayWidget) {
		displayWidget->startRealtimeAnimation();
	}

		deviceReader1 = new DeviceReader(1, 100); // 每100ms读取一次
		deviceReader2 = new DeviceReader(2, 50); // 每50ms读取一次,电磁位姿更新更快，读取频率也设的更高
	    // 创建线程并将 DeviceReader 移到线程中
		thread1 = new QThread(this);
		deviceReader1->moveToThread(thread1);
		connect(thread1, &QThread::started, deviceReader1, &DeviceReader::startReading);
		connect(deviceReader1, &DeviceReader::newODataAvailable, this, &MainWindow::updateODataLabel);

		thread2 = new QThread(this);
		deviceReader2->moveToThread(thread2);
		connect(thread2, &QThread::started, deviceReader2, &DeviceReader::startReading);
		connect(deviceReader2, &DeviceReader::newMDataAvailable, this, &MainWindow::updateMDataLabel);

		// 启动线程
		thread1->start();
		thread2->start();

		ui->m_systemstatus->setText("Tracking in progress");
		ui->m_systemstatus->setStyleSheet("color: green;");

		// 显示记录对话框
		if (!m_recordingDialog) {
			m_recordingDialog = new TrackingRecordingDialog(m_savePath, this);
			m_recordingDialog->setAttribute(Qt::WA_DeleteOnClose);
			connect(m_recordingDialog, &QObject::destroyed, [this]() {
				m_recordingDialog = nullptr;
				});
		}
		m_recordingDialog->show();
		m_recordingDialog->raise();
}

void MainWindow::OnStopTracking() {
	
	// 1. --- 关键修复：必须先停止后台线程，再停止硬件 ---
	// 停止线程1
	if (thread1) {
		disconnect(deviceReader1, &DeviceReader::newODataAvailable, this, &MainWindow::updateODataLabel);
		QMetaObject::invokeMethod(deviceReader1, "stop", Qt::QueuedConnection);
		thread1->quit();
		if (!thread1->wait(1000)) { // 给线程 1 秒时间退出
			thread1->terminate();
			thread1->wait();
		}
		deviceReader1->deleteLater();
		deviceReader1 = nullptr;
		thread1->deleteLater();
		thread1 = nullptr;
	}

	// 停止线程2
	if (thread2) {
		disconnect(deviceReader2, &DeviceReader::newMDataAvailable, this, &MainWindow::updateMDataLabel);
		QMetaObject::invokeMethod(deviceReader2, "stop", Qt::QueuedConnection);
		thread2->quit();
		if (!thread2->wait(1000)) {
			thread2->terminate();
			thread2->wait();
		}
		deviceReader2->deleteLater();
		deviceReader2 = nullptr;
		thread2->deleteLater();
		thread2 = nullptr;
	}

	// 2. 线程完全退出后，再停止设备硬件跟踪
	O_capi.stopTracking();
	M_capi.stopTracking();

	// 确保录制对话框也被清理
	if (m_recordingDialog) {
		m_recordingDialog->close();
		m_recordingDialog = nullptr;
	}

	// UI 状态恢复
	ui->m_systemstatus->setText("Tracking has been stopped");
	ui->m_systemstatus->setStyleSheet("color: orange;");
}

void MainWindow::updateODataLabel(const std::vector<ToolData>& tools)
{
	O_data = tools;

	if (m_recordingDialog) {
		m_recordingDialog->addOData(tools);
	}

	if (tools.empty()) {
		ui->m_OdataLabel->setText("O 1:\nstatus: No Tools Detected");
		return;
	}

	QString O_text;
	QString F_text;
	
	const ToolData& data = tools[0];
	const Transform& transform = data.transform;

	//只要DisplayWidget存在，就推送位姿,toolHandle==1对应标准针
	if (transform.toolHandle == 1 && !transform.isMissing()) {
		O_text = QString("O %1:\n"
			"  X=%2 mm, Y=%3 mm, Z=%4 mm\n"
			"  q0=%5, qx=%6, qy=%7, qz=%8\n"
			"  status: %9\n\n")
			.arg(1)
			.arg(transform.tx, 0, 'f', 2)
			.arg(transform.ty, 0, 'f', 2)
			.arg(transform.tz, 0, 'f', 2)
			.arg(transform.q0, 0, 'f', 4)
			.arg(transform.qx, 0, 'f', 4)
			.arg(transform.qy, 0, 'f', 4)
			.arg(transform.qz, 0, 'f', 4)
			.arg(transform.isMissing() ? "Missing" : "Normal");

		F_text = O_text;

		if (displayWidget) {
			vtkSmartPointer<vtkTransform> ndiTransform = vtkSmartPointer<vtkTransform>::New();
			
			// 1. 坐标系映射 (仅使用 RotateZ(-90) 即可实现 X=ty, Y=-tx, Z=tz 的右手机映射)
			// 移除之前的 RotateX(-90)，因为它会使坐标系变为左手系，导致旋转反向
			ndiTransform->RotateZ(-90.0);

			// 2. 应用原始 NDI 数据
			ndiTransform->Translate(transform.tx, transform.ty, transform.tz);

			double angle = 0.0;
			if (transform.q0 >= 1.0) angle = 0.0;
			else if (transform.q0 <= -1.0) angle = 2.0 * M_PI;
			else angle = 2.0 * acos(transform.q0);

			ndiTransform->RotateWXYZ(
				vtkMath::DegreesFromRadians(angle), 
				transform.qx, transform.qy, transform.qz
			);

			displayWidget->updateExternalTran(ndiTransform);
		}

	}
	else {
		O_text = QString("O 1:\n"
			"status: Missing\n\n");

		// 安全检查：确保 M_data 已经初始化，且 O_data 至少包含两个工具（Marker2）
		if (M_data.empty() || O_data.size() < 2) {
			F_text = "Fusion: Waiting for more tools/data...";
		}
		else {
			cv::Mat Marker1_2EMsensor;
			loadMatFromTxt("Marker1_2EMsensor.txt", Marker1_2EMsensor);
			cv::Mat EM_2Marker2;
			loadMatFromTxt("EM_2Marker2.txt", EM_2Marker2);

			if (Marker1_2EMsensor.empty() || EM_2Marker2.empty()) {
				F_text = "Fusion: Calibration files error!";
			}
			else {
				const ToolData& data_M = M_data[0];
				const Transform& transform_M = data_M.transform;      //包含实时EMsensor_2Aurora (q0,qx,qy,qz,tx,ty,tz)
				const ToolData& data_O2 = O_data[1];
				const Transform& transform_O2 = data_O2.transform;    //包含实时Marker2_2Vega (q0,qx,qy,qz,tx,ty,tz)

				cv::Mat EMsensor_2Aurora = transformToMatrix(transform_M);
				cv::Mat Marker2_2Vega = transformToMatrix(transform_O2);

				// 检查生成的变换矩阵是否有效
				if (EMsensor_2Aurora.empty() || Marker2_2Vega.empty()) {
					F_text = "Fusion: Transform matrix error!";
				}
				else {
					cv::Mat output_Marker1_2Vega = Marker2_2Vega * EM_2Marker2 * EMsensor_2Aurora * Marker1_2EMsensor;

					double q0, qx, qy, qz, tx, ty, tz;
					matrixToTransform(output_Marker1_2Vega, q0, qx, qy, qz, tx, ty, tz);

					F_text = QString("Fusion :\n"
						"  X=%2 mm, Y=%3 mm, Z=%4 mm\n"
						"  q0=%5, qx=%6, qy=%7, qz=%8\n"
						"  status: %9\n\n")
						.arg(tx, 0, 'f', 2)
						.arg(ty, 0, 'f', 2)
						.arg(tz, 0, 'f', 2)
						.arg(q0, 0, 'f', 4)
						.arg(qx, 0, 'f', 4)
						.arg(qy, 0, 'f', 4)
						.arg(qz, 0, 'f', 4)
						.arg("Normal");

					if (displayWidget) {
						vtkSmartPointer<vtkTransform> ndiTransform = vtkSmartPointer<vtkTransform>::New();

						// 1. 坐标映射 (右手系)
						ndiTransform->RotateZ(-90.0);

						// 2. 原始数据
						ndiTransform->Translate(tx, ty, tz);

						double angle = 0.0;
						if (q0 >= 1.0) angle = 0.0;
						else if (q0 <= -1.0) angle = 2.0 * M_PI;
						else angle = 2.0 * acos(q0);

						ndiTransform->RotateWXYZ(
							vtkMath::DegreesFromRadians(angle),
							qx, qy, qz
						);

						displayWidget->updateExternalTran(ndiTransform);
					}
				}
			}
		}
	}

	//QLabel 只显示最近一次结果
	ui->m_OdataLabel->setText(O_text);
	ui->m_FusionLabel->setText(F_text);

	//追加写到日志文件
	QFile file("tracking_log.txt");
	if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
		QTextStream out(&file);
		out.setCodec("UTF-8");
		out << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz") << "\n";
		out << O_text << "\n";
	}
}


void MainWindow::updateMDataLabel(const std::vector<ToolData>& tools) {

	M_data = tools;

	if (m_recordingDialog) {
		m_recordingDialog->addMData(tools);
	}

	QString text;
	for (size_t i = 0; i < tools.size(); ++i) {
		const ToolData& data = tools[i];
		const Transform& transform = data.transform;

		text += QString("M %1:\n"
			"X=%2 mm, Y=%3 mm, Z=%4 mm\n"
			"q0=%5, qx=%6, qy=%7, qz=%8\n"
			"status: %9\n\n")
			.arg(i + 1)
			.arg(transform.tx, 0, 'f', 2)
			.arg(transform.ty, 0, 'f', 2)
			.arg(transform.tz, 0, 'f', 2)
			.arg(transform.q0, 0, 'f', 4)
			.arg(transform.qx, 0, 'f', 4)
			.arg(transform.qy, 0, 'f', 4)
			.arg(transform.qz, 0, 'f', 4)
			.arg(transform.isMissing() ? "Missing" : "Normal");

	
	}
	ui->m_MdataLabel->setText(text);
}


void MainWindow::onDisplayBtnClicked()
{
	if (!displayWidget)
	{
		displayWidget = new DisplayWidget(nullptr); // 独立窗口
		displayWidget->setAttribute(Qt::WA_DeleteOnClose);

		// 窗口关闭后把指针清空，避免悬空
		connect(displayWidget, &QObject::destroyed, [this]() {
			displayWidget = nullptr;
			});
	}

	displayWidget->show();
	displayWidget->raise();
	displayWidget->activateWindow();

	 // 使用相对路径或检查文件是否存在
	 QString modelPath = QCoreApplication::applicationDirPath() + "/tools/ToolScribing2.stl";
	 if (!QFile::exists(modelPath)) {
		 // 如果相对路径不存在，尝试原始硬编码路径作为兜底，但给出警告
		 modelPath = "D:/Optomagnetic-tracking/OpticalMagneticNavigation/tools/ToolScribing2.stl";
	 }
	 
	displayWidget->loadSTLModel(modelPath.toStdString());

	// 如果当前已经在跟踪中，新打开的显示窗口也要启动动画刷新
	if ((thread1 && thread1->isRunning()) || (thread2 && thread2->isRunning())) {
		displayWidget->startRealtimeAnimation();
	}

	//不启动动画刷新，除非 OnStartTracking() 调用
}



void MainWindow::OnSelectBtnClicked()
{
	QString selectedPath = QFileDialog::getExistingDirectory(this, tr("选择数据保存路径"),
		"", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	if (!selectedPath.isEmpty()) {
		m_savePath = selectedPath;
		ui->m_DisplaySavePathLineEdit->setText(m_savePath);
		qDebug() << "Selected Path:" << m_savePath;
	}
}

void MainWindow::quaternionToEuler(double w, double x, double y, double z,
	double& roll, double& pitch, double& yaw)
{
	double ysqr = y * y;

	double t0 = +2.0 * (w * x + y * z);
	double t1 = +1.0 - 2.0 * (x * x + ysqr);
	roll = std::atan2(t0, t1) * 180.0 / M_PI;

	double t2 = +2.0 * (w * y - z * x);
	t2 = t2 > 1.0 ? 1.0 : (t2 < -1.0 ? -1.0 : t2);
	pitch = std::asin(t2) * 180.0 / M_PI;

	double t3 = +2.0 * (w * z + x * y);
	double t4 = +1.0 - 2.0 * (ysqr + z * z);
	yaw = std::atan2(t3, t4) * 180.0 / M_PI;
}


void MainWindow::loadMatFromTxt(const std::string &filename, cv::Mat &mat)
{
    QString fullPath = m_savePath + "/" + QString::fromStdString(filename); // 拼接完整路径
    // 打开文件
    std::ifstream file(fullPath.toStdString()); // 使用完整路径打开文件
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << fullPath.toStdString() << std::endl; // 打印完整路径
        mat = cv::Mat(); // 确保返回空矩阵
        return;
    }

    std::vector<std::vector<double>> tempData; // 临时存储数据

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<double> rowData;
        double value;
        while (ss >> value) {
            rowData.push_back(value);
        }
        if (!rowData.empty()) {
            tempData.push_back(rowData);
        }
    }

    // 检查是否有数据，防止 tempData[0] 崩溃
    if (tempData.empty() || tempData[0].empty()) {
        std::cerr << "文件内容为空或格式错误: " << fullPath.toStdString() << std::endl;
        mat = cv::Mat();
        file.close();
        return;
    }

    // 将数据转换为cv::Mat
    int rows = (int)tempData.size();
    int cols = (int)tempData[0].size();
    mat = cv::Mat(rows, cols, CV_64F);

    for (int i = 0; i < rows; ++i) {
        // 确保每一列的大小一致，防止不规则数据导致崩溃
        for (int j = 0; j < (int)tempData[i].size() && j < cols; ++j) {
            mat.at<double>(i, j) = tempData[i][j];
        }
    }

    file.close();
    //std::cout << "数据已成功加载到 cv::Mat 中。" << std::endl;
}


cv::Mat MainWindow::transformToMatrix(const Transform& transform_O2) {

	double q0 = transform_O2.q0;
	double qx = transform_O2.qx;
	double qy = transform_O2.qy;
	double qz = transform_O2.qz;

	double R[3][3] = {
		{1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - q0 * qz), 2 * (qx * qz + q0 * qy)},
		{2 * (qx * qy + q0 * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - q0 * qx)},
		{2 * (qx * qz - q0 * qy), 2 * (qy * qz + q0 * qx), 1 - 2 * (qx * qx + qy * qy)}
	};


	cv::Mat transformMat = cv::Mat::eye(4, 4, CV_64F);  


	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			transformMat.at<double>(i, j) = R[i][j];
		}
	}

	transformMat.at<double>(0, 3) = transform_O2.tx;
	transformMat.at<double>(1, 3) = transform_O2.ty;
	transformMat.at<double>(2, 3) = transform_O2.tz;

	return transformMat;
}



// 从旋转矩阵转换为四元数
void MainWindow::rotationMatrixToQuaternion(const cv::Mat& R, double& q0, double& qx, double& qy, double& qz)
{
	double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

	if (trace > 0) {
		double s = 0.5 / std::sqrt(trace + 1.0);
		q0 = 0.25 / s;
		qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
		qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
		qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
	}
	else {
		if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2)) {
			double s = 2.0 * std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2));
			q0 = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
			qx = 0.25 * s;
			qy = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
			qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
		}
		else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
			double s = 2.0 * std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2));
			q0 = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
			qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
			qy = 0.25 * s;
			qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
		}
		else {
			double s = 2.0 * std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1));
			q0 = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
			qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
			qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
			qz = 0.25 * s;
		}
	}
}

// 从变换矩阵提取四元数和位移
void MainWindow::matrixToTransform(const cv::Mat& transformMat, double& q0, double& qx, double& qy, double& qz, double& tx, double& ty, double& tz)
{
	
	cv::Mat R = transformMat(cv::Rect(0, 0, 3, 3));

	
	rotationMatrixToQuaternion(R, q0, qx, qy, qz);

	
	tx = transformMat.at<double>(0, 3);
	ty = transformMat.at<double>(1, 3);
	tz = transformMat.at<double>(2, 3);
}





//void MainWindow::PinErrorRMS()
//{
//	if (rotation.size() < 3)
//	{
//		QMessageBox msgBox;
//		msgBox.setStyleSheet("background-color:white");
//		msgBox.setText(QString::fromLocal8Bit("采集点过少"));
//		msgBox.exec();
//		return;
//	}
//	if (rotation.size() != translation.size())
//	{
//		QMessageBox msgBox;
//		msgBox.setStyleSheet("background-color:white");
//		msgBox.setText(QString::fromLocal8Bit("Nums Error"));
//		msgBox.exec();
//		return;
//	}
//	size_t sz = rotation.size();
//	double rmsTotal = 0.0;
//	std::vector<cv::Mat> hrot, vtan;
//	cv::Mat vrot, tanV;
//	cv::Mat pos;
//	//std::vector<cv::Mat> postion;
//	//RX+t-X0=0 can be tansformed to [R -I]*[X^T X0^T]^T = -t
//	for (int i = 0; i < sz; i++) {
//		cv::Mat rot = cv::Mat::ones(3, 6, CV_64F);
//		cv::hconcat(rotation.at(i), -cv::Mat::eye(3, 3, CV_64F), rot);
//		hrot.push_back(rot);
//		//std::cout << rotation.at(i) << std::endl;
//		cv::Mat tan;
//		tan = cv::Mat(translation.at(i));
//		vtan.push_back(tan);
//	}
//	cv::vconcat(hrot, vrot);
//	// 位移矩阵
//	cv::vconcat(vtan, tanV);
//
//	// 计算pos
//	cv::Mat rotInv;
//	cv::invert(vrot, rotInv, cv::DECOMP_SVD);
//
//	pos = rotInv * tanV * -1;
//	cv::Point3d dstPos, srcPos;
//	dstPos.x = pos.at<double>(0, 0);
//	dstPos.y = pos.at<double>(1, 0);
//	dstPos.z = pos.at<double>(2, 0);
//	srcPos.x = pos.at<double>(3, 0);
//	srcPos.y = pos.at<double>(4, 0);
//	srcPos.z = pos.at<double>(5, 0);
//
//	cv::Mat sqrErrors = vrot * pos + tanV;
//	cv::Mat sqrErrorSum = sqrErrors.t() * sqrErrors;
//	double sqrtError = sqrt(sqrErrorSum.at<double>(0, 0) / sz);
//
//	QFile file("ProbePosition.txt");
//	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
//		return;
//	}
//	QTextStream stream(&file);
//	QDateTime curTime = QDateTime::currentDateTime();
//	stream << "Create time: " << curTime.toString("yyyy-MM-dd hh:mm:ss") << "\n";
//	stream << "Probe position in tool coordinate: " << dstPos.x << ", " << dstPos.y << ", " <<"\n" ;