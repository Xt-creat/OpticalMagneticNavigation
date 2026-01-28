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

	// 设置标签的基本属性
	auto setupDataLabel = [](QLabel* label) {
		label->setMinimumHeight(100);
		label->setWordWrap(true);
		label->setTextInteractionFlags(Qt::TextSelectableByMouse); // 允许选择/复制文本
		label->setAlignment(Qt::AlignTop | Qt::AlignLeft);
		label->setStyleSheet(
			"QLabel {"
			"  background-color: #f8f9fa;"
			"  border: 1px solid #dee2e6;"
			"  border-radius: 6px;"
			"  padding: 8px;"
			"  font-family: 'Segoe UI', 'Consolas', monospace;"
			"  font-size: 18px;"
			"  font-weight: bold;"
			"  color: #495057;"
			"  line-height: 1.2;"
			"}"
		);
	};

	setupDataLabel(ui->m_OdataLabel);
	ui->m_OdataLabel->setMinimumHeight(200); // 调大光学数据的最小高度
	
	setupDataLabel(ui->m_MdataLabel);
	setupDataLabel(ui->m_FusionLabel);

	ui->m_systemstatus->setMaximumHeight(40); // 限制系统状态区域的纵向尺寸
	ui->m_systemstatus->setStyleSheet(
		"QLabel {"
		"  font-weight: bold;"
		"  font-size: 18px;"
		"  padding: 5px;"
		"  color: #495057;"
		"}"
	);

	connect(ui->m_SelectSavePathBtn, &QPushButton::clicked, this, &MainWindow::OnSelectBtnClicked);     //选择数据存储路径
	m_NDIWorker = new NDIWorker;
	ui->m_LinkBtn->setCheckable(false);
	connect(ui->m_LinkBtn, &QPushButton::clicked, m_NDIWorker, &NDIWorker::LinkNDI);                     //连接设备
	connect(m_NDIWorker, &NDIWorker::linkStatusChanged, this, &MainWindow::updateSystemStatus);
	connect(ui->m_CalibrateBtn, &QPushButton::clicked, this, &MainWindow::OnStartBtnClicked);               //系统标定
	
	ui->m_TrackingBtn->setCheckable(true);
	connect(ui->m_TrackingBtn, &QPushButton::clicked, this, &MainWindow::OnStartTracking);                 //开始/停止跟踪
	
	connect(ui->m_StopBtn, &QPushButton::clicked, this, &MainWindow::OnStopTracking);             //数据记录
	ui->m_StopBtn->setText(QString::fromLocal8Bit("数据记录"));

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
	if (!m_connected) {
		QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("设备未连接，请先连接设备！"));
		return;
	}

	// 检查是否正在跟踪，标定需要实时数据
	bool isTracking = (thread1 && thread1->isRunning()) || (thread2 && thread2->isRunning());
	if (!isTracking) {
		QMessageBox::warning(this, QString::fromLocal8Bit("警告"), QString::fromLocal8Bit("请先开启“跟踪”模式后再进行标定数据采集！"));
		return;
	}

	if (!m_calibrationDialog) {
		m_calibrationDialog = new CalibrationDialog(m_savePath, this);
		m_calibrationDialog->setAttribute(Qt::WA_DeleteOnClose);
		connect(m_calibrationDialog, &QObject::destroyed, [this]() {
			m_calibrationDialog = nullptr;
		});
		m_calibrationDialog->show(); // 非模态显示，这样主界面依然可以操作和刷新
	}
	m_calibrationDialog->raise();
	m_calibrationDialog->activateWindow();
}



void MainWindow::updateSystemStatus(bool isConnected) {
	if (isConnected) {
		m_connected = isConnected;
		ui->m_systemstatus->setText("Device connection successful");
		ui->m_systemstatus->setStyleSheet("color: #28a745; font-weight: bold; font-size: 18px;"); 
	}
	else {
		ui->m_systemstatus->setText("Device not connected");
		ui->m_systemstatus->setStyleSheet("color: #dc3545; font-weight: bold; font-size: 18px;");
	}

}


void MainWindow::OnStartTracking()
{
	if (!m_connected) {
		ui->m_systemstatus->setText(QString::fromLocal8Bit("设备未连接"));
		ui->m_systemstatus->setStyleSheet("color: #dc3545; font-weight: bold; font-size: 18px;");
		ui->m_TrackingBtn->setChecked(false);
		return;
	}

	bool isTracking = ui->m_TrackingBtn->isChecked();

	if (isTracking) {
		startTrackingThreads();
	} else {
		stopTrackingThreads();
	}
}

void MainWindow::startTrackingThreads()
{
	O_capi.startTracking();
	M_capi.startTracking();

	if (displayWidget) {
		displayWidget->startRealtimeAnimation();
	}

	deviceReader1 = new DeviceReader(1, 100); 
	deviceReader2 = new DeviceReader(2, 100); 
	
	thread1 = new QThread(this);
	deviceReader1->moveToThread(thread1);
	connect(thread1, &QThread::started, deviceReader1, &DeviceReader::startReading);
	connect(deviceReader1, &DeviceReader::newODataAvailable, this, &MainWindow::updateODataLabel);

	thread2 = new QThread(this);
	deviceReader2->moveToThread(thread2);
	connect(thread2, &QThread::started, deviceReader2, &DeviceReader::startReading);
	connect(deviceReader2, &DeviceReader::newMDataAvailable, this, &MainWindow::updateMDataLabel);

	thread1->start();
	thread2->start();

	ui->m_systemstatus->setText(QString::fromLocal8Bit("跟踪中..."));
	ui->m_systemstatus->setStyleSheet("color: #28a745; font-weight: bold; font-size: 18px;");
	ui->m_TrackingBtn->setText(QString::fromLocal8Bit("停止跟踪"));
}

void MainWindow::stopTrackingThreads()
{
	if (thread1) {
		disconnect(deviceReader1, &DeviceReader::newODataAvailable, this, &MainWindow::updateODataLabel);
		QMetaObject::invokeMethod(deviceReader1, "stop", Qt::QueuedConnection);
		thread1->quit();
		if (!thread1->wait(1000)) {
			thread1->terminate();
			thread1->wait();
		}
		deviceReader1->deleteLater();
		deviceReader1 = nullptr;
		thread1->deleteLater();
		thread1 = nullptr;
	}

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

	O_capi.stopTracking();
	M_capi.stopTracking();

	ui->m_systemstatus->setText(QString::fromLocal8Bit("跟踪已停止"));
	ui->m_systemstatus->setStyleSheet("color: #fd7e14; font-weight: bold; font-size: 18px;");
	ui->m_TrackingBtn->setText(QString::fromLocal8Bit("开始跟踪"));
}

void MainWindow::OnStopTracking() {
	// 此函数现在对应 UI 上的“数据记录”按钮 (m_StopBtn)
	if (!((thread1 && thread1->isRunning()) || (thread2 && thread2->isRunning()))) {
		QMessageBox::information(this, QString::fromLocal8Bit("提示"), QString::fromLocal8Bit("请先开启“跟踪”模式后再进行数据记录！"));
		return;
	}

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

void MainWindow::updateODataLabel(const std::vector<ToolData>& tools)
{
	O_data = tools;

	if (m_recordingDialog) {
		m_recordingDialog->addOData(tools);
	}

	if (m_calibrationDialog) {
		m_calibrationDialog->addOData(tools);
	}

	QString header = QString("<div style='font-size: 18px; font-weight: bold; color: #495057; margin-bottom: 10px;'>%1</div>")
		.arg(QString::fromLocal8Bit("光学数据"));
	QString fusionHeader = QString("<div style='font-size: 18px; font-weight: bold; color: #495057; margin-bottom: 10px;'>%1</div>")
		.arg(QString::fromLocal8Bit("融合数据"));

	if (tools.empty()) {
		ui->m_OdataLabel->setText(header + "<b style='color: #dc3545;'>O: No Tools Detected</b>");
		ui->m_FusionLabel->setText(fusionHeader + "<i style='color: #6c757d; font-size:14px;'>Fusion: Waiting for tools...</i>");
		return;
	}

	QString O_text = header + "<div style='font-family: Segoe UI, sans-serif;'>";
	QString F_text = fusionHeader;
	
	for (size_t i = 0; i < tools.size() && i < 3; ++i) {
		const ToolData& data = tools[i];
		const Transform& transform = data.transform;
		QString statusColor = transform.isMissing() ? "#dc3545" : "#28a745";
		QString statusText = transform.isMissing() ? "Missing" : "Normal";

		QString dataText = "";
		if (!transform.isMissing()) {
			dataText = QString("X:%1, Y:%2, Z:%3 mm | Rot: q0:%4, qx:%5, qy:%6, qz:%7")
				.arg(transform.tx, 7, 'f', 2)
				.arg(transform.ty, 7, 'f', 2)
				.arg(transform.tz, 7, 'f', 2)
				.arg(transform.q0, 7, 'f', 4)
				.arg(transform.qx, 7, 'f', 4)
				.arg(transform.qy, 7, 'f', 4)
				.arg(transform.qz, 7, 'f', 4);
		}

		O_text += QString(
			"<div style='margin-bottom: 15px; border-left: 5px solid %1; padding-left: 8px;'>"
			"<b>O%2</b> <span style='color:%1; font-weight:bold;'>[%3]</span> "
			"<span style='font-family:Consolas; font-size:16px; font-weight:normal;'>%4</span>"
			"</div>"
		)
		.arg(statusColor)
		.arg(i + 1)
		.arg(statusText)
		.arg(dataText);

		// 只要第一个工具(通常是标准针)在线，就更新可视化
		if (i == 0 && transform.toolHandle == 1 && !transform.isMissing()) {
			F_text = fusionHeader + O_text.mid(header.length()) + "</div>";

		if (displayWidget) {
			vtkSmartPointer<vtkTransform> ndiTransform = vtkSmartPointer<vtkTransform>::New();
			
			// 1. 世界坐标系对齐 (将 VTK 空间旋转到符合 NDI 的习惯)
			ndiTransform->RotateX(-90.0);

			// 2. 坐标系映射 (核心：通过旋转实现 ty, -tx, tz 的映射效果，并同步旋转轴)
			ndiTransform->RotateZ(-90.0);

			// 3. 应用原始 NDI 数据 (不进行手动交换)
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
	}
	O_text += "</div>";

	// 如果第一个工具丢失，尝试 Fusion 逻辑
	if (tools[0].transform.isMissing()) {
		// 安全检查：确保 M_data 已经初始化，且 O_data 至少包含三个工具（需使用 Otool3 作为参考）
		if (M_data.empty() || O_data.size() < 3) {
			F_text = fusionHeader + "<i style='color: #6c757d; font-size:14px;'>Fusion: Waiting for more tools...</i>";
		}
		else {
			cv::Mat Marker1_2EMsensor;
			loadMatFromTxt("Marker1_2EMsensor.txt", Marker1_2EMsensor);
			cv::Mat EM_2Marker2;
			loadMatFromTxt("EM_2Marker2.txt", EM_2Marker2);

			if (Marker1_2EMsensor.empty() || EM_2Marker2.empty()) {
				F_text = "<b style='color: #dc3545;'>Fusion: Calibration files error!</b>";
			}
			else {
				const ToolData& data_Mtool = M_data[0];
				const Transform& transform_Mtool = data_Mtool.transform;      //包含实时EMsensor_2Aurora (q0,qx,qy,qz,tx,ty,tz)
				const ToolData& data_Otool3 = O_data[2];
				const Transform& transform_Otool3 = data_Otool3.transform;    //包含实时Marker2_2Vega (q0,qx,qy,qz,tx,ty,tz)

				cv::Mat EMsensor_2Aurora = transformToMatrix(transform_Mtool);
				cv::Mat Marker2_2Vega = transformToMatrix(transform_Otool3);

				// 检查生成的变换矩阵是否有效
				if (EMsensor_2Aurora.empty() || Marker2_2Vega.empty()) {
					F_text = "<b style='color: #dc3545;'>Fusion: Transform matrix error!</b>";
				}
				else {
					cv::Mat output_Marker1_2Vega = Marker2_2Vega * EM_2Marker2 * EMsensor_2Aurora * Marker1_2EMsensor;

					double q0, qx, qy, qz, tx, ty, tz;
					matrixToTransform(output_Marker1_2Vega, q0, qx, qy, qz, tx, ty, tz);

					F_text = fusionHeader + QString(
						"<div style='border-left: 5px solid #007bff; padding-left: 8px;'>"
						"<b style='color:#007bff;'>FUSION</b> "
						"<span style='font-family:Consolas; font-size:16px; font-weight:normal;'>"
						"X:%1, Y:%2, Z:%3 mm | Rot: q0:%4, qx:%5, qy:%6, qz:%7"
						"</span></div>")
						.arg(tx, 7, 'f', 2)
						.arg(ty, 7, 'f', 2)
						.arg(tz, 7, 'f', 2)
						.arg(q0, 7, 'f', 4)
						.arg(qx, 7, 'f', 4)
						.arg(qy, 7, 'f', 4)
						.arg(qz, 7, 'f', 4);

					if (displayWidget) {
						vtkSmartPointer<vtkTransform> ndiTransform = vtkSmartPointer<vtkTransform>::New();

						// 1. 世界对齐
						ndiTransform->RotateX(-90.0);

						// 2. 坐标映射
						ndiTransform->RotateZ(-90.0);

						// 3. 原始数据
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

	// 使用 setText 设置 HTML 文本
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

	if (m_calibrationDialog) {
		m_calibrationDialog->addMData(tools);
	}

	QString header = QString("<div style='font-size: 18px; font-weight: bold; color: #495057; margin-bottom: 10px;'>%1</div>")
		.arg(QString::fromLocal8Bit("电磁数据"));

	QString text = header + "<div style='font-family: Segoe UI, sans-serif;'>";
	for (size_t i = 0; i < tools.size(); ++i) {
		const ToolData& data = tools[i];
		const Transform& transform = data.transform;
		QString statusColor = transform.isMissing() ? "#dc3545" : "#17a2b8"; // 电磁用青蓝色区分
		QString statusText = transform.isMissing() ? "Missing" : "Normal";

		QString dataText = "";
		if (!transform.isMissing()) {
			dataText = QString("X:%1, Y:%2, Z:%3 mm | Rot: q0:%4, qx:%5, qy:%6, qz:%7")
				.arg(transform.tx, 7, 'f', 2)
				.arg(transform.ty, 7, 'f', 2)
				.arg(transform.tz, 7, 'f', 2)
				.arg(transform.q0, 7, 'f', 4)
				.arg(transform.qx, 7, 'f', 4)
				.arg(transform.qy, 7, 'f', 4)
				.arg(transform.qz, 7, 'f', 4);
		}

		text += QString(
			"<div style='margin-bottom: 15px; border-left: 5px solid %1; padding-left: 8px;'>"
			"<b>M%2</b> <span style='color:%1; font-weight:bold;'>[%3]</span> "
			"<span style='font-family:Consolas; font-size:16px; font-weight:normal;'>%4</span>"
			"</div>"
		)
		.arg(statusColor)
		.arg(i + 1)
		.arg(statusText)
		.arg(dataText);
	}
	text += "</div>";
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