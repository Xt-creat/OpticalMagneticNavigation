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

	// Configure data labels.
	auto setupDataLabel = [](QLabel* label) {
		label->setMinimumHeight(100);
		label->setWordWrap(true);
		label->setTextInteractionFlags(Qt::TextSelectableByMouse);
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
			"}");
	};

	setupDataLabel(ui->m_OdataLabel);
	ui->m_OdataLabel->setMinimumHeight(200);
	
	setupDataLabel(ui->m_MdataLabel);
	setupDataLabel(ui->m_FusionLabel);

	ui->m_systemstatus->setMaximumHeight(40);
	ui->m_systemstatus->setStyleSheet(
		"QLabel {"
		"  font-weight: bold;"
		"  font-size: 18px;"
		"  padding: 5px;"
		"  color: #495057;"
		"}"
	);

	connect(ui->m_SelectSavePathBtn, &QPushButton::clicked, this, &MainWindow::OnSelectBtnClicked);
	m_NDIWorker = new NDIWorker;
	ui->m_LinkBtn->setCheckable(false);
	connect(ui->m_LinkBtn, &QPushButton::clicked, m_NDIWorker, &NDIWorker::LinkNDI);
	connect(m_NDIWorker, &NDIWorker::linkStatusChanged, this, &MainWindow::updateSystemStatus);
	connect(ui->m_CalibrateBtn, &QPushButton::clicked, this, &MainWindow::OnStartBtnClicked);
	
	ui->m_TrackingBtn->setCheckable(true);
	connect(ui->m_TrackingBtn, &QPushButton::clicked, this, &MainWindow::OnStartTracking);
	
	connect(ui->m_StopBtn, &QPushButton::clicked, this, &MainWindow::OnStopTracking);
	ui->m_StopBtn->setText("Record Data");

	connect(ui->m_DisplayBtn, &QPushButton::clicked, this, &MainWindow::onDisplayBtnClicked);

	ui->m_DisplaySavePathLineEdit->setEnabled(false);

	// Sampling frequency config: 10-30 Hz, default 10 Hz
	ui->m_SleepSpinBox->setRange(10, 30);
	ui->m_SleepSpinBox->setValue(10);
	ui->m_SleepSpinBox->setSuffix(" Hz");
	ui->m_SleepSpinBox->setToolTip("Sampling frequency (10-30 Hz)");
	
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
		QMessageBox::warning(this, "Warning", "Device is not connected. Please connect first.");
		return;
	}

	// Calibration requires live tracking data.
	bool isTracking = (thread1 && thread1->isRunning()) || (thread2 && thread2->isRunning());
	if (!isTracking) {
		QMessageBox::warning(this, "Warning", "Please start tracking before calibration data acquisition.");
		return;
	}

	if (!m_calibrationDialog) {
		m_calibrationDialog = new CalibrationDialog(m_savePath, this);
		m_calibrationDialog->setAttribute(Qt::WA_DeleteOnClose);
		connect(m_calibrationDialog, &QObject::destroyed, [this]() {
			m_calibrationDialog = nullptr;
		});
		m_calibrationDialog->show();
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
		ui->m_systemstatus->setText("Device not connected");
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

	const int sampleFreqHz = ui->m_SleepSpinBox->value();
	const int updateIntervalMs = 1000 / sampleFreqHz;
	deviceReader1 = new DeviceReader(1, updateIntervalMs);
	deviceReader2 = new DeviceReader(2, updateIntervalMs);
	
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

	ui->m_systemstatus->setText("Tracking...");
	ui->m_systemstatus->setStyleSheet("color: #28a745; font-weight: bold; font-size: 18px;");
	ui->m_TrackingBtn->setText("Stop Tracking");
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

	ui->m_systemstatus->setText("Tracking stopped");
	ui->m_systemstatus->setStyleSheet("color: #fd7e14; font-weight: bold; font-size: 18px;");
	ui->m_TrackingBtn->setText("Start Tracking");
}

void MainWindow::OnStopTracking() {
	// This slot handles the data recording button.
	if (!((thread1 && thread1->isRunning()) || (thread2 && thread2->isRunning()))) {
		QMessageBox::information(this, "Info", "Please start tracking before recording data.");
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
		.arg("Optical Data");
	QString fusionHeader = QString("<div style='font-size: 18px; font-weight: bold; color: #495057; margin-bottom: 10px;'>%1</div>")
		.arg("Fusion Data");

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

		// Update visualization if the first tool is valid.
		if (i == 0 && transform.toolHandle == 1 && !transform.isMissing()) {
			F_text = fusionHeader + O_text.mid(header.length()) + "</div>";

		if (displayWidget) {
			vtkSmartPointer<vtkTransform> ndiTransform = vtkSmartPointer<vtkTransform>::New();
			
			// 1) World axis alignment
			ndiTransform->RotateX(-90.0);

			// 2) Coordinate mapping
			ndiTransform->RotateZ(-90.0);

			// 3) Apply raw NDI data
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

	// Try fusion logic when first tool is missing.
	if (tools[0].transform.isMissing()) {
		// Safety check before fusion.
		if (M_data.empty() || O_data.size() < 2) {
			F_text = fusionHeader + "<i style='color: #6c757d; font-size:14px;'>Fusion: Waiting for more tools...</i>";
		}
		else {
			cv::Mat tool1_2EMsensor;
			loadMatFromTxt("tool1_2EMsensor.txt", tool1_2EMsensor);

			cv::Mat EM_2tool2;
			loadMatFromTxt("EM_2tool2.txt", EM_2tool2);

			if (tool1_2EMsensor.empty() || EM_2tool2.empty()) {
				F_text = "<b style='color: #dc3545;'>Fusion: Calibration files error!</b>";
			}
			else {
				const ToolData& data_Mtool = M_data[0];
				const Transform& transform_Mtool = data_Mtool.transform;
				const ToolData& data_Otool2 = O_data[1];
				const Transform& transform_Otool2 = data_Otool2.transform;

				cv::Mat EMsensor_2Aurora = transformToMatrix(transform_Mtool);
				cv::Mat tool2_2Vega = transformToMatrix(transform_Otool2);

				// Validate generated transform matrices.
				if (EMsensor_2Aurora.empty() || tool2_2Vega.empty()) {
					F_text = "<b style='color: #dc3545;'>Fusion: Transform matrix error!</b>";
				}
				else {
					cv::Mat output_tool1_2Vega = tool2_2Vega * EM_2tool2 * EMsensor_2Aurora * tool1_2EMsensor;

					double q0, qx, qy, qz, tx, ty, tz;
					matrixToTransform(output_tool1_2Vega, q0, qx, qy, qz, tx, ty, tz);

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

						// 1) World alignment
						ndiTransform->RotateX(-90.0);

						// 2) Coordinate mapping
						ndiTransform->RotateZ(-90.0);

						// 3) Raw data
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

	// Apply formatted HTML to labels.
	ui->m_OdataLabel->setText(O_text);
	ui->m_FusionLabel->setText(F_text);

	// Append to log file.
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
		.arg("EM Data");

	QString text = header + "<div style='font-family: Segoe UI, sans-serif;'>";
	for (size_t i = 0; i < tools.size(); ++i) {
		const ToolData& data = tools[i];
		const Transform& transform = data.transform;
		QString statusColor = transform.isMissing() ? "#dc3545" : "#17a2b8";
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
		displayWidget = new DisplayWidget(nullptr);
		displayWidget->setAttribute(Qt::WA_DeleteOnClose);

		// Clear pointer after window is closed.
		connect(displayWidget, &QObject::destroyed, [this]() {
			displayWidget = nullptr;
			});
	}

	displayWidget->show();
	displayWidget->raise();
	displayWidget->activateWindow();

	 // Use relative path and check file existence.
	 QString modelPath = QCoreApplication::applicationDirPath() + "/tools/ToolScribing2.stl";
	 if (!QFile::exists(modelPath)) {
		 // Fallback to hard-coded path if needed.
		 modelPath = "D:/Optomagnetic-tracking/OpticalMagneticNavigation/tools/ToolScribing2.stl";
	 }
	 
	displayWidget->loadSTLModel(modelPath.toStdString());

	// If tracking is active, start rendering updates for new window.
	if ((thread1 && thread1->isRunning()) || (thread2 && thread2->isRunning())) {
		displayWidget->startRealtimeAnimation();
	}

	// Do not auto-start animation unless tracking starts.
}



void MainWindow::OnSelectBtnClicked()
{
	QString selectedPath = QFileDialog::getExistingDirectory(this, tr("Select data save path"),
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
    QString fullPath = m_savePath + "/" + QString::fromStdString(filename);
    std::ifstream file(fullPath.toStdString());
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << fullPath.toStdString() << std::endl;
        mat = cv::Mat();
        return;
    }

    std::vector<std::vector<double>> tempData;

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

    // Check data validity.
    if (tempData.empty() || tempData[0].empty()) {
        std::cerr << "Empty file or invalid format: " << fullPath.toStdString() << std::endl;
        mat = cv::Mat();
        file.close();
        return;
    }

    // Convert parsed data to cv::Mat.
    int rows = (int)tempData.size();
    int cols = (int)tempData[0].size();
    mat = cv::Mat(rows, cols, CV_64F);

    for (int i = 0; i < rows; ++i) {
        // Guard against ragged rows.
        for (int j = 0; j < (int)tempData[i].size() && j < cols; ++j) {
            mat.at<double>(i, j) = tempData[i][j];
        }
    }

    file.close();
    //std::cout << "Loaded data to cv::Mat." << std::endl;
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



// Convert rotation matrix to quaternion.
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

// Extract quaternion and translation from transform matrix.
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
//		msgBox.setText("Too few samples");
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
//	// Translation matrix
//	cv::vconcat(vtan, tanV);
//
//	// Compute pos
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