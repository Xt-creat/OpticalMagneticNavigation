#include <iostream>
#include <QPushButton>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QString>
#include <QThread>
#include <QDateTime>
#include <QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "mainwindow.moc"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
	m_StartTimePoint(0.0)
{
    ui->setupUi(this);
	connect(ui->m_SelectSavePathBtn, &QPushButton::clicked, this, &MainWindow::OnSelectBtnClicked);
	connect(ui->m_StartBtn, &QPushButton::clicked, this, &MainWindow::OnStartBtnClicked);
	connect(ui->m_StopBtn, &QPushButton::clicked, this, &MainWindow::OnStopBtnClicked);

	ui->m_DisplaySavePathLineEdit->setEnabled(false);
	ui->m_DisplayCaptureLabel->setScaledContents(true);
	ui->m_SleepSpinBox->setValue(30);

	m_LoadFramesThread = new QThread;
	m_LoadFramesWorker = new LoadCaptureUSWorker;
	m_LoadFramesWorker->moveToThread(m_LoadFramesThread);
	connect(m_LoadFramesThread, &QThread::started, m_LoadFramesWorker, &LoadCaptureUSWorker::start);
	connect(m_LoadFramesWorker, &LoadCaptureUSWorker::sglFinished, this, &MainWindow::ProcessCaptureState);
	connect(m_LoadFramesWorker, &LoadCaptureUSWorker::loadPerFrame, this, &MainWindow::DisplayNewFrame);

	void(QComboBox::*comboBoxSignal)(int) = &QComboBox::currentIndexChanged;

	connect(ui->m_DisplayModeComboBox, comboBoxSignal, this, &MainWindow::OnModelComboBoxChanged);
	connect(ui->m_SizeComboBox, comboBoxSignal, this, &MainWindow::OnSizeComboBoxChanged);

	m_WriteFramesThread = new QThread;
	m_WriteFramesWorker = new WriteCaptureUSWorker;
	m_WriteFramesWorker->moveToThread(m_WriteFramesThread);
	connect(ui->m_LinkBtn, &QPushButton::toggled, m_WriteFramesWorker, &WriteCaptureUSWorker::LinkNDI);
	//connect(m_WriteFramesThread, &QThread::started, m_WriteFramesWorker, &WriteCaptureUSWorker::start);
	qRegisterMetaType< cv::Mat >("cv::Mat");
	connect(m_LoadFramesWorker, &LoadCaptureUSWorker::capturePerFrame, m_WriteFramesWorker, &WriteCaptureUSWorker::Write);
	connect(m_WriteFramesWorker, &WriteCaptureUSWorker::WriteOneFrameFinished, this, &MainWindow::UpdateDisplayInfo);
	m_WriteFramesThread->start();
}

MainWindow::~MainWindow()
{
    delete ui;
	if (!m_LoadFramesThread->isFinished())
	{
		m_LoadFramesThread->terminate();
		m_LoadFramesThread->wait();
	}
	delete m_LoadFramesWorker;
	delete m_LoadFramesThread;
	if (!m_WriteFramesThread->isFinished())
	{
		m_WriteFramesThread->terminate();
		m_WriteFramesThread->wait();
	}
	delete m_WriteFramesWorker;
	delete m_WriteFramesThread;
}

void MainWindow::OnStartBtnClicked()
{
	if (ui->m_DisplaySavePathLineEdit->text().isEmpty())
	{
		return;
	}
	if (m_LoadFramesThread->isRunning())
	{
		return;
	}
	QString dirName = ui->m_DisplaySavePathLineEdit->text() + "/" + QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
	QDir dir;
	bool mkdirOk = false;
	mkdirOk = dir.mkdir(dirName);
	if (!mkdirOk)
	{
		std::cout<<"mkdir error"<<std::endl;
		return;
	}
	m_WriteFramesWorker->SetWritePath(dirName);
	m_LoadFramesWorker->SetSleepMs(ui->m_SleepSpinBox->value());

	m_LoadFramesThread->start();
	ui->m_DisplayCaptureTimeLabel->setText(QString::fromLocal8Bit("用时%1秒").arg(0));
	m_StartTimePoint = QDateTime::currentMSecsSinceEpoch();
}

void MainWindow::OnStopBtnClicked()
{
	m_LoadFramesWorker->stop();
}

void MainWindow::OnSizeComboBoxChanged(int index = 0)
{
	m_LoadFramesWorker->SetSize(index);
}

void MainWindow::OnModelComboBoxChanged(int index=0)
{
	m_LoadFramesWorker->SetModel(index);
}

void MainWindow::ProcessCaptureState()
{
	m_LoadFramesThread->terminate();
	m_LoadFramesThread->wait();
}

void MainWindow::DisplayNewFrame(const QImage &frame)
{
	QPixmap img = QPixmap::fromImage(frame);
	ui->m_DisplayCaptureLabel->setPixmap(QPixmap::fromImage(frame));
}

void MainWindow::UpdateDisplayInfo(int frames)
{
	ui->m_DisplayCaptureFramesLabel->setText(QString::fromLocal8Bit("采集%1帧").arg(frames));
	qint64 time = QDateTime::currentMSecsSinceEpoch() - m_StartTimePoint;
	ui->m_DisplayCaptureTimeLabel->setText(QString::fromLocal8Bit("用时%1秒").arg(time/1000.0));
}

void MainWindow::OnSelectBtnClicked()
{
	QString savePath = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
		"E:/",QFileDialog::ShowDirsOnly| QFileDialog::DontResolveSymlinks);
	ui->m_DisplaySavePathLineEdit->setText(savePath);
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