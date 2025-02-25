#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include <LoadCaptureUS.h>
#include <WriteCaptureUS.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void OnSelectBtnClicked();
	void OnStartBtnClicked();
	void OnStopBtnClicked();

	void OnModelComboBoxChanged(int);
	void OnSizeComboBoxChanged(int);
	void ProcessCaptureState();

	void DisplayNewFrame(const QImage&);
	void UpdateDisplayInfo(int);

	////四元数和平移转矩阵
	//void MainWindow::PinErrorRMS();

private:
    Ui::MainWindow *ui;
	QThread *m_LoadFramesThread;
	QThread *m_WriteFramesThread;
	LoadCaptureUSWorker *m_LoadFramesWorker;
	WriteCaptureUSWorker *m_WriteFramesWorker;

	qint64 m_StartTimePoint;
};

#endif // MAINWINDOW_H
