#ifndef __LOADCAPTUREUS_H__
#define __LOADCAPTUREUS_H__

#include <QObject>

#include <opencv2/opencv.hpp>
#include <QVector>

class LoadCaptureUSWorker : public QObject
{
	Q_OBJECT
public:
	LoadCaptureUSWorker();
	~LoadCaptureUSWorker();

	enum DisplayMode
	{
		Single, Double,
	};

	enum CaptureSize
	{
		Fit, Full,
	};
	void SetModel(int);
	void SetSize(int);
	void SetSleepMs(int);

signals:
	void sglFinished(); // Ω· ¯–≈∫≈
	void loadPerFrame(const QImage&);
	void capturePerFrame(const cv::Mat&);
public slots:
	void start();
	void stop();

private:
	bool m_IsStart;
	QVector<cv::Rect> m_Rois;  // single or double
	QVector<cv::Rect> m_BlackPatchRois; // single or double
	DisplayMode m_Model;
	CaptureSize m_Size;
	int m_SleepMs;
};
#endif //__LOADCAPTUREUS_H__

