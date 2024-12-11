#ifndef __WRITECAPTUREUS_H__
#define __WRITECAPTUREUS_H__

#include <QObject>
#include <opencv2/opencv.hpp>

class WriteCaptureUSWorker : public QObject
{
	Q_OBJECT
public:
	WriteCaptureUSWorker();
	~WriteCaptureUSWorker();

	void SetWritePath(const QString&);
signals:
	void WriteOneFrameFinished(int);
public slots:
	void Write(const cv::Mat&);
	void LinkNDI(bool);

private:
	int m_NumberOfImages;
	QString m_Path;
	QVector<double> GetPosition();

	void onErrorPrintDebugMessage(std::string, int);

	bool m_LinkNDI;
};
#endif //__WRITECAPTUREUS_H__
