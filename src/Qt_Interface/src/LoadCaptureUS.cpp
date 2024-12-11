#include <LoadCaptureUS.h>
#include <QImage>
#include <QThread>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//#include "LoadCaptureUS.moc"

LoadCaptureUSWorker::LoadCaptureUSWorker()
{
	m_IsStart = false;
	m_Model = DisplayMode::Single;
	m_Size = CaptureSize::Fit;
	m_SleepMs = 30;
	m_Rois << cv::Rect(451, 180, 865, 674) << cv::Rect(333, 183, 550, 633);
	m_BlackPatchRois << cv::Rect(245, 3, 40, 26) << cv::Rect(91, 0, 30, 25);
	// above args are fixed on mindray resona7 sc5-1u.
}

LoadCaptureUSWorker::~LoadCaptureUSWorker()
{
}

void LoadCaptureUSWorker::SetSleepMs(int ms = 30)
{
	m_SleepMs = ms;
}

void LoadCaptureUSWorker::SetSize(int index = 0)
{
	m_Size = CaptureSize(index);
}

void LoadCaptureUSWorker::SetModel(int index=0)
{
	m_Model = DisplayMode(index);
}

void LoadCaptureUSWorker::start()
{
	m_IsStart = true;
	cv::VideoCapture capture;
	capture.open(1);
	if (!capture.isOpened())
	{
		std::cout << "video 1 not open." << std::endl << "try to open vedeo 0" <<  std::endl;
		capture.open(0);
		if (!capture.isOpened())
		{
			std::cout << "video 0 not open." << std::endl;
			capture.release();
			emit sglFinished();
			return;
		}
		else
		{
			std::cout << "video 0 is opended." << std::endl;
		}
	}
	else
	{
		std::cout << "video 1 is opened." << std::endl;
	}
	capture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
	float width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
	float height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
	std::cout << "Frame size : " << width << " x " << height << std::endl;
	if (width != 1920 || height != 1080)
	{
		std::cout<<"resolution is wrong!"<<std::endl;
		capture.release();
		emit sglFinished();
		return;
	}
	QImage img;
	cv::Mat frame;
	cv::Mat grayFrame;
	while (m_IsStart)
	{
		//¶ÁÈ¡µ±Ç°Ö¡
		capture >> frame;
		if (m_Size == CaptureSize::Fit)
		{
			frame = frame(m_Rois.at(m_Model));
		}


		if (!frame.empty())
		{
			if (frame.channels() == 3)
			{
				cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
			}
			else if (frame.channels() == 1)
			{
				grayFrame = frame;
			}
			else
			{
				std::cout<<"capture channel error"<<std::endl;
				capture.release();
				emit sglFinished();
				return;
			}
			double minVal;
			double maxVal;
			cv::Point minLoc;
			cv::Point maxLoc;
			minMaxLoc(grayFrame, &minVal, &maxVal, &minLoc, &maxLoc);
			if (maxVal < 1)
			{
				std::cout << "skip black." << std::endl;
				continue;
			}
			if (m_Size == CaptureSize::Fit)
			{
				grayFrame(m_BlackPatchRois.at(m_Model)) = 0;
			}
			emit capturePerFrame(grayFrame);
			img = QImage((const uchar*)(grayFrame.data), grayFrame.cols, grayFrame.rows, grayFrame.cols*grayFrame.channels(), QImage::Format_Indexed8);
			emit loadPerFrame(img);
		}
		else
		{
			capture.release();
			emit sglFinished();
			return ;
		}
		QThread::msleep(m_SleepMs);
	}
	capture.release();
	emit sglFinished();
}

void LoadCaptureUSWorker::stop()
{
	this->m_IsStart = false;
}