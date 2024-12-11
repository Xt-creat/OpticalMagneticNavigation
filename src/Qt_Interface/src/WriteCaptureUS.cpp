#include <WriteCaptureUS.h>
#include <QImage>
#include <QThread>
#include <QLatin1Char>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <WriteCaptureUS.h>

#include <CombinedApi.h>
#include<fstream>

//#include "WriteCaptureUS.moc"


static CombinedApi capi = CombinedApi();
static std::string cipher = "";

WriteCaptureUSWorker::WriteCaptureUSWorker()
{
	m_NumberOfImages = 0;
	m_LinkNDI = false;
}

WriteCaptureUSWorker::~WriteCaptureUSWorker()
{
	if (m_LinkNDI)
	{
		onErrorPrintDebugMessage("capi.stopTracking()", capi.stopTracking());
	}
}

void WriteCaptureUSWorker::SetWritePath(const QString& path)
{
	m_Path = path;
	m_NumberOfImages = 0;
}

void WriteCaptureUSWorker::LinkNDI(bool checked)
{
	if (checked)
	{
		if (capi.connect("COM4", Protocol::TCP, cipher) != 0)
		{
			std::cout << "Connection Failed!" << std::endl;
			std::cout << "Please check port number or power!";
			//need to exit safely.
			return;
		}

		onErrorPrintDebugMessage("capi.initialize()", capi.initialize());
		// Initialize and enable tools
		std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
		for (int i = 0; i < portHandles.size(); i++)
		{
			onErrorPrintDebugMessage("capi.portHandleInitialize()", capi.portHandleInitialize(portHandles[i].getPortHandle()));
			onErrorPrintDebugMessage("capi.portHandleEnable()", capi.portHandleEnable(portHandles[i].getPortHandle()));
		}

		onErrorPrintDebugMessage("capi.startTracking()", capi.startTracking());
		m_LinkNDI = true;
	}
	else
	{
		onErrorPrintDebugMessage("capi.stopTracking()", capi.stopTracking());
		m_LinkNDI = false;
	}
}

void WriteCaptureUSWorker::Write(const cv::Mat& frame)
{
	QString name = QString("%1.png").arg(m_NumberOfImages, 6, 10, QLatin1Char('0'));
	cv::imwrite((m_Path + "/" + name).toLocal8Bit().toStdString(), frame);

	if (m_LinkNDI)
	{
		std::ofstream outfile((m_Path + "/tracking.txt").toLocal8Bit().toStdString(),std::ios::app);
		if (!outfile.is_open())
		{
			std::cout << "can not open tracking file to write" << std::endl;
			return;
		}

		QVector<double> trackResult = GetPosition();
		for (double value : trackResult)
		{
			outfile << value << " ";
		}
		outfile << std::endl;
		outfile.close();
	}

	++m_NumberOfImages;
	emit WriteOneFrameFinished(m_NumberOfImages);
}

QVector<double> WriteCaptureUSWorker::GetPosition()
{
	QVector<double> result;
	std::vector<ToolData> trackValue = capi.getTrackingDataBX();
	assert(trackValue.size() == 1);
	if (!trackValue[0].transform.isMissing())
	{
		float w = trackValue[0].transform.q0;
		float x = trackValue[0].transform.qx;
		float y = trackValue[0].transform.qy;
		float z = trackValue[0].transform.qz;

		float xx = x * x;
		float yy = y * y;
		float zz = z * z;
		float xy = x * y;
		float wz = w * z;
		float wy = w * y;
		float xz = x * z;
		float yz = y * z;
		float wx = w * x;

		result << 1.0f - 2 * (yy + zz) << 2 * (xy + wz) << 2 * (xz - wy);
		result << 2 * (xy - wz) << 1.0f - 2 * (xx + zz) << 2 * (yz + wx);
		result << 2 * (xz + wy) << 2 * (yz - wx) << 1.0f - 2 * (xx + yy);
		result << trackValue[0].transform.tx << trackValue[0].transform.ty << trackValue[0].transform.tz;
	}
	return result;
}

void WriteCaptureUSWorker::onErrorPrintDebugMessage(std::string methodName, int errorCode)
{
	if (errorCode < 0)
	{
		std::cout << methodName << " failed: " << capi.errorToString(errorCode) << std::endl;
	}
}