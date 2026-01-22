#ifdef _WIN32
#define ACCESS _access
#include <conio.h>   // for _kbhit()
#include <io.h>      // for _access_s()
#include <windows.h> // for Sleep(ms)
#else
#define ACCESS access
#include <unistd.h>     // for POSIX sleep(sec), and access()
#include <sys/ioctl.h>
#endif



#include "NDIWorker.h"
#include <CombinedApi.h>
#include<fstream>
#include<iostream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <thread>


static bool useEncryption = false;
static std::string cipher = "";

extern CombinedApi O_capi;
extern CombinedApi M_capi;

//CombinedApi O_capi = CombinedApi();
//CombinedApi M_capi = CombinedApi();

NDIWorker::NDIWorker()
{
	device1 = "P9-01077";
	otool1_rom = "Otool1.rom";
	otool2_rom = "Otool2.rom";
	otool3_rom = "Otool3.rom";
	device2 = "COM3";
	mtool_rom = "080061.rom";

	toolDefinitions1 = std::vector<std::string>();
	toolDefinitions1.push_back(otool1_rom);
	toolDefinitions1.push_back(otool2_rom);
	toolDefinitions1.push_back(otool3_rom);
	toolDefinitions2 = std::vector<std::string>();
	toolDefinitions2.push_back(mtool_rom);

}

NDIWorker::~NDIWorker()
{
	// Stop tracking (back to configuration mode)
	std::cout << std::endl << "Leaving tracking mode and returning to configuration mode..." << std::endl;
	onErrorPrintDebugMessage(O_capi, "capi1.stopTracking()", O_capi.stopTracking());
	onErrorPrintDebugMessage(M_capi, "capi2.stopTracking()", M_capi.stopTracking());
	std::cout << "CAPI demonstration complete. Press any key to exit." << std::endl;
}

void NDIWorker::LinkNDI(bool checked) {
	
	std::cout << "device1:"<< device1 << std::endl;
	//VEGA连接
	if (O_capi.connect(device1, useEncryption ? Protocol::SecureTCP : Protocol::TCP, cipher) != 0)
		//useEncryption == false,没有启用加密，启用普通 TCP连接协议
	{
		// Print the error and exit if we can't connect to a device
		std::cout << "Connection1 Failed!" << std::endl;
		std::cout << "Press Enter to continue...";
		//std::cin.ignore();
		return;
	}
	std::cout << "Device1 Connected!" << std::endl;
	// Wait a second - needed to support connecting to LEMO Vega
	sleepSeconds(1);
	onErrorPrintDebugMessage(O_capi,"O_capi.initialize()", O_capi.initialize());
	std::cout << O_capi.getTrackingDataTX() << std::endl;

	configureUserParameters(O_capi);




	if (toolDefinitions1.size() > 0)
	{
		std::cout << "Loading Tool Definitions1 (.rom files) ..." << std::endl;
		for (int f = 0; f < toolDefinitions1.size(); f++)
		{
			std::cout << "Loading: " << toolDefinitions1[f] << std::endl;
			loadTool(O_capi,toolDefinitions1[f].c_str());
		}
	}

	std::vector<ToolData> enabledTools1 = std::vector<ToolData>();
	initializeAndEnableTools(O_capi,enabledTools1);

	
	if (enabledTools1.size() == 0)
	{
		std::cout << "No tools detected. To load passive tools, specify: --tools=[tool1.rom],[tool2.rom]" << std::endl;
	}

	//Aurora连接
	if (M_capi.connect(device2, useEncryption ? Protocol::SecureTCP : Protocol::TCP, cipher) != 0)
	{
		// Print the error and exit if we can't connect to a device
		std::cout << "Connection2 Failed!" << std::endl;
		std::cout << "Press Enter to continue...";
		//std::cin.ignore();
		return;
	}
	std::cout << "Device2 Connected!" << std::endl;
	// Wait a second - needed to support connecting to LEMO Vega
	sleepSeconds(1);

	// Initialize the system. This clears all previously loaded tools, unsaved settings etc...
	onErrorPrintDebugMessage(M_capi,"M_capi.initialize()", M_capi.initialize());

	// Demonstrate error handling by asking for tracking data in the wrong mode
	std::cout << M_capi.getTrackingDataTX() << std::endl;

	// Demonstrate getting/setting user parameters
	configureUserParameters(M_capi);

	// Load any passive tool definitions from a .rom files
	if (toolDefinitions2.size() > 0)
	{
		std::cout << "Loading Tool Definitions2 (.rom files) ..." << std::endl;
		for (int f = 0; f < toolDefinitions2.size(); f++)
		{
			std::cout << "Loading: " << toolDefinitions2[f] << std::endl;
			loadTool(M_capi,toolDefinitions2[f].c_str());
		}
	}

	std::vector<ToolData> enabledTools2 = std::vector<ToolData>();
	initializeAndEnableTools(M_capi, enabledTools2);

	// Print an error if no tools were specified
	if (enabledTools2.size() == 0)
	{
		std::cout << "No tools detected. To load passive tools, specify: --tools=[tool1.rom],[tool2.rom]" << std::endl;
	}

	// Once the system is put into tracking mode, data is returned for whatever tools are enabled
	std::cout << "Entering tracking mode..." << std::endl;
	onErrorPrintDebugMessage(O_capi,"O_capi.startTracking()", O_capi.startTracking());
	onErrorPrintDebugMessage(M_capi,"M_capi.startTracking()", M_capi.startTracking());

	//O_capi.stopTracking();
	//M_capi.stopTracking();

	//O_capi.startTracking();
	//M_capi.startTracking();

	printTrackingData(O_capi, M_capi);



	m_LinkNDI = true;

	if (m_LinkNDI) {
		emit linkStatusChanged(true); // 发射连接成功信号
	}

	
}

void NDIWorker::sleepSeconds(unsigned numSeconds)
{
#ifdef _WIN32
	Sleep((DWORD)1000 * numSeconds); // Sleep(ms)
#else
	sleep(numSeconds); // sleep(sec)
#endif
}


void NDIWorker::configureUserParameters(CombinedApi& capi)
{
	std::cout << capi.getUserParameter("Param.User.String0") << std::endl;
	onErrorPrintDebugMessage(capi,"capi1.setUserParameter(Param.User.String0, customString)", capi.setUserParameter("Param.User.String0", "customString"));
	std::cout << capi.getUserParameter("Param.User.String0") << std::endl;
	onErrorPrintDebugMessage(capi,"capi1.setUserParameter(Param.User.String0, emptyString)", capi.setUserParameter("Param.User.String0", ""));
}

void NDIWorker::onErrorPrintDebugMessage(CombinedApi& capi,std::string methodName, int errorCode)
{
	if (errorCode < 0)
	{
		std::cout << methodName << " failed: " << capi.errorToString(errorCode) << std::endl;
	}
}

void NDIWorker::loadTool(CombinedApi& capi,const char* toolDefinitionFilePath)
{
	// Request a port handle to load a passive tool into
	int portHandle = capi.portHandleRequest();
	onErrorPrintDebugMessage(capi,"capi.portHandleRequest()", portHandle);

	// Load the .rom file using the previously obtained port handle
	capi.loadSromToPort(toolDefinitionFilePath, portHandle);
}


void NDIWorker::initializeAndEnableTools(CombinedApi& capi,std::vector<ToolData>& enabledTools)
{


	std::cout << std::endl << "Initializing and enabling tools..." << std::endl;

	// Initialize and enable tools
	std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);

	//打印capi的portHandles数目
	std::cout << "nums in portHandles after Initialize and enable tools" << portHandles.size() << std::endl;
	for (int i = 0; i < portHandles.size(); i++)
	{
		std::cout << portHandles[i].getPortHandle() << std::endl;
	}

	for (int i = 0; i < portHandles.size(); i++)
	{

		onErrorPrintDebugMessage(capi,"capi.portHandleInitialize()", capi.portHandleInitialize(portHandles[i].getPortHandle()));
		onErrorPrintDebugMessage(capi,"capi.portHandleEnable()", capi.portHandleEnable(portHandles[i].getPortHandle()));
	}

	// Print all enabled tools
	portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
	for (int i = 0; i < portHandles.size(); i++)
	{
		std::cout << portHandles[i].toString() << std::endl;
	}

	// Lookup and store the serial number for each enabled tool
	for (int i = 0; i < portHandles.size(); i++)
	{
		enabledTools.push_back(ToolData());
		enabledTools.back().transform.toolHandle = (uint16_t)capi.stringToInt(portHandles[i].getPortHandle());
		enabledTools.back().toolInfo = getToolInfo(capi,portHandles[i].getPortHandle());
	}


}


std::string NDIWorker::getToolInfo(CombinedApi& capi,std::string toolHandle)
{
	// Get the port handle info from PHINF
	PortHandleInfo info = capi.portHandleInfo(toolHandle);

	// Return the ID and SerialNumber the desired string format
	std::string outputString = info.getToolId();
	outputString.append(" s/n:").append(info.getSerialNumber());
	return outputString;
}

void NDIWorker::printTrackingData(CombinedApi& capi1, CombinedApi& capi2)
//两个设备采集数据并打印
{
	// Start tracking, output a few frames of data, and stop tracking

	for (int i = 0; i < 5; i++)
	{
		// VEGA数据采集和打印
	    //std::cout << capi1.getTrackingDataTX() << std::endl;
		std::cout << "VEGA" << std::endl;
		std::vector<ToolData> toolData1 = capi1.getTrackingDataBX2();

		// Print to stdout in similar format to CSV
		std::cout << "[alerts] [buttons] Frame#,ToolHandle,Face#,timespec_s,timespec_ms,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers,State,Tx,Ty,Tz" << std::endl;
		// 工具数量
		std::cout << "toolData size: " << toolData1.size() << std::endl;
		for (int i = 0; i < toolData1.size(); i++)
		{
			printToolData(toolData1[i]);
		}
		// Aurora数据采集和打印
		//std::cout << capi2.getTrackingDataTX() << std::endl;
		std::cout << "Aurora" << std::endl;
		std::vector<ToolData> toolData2 = capi2.getTrackingDataBX();

		// Print to stdout in similar format to CSV
		std::cout << "[alerts] [buttons] Frame#,ToolHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers,State,Tx,Ty,Tz" << std::endl;
		// 工具数量
		std::cout << "toolData size: " << toolData2.size() << std::endl;
		for (int i = 0; i < toolData2.size(); i++)
		{
			printToolData(toolData2[i]);
		}
		//延时采集
		std::cout << "-----------------------------------------------------------------" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	}
}

void NDIWorker::printToolData(const ToolData& toolData)
{
	if (toolData.systemAlerts.size() > 0)
		//toolData 中存在系统警报，则输出警报的数量
	{
		std::cout << "[" << toolData.systemAlerts.size() << " alerts] ";
		for (int a = 0; a < toolData.systemAlerts.size(); a++)
		{
			std::cout << toolData.systemAlerts[a].toString() << std::endl;
		}
	}

	if (toolData.buttons.size() > 0)
		//如果 toolData 中有按钮状态，则输出按钮的状态
	{
		std::cout << "[buttons: ";
		for (int b = 0; b < toolData.buttons.size(); b++)
		{
			std::cout << ButtonState::toString(toolData.buttons[b]) << " ";
		}
		std::cout << "] ";
	}
	std::cout << toolDataToCSV(toolData) << std::endl;
}

std::string NDIWorker::toolDataToCSV(const ToolData& toolData)
{
	std::stringstream stream;
	stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
	stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
		<< "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
	stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";

	// 分开输出 timespec_s 和 timespec_ns
	stream << toolData.timespec_s << "," << toolData.timespec_ns << ",";


	if (toolData.transform.isMissing())
	{
		stream << "Missing,,,,,,,,";
	}
	else
	{
		stream << TransformStatus::toString(toolData.transform.getErrorCode()) << ","
			<< toolData.transform.q0 << "," << toolData.transform.qx << "," << toolData.transform.qy << "," << toolData.transform.qz << ","
			<< toolData.transform.tx << "," << toolData.transform.ty << "," << toolData.transform.tz << "," << toolData.transform.error;
	}

	// Each marker is printed as: status,tx,ty,tz
	stream << "," << toolData.markers.size();
	for (int i = 0; i < toolData.markers.size(); i++)
	{
		stream << "," << MarkerStatus::toString(toolData.markers[i].status);
		if (toolData.markers[i].status == MarkerStatus::Missing)
		{
			stream << ",,,";
		}
		else
		{
			stream << "," << toolData.markers[i].x << "," << toolData.markers[i].y << "," << toolData.markers[i].z;
		}
	}
	return stream.str();
}