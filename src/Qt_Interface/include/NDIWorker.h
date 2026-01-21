#pragma once

#include <QObject>
#include <CombinedApi.h>

class NDIWorker  : public QObject
{
	Q_OBJECT

public:
	NDIWorker();
	~NDIWorker();
	void printTrackingData(CombinedApi& capi1, CombinedApi& capi2);


signals:
	void linkStatusChanged(bool isConnected); // ÐÂÔöÐÅºÅ
	

public slots:
	void LinkNDI(bool);


private:
	void sleepSeconds(unsigned numSeconds);
	void configureUserParameters(CombinedApi& capi);
	void onErrorPrintDebugMessage(CombinedApi& capi, std::string methodName, int errorCode);
	void loadTool(CombinedApi& capi, const char* toolDefinitionFilePath);
	void initializeAndEnableTools(CombinedApi& capi, std::vector<ToolData>& enabledTools);
	std::string getToolInfo(CombinedApi& capi, std::string toolHandle);

	void printToolData(const ToolData& toolData);
	std::string toolDataToCSV(const ToolData& toolData);

	std::string device1;
	std::string tool1 ;
	std::string tool2;
	std::string tool4;
	std::string device2;
	std::string tool3;

	std::vector<std::string> toolDefinitions1;
	std::vector<std::string> toolDefinitions2;

	bool m_LinkNDI;
	

	
};
