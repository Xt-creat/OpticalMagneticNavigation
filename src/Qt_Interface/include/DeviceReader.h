#pragma once

#include <QObject>
#include <QTimer>
#include <CombinedApi.h>


class DeviceReader : public QObject {
	Q_OBJECT
public:
	DeviceReader(int deviceId, int updateRate);
	~DeviceReader();

signals:
	void newODataAvailable(const std::vector<ToolData> &data);  // 设备1（O_capi）数据信号
	void newMDataAvailable(const std::vector<ToolData> &data);  // 设备2（M_capi）数据信号

public slots:
	void readData();
	void startReading(); // 启动定时器
	void stop();   

private:
	int deviceId;
	int updateRate; // 更新频率（ms）
	QTimer* timer;
	std::vector<ToolData> toolData1;
	std::vector<ToolData> toolData2;
};