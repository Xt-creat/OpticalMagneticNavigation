#include "DeviceReader.h"
#include<iostream>
#include <CombinedApi.h>


extern CombinedApi O_capi;
extern CombinedApi M_capi;

DeviceReader::DeviceReader(int deviceId, int updateRate)
	: deviceId(deviceId), updateRate(updateRate) {
	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &DeviceReader::readData);
}

DeviceReader::~DeviceReader() {
	delete timer;
}

void DeviceReader::startReading() {
	timer->start(updateRate);
}

void DeviceReader::stop() {
	if (timer && timer->isActive()) {
		timer->stop();
	}
}

void DeviceReader::readData() {
	// 读取设备数据，假设使用 capi1 和 capi2 从不同设备获取数据
	if (deviceId == 1) {
		toolData1 = O_capi.getTrackingDataBX2();
		emit newODataAvailable(toolData1);  // 发射设备1数据
	}
	else if (deviceId == 2) {
		toolData2 = M_capi.getTrackingDataBX();
		emit newMDataAvailable(toolData2);  // 发射设备2数据
	}
}

