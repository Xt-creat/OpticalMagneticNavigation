#ifndef OMCalibrate_H
#define OMCalibrate_H

#include <opencv2/opencv.hpp>
#include "CombinedApi.h"
#include "ToolData.h"
#include "ToolRegi.h"
#include <opencv2/opencv.hpp>

struct TrackedData
{
	std::string name = "";
	double quat[4]; //四元数
	double t[3];    //平移
	double FRE=0;
	cv::Mat R;  //旋转矩阵
};




class OMCalibrate
{
public:
	OMCalibrate();

	~OMCalibrate();

	void HandeyeCalibrate();  //求解两次AX=XB，计算M12EMS和EM2M2

	void HandeyeCalibrate1();

	void HandeyeCalibrate2();//计算EMS2M1

	void HandeyeCalibrate3();//求解AX=YB
	
	void PrintTrackedData(const std::vector<TrackedData>& m_data);



//private:
	std::vector<cv::Mat> R_Marker1_2Vega;     //读取数据存储
	std::vector<cv::Mat> t_Marker1_2Vega;
	std::vector<cv::Mat> Marker1_2Vega;
	std::vector<cv::Mat> Vega_2Marker1;

	std::vector<cv::Mat> R_Marker2_2Vega;       
	std::vector<cv::Mat> t_Marker2_2Vega;
	std::vector<cv::Mat> Marker2_2Vega;
	std::vector<cv::Mat> Vega_2Marker2;

	std::vector<cv::Mat> R_EMsensor_2Aurora;
	std::vector<cv::Mat> t_EMsensor_2Aurora;
	std::vector<cv::Mat> EMsensor_2Aurora;
	std::vector<cv::Mat> Aurora_2EMsensor;

	std::vector<cv::Mat> R_Marker2_2Marker1;
	std::vector<cv::Mat> t_Marker2_2Marker1;
	std::vector<cv::Mat> Marker2_2Marker1;


	cv::Mat R_EMsensor_2Marker1;  //待求转换矩阵
	cv::Mat t_EMsensor_2Marker1;
	cv::Mat EMsensor_2Marker1;
	cv::Mat Marker1_2EMsensor;
	cv::Mat R_EM_2Marker2;
	cv::Mat t_EM_2Marker2;
	cv::Mat EM_2Marker2;
	cv::Mat R_Aurora2Vega;
	cv::Mat t_Aurora2Vega;
	cv::Mat Aurora2Vega;

	std::vector<TrackedData> M1_data;
	std::vector<TrackedData> M2_data;
	std::vector<TrackedData> EMSensor_data;

	void ReadRecordData(std::vector<TrackedData>& m_data, const std::string& filename, int groups,int interval, int start);
	void Quat2Matrices(std::vector<TrackedData>& data);
	void GetRt();

	cv::Mat createTransformationMatrix(const cv::Mat& R, const cv::Mat& t);

};












#endif
