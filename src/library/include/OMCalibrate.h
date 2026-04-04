#ifndef OMCalibrate_H
#define OMCalibrate_H

// 添加导出宏（与 CMake 的 DEFINE_SYMBOL 匹配）
#ifdef _WIN32
#ifdef CAPICOMMON_EXPORTS
#define CAPICOMMON_API __declspec(dllexport)
#else
#define CAPICOMMON_API __declspec(dllimport)
#endif
#else
#define CAPICOMMON_API __attribute__ ((visibility ("default")))
#endif

#include <opencv2/opencv.hpp>
#include "CombinedApi.h"
#include "ToolData.h"
#include "ToolRegi.h"


struct TrackedData
{
	std::string name = "";
	double quat[4]; //四元数
	double t[3];    //平移
	double FRE=0;
	cv::Mat R;  //旋转矩阵
};

struct CalibrationError {
	double avgTranslationError;
	double avgRotationError;
	std::vector<double> translationErrors;
	std::vector<double> rotationErrors;
};


class CAPICOMMON_API OMCalibrate
{
public:
	OMCalibrate();
	OMCalibrate(const std::string& path, int group = 30);

	~OMCalibrate();

	void LoadData(const std::string& path, int group = 30);

	void PrepareCalibration();

	void RemoveGroup(int index);

	void HandeyeCalibrate();  //求解两次AX=XB，计算tool1_2EMsensor和EM_2tool2

	void HandeyeCalibrate1();

	void HandeyeCalibrate2();//计算EMS2M1

	void HandeyeCalibrate3();//求解AX=YB
	
	void PrintTrackedData(const std::vector<TrackedData>& m_data);

	void saveVectorMatToTxt(const std::vector<cv::Mat>& matrices, const std::string& filename);

	void saveTrackedDataToCsv(const std::vector<TrackedData>& data, const std::string& filename);

	CalibrationError EvaluateCalibration();


//private:
	std::vector<cv::Mat> R_tool1_2Vega;     //读取数据存储
	std::vector<cv::Mat> t_tool1_2Vega;
	std::vector<cv::Mat> tool1_2Vega;
	std::vector<cv::Mat> Vega_2tool1;

	std::vector<cv::Mat> R_tool2_2Vega;
	std::vector<cv::Mat> t_tool2_2Vega;
	std::vector<cv::Mat> tool2_2Vega;
	std::vector<cv::Mat> Vega_2tool2;

	std::vector<cv::Mat> R_EMsensor_2Aurora;
	std::vector<cv::Mat> t_EMsensor_2Aurora;
	std::vector<cv::Mat> EMsensor_2Aurora;
	std::vector<cv::Mat> Aurora_2EMsensor;

	std::vector<cv::Mat> R_tool2_2tool1;
	std::vector<cv::Mat> t_tool2_2tool1;
	std::vector<cv::Mat> tool2_2tool1;

	std::vector<cv::Mat> tool1_2tool2;


	cv::Mat R_EMsensor_2tool1;  //待求转换矩阵
	cv::Mat t_EMsensor_2tool1;
	cv::Mat EMsensor_2tool1;
	cv::Mat tool1_2EMsensor;
	cv::Mat R_EM_2tool2;
	cv::Mat t_EM_2tool2;
	cv::Mat EM_2tool2;
	cv::Mat R_Aurora2Vega;
	cv::Mat t_Aurora2Vega;
	cv::Mat Aurora2Vega;

	std::vector<TrackedData> tool1_data;
	std::vector<TrackedData> tool2_data;
	std::vector<TrackedData> EMSensor_data;

	void ReadRecordData(std::vector<TrackedData>& m_data, const std::string& filename, int groups,int interval, int start);
	void Quat2Matrices(std::vector<TrackedData>& data);
	void GetRt();

	cv::Mat createTransformationMatrix(const cv::Mat& R, const cv::Mat& t);

	// --- 新增: Proposed1 算法与优化支持函数 ---
	// 执行完整的 Proposed1 标定流水线，返回被剔除的异常组别索引 (基于原始组号)
	std::vector<int> ProposedCalibration(int num_to_skip = 5);

	// 非线性优化 AX = XB (Gauss-Newton / LM 迭代)
	cv::Mat solveAXXB_LM(const std::vector<cv::Mat>& A, const std::vector<cv::Mat>& B, const cv::Mat& initial_X);

	// 基于 3D 虚拟点云的 SVD 基座优化 (计算 EM_2tool2)
	cv::Mat optimizeBaseToGridSVD(const std::vector<cv::Mat>& tool1_2tool2_vec, const std::vector<cv::Mat>& EMsensor_2Aurora_vec, const cv::Mat& EMsensor_2tool1);


};



#endif

