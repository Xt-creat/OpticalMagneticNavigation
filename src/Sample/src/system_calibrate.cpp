#include <opencv2/opencv.hpp>
#include "OMCalibrate.h"
#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataWriter.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPointData.h>
#include <numeric>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vector>
#include <vtkProperty.h>
#include <vtkAutoInit.h> // 必须添加此头文件
#include <Eigen/Dense> // For Vector3d
using namespace Eigen;
using Vector3d = Eigen::Vector3d;

#define M_PI 3.14159265358979323846

VTK_MODULE_INIT(vtkInteractionStyle); // 手动初始化交互模块
VTK_MODULE_INIT(vtkRenderingOpenGL2); // 如果需要渲染，也初始化 OpenGL

// 将cv::Mat旋转矩阵转换为欧拉角
Vector3d rotationMatrixToEulerAngles(const cv::Mat& rotationMatrix) {
	// 确保输入是3x3的旋转矩阵
	if (rotationMatrix.rows != 3 || rotationMatrix.cols != 3) {
		throw std::invalid_argument("Input must be a 3x3 rotation matrix.");
	}

	// 将cv::Mat转换为Eigen的旋转矩阵
	Matrix3d R;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			R(i, j) = rotationMatrix.at<double>(i, j);
		}
	}

	// 从旋转矩阵计算欧拉角
	Vector3d eulerAngles = R.eulerAngles(2, 1, 0); // ZYX顺序
	return eulerAngles * (180.0 / M_PI); // 转换为度
}


// 函数：提取平移向量并将其作为点云返回
std::vector<cv::Point3f> extractTranslationVectors(const std::vector<cv::Mat>& transformMatrices) {
	std::vector<cv::Point3f> translationVectors; // 用于存储平移向量作为点云

	for (size_t i = 0; i < transformMatrices.size(); ++i) {
		const cv::Mat& transformationMatrix = transformMatrices[i];

		// 确保变换矩阵是 4x4 的
		if (transformationMatrix.rows == 4 && transformationMatrix.cols == 4) {
			// 提取平移向量（矩阵的最后一列，前三个元素）
			cv::Point3f translation(
				transformationMatrix.at<float>(0, 3),
				transformationMatrix.at<float>(1, 3),
				transformationMatrix.at<float>(2, 3)
			);

			// 将平移向量添加到点云中
			translationVectors.push_back(translation);
		}
		else {
			std::cerr << "Warning: The matrix is not 4x4, skipping this transformation matrix." << std::endl;
		}
	}

	return translationVectors; // 返回提取的平移向量点云
}


// 函数：提取变换矩阵中的平移向量，并将其存储到 std::vector<cv::Mat> 中
void extractTranslationVectorsToMat(const std::vector<cv::Mat>& Marker1_2Vega, std::vector<cv::Mat>& t_Marker1_2Vega) {
	for (size_t i = 0; i < Marker1_2Vega.size(); ++i) {
		const cv::Mat& transformationMatrix = Marker1_2Vega[i];

		// 确保变换矩阵是 4x4 的
		if (transformationMatrix.rows == 4 && transformationMatrix.cols == 4) {
			// 提取平移向量（矩阵的最后一列，前三个元素）
			cv::Mat translation = transformationMatrix(cv::Range(0, 3), cv::Range(3, 4));

			// 将平移向量（3x1矩阵）添加到 t_Marker1_2Vega
			t_Marker1_2Vega.push_back(translation);
		}
		else {
			std::cerr << "Warning: The matrix is not 4x4, skipping this transformation matrix." << std::endl;
		}
	}
}


// 计算点云配准的刚性变换矩阵（旋转和平移）
bool computeRigidTransform(const std::vector<cv::Mat>& sourcePoints,
	const std::vector<cv::Mat>& targetPoints,
	cv::Mat& rotationMatrix, cv::Mat& translationVector) {
	// 确保两个点云的点数相同
	if (sourcePoints.size() != targetPoints.size() || sourcePoints.empty()) {
		std::cerr << "Error: Point clouds must have the same number of points and be non-empty." << std::endl;
		return false;
	}

	// 计算源点云和目标点云的质心
	cv::Mat centroidSource = cv::Mat::zeros(3, 1, CV_32F);
	cv::Mat centroidTarget = cv::Mat::zeros(3, 1, CV_32F);
	for (size_t i = 0; i < sourcePoints.size(); ++i) {
		centroidSource += sourcePoints[i];
		centroidTarget += targetPoints[i];
	}
	centroidSource /= static_cast<float>(sourcePoints.size());
	centroidTarget /= static_cast<float>(targetPoints.size());

	// 去质心并计算协方差矩阵
	cv::Mat H = cv::Mat::zeros(3, 3, CV_32F);
	for (size_t i = 0; i < sourcePoints.size(); ++i) {
		cv::Mat centeredSource = sourcePoints[i] - centroidSource;
		cv::Mat centeredTarget = targetPoints[i] - centroidTarget;
		H += centeredSource * centeredTarget.t();
	}

	// 使用 SVD 求解旋转矩阵
	cv::Mat U, S, Vt;
	cv::SVD::compute(H, S, U, Vt);
	rotationMatrix = Vt.t() * U.t();

	// 确保旋转矩阵的正确性（防止反射变换）
	if (cv::determinant(rotationMatrix) < 0) {
		Vt.row(2) *= -1;
		rotationMatrix = Vt.t() * U.t();
	}

	// 计算平移向量
	translationVector = centroidTarget - rotationMatrix * centroidSource;

	return true;
}



// 将 3x1 列向量转换为 4x1 并进行变换
std::vector<cv::Mat> applyTransformToVectors(const std::vector<cv::Mat>& points, const cv::Mat& transformMatrix) {
	std::vector<cv::Mat> transformedPoints;

	cv::Mat transformMatrix32F;
	transformMatrix.convertTo(transformMatrix32F, CV_32F);

	for (const auto& point : points) {
		// 将 3x1 列向量转换为 4x1 齐次坐标
		cv::Mat homogeneousPoint = cv::Mat::ones(4, 1, CV_32F);
		point.copyTo(homogeneousPoint(cv::Range(0, 3), cv::Range(0, 1)));
		//std::cout <<"transformMatrix"<< transformMatrix << endl;
		//std::cout <<"homogeneousPoint"<< homogeneousPoint << endl;

		// 进行 4x4 变换
		cv::Mat transformedPoint = transformMatrix32F * homogeneousPoint;
		//std::cout << "continue?"  << endl;
		// 将结果转换回 3x1
		transformedPoints.push_back(transformedPoint(cv::Range(0, 3), cv::Range(0, 1)).clone());
	}

	return transformedPoints;
}



void visualizePointSets(const std::vector<Vector3d>& T1, const std::vector<Vector3d>& T2) {
	if (T1.empty() || T2.empty()) {
		std::cerr << "One or both point sets are empty. Nothing to visualize." << std::endl;
		return;
	}

	auto pointsT1 = vtkSmartPointer<vtkPoints>::New();
	auto pointsT2 = vtkSmartPointer<vtkPoints>::New();

	for (const auto& point : T1) {
		pointsT1->InsertNextPoint(point.x(), point.y(), point.z());
	}
	for (const auto& point : T2) {
		pointsT2->InsertNextPoint(point.x(), point.y(), point.z());
	}

	auto polyDataT1 = vtkSmartPointer<vtkPolyData>::New();
	polyDataT1->SetPoints(pointsT1);

	auto polyDataT2 = vtkSmartPointer<vtkPolyData>::New();
	polyDataT2->SetPoints(pointsT2);

	auto glyphFilterT1 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterT1->SetInputData(polyDataT1);
	glyphFilterT1->Update();

	auto glyphFilterT2 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterT2->SetInputData(polyDataT2);
	glyphFilterT2->Update();

	auto mapperT1 = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperT1->SetInputData(glyphFilterT1->GetOutput());

	auto mapperT2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperT2->SetInputData(glyphFilterT2->GetOutput());

	auto actorT1 = vtkSmartPointer<vtkActor>::New();
	actorT1->SetMapper(mapperT1);
	actorT1->GetProperty()->SetColor(1.0, 0.0, 0.0);
	actorT1->GetProperty()->SetPointSize(5); // 设置点大小

	auto actorT2 = vtkSmartPointer<vtkActor>::New();
	actorT2->SetMapper(mapperT2);
	actorT2->GetProperty()->SetColor(0.0, 1.0, 0.0);
	actorT2->GetProperty()->SetPointSize(5);

	auto renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(actorT1);
	renderer->AddActor(actorT2);
	renderer->SetBackground(0.1, 0.1, 0.1);

	auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(800, 600);

	renderWindow->Render();

	// 添加交互器保持窗口
	auto interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);
	interactor->Start();
}



// 配准函数
void pointSetRegistration(const std::vector<Vector3d>& T1,
	const std::vector<Vector3d>& T2,
	Matrix3d& R,
	Vector3d& t) {
	// 检查点集大小是否一致
	if (T1.size() != T2.size() || T1.empty()) {
		throw std::runtime_error("Point sets are of unequal size or empty.");
	}

	// 1. 计算质心
	Vector3d centroid_T1 = Vector3d::Zero();
	Vector3d centroid_T2 = Vector3d::Zero();
	for (size_t i = 0; i < T1.size(); ++i) {
		centroid_T1 += T1[i];
		centroid_T2 += T2[i];
	}
	centroid_T1 /= T1.size();
	centroid_T2 /= T2.size();

	// 2. 去中心化
	std::vector<Vector3d> T1_centered, T2_centered;
	for (size_t i = 0; i < T1.size(); ++i) {
		T1_centered.push_back(T1[i] - centroid_T1);
		T2_centered.push_back(T2[i] - centroid_T2);
	}

	// 3. 计算协方差矩阵
	Matrix3d H = Matrix3d::Zero();
	for (size_t i = 0; i < T1_centered.size(); ++i) {
		H += T1_centered[i] * T2_centered[i].transpose();
	}

	// 4. 奇异值分解求旋转矩阵 R
	JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
	Matrix3d U = svd.matrixU();
	Matrix3d V = svd.matrixV();
	R = V * U.transpose();

	// 如果 det(R) < 0, 需要修正
	if (R.determinant() < 0) {
		V.col(2) *= -1.0;
		R = V * U.transpose();
	}

	// 5. 计算平移向量 t
	t = centroid_T2 - R * centroid_T1;
}


// 计算配准误差
void calculateRegistrationError(const std::vector<Vector3d>& T1,
	const std::vector<Vector3d>& T2,
	const Matrix3d& R,
	const Vector3d& t) {
	if (T1.size() != T2.size() || T1.empty()) {
		throw std::runtime_error("Point sets are of unequal size or empty.");
	}

	double totalSquaredError = 0.0;
	double totalError = 0.0;
	double maxError = 0.0;

	for (size_t i = 0; i < T1.size(); ++i) {
		// 配准后的点
		Vector3d transformedPoint = R * T1[i] + t;
		// 欧几里得误差
		double error = (T2[i] - transformedPoint).norm();
		totalSquaredError += error * error;
		totalError += error;
		maxError = std::max(maxError, error);
	}

	// 计算 RMSE 和 ME
	double rmse = std::sqrt(totalSquaredError / T1.size());
	double meanError = totalError / T1.size();

	// 输出结果
	std::cout << "RMSE: " << rmse << "\n";
	std::cout << "Mean Error: " << meanError << "\n";
	std::cout << "Max Error: " << maxError << "\n";
}

void ErrorMetric1(const OMCalibrate& OMC, const cv::Mat RecordEMsensor_2Marker1, cv::Mat RecordEM_2Marker2) {
	cv::Mat RecordMarker1_2EMsensor;
	cv::invert(RecordEMsensor_2Marker1, RecordMarker1_2EMsensor, cv::DECOMP_SVD);

	cv::Mat RecordMarker2_2EM;
	cv::invert(RecordEM_2Marker2, RecordMarker2_2EM, cv::DECOMP_SVD);

	cv::Mat Marker1_2EMsensor;
	cv::invert(OMC.EMsensor_2Marker1, Marker1_2EMsensor, cv::DECOMP_SVD);

	cv::Mat output_Marker1_2Vega;
	cv::Mat Marker1_2Aurora_Path1;
	cv::Mat Marker1_2Aurora_Path2;

	std::vector<Vector3d> T1;
	std::vector<Vector3d> T2;


	std::vector<double> TranslationError;
	std::vector<double> RotationError;

	//用于计算output_Marker1_2Vega和真实Marker1_2Vega的误差
	for (int i = 0; i < OMC.R_Marker1_2Vega.size(); i++) {
		//test_Marker2_2Vega = OMC.createTransformationMatrix(OMC.R_Marker2_2Vega[i], OMC.t_Marker2_2Vega[i]);
		//test_EMsensor_2Aurora = OMC.createTransformationMatrix(OMC.R_EMsensor_2Aurora[i], OMC.t_EMsensor_2Aurora[i]);

		//output_Marker1_2Vega = test_Marker2_2Vega * OMC.EM_2Marker2*test_EMsensor_2Aurora*Marker1_2EMsensor;

		//output_Marker1_2Vega2 =  OMC.Aurora2Vega*test_EMsensor_2Aurora*Marker1_2EMsensor;

		//验证在标定数据集的误差
		//output_Marker1_2Vega = OMC.Aurora2Vega * OMC.EMsensor_2Aurora[i] * Marker1_2EMsensor;
		//output_Marker1_2Vega = OMC.Marker2_2Vega[i] * OMC.EM_2Marker2 * OMC.EMsensor_2Aurora[i] * Marker1_2EMsensor;


		//验证标定的固定变换矩阵在其他数据集可用
		output_Marker1_2Vega = OMC.Marker2_2Vega[i] * RecordEM_2Marker2 * OMC.EMsensor_2Aurora[i] * RecordMarker1_2EMsensor;



		//计算的器械位姿
		std::cout << "Group" << i + 1 << std::endl;
		std::cout << std::endl;
		std::cout << "output_Marker1_2Vega" << output_Marker1_2Vega << std::endl;
		std::cout << std::endl;
		Vector3d translationOutput;
		translationOutput << output_Marker1_2Vega.at<double>(0, 3), output_Marker1_2Vega.at<double>(1, 3), output_Marker1_2Vega.at<double>(2, 3);
		std::cout << "Translation Vector: " << translationOutput.transpose() << std::endl;
		std::cout << std::endl;
		cv::Mat rotationMatrix = output_Marker1_2Vega(cv::Rect(0, 0, 3, 3));
		Vector3d eulerAnglesOutput = rotationMatrixToEulerAngles(rotationMatrix);
		std::cout << "Euler Angles (degrees): " << eulerAnglesOutput.transpose() << std::endl;
		std::cout << std::endl;

		//真实的器械位姿
		std::cout << "truth_Marker1_2Vega" << OMC.Marker1_2Vega[i] << std::endl;
		std::cout << std::endl;
		std::cout << "Translation: " << OMC.t_Marker1_2Vega[i] << std::endl;
		std::cout << std::endl;
		Vector3d translationTruth;
		translationTruth << OMC.Marker1_2Vega[i].at<double>(0, 3), OMC.Marker1_2Vega[i].at<double>(1, 3), OMC.Marker1_2Vega[i].at<double>(2, 3);
		Vector3d eulerAnglesTruth = rotationMatrixToEulerAngles(OMC.R_Marker1_2Vega[i]);
		std::cout << "Euler Angles (degrees): " << eulerAnglesTruth.transpose() << std::endl;
		std::cout << std::endl;

		//计算平移误差和旋转误差
		double DistanceError = (translationOutput - translationTruth).norm();
		std::cout << "DistanceError " << DistanceError << std::endl;
		TranslationError.push_back(DistanceError);
		double AngleError = (eulerAnglesOutput - eulerAnglesTruth).norm();
		RotationError.push_back(AngleError);
		std::cout << "AngleError " << AngleError << std::endl;

	}
	//// 计算 TranslationError 的Mean/Min/Max
	double translationErrorMean = std::accumulate(TranslationError.begin(), TranslationError.end(), 0.0) / TranslationError.size();
	std::cout << "Average Translation Error: " << translationErrorMean << std::endl;
	double maxTranslationError = *std::max_element(TranslationError.begin(), TranslationError.end());
	double minTranslationError = *std::min_element(TranslationError.begin(), TranslationError.end());
	std::cout << "Max Translation Error: " << maxTranslationError << std::endl;
	std::cout << "Min Translation Error: " << minTranslationError << std::endl;

	// 计算 RotationError 的Mean/Min/Max
	double rotationErrorMean = std::accumulate(RotationError.begin(), RotationError.end(), 0.0) / RotationError.size();
	std::cout << "Average Rotation Error: " << rotationErrorMean << std::endl;
	double maxRotationError = *std::max_element(RotationError.begin(), RotationError.end());
	double minRotationError = *std::min_element(RotationError.begin(), RotationError.end());
	std::cout << "Max Rotation Error: " << maxRotationError << std::endl;
	std::cout << "Min Rotation Error: " << minRotationError << std::endl;
}

void ErrorMetric2(const OMCalibrate& OMC, const cv::Mat RecordEMsensor_2Marker1, cv::Mat RecordEM_2Marker2) {
	//Methods for Simultaneous Robot-World-Hand--Eye Calibration- A Comparative Study




}




//int main()
//{
//	OMCalibrate OMC;
//	//std::string filename = "D:/Optomagnetic-tracking/CombinedAPIsample_v2/Vega_Collected_data.csv";
//	//OMC.ReadtRecordData(filename,12,9);
//	//OMC.PrintTrackedData();
//	
//	
//
//	OMC.HandeyeCalibrate3();
//	std::cout << "Aurora2Vega" << OMC.Aurora2Vega << std::endl;
//	std::cout << std::endl;
//	std::cout << "EMsensor_2Marker1" << OMC.EMsensor_2Marker1 << std::endl;
//	std::cout << std::endl;
//	std::cout << "EM_2Marker2" << OMC.EM_2Marker2 << std::endl;
//	std::cout << std::endl;
//
//
//	//Aurora_Collected_data_Staticabc.csv和Vega_Collected_data_Staticabc.csv标定结果
//	//使用算法：cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH
//	/*cv::Mat RecordEMsensor_2Marker1 = (cv::Mat_<double>(4, 4) <<
//		0.9688130440658627, -0.2125049752086569, 0.1274477193181763, -14.10909762500515,
//	-0.1141237264900248, 0.07388013079954617, 0.9907156510952434, 1.402743103227017,
//	-0.2199478590481539, -0.9743630544025209, 0.04732417474741032, 40.39570284928978,
//	0, 0, 0,1);
//	cv::Mat RecordEM_2Marker2 = (cv::Mat_<double>(4, 4) <<
//		0.7068566913459077, 0.03909418580624402, -0.7062756278788491, 61.42075358060455,
//	-0.7063178856807577, -0.01508595729632206, -0.707734030734652, -8.047828626211167,
//	-0.03832312966055588, 0.9991216435092377, 0.01694931280081335, -124.1516367177233,
//	0, 0, 0, 1);*/
//
//	//Aurora_Collected_data_Staticabc.csv和Vega_Collected_data_Staticabc.csv标定结果
//	//使用算法：cv::CALIB_ROBOT_WORLD_HAND_EYE_LI
//	/*cv::Mat RecordEMsensor_2Marker1 = (cv::Mat_<double>(4, 4) <<
//		0.9693259355208593, -0.2084142597946028, 0.130271743063791, -15.28693847924255,
//	-0.1193484177093449, 0.06421024858526503, 0.9907739394871538, 2.128075882125435,
//	-0.2148561982277594, -0.9759306021899744, 0.03736674714538873, 40.29436060036536,
//	0, 0, 0, 1);
//	cv::Mat RecordEM_2Marker2 = (cv::Mat_<double>(4, 4) <<
//		0.702498990237248, 0.0430812595948305, -0.7103795983749606, 59.8414559322578,
//	-0.7104176654528762, -0.01708677606742617, -0.7035728694997463, -8.201175174872091,
//	-0.04244890255509942, 0.9989254462447857, 0.01860224493235183, -123.850180206467,
//	0, 0, 0, 1);*/
//
//    //Aurora_Collected_data_Static_test27.csv和Vega_Collected_data_Static_test27.csv标定结果
//	//使用算法：cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH
//	/*cv::Mat RecordEMsensor_2Marker1 = (cv::Mat_<double>(4, 4) <<
//		0.9671755070005249, -0.2204001372249867, 0.1264725984926568, -17.03223534906297,
//	-0.1072818234213993, 0.09701563854092207, 0.9894839949397247, 1.45658943773898,
//	-0.2303522281673381, -0.9705728954538678, 0.07018622078897263, 42.13842688132948,
//	0, 0, 0, 1);
//	cv::Mat RecordEM_2Marker2 = (cv::Mat_<double>(4, 4) <<
//		0.7201568526430151, 0.002985389602308708, -0.6938048681295261, 66.91543934916911,
//	-0.6938105802379644, 0.001667324651977747, -0.7201556073366118, -9.023501130065338,
//	-0.0009931471018925567, 0.9999941537216244, 0.003272030165916567, -131.3936875244864,
//	0, 0, 0, 1);*/
//
//	//
//	/*cv::Mat RecordEMsensor_2Marker1 = (cv::Mat_<double>(4, 4) <<
//		0.9630136298295092, -0.2405277446141953, 0.1214543240621798, -19.2533983122965,
//	-0.1070406403239078, 0.07215996346702896, 0.9916326139208433, -0.4800268993572878,
//	-0.2472792956994989, -0.9679562716069592, 0.04374478454757944, 36.82410188973542,
//	0, 0, 0, 1);
//	cv::Mat RecordEM_2Marker2 = (cv::Mat_<double>(4, 4) <<
//		0.6990778776881925, 0.02703853315331224, -0.7145341409980979, 62.40616427236554,
//	-0.7146560362853871, -0.006558305832288336, -0.69944530767278, -2.278167750799835,
//	-0.02359810856471617, 0.999612878243136, 0.01473848440785097, -132.6401500243014,
//	0, 0, 0, 1);*/
//
//	//Matlab计算结果（32数据集）
//	/*cv::Mat RecordEMsensor_2Marker1 = (cv::Mat_<double>(4, 4) <<
//		0.9672, - 0.1063, - 0.2308 ,  24.7655,
//		- 0.2231,    0.0797 ,- 0.9715 ,  36.2172,
//		0.1216,    0.9911 ,   0.0534 ,- 1.8481,
//		0   ,      0      ,   0 ,   1.0000//		);
//	cv::Mat RecordEM_2Marker2 = (cv::Mat_<double>(4, 4) <<
//		0.7131 ,- 0.7004 ,- 0.0287, - 54.1880,
//		0.0291, - 0.0114 ,   0.9995 , 125.2873,
//		- 0.7004, - 0.7136,    0.0123 ,  40.3016,
//		0   ,      0     ,    0  ,  1.0000//		);*/
//
//	//Matlab计算结果（60数据集）
//	cv::Mat RecordEMsensor_2Marker1 = (cv::Mat_<double>(4, 4) <<
//		0.9639, - 0.2150  ,  0.1569  ,  1.3373,
//		- 0.1464   , 0.0639 ,   0.9872 ,- 0.9964,
//		- 0.2222, - 0.9745 ,   0.0301,   43.6812,
//		0    ,     0      ,   0  ,  1.0000////		);
//	cv::Mat RecordEM_2Marker2 = (cv::Mat_<double>(4, 4) <<
//		0.6912  ,  0.0266, - 0.7222  , 58.3635,
//		- 0.7214, - 0.0347, - 0.6917, - 8.8387,
//		- 0.0435 ,   0.9990 ,- 0.0048 ,- 118.5730,
//		0     ,    0    ,     0   , 1.0000//		);
//	
//
//	ErrorMetric1(OMC, RecordEMsensor_2Marker1, RecordEM_2Marker2);
//
//
//
//	
//
//
//
//	//for (int i = 0; i < OMC.Marker1_2Vega.size(); i++) {
//	//	//test_Marker2_2Vega = OMC.createTransformationMatrix(OMC.R_Marker2_2Vega[i], OMC.t_Marker2_2Vega[i]);
//	//	//test_EMsensor_2Aurora = OMC.createTransformationMatrix(OMC.R_EMsensor_2Aurora[i], OMC.t_EMsensor_2Aurora[i]);
//
//	//	//output_Marker1_2Vega = test_Marker2_2Vega * OMC.EM_2Marker2*test_EMsensor_2Aurora*Marker1_2EMsensor;
//
//	//	//output_Marker1_2Vega2 =  OMC.Aurora2Vega*test_EMsensor_2Aurora*Marker1_2EMsensor;
//
//	//	//验证在标定数据集的误差
//	//	//output_Marker1_2Vega = OMC.Aurora2Vega * OMC.EMsensor_2Aurora[i] * Marker1_2EMsensor;
//	//	//output_Marker1_2Vega = OMC.Marker2_2Vega[i]*OMC.EM_2Marker2 * OMC.EMsensor_2Aurora[i] * Marker1_2EMsensor;
//
//
//	//	//验证标定的固定变换矩阵在其他数据集可用
//	//	Marker1_2Aurora_Path1 = OMC.EMsensor_2Aurora[i] * RecordMarker1_2EMsensor;
//	//	Marker1_2Aurora_Path2 = RecordMarker2_2EM *OMC.Vega_2Marker2[i]*OMC.Marker1_2Vega[i];
//
//
//	//	//Marker1_2Aurora_Path1位姿
//	//	std::cout << "Group" << i + 1 << std::endl;
//	//	std::cout << std::endl;
//	//	std::cout << "Marker1_2Aurora_Path1" << Marker1_2Aurora_Path1 << std::endl;
//	//	std::cout << std::endl;
//	//	Vector3d translationOutput1;
//	//	translationOutput1 << Marker1_2Aurora_Path1.at<double>(0, 3), Marker1_2Aurora_Path1.at<double>(1, 3), Marker1_2Aurora_Path1.at<double>(2, 3);
//	//	T1.push_back(translationOutput1);
//	//	std::cout << "Translation Vector: " << translationOutput1.transpose() << std::endl;
//	//	std::cout << std::endl;
//	//	cv::Mat rotationMatrix1 = Marker1_2Aurora_Path1(cv::Rect(0, 0, 3, 3));
//	//	Vector3d eulerAnglesOutput1 = rotationMatrixToEulerAngles(rotationMatrix1);
//	//	std::cout << "Euler Angles (degrees): " << eulerAnglesOutput1.transpose() << std::endl;
//	//	std::cout << std::endl;
//
//	//	//Marker1_2Aurora_Path2位姿
//	//	std::cout << "Group" << i + 1 << std::endl;
//	//	std::cout << std::endl;
//	//	std::cout << "Marker1_2Aurora_Path2" << Marker1_2Aurora_Path2 << std::endl;
//	//	std::cout << std::endl;
//	//	Vector3d translationOutput2;
//	//	translationOutput2 << Marker1_2Aurora_Path2.at<double>(0, 3), Marker1_2Aurora_Path2.at<double>(1, 3), Marker1_2Aurora_Path2.at<double>(2, 3);
//	//	T2.push_back(translationOutput2);
//	//	std::cout << "Translation Vector: " << translationOutput2.transpose() << std::endl;
//	//	std::cout << std::endl;
//	//	cv::Mat rotationMatrix2 = Marker1_2Aurora_Path2(cv::Rect(0, 0, 3, 3));
//	//	Vector3d eulerAnglesOutput2 = rotationMatrixToEulerAngles(rotationMatrix2);
//	//	std::cout << "Euler Angles (degrees): " << eulerAnglesOutput2.transpose() << std::endl;
//
//	//	//计算平移误差和旋转误差
//	//	double DistanceError = (translationOutput1 - translationOutput2).norm();
//	//	std::cout << "DistanceError " << DistanceError << std::endl;
//	//	TranslationError.push_back(DistanceError);
//	//	double AngleError = (eulerAnglesOutput1 - eulerAnglesOutput2).norm();
//	//	RotationError.push_back(AngleError);
//	//	std::cout << "AngleError " << AngleError << std::endl;
//
//	//}
//	//
//
// //   std::cout << std::endl;
//
//	//// 计算 TranslationError 的Mean/Min/Max
//	//double translationErrorMean = std::accumulate(TranslationError.begin(), TranslationError.end(), 0.0) / TranslationError.size();
//	//std::cout << "Average Translation Error: " << translationErrorMean << std::endl;
//	//double maxTranslationError = *std::max_element(TranslationError.begin(), TranslationError.end());
//	//double minTranslationError = *std::min_element(TranslationError.begin(), TranslationError.end());
//	//std::cout << "Max Translation Error: " << maxTranslationError << std::endl;
//	//std::cout << "Min Translation Error: " << minTranslationError << std::endl;
//
//	//// 计算 RotationError 的Mean/Min/Max
//	//double rotationErrorMean = std::accumulate(RotationError.begin(), RotationError.end(), 0.0) / RotationError.size();
//	//std::cout << "Average Rotation Error: " << rotationErrorMean << std::endl;
//	//double maxRotationError = *std::max_element(RotationError.begin(), RotationError.end());
//	//double minRotationError = *std::min_element(RotationError.begin(), RotationError.end());
//	//std::cout << "Max Rotation Error: " << maxRotationError << std::endl;
//	//std::cout << "Min Rotation Error: " << minRotationError << std::endl;
//
//
//	//std::cout << std::endl;
//	//std::cout << "T1 size"<<T1.size()<<std::endl;
//
//
//	////visualizePointSets(T1, T2);
//	//
//
//	//// 配准结果
//	//Matrix3d R;
//	//Vector3d t;
//
//	//// 执行配准
//	//pointSetRegistration(T1, T2, R, t);
//
//	//// 输出结果
//	//std::cout << "Rotation Matrix R:\n" << R << "\n";
//	//std::cout << "Translation Vector t:\n" << t.transpose() << "\n";
//
//	//// 计算误差
//	//calculateRegistrationError(T1, T2, R, t);
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//	//std::vector<cv::Mat> t_Vega_2Marker1;
//	//std::vector<cv::Mat> t_Marker1_2Vega;
//	//std::vector<cv::Mat> t_Aurora_2EMSensor;
//	//std::vector<cv::Mat> t_EMsensor_2Aurora;
//	//// 调用函数提取平移向量并存储到 t_Marker1_2Vega
//	//extractTranslationVectorsToMat(OMC.Vega_2Marker1, t_Vega_2Marker1);
//	//extractTranslationVectorsToMat(OMC.Marker1_2Vega, t_Marker1_2Vega);
//	//extractTranslationVectorsToMat(OMC.Aurora_2EMsensor, t_Aurora_2EMSensor);
//	//extractTranslationVectorsToMat(OMC.EMsensor_2Aurora, t_EMsensor_2Aurora);
//
//	//// 打印提取的平移向量
//	//for (size_t i = 0; i < t_Vega_2Marker1.size(); ++i) {
//	//	std::cout << "t_Vega_2Marker1 " << i << " : \n" << t_Vega_2Marker1[i] << std::endl;
//	//}
//
//	////// 旋转矩阵和平移向量
//	////cv::Mat rotationMatrix, translationVector;
//
//	////// 计算刚性变换
//	////if (computeRigidTransform(t_Vega_2Marker1, t_Aurora_2EMSensor, rotationMatrix, translationVector)) {
//	////	std::cout << "Rotation Matrix: \n" << rotationMatrix << std::endl;
//	////	std::cout << "Translation Vector: \n" << translationVector << std::endl;
//	////}
//	////else {
//	////	std::cerr << "Failed to compute the rigid transform." << std::endl;
//	////}
//
//	// // 应用变换
//	//std::vector<cv::Mat> transformedPoints = applyTransformToVectors(t_Aurora_2EMSensor, RecordEMsensor_2Marker1);
//	//// 打印提取的平移向量
//	//for (size_t i = 0; i < transformedPoints.size(); ++i) {
//	//	std::cout << "t_Aurora_2EMSensor_convert " << i << " : \n" << transformedPoints[i] << std::endl;
//	//}
//
//	std::cin.get();
//	
//}