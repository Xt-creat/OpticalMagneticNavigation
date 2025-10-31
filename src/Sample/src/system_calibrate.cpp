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
#include <random>
#include <algorithm>

#include "ToolRegi.h"


using namespace Eigen;
using Vector3d = Eigen::Vector3d;

#define M_PI 3.14159265358979323846

VTK_MODULE_INIT(vtkInteractionStyle); // 手动初始化交互模块
VTK_MODULE_INIT(vtkRenderingOpenGL2); // 如果需要渲染，也初始化 OpenGL

// 将cv::Mat旋转误差矩阵转换为角度误差
double rotationMatrixToAngleError(const cv::Mat& rotationMatrix) {
	// 确保输入是3x3的旋转矩阵
	if (rotationMatrix.rows != 3 || rotationMatrix.cols != 3) {
		throw std::invalid_argument("Input must be a 3x3 rotation matrix.");
	}

	// 将cv::Mat转换为Eigen的旋转矩阵
	Eigen::Matrix3d R;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			R(i, j) = rotationMatrix.at<double>(i, j);
		}
	}

	// 使用 AngleAxis 提取旋转角度
	Eigen::AngleAxisd angleAxis(R);
	double angleInDegrees = angleAxis.angle() * (180.0 / M_PI);

	return angleInDegrees;
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



//void ErrorMetric1(const OMCalibrate& OMC, const cv::Mat RecordEMsensor_2Marker1, cv::Mat RecordEM_2Marker2) {
//	//Methods for Simultaneous Robot-World-Hand--Eye Calibration- A Comparative Study
//	cv::Mat RecordMarker1_2EMsensor;
//	cv::invert(RecordEMsensor_2Marker1, RecordMarker1_2EMsensor, cv::DECOMP_SVD);
//
//	cv::Mat RecordMarker2_2EM;
//	cv::invert(RecordEM_2Marker2, RecordMarker2_2EM, cv::DECOMP_SVD);
//
//	cv::Mat Marker1_2EMsensor;
//	cv::invert(OMC.EMsensor_2Marker1, Marker1_2EMsensor, cv::DECOMP_SVD);
//
//	cv::Mat output_Marker1_2Vega;
//	cv::Mat Marker1_2Aurora_Path1;
//	cv::Mat Marker1_2Aurora_Path2;
//
//	std::vector<Vector3d> T1;
//	std::vector<Vector3d> T2;
//
//
//	std::vector<double> TranslationError;
//	std::vector<double> RotationError;
//
//	for (int i = 0; i < OMC.R_Marker1_2Vega.size(); i++) {
//		//验证标定的固定变换矩阵在其他数据集可用
//		output_Marker1_2Vega = OMC.Marker2_2Vega[i] * RecordEM_2Marker2 * OMC.EMsensor_2Aurora[i] * RecordMarker1_2EMsensor;
//
//
//		//计算的器械位姿
//		std::cout << "Group" << i + 1 << std::endl;
//		std::cout << std::endl;
//		std::cout << "output_Marker1_2Vega" << output_Marker1_2Vega << std::endl;
//		std::cout << std::endl;
//		Vector3d translationOutput;
//		translationOutput << output_Marker1_2Vega.at<double>(0, 3), output_Marker1_2Vega.at<double>(1, 3), output_Marker1_2Vega.at<double>(2, 3);
//		std::cout << "Translation Vector: " << translationOutput.transpose() << std::endl;
//		std::cout << std::endl;
//		cv::Mat rotationMatrix = output_Marker1_2Vega(cv::Rect(0, 0, 3, 3));
//		Vector3d eulerAnglesOutput = rotationMatrixToEulerAngles(rotationMatrix);
//		std::cout << "Euler Angles (degrees): " << eulerAnglesOutput.transpose() << std::endl;
//		std::cout << std::endl;
//
//		//真实的器械位姿
//		std::cout << "truth_Marker1_2Vega" << OMC.Marker1_2Vega[i] << std::endl;
//		std::cout << std::endl;
//		std::cout << "Translation: " << OMC.t_Marker1_2Vega[i] << std::endl;
//		std::cout << std::endl;
//		Vector3d translationTruth;
//		translationTruth << OMC.Marker1_2Vega[i].at<double>(0, 3), OMC.Marker1_2Vega[i].at<double>(1, 3), OMC.Marker1_2Vega[i].at<double>(2, 3);
//		Vector3d eulerAnglesTruth = rotationMatrixToEulerAngles(OMC.R_Marker1_2Vega[i]);
//		std::cout << "Euler Angles (degrees): " << eulerAnglesTruth.transpose() << std::endl;
//		std::cout << std::endl;
//
//		//计算平移误差和旋转误差
//		double DistanceError = (translationOutput - translationTruth).norm();
//		std::cout << "DistanceError " << DistanceError << std::endl;
//		TranslationError.push_back(DistanceError);
//		double AngleError = (eulerAnglesOutput - eulerAnglesTruth).norm();
//		RotationError.push_back(AngleError);
//		std::cout << "AngleError " << AngleError << std::endl;
//
//	}
//
//	//// 计算 TranslationError 的Mean/Min/Max
//	double translationErrorMean = std::accumulate(TranslationError.begin(), TranslationError.end(), 0.0) / TranslationError.size();
//	std::cout << "Average Translation Error: " << translationErrorMean << std::endl;
//	double maxTranslationError = *std::max_element(TranslationError.begin(), TranslationError.end());
//	double minTranslationError = *std::min_element(TranslationError.begin(), TranslationError.end());
//	std::cout << "Max Translation Error: " << maxTranslationError << std::endl;
//	std::cout << "Min Translation Error: " << minTranslationError << std::endl;
//
//	// 计算 RotationError 的Mean/Min/Max
//	double rotationErrorMean = std::accumulate(RotationError.begin(), RotationError.end(), 0.0) / RotationError.size();
//	std::cout << "Average Rotation Error: " << rotationErrorMean << std::endl;
//	double maxRotationError = *std::max_element(RotationError.begin(), RotationError.end());
//	double minRotationError = *std::min_element(RotationError.begin(), RotationError.end());
//	std::cout << "Max Rotation Error: " << maxRotationError << std::endl;
//	std::cout << "Min Rotation Error: " << minRotationError << std::endl;
//
//
//
//}

void ErrorMetric2(const OMCalibrate& OMC, const cv::Mat RecordEMsensor_2Marker1, cv::Mat RecordEM_2Marker2) {
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
		//Vector3d eulerAnglesOutput = rotationMatrixToEulerAngles(rotationMatrix);
		//std::cout << "Euler Angles (degrees): " << eulerAnglesOutput.transpose() << std::endl;
		std::cout << std::endl;
		cv::Mat rotationMatrix_inv;
		cv::invert(rotationMatrix, rotationMatrix_inv, cv::DECOMP_SVD);


		//真实的器械位姿
		std::cout << "truth_Marker1_2Vega" << OMC.Marker1_2Vega[i] << std::endl;
		std::cout << std::endl;
		std::cout << "Translation: " << OMC.t_Marker1_2Vega[i] << std::endl;
		std::cout << std::endl;
		Vector3d translationTruth;
		translationTruth << OMC.Marker1_2Vega[i].at<double>(0, 3), OMC.Marker1_2Vega[i].at<double>(1, 3), OMC.Marker1_2Vega[i].at<double>(2, 3);
		//Vector3d eulerAnglesTruth = rotationMatrixToEulerAngles(OMC.R_Marker1_2Vega[i]);
		//std::cout << "Euler Angles (degrees): " << eulerAnglesTruth.transpose() << std::endl;
		std::cout << std::endl;

		//计算平移误差和旋转误差
		double DistanceError = (translationOutput - translationTruth).norm();
		std::cout << "DistanceError " << DistanceError << std::endl;
		TranslationError.push_back(DistanceError);

		cv::Mat delta_R;
		delta_R = OMC.R_Marker1_2Vega[i] * rotationMatrix_inv;

		std::cout << "delta_R[" << i << "] = " << std::endl << delta_R << std::endl;

		double angleError = rotationMatrixToAngleError(delta_R);
		RotationError.push_back(angleError);

		std::cout << "AngleError " << angleError << std::endl;

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


void ErrorCalculate(const OMCalibrate& OMC, std::vector<cv::Mat> estimate) {

	std::vector<double> TranslationError;
	std::vector<double> RotationError;

	for (int i = 0; i < OMC.R_Marker1_2Vega.size(); i++) {
	
		//计算的器械位姿分解
		Vector3d translationOutput;
		translationOutput << estimate[i].at<double>(0, 3), estimate[i].at<double>(1, 3), estimate[i].at<double>(2, 3);
		cv::Mat rotationMatrix = estimate[i](cv::Rect(0, 0, 3, 3));
		cv::Mat rotationMatrix_inv;
		cv::invert(rotationMatrix, rotationMatrix_inv, cv::DECOMP_SVD);

		//真实的器械位姿
		Vector3d translationTruth;
		translationTruth << OMC.Marker1_2Vega[i].at<double>(0, 3), OMC.Marker1_2Vega[i].at<double>(1, 3), OMC.Marker1_2Vega[i].at<double>(2, 3);

		//计算平移误差和旋转误差
		double DistanceError = (translationOutput - translationTruth).norm();
		//std::cout << "DistanceError[" << i << "] = " << DistanceError << std::endl;
		TranslationError.push_back(DistanceError);

		cv::Mat delta_R;
		delta_R = OMC.R_Marker1_2Vega[i] * rotationMatrix_inv;

		//std::cout << "delta_R[" << i << "] = " << std::endl << delta_R << std::endl;

		double angleError = rotationMatrixToAngleError(delta_R);
		RotationError.push_back(angleError);

		//std::cout << "AngleError " << angleError << std::endl;
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

void CompleteEstimate(const OMCalibrate& OMC, const cv::Mat RecordEMsensor_2Marker1, cv::Mat RecordEM_2Marker2, std::vector<std::vector<std::vector<double>>>& M1_markersEst) {
	//生成估计的Marker坐标

	std::vector<std::vector<double>> tool = { {0.0,0.0,0.0 },{0.0,28.59,41.02},{0.0,0.0,88.00},{0.0,-44.32,40.45} };
	
	cv::Mat RecordMarker1_2EMsensor;
	cv::invert(RecordEMsensor_2Marker1, RecordMarker1_2EMsensor, cv::DECOMP_SVD);

	cv::Mat RecordMarker2_2EM;
	cv::invert(RecordEM_2Marker2, RecordMarker2_2EM, cv::DECOMP_SVD);

	cv::Mat Marker1_2EMsensor;
	cv::invert(OMC.EMsensor_2Marker1, Marker1_2EMsensor, cv::DECOMP_SVD);

	cv::Mat output_Marker1_2Vega;

	for (int i = 0; i < OMC.R_Marker1_2Vega.size(); i++) {
		output_Marker1_2Vega = OMC.Marker2_2Vega[i] * RecordEM_2Marker2 * OMC.EMsensor_2Aurora[i] * RecordMarker1_2EMsensor;

		std::vector<std::vector<double>> oneGroup;

		for (const auto& pt : tool) {
			cv::Mat point = (cv::Mat_<double>(4, 1) << pt[0], pt[1], pt[2], 1.0);
			cv::Mat transformed = output_Marker1_2Vega * point;

			std::vector<double> transformedPoint = {
				transformed.at<double>(0, 0),
				transformed.at<double>(1, 0),
				transformed.at<double>(2, 0)
			};

			oneGroup.push_back(transformedPoint);
		}

		M1_markersEst.push_back(oneGroup);
	}

	
}

void CreateMixedMarkersN(const OMCalibrate& OMC,
	const std::vector<std::vector<std::vector<double>>>& M1_markersEst,
	std::vector<std::vector<std::vector<double>>>& M1_mixedMarkers,
	int numToReplace) // 替换个数：1, 2, or 3
{
	//随机用M1_markersEst的1/2/3个Marker取代OMC.M1_markers的相应位置，并另存在3个变量中
	std::random_device rd;
	std::mt19937 gen(rd());
	int groupCount = std::min((int)OMC.M1_markers.size(), (int)M1_markersEst.size());

	M1_mixedMarkers = OMC.M1_markers;

	for (int i = 0; i < groupCount; ++i) {
		// marker indices: [0, 1, 2, 3]
		std::vector<int> indices = { 0, 1, 2, 3 };
		std::shuffle(indices.begin(), indices.end(), gen);

		for (int k = 0; k < numToReplace; ++k) {
			int j = indices[k]; // 随机选中的marker index
			M1_mixedMarkers[i][j] = M1_markersEst[i][j];
		}
	}

}


vtkSmartPointer<vtkPoints> Convert2VtkPoints(const std::vector<std::vector<double>>& tool) {
	// 创建 vtkPoints 对象
	vtkSmartPointer<vtkPoints> Points;
	Points = vtkSmartPointer<vtkPoints>::New();

	// 遍历 std::vector<std::vector<double>> 并插入点
	for (const auto& point : tool) {
		if (point.size() == 3) { // 确保每个点有3个坐标
			Points->InsertNextPoint(point[0], point[1], point[2]);
		}
		else {
			// 处理错误情况：点的维度不是3
			std::cerr << "Point does not have 3 coordinates!" << std::endl;
		}
	}

	return Points; // 返回 vtkSmartPointer<vtkPoints>
}



void RegisterMixedMarkersToTool(
	const std::vector<std::vector<std::vector<double>>>& mixedMarkers,
	const std::vector<std::vector<double>>& toolTemplate,
	std::vector<cv::Mat>& outputTransforms)
{
	outputTransforms.clear(); // 清空原有数据

	// 固定工具坐标系转换为 vtkPoints
	vtkSmartPointer<vtkPoints> vtkToolPoints = Convert2VtkPoints(toolTemplate);

	for (const auto& group : mixedMarkers) {
		// 当前目标 marker 转换为 vtkPoints
		vtkSmartPointer<vtkPoints> vtkTargetPoints = Convert2VtkPoints(group);

		// 配准
		ToolRegi tool_Regi;
		tool_Regi.Tool = vtkToolPoints;
		tool_Regi.Target = vtkTargetPoints;
		tool_Regi.Register();

		// 获取 vtkMatrix4x4 并转成 cv::Mat
		vtkMatrix4x4* vtkMat = tool_Regi.GetMatrix();
		cv::Mat cvMat(4, 4, CV_64F);
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				cvMat.at<double>(r, c) = vtkMat->GetElement(r, c);
			}
		}

		outputTransforms.push_back(cvMat);
	}

}


cv::Mat RegisterPoints(const std::vector<std::vector<double>>& source,
	const std::vector<std::vector<double>>& target) {
	assert(source.size() == target.size());
	int N = source.size();

	Eigen::MatrixXd src(3, N), tgt(3, N);
	for (int i = 0; i < N; ++i) {
		src(0, i) = source[i][0];
		src(1, i) = source[i][1];
		src(2, i) = source[i][2];
		tgt(0, i) = target[i][0];
		tgt(1, i) = target[i][1];
		tgt(2, i) = target[i][2];
	}

	Eigen::Vector3d src_centroid = src.rowwise().mean();
	Eigen::Vector3d tgt_centroid = tgt.rowwise().mean();

	src.colwise() -= src_centroid;
	tgt.colwise() -= tgt_centroid;

	Eigen::Matrix3d H = src * tgt.transpose();

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

	if (R.determinant() < 0) {
		Eigen::Matrix3d V = svd.matrixV();
		V.col(2) *= -1;
		R = V * svd.matrixU().transpose();
	}

	Eigen::Vector3d t = tgt_centroid - R * src_centroid;

	// 转换为 cv::Mat
	cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			T.at<double>(r, c) = R(r, c);
		}
		T.at<double>(r, 3) = t(r);
	}

	return T;
}

int main32() {
	OMCalibrate OMC;

	OMC.HandeyeCalibrate3();
	std::cout << "EMsensor_2Marker1" << OMC.EMsensor_2Marker1 << std::endl;
	std::cout << std::endl;
	std::cout << "EM_2Marker2" << OMC.EM_2Marker2 << std::endl;
	std::cout << std::endl;

	return 0;
}




int main33()
{
	OMCalibrate OMC;
	//std::string filename = "D:/Optomagnetic-tracking/CombinedAPIsample_v2/Vega_Collected_data.csv";
	//OMC.ReadtRecordData(filename,12,9);
	//OMC.PrintTrackedData();
	
	//测试读取csv的markers坐标
	//for (size_t i = 0; i < OMC.M1_markers.size(); ++i) {
	//	std::cout << "Group " << i << ":\n";
	//	for (size_t j = 0; j < OMC.M1_markers[i].size(); ++j) {
	//		std::cout << "Marker " << j << ": ";
	//		for (double val : OMC.M1_markers[i][j]) {
	//			std::cout << val << " ";
	//		}
	//		std::cout << std::endl;
	//	}
	//}

	OMC.HandeyeCalibrate3();
	std::cout << "Aurora2Vega" << OMC.Aurora2Vega << std::endl;
	std::cout << std::endl;
	std::cout << "EMsensor_2Marker1" << OMC.EMsensor_2Marker1 << std::endl;
	std::cout << std::endl;
	std::cout << "EM_2Marker2" << OMC.EM_2Marker2 << std::endl;
	std::cout << std::endl;


	cv::Mat RecordEMsensor_2Marker1 = (cv::Mat_<double>(4, 4) <<
		0.9717 ,- 0.1995   , 0.1266 ,- 19.7548,
		- 0.1093  ,  0.0952  ,  0.9894  ,  2.9128,
		- 0.2094, - 0.9753 ,   0.0707   ,38.9164,
		0     ,    0   ,      0   , 1.0000
		);
	cv::Mat RecordEM_2Marker2 = (cv::Mat_<double>(4, 4) <<
		0.7185 , - 0.0045 ,- 0.6953 ,  67.8992,
		- 0.6953, - 0.0213, - 0.7182, - 3.8716,
		- 0.0117  ,  0.9996 ,- 0.0184 ,- 137.7597,
		0.0000 , 0.0000 , 0.0000  ,  1.0000
		);


	std::vector<std::vector<double>> tool = {
	{0.0, 0.0, 0.0},
	{0.0, 28.59, 41.02},
	{0.0, 0.0, 88.00},
	{0.0, -44.32, 40.45}
	};

	

	ErrorMetric2(OMC, RecordEMsensor_2Marker1, RecordEM_2Marker2);

	std::vector<std::vector<std::vector<double>>> M1_markersEst;
	CompleteEstimate(OMC, RecordEMsensor_2Marker1, RecordEM_2Marker2, M1_markersEst);

	std::vector<std::vector<std::vector<double>>> M1_mixedMarkers_1; // 每组替换1个
	std::vector<std::vector<std::vector<double>>> M1_mixedMarkers_2; // 每组替换2个
	std::vector<std::vector<std::vector<double>>> M1_mixedMarkers_3; // 每组替换3个

	CreateMixedMarkersN(OMC, M1_markersEst, M1_mixedMarkers_1, 1);
	CreateMixedMarkersN(OMC, M1_markersEst, M1_mixedMarkers_2, 2);
	CreateMixedMarkersN(OMC, M1_markersEst, M1_mixedMarkers_3, 3);


	//检查生成遮挡是否正确
	for (int i = 0; i < 5 && i < OMC.M1_markers.size() && i < M1_markersEst.size(); ++i) {
		std::cout << "==== Group " << i << " ====\n";

		for (int j = 0; j < 4; ++j) {
			const auto& original = OMC.M1_markers[i][j];
			const auto& estimated = M1_markersEst[i][j];
			const auto& mixed1 = M1_mixedMarkers_1[i][j];
			const auto& mixed2 = M1_mixedMarkers_2[i][j];
			const auto& mixed3 = M1_mixedMarkers_3[i][j];

			std::cout << "Marker " << j << ":\n";
			std::cout << "  Original : (" << original[0] << ", " << original[1] << ", " << original[2] << ")\n";
			std::cout << "  Estimated: (" << estimated[0] << ", " << estimated[1] << ", " << estimated[2] << ")\n";
			std::cout << "  Mixed-1  : (" << mixed1[0] << ", " << mixed1[1] << ", " << mixed1[2] << ")\n";
			std::cout << "  Mixed-2  : (" << mixed2[0] << ", " << mixed2[1] << ", " << mixed2[2] << ")\n";
			std::cout << "  Mixed-3  : (" << mixed3[0] << ", " << mixed3[1] << ", " << mixed3[2] << ")\n";
		}

		std::cout << std::endl;
	}

	std::vector<cv::Mat> M1_mixedMarkers_1_Transforms;
	//RegisterMixedMarkersToTool(M1_mixedMarkers_1, tool, M1_mixedMarkers_1_Transforms);

	std::cout << " 1个Marker遮挡" << endl;
	for (const auto& markerSet : M1_mixedMarkers_1) {
		cv::Mat T = RegisterPoints(tool, markerSet);
		M1_mixedMarkers_1_Transforms.push_back(T);
	}

	//for (size_t i = 0; i < std::min(OMC.Marker1_2Vega.size(), M1_mixedMarkers_1_Transforms.size()); ++i) {
	//	std::cout << "===== Group " << i << " =====" << std::endl;

	//	std::cout << "Original (OMC.Marker1_2Vega):" << std::endl;
	//	for (int r = 0; r < 4; ++r) {
	//		for (int c = 0; c < 4; ++c) {
	//			std::cout << std::fixed << std::setprecision(4)
	//				<< OMC.Marker1_2Vega[i].at<double>(r, c) << " ";
	//		}
	//		std::cout << std::endl;
	//	}

	//	std::cout << "Estimated (M1_mixedMarkers_1_Transforms):" << std::endl;
	//	for (int r = 0; r < 4; ++r) {
	//		for (int c = 0; c < 4; ++c) {
	//			std::cout << std::fixed << std::setprecision(4)
	//				<< M1_mixedMarkers_1_Transforms[i].at<double>(r, c) << " ";
	//		}
	//		std::cout << std::endl;
	//	}

	//	std::cout << std::endl;
	//}

	ErrorCalculate(OMC, M1_mixedMarkers_1_Transforms);

	std::cout << " 2个Marker遮挡" << endl;
	std::vector<cv::Mat> M1_mixedMarkers_2_Transforms;
	//RegisterMixedMarkersToTool(M1_mixedMarkers_1, tool, M1_mixedMarkers_1_Transforms);

	for (const auto& markerSet : M1_mixedMarkers_2) {
		cv::Mat T = RegisterPoints(tool, markerSet);
		M1_mixedMarkers_2_Transforms.push_back(T);
	}
	ErrorCalculate(OMC, M1_mixedMarkers_2_Transforms);

	std::cout << " 3个Marker遮挡" << endl;
	std::vector<cv::Mat> M1_mixedMarkers_3_Transforms;
	//RegisterMixedMarkersToTool(M1_mixedMarkers_1, tool, M1_mixedMarkers_1_Transforms);

	for (const auto& markerSet : M1_mixedMarkers_3) {
		cv::Mat T = RegisterPoints(tool, markerSet);
		M1_mixedMarkers_3_Transforms.push_back(T);
	}
	ErrorCalculate(OMC, M1_mixedMarkers_3_Transforms);

	



	//for (int i = 0; i < OMC.Marker1_2Vega.size(); i++) {
	//	//test_Marker2_2Vega = OMC.createTransformationMatrix(OMC.R_Marker2_2Vega[i], OMC.t_Marker2_2Vega[i]);
	//	//test_EMsensor_2Aurora = OMC.createTransformationMatrix(OMC.R_EMsensor_2Aurora[i], OMC.t_EMsensor_2Aurora[i]);

	//	//output_Marker1_2Vega = test_Marker2_2Vega * OMC.EM_2Marker2*test_EMsensor_2Aurora*Marker1_2EMsensor;

	//	//output_Marker1_2Vega2 =  OMC.Aurora2Vega*test_EMsensor_2Aurora*Marker1_2EMsensor;

	//	//验证在标定数据集的误差
	//	//output_Marker1_2Vega = OMC.Aurora2Vega * OMC.EMsensor_2Aurora[i] * Marker1_2EMsensor;
	//	//output_Marker1_2Vega = OMC.Marker2_2Vega[i]*OMC.EM_2Marker2 * OMC.EMsensor_2Aurora[i] * Marker1_2EMsensor;


	//	//验证标定的固定变换矩阵在其他数据集可用
	//	Marker1_2Aurora_Path1 = OMC.EMsensor_2Aurora[i] * RecordMarker1_2EMsensor;
	//	Marker1_2Aurora_Path2 = RecordMarker2_2EM *OMC.Vega_2Marker2[i]*OMC.Marker1_2Vega[i];


	//	//Marker1_2Aurora_Path1位姿
	//	std::cout << "Group" << i + 1 << std::endl;
	//	std::cout << std::endl;
	//	std::cout << "Marker1_2Aurora_Path1" << Marker1_2Aurora_Path1 << std::endl;
	//	std::cout << std::endl;
	//	Vector3d translationOutput1;
	//	translationOutput1 << Marker1_2Aurora_Path1.at<double>(0, 3), Marker1_2Aurora_Path1.at<double>(1, 3), Marker1_2Aurora_Path1.at<double>(2, 3);
	//	T1.push_back(translationOutput1);
	//	std::cout << "Translation Vector: " << translationOutput1.transpose() << std::endl;
	//	std::cout << std::endl;
	//	cv::Mat rotationMatrix1 = Marker1_2Aurora_Path1(cv::Rect(0, 0, 3, 3));
	//	Vector3d eulerAnglesOutput1 = rotationMatrixToEulerAngles(rotationMatrix1);
	//	std::cout << "Euler Angles (degrees): " << eulerAnglesOutput1.transpose() << std::endl;
	//	std::cout << std::endl;

	//	//Marker1_2Aurora_Path2位姿
	//	std::cout << "Group" << i + 1 << std::endl;
	//	std::cout << std::endl;
	//	std::cout << "Marker1_2Aurora_Path2" << Marker1_2Aurora_Path2 << std::endl;
	//	std::cout << std::endl;
	//	Vector3d translationOutput2;
	//	translationOutput2 << Marker1_2Aurora_Path2.at<double>(0, 3), Marker1_2Aurora_Path2.at<double>(1, 3), Marker1_2Aurora_Path2.at<double>(2, 3);
	//	T2.push_back(translationOutput2);
	//	std::cout << "Translation Vector: " << translationOutput2.transpose() << std::endl;
	//	std::cout << std::endl;
	//	cv::Mat rotationMatrix2 = Marker1_2Aurora_Path2(cv::Rect(0, 0, 3, 3));
	//	Vector3d eulerAnglesOutput2 = rotationMatrixToEulerAngles(rotationMatrix2);
	//	std::cout << "Euler Angles (degrees): " << eulerAnglesOutput2.transpose() << std::endl;

	//	//计算平移误差和旋转误差
	//	double DistanceError = (translationOutput1 - translationOutput2).norm();
	//	std::cout << "DistanceError " << DistanceError << std::endl;
	//	TranslationError.push_back(DistanceError);
	//	double AngleError = (eulerAnglesOutput1 - eulerAnglesOutput2).norm();
	//	RotationError.push_back(AngleError);
	//	std::cout << "AngleError " << AngleError << std::endl;

	//}
	//

 //   std::cout << std::endl;

	//// 计算 TranslationError 的Mean/Min/Max
	//double translationErrorMean = std::accumulate(TranslationError.begin(), TranslationError.end(), 0.0) / TranslationError.size();
	//std::cout << "Average Translation Error: " << translationErrorMean << std::endl;
	//double maxTranslationError = *std::max_element(TranslationError.begin(), TranslationError.end());
	//double minTranslationError = *std::min_element(TranslationError.begin(), TranslationError.end());
	//std::cout << "Max Translation Error: " << maxTranslationError << std::endl;
	//std::cout << "Min Translation Error: " << minTranslationError << std::endl;

	//// 计算 RotationError 的Mean/Min/Max
	//double rotationErrorMean = std::accumulate(RotationError.begin(), RotationError.end(), 0.0) / RotationError.size();
	//std::cout << "Average Rotation Error: " << rotationErrorMean << std::endl;
	//double maxRotationError = *std::max_element(RotationError.begin(), RotationError.end());
	//double minRotationError = *std::min_element(RotationError.begin(), RotationError.end());
	//std::cout << "Max Rotation Error: " << maxRotationError << std::endl;
	//std::cout << "Min Rotation Error: " << minRotationError << std::endl;


	//std::cout << std::endl;
	//std::cout << "T1 size"<<T1.size()<<std::endl;


	////visualizePointSets(T1, T2);
	//

	//// 配准结果
	//Matrix3d R;
	//Vector3d t;

	//// 执行配准
	//pointSetRegistration(T1, T2, R, t);

	//// 输出结果
	//std::cout << "Rotation Matrix R:\n" << R << "\n";
	//std::cout << "Translation Vector t:\n" << t.transpose() << "\n";

	//// 计算误差
	//calculateRegistrationError(T1, T2, R, t);


	//std::vector<cv::Mat> t_Vega_2Marker1;
	//std::vector<cv::Mat> t_Marker1_2Vega;
	//std::vector<cv::Mat> t_Aurora_2EMSensor;
	//std::vector<cv::Mat> t_EMsensor_2Aurora;
	//// 调用函数提取平移向量并存储到 t_Marker1_2Vega
	//extractTranslationVectorsToMat(OMC.Vega_2Marker1, t_Vega_2Marker1);
	//extractTranslationVectorsToMat(OMC.Marker1_2Vega, t_Marker1_2Vega);
	//extractTranslationVectorsToMat(OMC.Aurora_2EMsensor, t_Aurora_2EMSensor);
	//extractTranslationVectorsToMat(OMC.EMsensor_2Aurora, t_EMsensor_2Aurora);

	//// 打印提取的平移向量
	//for (size_t i = 0; i < t_Vega_2Marker1.size(); ++i) {
	//	std::cout << "t_Vega_2Marker1 " << i << " : \n" << t_Vega_2Marker1[i] << std::endl;
	//}

	////// 旋转矩阵和平移向量
	////cv::Mat rotationMatrix, translationVector;

	////// 计算刚性变换
	////if (computeRigidTransform(t_Vega_2Marker1, t_Aurora_2EMSensor, rotationMatrix, translationVector)) {
	////	std::cout << "Rotation Matrix: \n" << rotationMatrix << std::endl;
	////	std::cout << "Translation Vector: \n" << translationVector << std::endl;
	////}
	////else {
	////	std::cerr << "Failed to compute the rigid transform." << std::endl;
	////}

	// // 应用变换
	//std::vector<cv::Mat> transformedPoints = applyTransformToVectors(t_Aurora_2EMSensor, RecordEMsensor_2Marker1);
	//// 打印提取的平移向量
	//for (size_t i = 0; i < transformedPoints.size(); ++i) {
	//	std::cout << "t_Aurora_2EMSensor_convert " << i << " : \n" << transformedPoints[i] << std::endl;
	//}

	std::cin.get();
	return 0;
}


