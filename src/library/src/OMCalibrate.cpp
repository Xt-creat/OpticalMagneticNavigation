#include "OMCalibrate.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <numeric>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

OMCalibrate::OMCalibrate()
{
}

OMCalibrate::OMCalibrate(const std::string& path, int group)
{
	LoadData(path, group);
}

OMCalibrate::~OMCalibrate()
{
}

void OMCalibrate::LoadData(const std::string& path, int group)
{
	std::string filename1 = path + "/Vega.csv";
	std::string filename2 = path + "/Aurora.csv";
	
	M1_data.clear();
	M2_data.clear();
	EMSensor_data.clear();

	ReadRecordData(this->M1_data, filename1, group, 10, 9);
	Quat2Matrices(M1_data);

	ReadRecordData(this->M2_data, filename1, group, 10, 73);
	Quat2Matrices(M2_data);

	ReadRecordData(this->EMSensor_data, filename2, group, 10, 9);
	Quat2Matrices(EMSensor_data);

	saveTrackedDataToCsv(M1_data, path + "/Processed_M1.csv");
	saveTrackedDataToCsv(M2_data, path + "/Processed_M2.csv");
	saveTrackedDataToCsv(EMSensor_data, path + "/Processed_EMSensor.csv");

	PrepareCalibration();

	// 保存中间变换矩阵到 txt 文件
	saveVectorMatToTxt(Marker1_2Vega, path + "/Marker1_2Vega.txt");
	saveVectorMatToTxt(Marker2_2Vega, path + "/Marker2_2Vega.txt");
	saveVectorMatToTxt(EMsensor_2Aurora, path + "/EMsensor_2Aurora.txt");
	saveVectorMatToTxt(Aurora_2EMsensor, path + "/Aurora_2EMsensor.txt");
	saveVectorMatToTxt(Marker2_2Marker1, path + "/Marker2_2Marker1.txt");
	saveVectorMatToTxt(Marker1_2Marker2, path + "/Marker1_2Marker2.txt");
}

void OMCalibrate::PrepareCalibration()
{
	R_Marker1_2Vega.clear();
	t_Marker1_2Vega.clear();
	R_Marker2_2Vega.clear();
	t_Marker2_2Vega.clear();
	R_EMsensor_2Aurora.clear();
	t_EMsensor_2Aurora.clear();

	GetRt();

	Marker1_2Vega.clear();
	Marker2_2Vega.clear();
	EMsensor_2Aurora.clear();
	Vega_2Marker1.clear();
	Vega_2Marker2.clear();
	Aurora_2EMsensor.clear();

	for (int i = 0; i < (int)R_Marker1_2Vega.size(); i++) {
		Marker1_2Vega.push_back(createTransformationMatrix(R_Marker1_2Vega[i], t_Marker1_2Vega[i]));
		if (i < (int)R_Marker2_2Vega.size())
			Marker2_2Vega.push_back(createTransformationMatrix(R_Marker2_2Vega[i], t_Marker2_2Vega[i]));
		if (i < (int)R_EMsensor_2Aurora.size())
			EMsensor_2Aurora.push_back(createTransformationMatrix(R_EMsensor_2Aurora[i], t_EMsensor_2Aurora[i]));
	}

	for (int i = 0; i < (int)Marker1_2Vega.size(); i++) {
		cv::Mat invert_temp, invert_temp1, invert_temp2;
		cv::invert(Marker1_2Vega[i], invert_temp, cv::DECOMP_SVD);
		Vega_2Marker1.push_back(invert_temp);

		if (i < (int)Marker2_2Vega.size()) {
			cv::invert(Marker2_2Vega[i], invert_temp1, cv::DECOMP_SVD);
			Vega_2Marker2.push_back(invert_temp1);
		}
		if (i < (int)EMsensor_2Aurora.size()) {
			cv::invert(EMsensor_2Aurora[i], invert_temp2, cv::DECOMP_SVD);
			Aurora_2EMsensor.push_back(invert_temp2);
		}
	}

	Marker2_2Marker1.clear();
	R_Marker2_2Marker1.clear();
	t_Marker2_2Marker1.clear();
	Marker1_2Marker2.clear();

	for (int i = 0; i < (int)Marker1_2Vega.size(); i++) {
		if (i >= (int)Vega_2Marker1.size() || i >= (int)Marker2_2Vega.size()) break;
		cv::Mat Rt = Vega_2Marker1[i] * Marker2_2Vega[i];
		cv::Mat R = Rt(cv::Range(0, 3), cv::Range(0, 3));
		cv::Mat t = Rt(cv::Range(0, 3), cv::Range(3, 4));

		Marker2_2Marker1.push_back(Rt);
		R_Marker2_2Marker1.push_back(R);
		t_Marker2_2Marker1.push_back(t);

		if (i < (int)Vega_2Marker2.size()) {
			cv::Mat Rt2 = Vega_2Marker2[i] * Marker1_2Vega[i];
			Marker1_2Marker2.push_back(Rt2);
		}
	}
}

void OMCalibrate::RemoveGroup(int index)
{
	if (index >= 0 && index < (int)M1_data.size()) {
		M1_data.erase(M1_data.begin() + index);
	}
	if (index >= 0 && index < (int)M2_data.size()) {
		M2_data.erase(M2_data.begin() + index);
	}
	if (index >= 0 && index < (int)EMSensor_data.size()) {
		EMSensor_data.erase(EMSensor_data.begin() + index);
	}
	PrepareCalibration();
}

static double rotationMatrixToAngleError(const cv::Mat& rotationMatrix) {
	if (rotationMatrix.empty() || rotationMatrix.rows != 3 || rotationMatrix.cols != 3) return 0.0;
	Eigen::Matrix3d R;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			R(i, j) = rotationMatrix.at<double>(i, j);
	Eigen::AngleAxisd angleAxis(R);
	return angleAxis.angle() * (180.0 / M_PI);
}

CalibrationError OMCalibrate::EvaluateCalibration()
{
	CalibrationError error = {0, 0, {}, {}};
	if (Marker1_2Vega.empty() || EMsensor_2Marker1.empty() || EM_2Marker2.empty()) return error;

	cv::Mat RecordMarker1_2EMsensor;
	try { cv::invert(EMsensor_2Marker1, RecordMarker1_2EMsensor, cv::DECOMP_SVD); } catch (...) { return error; }

	for (int i = 0; i < (int)Marker1_2Vega.size(); i++) {
		if (i >= (int)Marker2_2Vega.size() || i >= (int)EMsensor_2Aurora.size()) break;
		if (Marker2_2Vega[i].empty() || EMsensor_2Aurora[i].empty()) continue;

		cv::Mat output_Marker1_2Vega = Marker2_2Vega[i] * EM_2Marker2 * EMsensor_2Aurora[i] * RecordMarker1_2EMsensor;
		if (output_Marker1_2Vega.empty() || output_Marker1_2Vega.rows < 4) continue;

		Eigen::Vector3d tOut(output_Marker1_2Vega.at<double>(0, 3), output_Marker1_2Vega.at<double>(1, 3), output_Marker1_2Vega.at<double>(2, 3));
		Eigen::Vector3d tTruth(Marker1_2Vega[i].at<double>(0, 3), Marker1_2Vega[i].at<double>(1, 3), Marker1_2Vega[i].at<double>(2, 3));
		
		double distErr = (tOut - tTruth).norm();
		error.translationErrors.push_back(distErr);

		cv::Mat rOut = output_Marker1_2Vega(cv::Rect(0, 0, 3, 3));
		cv::Mat rTruth = Marker1_2Vega[i](cv::Rect(0, 0, 3, 3));
		cv::Mat rOutInv;
		cv::invert(rOut, rOutInv, cv::DECOMP_SVD);
		error.rotationErrors.push_back(rotationMatrixToAngleError(rTruth * rOutInv));
	}

	if (!error.translationErrors.empty()) {
		error.avgTranslationError = std::accumulate(error.translationErrors.begin(), error.translationErrors.end(), 0.0) / error.translationErrors.size();
		error.avgRotationError = std::accumulate(error.rotationErrors.begin(), error.rotationErrors.end(), 0.0) / error.rotationErrors.size();
	}
	return error;
}

void OMCalibrate::HandeyeCalibrate3()
{
	if (R_Marker2_2Marker1.empty() || R_EMsensor_2Aurora.empty()) return;

	std::vector<cv::Mat> R_EM2EMS, t_EM2EMS;
	for (int i = 0; i < (int)R_EMsensor_2Aurora.size(); i++) {
		cv::Mat R_inv;
		cv::invert(R_EMsensor_2Aurora[i], R_inv, cv::DECOMP_SVD);
		R_EM2EMS.push_back(R_inv);
		t_EM2EMS.push_back(-R_inv * t_EMsensor_2Aurora[i]);
	}

	cv::Mat R_EM2M2, t_EM2M2, R_EMS2M1, t_EMS2M1;
	try {
		cv::calibrateRobotWorldHandEye(R_Marker2_2Marker1, t_Marker2_2Marker1, R_EM2EMS, t_EM2EMS,
			R_EM2M2, t_EM2M2, R_EMS2M1, t_EMS2M1, cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH);
	} catch (...) { return; }

	EM_2Marker2 = createTransformationMatrix(R_EM2M2, t_EM2M2);
	EMsensor_2Marker1 = createTransformationMatrix(R_EMS2M1, t_EMS2M1);
	cv::invert(EMsensor_2Marker1, Marker1_2EMsensor, cv::DECOMP_SVD);
}

void OMCalibrate::GetRt()
{
	for (int i = 0; i < (int)this->M1_data.size();i++) {
		R_Marker1_2Vega.push_back(this->M1_data[i].R);
		t_Marker1_2Vega.push_back(cv::Mat(3, 1, CV_64F, this->M1_data[i].t).clone());

		if(i < (int)M2_data.size()) {
			R_Marker2_2Vega.push_back(M2_data[i].R);
			t_Marker2_2Vega.push_back(cv::Mat(3, 1, CV_64F, M2_data[i].t).clone());
		}
		if(i < (int)EMSensor_data.size()) {
			R_EMsensor_2Aurora.push_back(EMSensor_data[i].R);
			t_EMsensor_2Aurora.push_back(cv::Mat(3, 1, CV_64F, EMSensor_data[i].t).clone());
		}
	}
}

void OMCalibrate::ReadRecordData(std::vector<TrackedData>& m_data,const std::string& filename,int groups,int interval, int start) {
	std::ifstream file(filename);
	std::string line;
	if (!file.is_open()) return;

	for (int i = 0; i < 2; ++i) std::getline(file, line); // 跳过表头

	int dataCount = 0;
	while (dataCount < groups) {
		std::vector<TrackedData> groupRawData;
		for (int i = 0; i < interval; ++i) {
			if (!std::getline(file, line)) break;
			std::stringstream ss(line);
			std::string cell;
			TrackedData data;
			int column = 0;
			bool valid = false;
			while (std::getline(ss, cell, ',')) {
				if (column == start - 2) { if (cell == "Enabled" || cell == "OK") valid = true; else break; }
				try {
					if (column >= start-1 && column < start+3) data.quat[column - start+1] = std::stod(cell);
					else if (column >= start + 3 && column < start+6) data.t[column - start - 3] = std::stod(cell);
				} catch (...) { valid = false; break; }
				column++;
			}
			if (valid) groupRawData.push_back(data);
		}
		if (groupRawData.size() < 6) { dataCount++; continue; }
		double cX = 0, cY = 0, cZ = 0;
		for (const auto& d : groupRawData) { cX += d.t[0]; cY += d.t[1]; cZ += d.t[2]; }
		cX /= groupRawData.size(); cY /= groupRawData.size(); cZ /= groupRawData.size();
		struct IndexedError { int index; double dist; };
		std::vector<IndexedError> errors;
		for (int i = 0; i < (int)groupRawData.size(); ++i) {
			double dx = groupRawData[i].t[0] - cX, dy = groupRawData[i].t[1] - cY, dz = groupRawData[i].t[2] - cZ;
			errors.push_back({ i, std::sqrt(dx*dx + dy*dy + dz*dz) });
		}
		std::sort(errors.begin(), errors.end(), [](const IndexedError& a, const IndexedError& b) { return a.dist < b.dist; });
		int keepCount = 6;
		double sumQ[4] = {0}, sumT[3] = {0};
		for (int i = 0; i < keepCount; ++i) {
			int idx = errors[i].index;
			for (int j = 0; j < 4; j++) sumQ[j] += groupRawData[idx].quat[j];
			for (int j = 0; j < 3; j++) sumT[j] += groupRawData[idx].t[j];
		}
		TrackedData avgData;
		for (int j = 0; j < 4; j++) avgData.quat[j] = sumQ[j] / keepCount;
		for (int j = 0; j < 3; j++) avgData.t[j] = sumT[j] / keepCount;
		m_data.push_back(avgData);
		dataCount++;
	}
}

void OMCalibrate::Quat2Matrices(std::vector<TrackedData>& all_data)
{
	for (int i = 0; i < (int)all_data.size();i++) {
		double* q = all_data[i].quat;
		double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
		if (norm > 0) {
			for(int j=0; j<4; j++) q[j] /= norm;
			all_data[i].R = (cv::Mat_<double>(3, 3) <<
				1 - 2*(q[2]*q[2] + q[3]*q[3]), 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2]),
				2*(q[1]*q[2] + q[0]*q[3]), 1 - 2*(q[1]*q[1] + q[3]*q[3]), 2*(q[2]*q[3] - q[0]*q[1]),
				2*(q[1]*q[3] - q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
		} else all_data[i].R = cv::Mat::eye(3, 3, CV_64F);
	}
}

cv::Mat OMCalibrate::createTransformationMatrix(const cv::Mat& R, const cv::Mat& t) {
	if (R.empty() || t.empty()) return cv::Mat::eye(4, 4, CV_64F);
	cv::Mat M = cv::Mat::eye(4, 4, CV_64F);
	R.copyTo(M(cv::Rect(0, 0, 3, 3)));
	M.at<double>(0, 3) = t.at<double>(0);
	M.at<double>(1, 3) = t.at<double>(1);
	M.at<double>(2, 3) = t.at<double>(2);
	return M;
}

void OMCalibrate::saveTrackedDataToCsv(const std::vector<TrackedData>& data, const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) return;
	file << "Group,Q0,Qx,Qy,Qz,Tx,Ty,Tz\n";
	for (size_t i = 0; i < data.size(); ++i) {
		file << i + 1 << "," << data[i].quat[0] << "," << data[i].quat[1] << "," << data[i].quat[2] << "," << data[i].quat[3] << ","
			<< data[i].t[0] << "," << data[i].t[1] << "," << data[i].t[2] << "\n";
	}
	file.close();
}

void OMCalibrate::saveVectorMatToTxt(const std::vector<cv::Mat>& matrices, const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) return;
	for (size_t i = 0; i < matrices.size(); ++i) {
		file << "Matrix " << i + 1 << " (" << matrices[i].rows << "x" << matrices[i].cols << "):\n";
		for (int r = 0; r < matrices[i].rows; ++r) {
			for (int c = 0; c < matrices[i].cols; ++c) file << matrices[i].at<double>(r, c) << " ";
			file << "\n";
		}
		file << "\n";
	}
	file.close();
}

void OMCalibrate::PrintTrackedData(const std::vector<TrackedData>& m_data) {
	for (const auto& d : m_data) {
		std::cout << "Quaternion: " << d.quat[0] << " " << d.quat[1] << " " << d.quat[2] << " " << d.quat[3] << "\n";
		std::cout << "Translation: " << d.t[0] << " " << d.t[1] << " " << d.t[2] << "\n-----------------------------\n";
	}
}

void OMCalibrate::HandeyeCalibrate() { /* ... 不再使用，保留定义 ... */ }
void OMCalibrate::HandeyeCalibrate1() { /* ... 不再使用，保留定义 ... */ }
void OMCalibrate::HandeyeCalibrate2() { /* ... 不再使用，保留定义 ... */ }
