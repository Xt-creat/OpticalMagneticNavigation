#include "OMCalibrate.h"


OMCalibrate::OMCalibrate()
{
	
	std::string filename1 = "Vega.csv";
	std::string filename2 = "Aurora.csv";
	//std::cout << "M1 data" << std::endl;

	int group = 30;
	
	ReadRecordData(this->M1_data, filename1,group, 12, 9);
	
	Quat2Matrices(M1_data);
	//PrintTrackedData(this->M1_data);
	

	//std::cout << "M2  data" << std::endl;
	ReadRecordData(this->M2_data, filename1,group, 12, 41);
	Quat2Matrices(M2_data);
	//PrintTrackedData(this->M2_data);

	//std::cout << "EMSensor  data" << std::endl;
	ReadRecordData(this->EMSensor_data, filename2,group, 12, 9);
	Quat2Matrices(EMSensor_data);
	//PrintTrackedData(this->EMSensor_data);

	GetRt();


	
	for (int i = 0; i < R_Marker1_2Vega.size(); i++) {
		Marker1_2Vega.push_back(createTransformationMatrix(R_Marker1_2Vega[i], t_Marker1_2Vega[i]));        
		Marker2_2Vega.push_back(createTransformationMatrix(R_Marker2_2Vega[i], t_Marker2_2Vega[i]));
		EMsensor_2Aurora.push_back(createTransformationMatrix(R_EMsensor_2Aurora[i], t_EMsensor_2Aurora[i]));

	}


	for (int i = 0; i < Marker1_2Vega.size(); i++) {
		cv::Mat invert_temp;  
		cv::Mat invert_temp1;
		cv::Mat invert_temp2;

		cv::invert(Marker1_2Vega[i], invert_temp, cv::DECOMP_SVD);
		Vega_2Marker1.push_back(invert_temp);
		cv::invert(Marker2_2Vega[i], invert_temp1, cv::DECOMP_SVD);
		Vega_2Marker2.push_back(invert_temp1);
		cv::invert(EMsensor_2Aurora[i], invert_temp2, cv::DECOMP_SVD);
		Aurora_2EMsensor.push_back(invert_temp2);
		
	}
	
	
	//M2_2M1、M1_2M2数据
	for (int i = 0; i < Marker1_2Vega.size(); i++) {
		cv::Mat Rt;
		cv::Mat Rt2;
		Rt = Vega_2Marker1[i] * Marker2_2Vega[i];
		

		// 提取旋转矩阵 R (3x3)
		cv::Mat R = Rt(cv::Range(0, 3), cv::Range(0, 3));

		// 提取平移向量 t (3x1)
		cv::Mat t = Rt(cv::Range(0, 3), cv::Range(3, 4));
		
		Marker2_2Marker1.push_back(Rt);
		R_Marker2_2Marker1.push_back(R);
		t_Marker2_2Marker1.push_back(t);

		Rt2= Vega_2Marker2[i] * Marker1_2Vega[i];
		Marker1_2Marker2.push_back(Rt2);
	}
	

	saveVectorMatToTxt(Marker2_2Marker1, "Marker2_2Marker1.txt");
	saveVectorMatToTxt(Marker1_2Marker2, "Marker1_2Marker2.txt");
	saveVectorMatToTxt(EMsensor_2Aurora, "EMsensor_2Aurora.txt");
	saveVectorMatToTxt(Aurora_2EMsensor,"Aurora_2EMsensor.txt");
	saveVectorMatToTxt(Marker1_2Vega, "Marker1_2Vega.txt");
	saveVectorMatToTxt(Marker2_2Vega, "Marker2_2Vega.txt");

	//saveVectorMatToTxt(Marker1_2EMsensor, "Marker1_2EMsensor.txt");
	//saveVectorMatToTxt(EM_2Marker2, "EM_2Marker2.txt");

}

OMCalibrate::~OMCalibrate()
{
}




void OMCalibrate::HandeyeCalibrate()
{
	
	//cv::calibrateHandEye求解AX=XB问题
	

	cv::Mat R_EM2M2, t_EM2M2;
	cv::Mat R_M1_2EMS, t_M1_2EMS;

	cv::calibrateHandEye(
		R_Marker2_2Marker1,
		t_Marker2_2Marker1,
		R_EMsensor_2Aurora,
		t_EMsensor_2Aurora,
		R_EM2M2,
		t_EM2M2,
		cv::CALIB_HAND_EYE_TSAI 
	);
	cv::calibrateHandEye(
		R_EMsensor_2Aurora,
		t_EMsensor_2Aurora,
		R_Marker2_2Marker1,
		t_Marker2_2Marker1,
		R_M1_2EMS,
		t_M1_2EMS,
		cv::CALIB_HAND_EYE_TSAI 
	);


	
	EM_2Marker2 = createTransformationMatrix(R_EM2M2, t_EM2M2);
	Marker1_2EMsensor = createTransformationMatrix(R_M1_2EMS, t_M1_2EMS);
	cv::invert(Marker1_2EMsensor, EMsensor_2Marker1, cv::DECOMP_SVD);
}


void OMCalibrate::HandeyeCalibrate1()
{

	//cv::calibrateHandEye求解AX=XB问题
	// 
	std::vector<cv::Mat> R_EMS2EM = this->R_EMsensor_2Aurora, t_EMS2EM = this->t_EMsensor_2Aurora;
	std::vector<cv::Mat> R_M1_2O = this->R_Marker1_2Vega, t_M1_2O = this->t_Marker1_2Vega;

	//求解固定变换Aurora2Vega，需要Vega2Marker1的转换矩阵，所以需要对Marker1_2Vega求逆 
	//求解固定变换EMsensor_2Marker1，需要Aurora2EMsensor的转换矩阵，所以需要对EMsensor_2Aurora求逆 


	std::vector<cv::Mat> R_O2M1, t_O2M1;
	std::vector<cv::Mat> R_EM2EMS, t_EM2EMS;

	for (int i = 0; i < R_M1_2O.size(); i++) {
		cv::Mat R_inv;
		cv::invert(R_M1_2O[i], R_inv, cv::DECOMP_SVD);
		cv::Mat t_inv = -R_inv * t_M1_2O[i];
		R_O2M1.push_back(R_inv);
		t_O2M1.push_back(t_inv);
	}

	for (int i = 0; i < R_EMS2EM.size(); i++) {
		cv::Mat R_inv;
		cv::invert(R_EMS2EM[i], R_inv, cv::DECOMP_SVD);
		cv::Mat t_inv = -R_inv * t_EMS2EM[i];
		R_EM2EMS.push_back(R_inv);
		t_EM2EMS.push_back(t_inv);
	}

	cv::Mat R_EM2O, t_EM2O;
	cv::Mat R_EMS2M1, t_EMS2M1;

	cv::calibrateHandEye(
		R_O2M1,
		t_O2M1,
		R_EMS2EM,
		t_EMS2EM,
		R_EM2O,
		t_EM2O,
		cv::CALIB_HAND_EYE_TSAI
	);
	cv::calibrateHandEye(
		R_M1_2O,
		t_M1_2O,
		R_EM2EMS,
		t_EM2EMS,
		R_EMS2M1,
		t_EMS2M1,
		cv::CALIB_HAND_EYE_TSAI
	);


	this->R_Aurora2Vega = R_EM2O;
	this->t_Aurora2Vega = t_EM2O;
	this->R_EMsensor_2Marker1 = R_EMS2M1;
	this->t_EMsensor_2Marker1 = t_EMS2M1;
	Aurora2Vega = createTransformationMatrix(R_Aurora2Vega, t_Aurora2Vega);
	EMsensor_2Marker1 = createTransformationMatrix(R_EMsensor_2Marker1, t_EMsensor_2Marker1);

}


void OMCalibrate::HandeyeCalibrate2()
{


	std::vector<cv::Mat> R_EMS2EM = this->R_EMsensor_2Aurora, t_EMS2EM = this->t_EMsensor_2Aurora;
	std::vector<cv::Mat> R_M1_2O = this->R_Marker1_2Vega, t_M1_2O = this->t_Marker1_2Vega;

	


	std::vector<cv::Mat> R_O2M1, t_O2M1;
	

	for (int i = 0; i < R_M1_2O.size(); i++) {
		cv::Mat R_inv;
		cv::invert(R_M1_2O[i], R_inv, cv::DECOMP_SVD);
		cv::Mat t_inv = -R_inv * t_M1_2O[i];
		R_O2M1.push_back(R_inv);
		t_O2M1.push_back(t_inv);
	}



	cv::Mat R_EM2O, t_EM2O;
	cv::Mat R_M1_2EMS, t_M1_2EMS;
	cv::Mat R_EMS2M1, t_EMS2M1;

	cv::calibrateHandEye(
        R_EMS2EM,
		t_EMS2EM,
		R_O2M1,
		t_O2M1,
		R_M1_2EMS,
		t_M1_2EMS,
		cv::CALIB_HAND_EYE_TSAI
	);

	
	cv::invert(R_M1_2EMS, R_EMS2M1, cv::DECOMP_SVD);
	t_EMS2M1 = -R_EMS2M1 * t_M1_2EMS;
	
	


	this->R_EMsensor_2Marker1 = R_EMS2M1;
	this->t_EMsensor_2Marker1 = t_EMS2M1;
	EMsensor_2Marker1 = createTransformationMatrix(R_EMsensor_2Marker1, t_EMsensor_2Marker1);

	  //用采集的第6组数据计算固定矩阵Aurora2Vega、EM_2Marker2
	Aurora2Vega = Marker1_2Vega[1] * EMsensor_2Marker1 *Aurora_2EMsensor[1];

	EM_2Marker2 = Vega_2Marker2[1] *Aurora2Vega;
}

void OMCalibrate::HandeyeCalibrate3()
{
	
	//cv::calibrateHandEye求解AX=YB问题
	// 
	std::vector<cv::Mat> R_EMS2EM = this->R_EMsensor_2Aurora, t_EMS2EM = this->t_EMsensor_2Aurora;
	std::vector<cv::Mat> R_M1_2O = this->R_Marker1_2Vega, t_M1_2O = this->t_Marker1_2Vega;

	//求解固定变换Aurora2Vega，需要Vega2Marker1的转换矩阵，所以需要对Marker1_2Vega求逆 
	//求解固定变换EMsensor_2Marker1，需要Aurora2EMsensor的转换矩阵，所以需要对EMsensor_2Aurora求逆 


	std::vector<cv::Mat> R_O2M1, t_O2M1;
	std::vector<cv::Mat> R_EM2EMS, t_EM2EMS;

	for (int i = 0; i < R_M1_2O.size(); i++) {
		cv::Mat R_inv;
		cv::invert(R_M1_2O[i], R_inv, cv::DECOMP_SVD);
		cv::Mat t_inv = -R_inv * t_M1_2O[i];
		R_O2M1.push_back(R_inv);
		t_O2M1.push_back(t_inv);
	}

	for (int i = 0; i < R_EMS2EM.size(); i++) {
		cv::Mat R_inv;
		cv::invert(R_EMS2EM[i], R_inv, cv::DECOMP_SVD);
		cv::Mat t_inv = -R_inv * t_EMS2EM[i];
		R_EM2EMS.push_back(R_inv);
		t_EM2EMS.push_back(t_inv);
	}

	//cv::Mat R_EM2O, t_EM2O;
	cv::Mat R_EM2M2, t_EM2M2;
	cv::Mat R_EMS2M1, t_EMS2M1;

	//cv::calibrateRobotWorldHandEye(
	//	R_O2M1, t_O2M1,
	//	R_EM2EMS, t_EM2EMS,
	//	R_EM2O, t_EM2O,
	//	R_EMS2M1, t_EMS2M1,
	//	cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH
	//);
	cv::calibrateRobotWorldHandEye(
		R_Marker2_2Marker1, t_Marker2_2Marker1,
		R_EM2EMS, t_EM2EMS,
		R_EM2M2, t_EM2M2,
		R_EMS2M1, t_EMS2M1,
		cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH
	);

	//this->R_Aurora2Vega = R_EM2O;
	//this->t_Aurora2Vega = t_EM2O;
	this->R_EM_2Marker2 = R_EM2M2;
	this->t_EM_2Marker2 = t_EM2M2;
	this->R_EMsensor_2Marker1 = R_EMS2M1;
	this->t_EMsensor_2Marker1 = t_EMS2M1;
	//Aurora2Vega = createTransformationMatrix(R_Aurora2Vega, t_Aurora2Vega);
	EM_2Marker2 = createTransformationMatrix(R_EM_2Marker2, t_EM_2Marker2);
	EMsensor_2Marker1 = createTransformationMatrix(R_EMsensor_2Marker1, t_EMsensor_2Marker1);

	cv::invert(EMsensor_2Marker1, Marker1_2EMsensor, cv::DECOMP_SVD);
}






void OMCalibrate::GetRt()
{
	//std::cout << "M1_data.size" << M1_data.size() << std::endl;

	for (int i = 0; i < this->M1_data.size();i++) {
		R_Marker1_2Vega.push_back(this->M1_data[i].R);
		cv::Mat matT(3, 1, CV_64F, this->M1_data[i].t);
		t_Marker1_2Vega.push_back(matT);

		R_Marker2_2Vega.push_back(M2_data[i].R);
		cv::Mat matT1(3, 1, CV_64F, M2_data[i].t);
		t_Marker2_2Vega.push_back(matT1);

		R_EMsensor_2Aurora.push_back(EMSensor_data[i].R);
		cv::Mat matT2(3, 1, CV_64F, EMSensor_data[i].t);
		t_EMsensor_2Aurora.push_back(matT2);
	}
	//std::cout << "R_Marker1_2Vega 0 " << R_Marker1_2Vega[0] << std::endl;
	//std::cout << "t_Marker1_2Vega 0 " << t_Marker1_2Vega[0] << std::endl;
}



void OMCalibrate::ReadRecordData(std::vector<TrackedData>& m_data,const std::string& filename,int groups,int interval, int start) {
	std::ifstream file(filename);
	std::string line;
	TrackedData data1;

	if (!file.is_open()) {
		std::cerr << "Error opening file: " << filename << std::endl;
		return;
	}

		// Skip the first 6 lines
	for (int i = 0; i < 6; ++i) {
		std::getline(file, line);
	}
	int dataCount = 0;
	while (dataCount < groups) {
		// Read 10 groups of data
		if (std::getline(file, line)) {
			std::stringstream ss(line);
			std::string cell;
			int column = 0;

			// 按照逗号分割每一行
			while (std::getline(ss, cell, ',')) {
				if (column >= start-1 && column < start+3) { // 第9-12列 (索引8-11)
					data1.quat[column - start+1] = std::stod(cell); // 转换为double并存入quat
				}
				else if (column >= start + 3 && column < start+6) { // 第13-15列 (索引12-14)
					data1.t[column -start - 3] = std::stod(cell); // 转换为double并存入t
				}
				column++;
			}
		}

		// Read FRE and any other fields as necessary
		// e.g., ss >> data1.FRE; if it is in the file
		m_data.push_back(data1); // Add the new data to m_data

		// Skip the next 11 lines (total 12 lines)
		for (int i = 0; i < interval-1; ++i) {
			std::getline(file, line);
		}

		++dataCount;
	}

}




void OMCalibrate::PrintTrackedData(const std::vector<TrackedData>& m_data) {
	for (const auto& data : m_data) {
		std::cout << "Name: " << data.name << std::endl;
		std::cout << "Quaternion: ";
		for (double q : data.quat) {
			std::cout << q << " ";
		}
		std::cout << std::endl;

		std::cout << "Translation: ";
		for (double t : data.t) {
			std::cout << t << " ";
		}
		std::cout << std::endl;

		std::cout << "FRE: " << data.FRE << std::endl;
		std::cout << "Rotation Matrix: " << data.R << std::endl; // Assuming R is printed correctly
		std::cout << "-----------------------------" << std::endl;
	}
}




void OMCalibrate::Quat2Matrices(std::vector<TrackedData>& all_data)
{
	TrackedData data;
	for (int i = 0; i < all_data.size();i++) {
		// 四元数归一化
		data = all_data[i];
		double norm = std::sqrt(data.quat[0] * data.quat[0] +
			data.quat[1] * data.quat[1] +
			data.quat[2] * data.quat[2] +
			data.quat[3] * data.quat[3]);

		cv::Mat R = cv::Mat::eye(3, 3, CV_64F); // 默认单位矩阵

		if (norm > 0)
		{
			double q[4] = { data.quat[0] / norm, data.quat[1] / norm,
						   data.quat[2] / norm, data.quat[3] / norm };

			// 计算旋转矩阵
			R = (cv::Mat_<double>(3, 3) <<
				1 - 2 * (q[2] * q[2] + q[3] * q[3]), 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2]),
				2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[1] * q[1] + q[3] * q[3]), 2 * (q[2] * q[3] - q[0] * q[1]),
				2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
		}

		all_data[i].R = R;

	}
	
}


cv::Mat OMCalibrate::createTransformationMatrix(const cv::Mat& R, const cv::Mat& t)
{

	if (R.rows != 3 || R.cols != 3) {
		throw std::invalid_argument("R must be a 3x3 rotation matrix.");
	}
	if (t.rows != 3 || t.cols != 1) {
		throw std::invalid_argument("t must be a 3x1 translation vector.");
	}

	
	cv::Mat transformMatrix = cv::Mat::zeros(4, 4, CV_64F);

	// 将旋转矩阵复制到变换矩阵的左上角
	R.copyTo(transformMatrix(cv::Rect(0, 0, 3, 3)));

	// 设置平移部分
	transformMatrix.at<double>(0, 3) = t.at<double>(0);
	transformMatrix.at<double>(1, 3) = t.at<double>(1);
	transformMatrix.at<double>(2, 3) = t.at<double>(2);
	transformMatrix.at<double>(3, 3) = 1.0; // 齐次坐标最后一行

	return transformMatrix;
}

void OMCalibrate::saveVectorMatToTxt(const std::vector<cv::Mat>& matrices, const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return;
	}

	for (size_t i = 0; i < matrices.size(); ++i) {
		file << "Matrix " << i + 1 << " (" << matrices[i].rows << "x" << matrices[i].cols << "):\n";
		for (int row = 0; row < matrices[i].rows; ++row) {
			for (int col = 0; col < matrices[i].cols; ++col) {
				file << matrices[i].at<double>(row, col) << " ";  // Assume double precision
			}
			file << "\n";
		}
		file << "\n";  // Separate matrices with an empty line
	}

	file.close();
}


