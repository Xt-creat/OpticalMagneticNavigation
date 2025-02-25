#include "ToolRegi.h"


vtkSmartPointer<vtkPoints> ConvertToVtkPoints(const std::vector<std::vector<double>>& tool) {
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

// 打印 TrackingData 内容的函数
void printTrackingData(const TrackingData& data) {
	std::cout << "Name: " << data.name << std::endl;
	std::cout << "Quaternion: ["
		<< data.quat[0] << ", "
		<< data.quat[1] << ", "
		<< data.quat[2] << ", "
		<< data.quat[3] << "]" << std::endl;
	std::cout << "Translation: ["
		<< data.t[0] << ", "
		<< data.t[1] << ", "
		<< data.t[2] << "]" << std::endl;
	std::cout << "FRE: " << data.FRE << std::endl;
	std::cout << "Match: " << data.match << std::endl;

	std::cout << "Tooltip: ["
		<< data.tooltip[0] << ", "
		<< data.tooltip[1] << ", "
		<< data.tooltip[2] << "]" << std::endl;

	std::cout << "Number of Points in spCord: " << data.spCord->GetNumberOfPoints() << std::endl;
	std::cin.get();
}

//example1  --tool  8700339.rom:
//tool = { {0.0,0.0,0.0 },{0.0,28.59,41.02},{0.0,0.0,88.00},{0.0,-44.32,40.45} };
//target = { {-157.679,-176.634,-1292.75 },{-187.989,-213.682,-1277.59},{-161.993,-253.648,-1250.46},{-116.234,-210.168,-1264.92} };
//NDI计算输出的四元数和平移：{0.561559,0.310384,-0.404376,0.651761}   {- 157.739, - 176.598, - 1292.48}     error:0.275648

//example2  --tool  20241023.rom:
//tool = { {0.0,0.0,0.0 },{-50.72,6.29,0},{0.0,63.21,0} };
//target = { {10.6416,-143.197,-1146.8 },{-31.5639,-170.643,-1155.67},{-30.5452,-95.4675,-1143.57} };
//NDI计算输出的四元数和平移：{0.934045,0.0556337,-0.0764159,0.344421},{10.6446,-143.226,-1146.81}     error:0.0462668


int main4()
{
	std::vector<std::vector<double>> tool;
	tool = { {0.0,0.0,0.0 },{-50.72,6.29,0},{0.0,63.21,0} };
	std::vector<std::vector<double>> target;
	target = { {10.6416,-143.197,-1146.8 },{-31.5639,-170.643,-1155.67},{-30.5452,-95.4675,-1143.57} };
	


	ToolRegi tool_Regi;
	tool_Regi.Tool = ConvertToVtkPoints(tool);
	tool_Regi.Target = ConvertToVtkPoints(target);
	tool_Regi.Register();
	
	vtkMatrix4x4 *reg_Matrix;
	reg_Matrix = tool_Regi.GetMatrix();

	TrackingData trackingData;
	trackingData = tool_Regi.GetTrackingData();
	printTrackingData(trackingData);

	return 0;

}