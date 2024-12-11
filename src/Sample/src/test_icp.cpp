#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataWriter.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <iostream>

//// 生成一个随机点云
//vtkSmartPointer<vtkPolyData> generateRandomPointCloud(int numPoints) {
//	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//	for (int i = 0; i < numPoints; ++i) {
//		points->InsertNextPoint(rand() % 100, rand() % 100, rand() % 100);
//	}
//
//	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
//	polyData->SetPoints(points);
//
//	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
//	vertexFilter->SetInputData(polyData);
//	vertexFilter->Update();
//
//	return vertexFilter->GetOutput();
//}
//
//// 保存点云到文件
//void savePointCloud(vtkSmartPointer<vtkPolyData> cloud, const std::string& filename) {
//	vtkSmartPointer<vtkPolyDataWriter> writer = vtkSmartPointer<vtkPolyDataWriter>::New();
//	writer->SetFileName(filename.c_str());
//	writer->SetInputData(cloud);
//	writer->Write();
//}
//
//int main6() {
//	vtkSmartPointer<vtkPolyData> cloud = generateRandomPointCloud(100);
//	savePointCloud(cloud, "point_cloud.vtk");
//
//	std::cout << "Point cloud saved as 'point_cloud.vtk'." << std::endl;
//
//	return 0;
//}