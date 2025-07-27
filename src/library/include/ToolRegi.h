#ifndef ToolRegi_H
#define ToolRegi_H

#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkMatrix4x4.h>
#include <vtkLandmarkTransform.h>
#include <vtkKdTree.h>
#include <string>
#include <math.h>

struct TrackingData
{
	std::string name = "";
	double quat[4];
	double t[3];
	double FRE;
	int match;
	vtkPoints* spCord = vtkPoints::New();
	double tooltip[3] = { 0., 0., 0. };
};

//基于RANSAC的二点-三点匹配，使用LansmarkTransform计算工具在三维空间的旋转四元数和平移向量
class ToolRegi
{
public:
	ToolRegi();
	~ToolRegi();

	//void LoadToolStorage(std::string filename);

	//输入的source是存储的工具点集，target是三维重建的坐标点集合
	void SetInputData(vtkPoints *source, vtkPoints *target);
	//开始配准
	bool Register();

	//获取跟踪数据
	TrackingData GetTrackingData();
	void GetTrackingData(TrackingData &data);
	//工具匹配误差RMS
	double GetFRE();
	//工具匹配刚性变换
	vtkMatrix4x4 *GetMatrix();

	//两个三维点的欧式距离
	static double EucDist(double *a, double *b);

	//四元数旋转
	static void QuaternionRotate(double *src, double *q, double *dst);

	//四元数和平移转矩阵
	static void Quat2Mat(double *q, double *t, double *mat);

//protected:
	//三角形按排序
	void ArrangeTri(vtkPoints *src, vtkPoints *out);
	bool TriMatch(vtkPoints *a, vtkPoints *b);
	double GetFRE(vtkLandmarkTransform *filter);
	void Mat2Quat(vtkMatrix4x4 *mat, double *q, double *t);

	void Mat2Quat2(vtkMatrix4x4 * mat, double * q, double * t);

	TrackingData m_Data;     //存储了Tool和Target配准后的四元数和平移，配准误差，针尖位置Target中用于配准计算的点集
	vtkSmartPointer<vtkPoints> Tool;
	vtkSmartPointer<vtkPoints> Target;
	vtkMatrix4x4 *RegMatrix;
	double FRE;
};

#endif
