#include "ToolRegi.h"

ToolRegi::ToolRegi()
{
	this->Tool = vtkSmartPointer<vtkPoints>::New();
	this->Target = vtkSmartPointer<vtkPoints>::New();
	this->RegMatrix = vtkMatrix4x4::New();
}

ToolRegi::~ToolRegi()
{
}

//void ToolRegi::LoadToolStorage(std::string filename) {
//	std::vector<std::vector<double>> tool;
//	tool = { {0.0,0.0,0.0 },{0.0,28.59,41.02},{0.0,0.0,88.00},{0.0,-44.32,40.45}};
//
//}

void ToolRegi::SetInputData(vtkPoints * source, vtkPoints * target)
{
	this->Tool->DeepCopy(source);
	//改成从内存中读入
	this->Target->DeepCopy(target);
}

bool ToolRegi::Register()
{
	vtkSmartPointer<vtkPoints> a0 = vtkSmartPointer<vtkPoints>::New();
	a0->InsertNextPoint(this->Tool->GetPoint(0));
	a0->InsertNextPoint(this->Tool->GetPoint(1));
	a0->InsertNextPoint(this->Tool->GetPoint(2));
	vtkSmartPointer<vtkPoints> a = vtkSmartPointer<vtkPoints>::New();
	//a中存放按一定顺序排列的Tool点集
	ArrangeTri(a0, a);
	//创建vtkLandmarkTransform类的对象filter 进行刚性体变换，更新配准矩阵
	vtkNew<vtkLandmarkTransform> filter;
	double aa[3], bb[3], cc[3][3];
	// 这里的cc是三角形的三个顶点
	this->Tool->GetPoint(0, cc[0]);
	this->Tool->GetPoint(1, cc[1]);
	this->Tool->GetPoint(2, cc[2]);
	// If the number of points in the target is less than 100, we use a brute-force method to find the best match.
	if (this->Target->GetNumberOfPoints() <= 100)
	{
		// First, we find the closest point in the target to each point in the tool.
		for (int i = 0; i < this->Target->GetNumberOfPoints() - 2; i++)
		{
			this->Target->GetPoint(i, aa);
			for (int j = i + 1; j < this->Target->GetNumberOfPoints() - 1; j++)
			{
				this->Target->GetPoint(j, bb);
				// baba is the distance between the two points
				double baba = EucDist(aa, bb);

				//判断两点之间的距离是否落在三角形边长附近
				if (abs(baba - EucDist(cc[0], cc[1])) >= 1.5
					&& abs(baba - EucDist(cc[1], cc[2])) >= 1.5
					&& abs(baba - EucDist(cc[2], cc[0])) >= 1.5)
					continue;

				// The third point.
				for (int k = j + 1; k < this->Target->GetNumberOfPoints(); k++)
				{
					//if (k == j)continue;
					vtkSmartPointer<vtkPoints> b0 = vtkSmartPointer<vtkPoints>::New();
					b0->InsertNextPoint(aa);
					b0->InsertNextPoint(bb);
					b0->InsertNextPoint(this->Target->GetPoint(k));
					vtkSmartPointer<vtkPoints> b = vtkSmartPointer<vtkPoints>::New();
					// Arrange the points in a triangle.
					ArrangeTri(b0, b);

					//如果两三角形满足TriMatch(a, b)，则进行配准
					if (TriMatch(a, b))
					{
						// The points are matched using a rigid body transformation.
						filter->SetSourceLandmarks(a);
						filter->SetTargetLandmarks(b);
						filter->SetModeToRigidBody();
						filter->Update();

						//如果Tool点集超过3个点，则将这些点用于进一步的优化转换矩阵，确保通过更多的点来提升配准的准确性。
						if (this->Tool->GetNumberOfPoints() >= 4)
						{
							double mindist = 100.0;
							for (int tempi = 3; tempi < this->Tool->GetNumberOfPoints(); tempi++)
							{
								mindist = 100.0;
								double* t = filter->TransformDoublePoint(this->Tool->GetPoint(tempi));
								int index = 0;
								for (int p = 0; p < this->Target->GetNumberOfPoints(); p++)
								{
									double d = EucDist(t, this->Target->GetPoint(p));
									if (d < mindist)
									{
										index = p;
										mindist = d;
										//cout << "min: " << mindist << endl;
									}
								}
								if (mindist > 10.0)break;
								a->InsertNextPoint(this->Tool->GetPoint(tempi));
								b->InsertNextPoint(this->Target->GetPoint(index));
							}
							if (mindist > 10.0)continue;
							vtkNew<vtkLandmarkTransform> filter2;
							filter2->SetSourceLandmarks(a);
							filter2->SetTargetLandmarks(b);
							filter2->SetModeToRigidBody();
							filter2->Update();

							this->RegMatrix->DeepCopy(filter2->GetMatrix());

							double err = 0.0;
							for (int i = 0; i < filter2->GetSourceLandmarks()->GetNumberOfPoints(); i++)
							{
								err += pow(EucDist(filter2->TransformDoublePoint(filter2->GetSourceLandmarks()->GetPoint(i)),
									filter2->GetTargetLandmarks()->GetPoint(i)), 2);
							}

							this->FRE = sqrt(err / filter2->GetSourceLandmarks()->GetNumberOfPoints());
							Mat2Quat(this->RegMatrix, this->m_Data.quat, this->m_Data.t);
							//cout << "qqq: " << m_Data.quat[0] << " " << m_Data.quat[1] << " " << m_Data.quat[2] << " " << m_Data.quat[3] << " " << endl;
							//cout << "ttt: " << m_Data.t[0] << " " << m_Data.t[1] << " " << m_Data.t[2] << endl;
							//cout << "FRE is: " << this->FRE << "mm" << endl;
							this->m_Data.FRE = this->FRE;
							this->m_Data.spCord->DeepCopy(b);
							return true;
						}
						else
						{
							this->RegMatrix->DeepCopy(filter->GetMatrix());

							double err = 0.0;
							for (int i = 0; i < filter->GetSourceLandmarks()->GetNumberOfPoints(); i++)
							{
								err += pow(EucDist(filter->TransformDoublePoint(filter->GetSourceLandmarks()->GetPoint(i)),
									filter->GetTargetLandmarks()->GetPoint(i)), 2);
							}
							this->FRE = sqrt(err / filter->GetSourceLandmarks()->GetNumberOfPoints());
							Mat2Quat(this->RegMatrix, this->m_Data.quat, this->m_Data.t);
							this->m_Data.FRE = this->FRE;
							this->m_Data.spCord->DeepCopy(b);
							//cout << "FRE is: " << this->FRE << "mm" << endl;
							return true;
						}
					}
				}
			}

		}
	}
	else // If the number of points in the target is greater than 100, we use KdTree to find the best match.
	{
		//// 使用KdTree检索
		//vtkSmartPointer<vtkKdTree> kdTree = vtkSmartPointer<vtkKdTree>::New();
		//kdTree->BuildLocatorFromPoints(this->Target);

		//double closestPoint[3];
		//vtkIdList *closestPointId1, *closestPointId2;
		//kdTree->FindPointsWithinRadius(EucDist(cc[0], cc[1]), cc[0], closestPointId1);
		//kdTree->FindPointsWithinRadius(EucDist(cc[0], cc[2]), cc[0], closestPointId2);
		//this->Target->GetPoint(closestPointId, aa);
		//this->Target->GetPoint(closestPointId, bb);
		//this->Target->GetPoint(closestPointId, cc[2]);

		//vtkSmartPointer<vtkPoints> b0 = vtkSmartPointer<vtkPoints>::New();
		//b0->InsertNextPoint(cc[0]);
		//b0->InsertNextPoint(aa);
		//b0->InsertNextPoint(bb);
		//vtkSmartPointer<vtkPoints> b = vtkSmartPointer<vtkPoints>::New();
		//ArrangeTri(b0, b);

		//if (TriMatch(a, b))
		//{
		//	filter->SetSourceLandmarks(a);
		//	filter->SetTargetLandmarks(b);
		//	filter->SetModeToRigidBody();
		//	filter->Update();
		//	// 处理匹配结果
		//	// ...
		//	return true;
		//}
	}

	//cout << "No tool tracked!" << endl;
	return false;
}

TrackingData ToolRegi::GetTrackingData()
{
	return this->m_Data;
}

void ToolRegi::GetTrackingData(TrackingData & data)
{
	data.FRE = this->m_Data.FRE;
	data.quat[0] = this->m_Data.quat[0];
	data.quat[1] = this->m_Data.quat[1];
	data.quat[2] = this->m_Data.quat[2];
	data.quat[3] = this->m_Data.quat[3];
	data.t[0] = this->m_Data.t[0];
	data.t[1] = this->m_Data.t[1];
	data.t[2] = this->m_Data.t[2];
	data.spCord->DeepCopy(this->m_Data.spCord);
}

double ToolRegi::GetFRE()
{
	return this->FRE;
}

vtkMatrix4x4 * ToolRegi::GetMatrix()
{
	return this->RegMatrix;
}

double ToolRegi::EucDist(double * a, double * b)
{
	return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));
}

void ToolRegi::QuaternionRotate(double * src, double * q, double * dst)
//使用四元数进行三维空间中的向量旋转
{
	double q01, q02, q03, q11, q12, q13, q22, q23, q33;
	//q00 = 2 * q[0] * q[0];
	q01 = 2 * q[0] * q[1];
	q02 = 2 * q[0] * q[2];
	q03 = 2 * q[0] * q[3];
	q11 = 2 * q[1] * q[1];
	q12 = 2 * q[1] * q[2];
	q13 = 2 * q[1] * q[3];
	q22 = 2 * q[2] * q[2];
	q23 = 2 * q[2] * q[3];
	q33 = 2 * q[3] * q[3];
	dst[0] = (1 - q22 - q33)*src[0] + (q12 - q03)*src[1] + (q13 + q02)*src[2];
	dst[1] = (q12 + q03)*src[0] + (1 - q11 - q33)*src[1] + (q23 - q01)*src[2];
	dst[2] = (q13 - q02)*src[0] + (q23 + q01)*src[1] + (1 - q11 - q22)*src[2];
	return;
}

void ToolRegi::Quat2Mat(double * q, double * t, double * mat)
//四元数+位移 ——> 旋转矩阵
{
	double q01, q02, q03, q11, q12, q13, q22, q23, q33;
	//q00 = 2 * q[0] * q[0];
	q01 = 2 * q[0] * q[1];
	q02 = 2 * q[0] * q[2];
	q03 = 2 * q[0] * q[3];
	q11 = 2 * q[1] * q[1];
	q12 = 2 * q[1] * q[2];
	q13 = 2 * q[1] * q[3];
	q22 = 2 * q[2] * q[2];
	q23 = 2 * q[2] * q[3];
	q33 = 2 * q[3] * q[3];

	mat[0] = 1.0 - q22 - q33;
	mat[1] = q12 - q03;
	mat[2] = q13 + q02;
	mat[3] = t[0];
	mat[4] = q12 + q03;
	mat[5] = 1.0 - q11 - q33;
	mat[6] = q23 - q01;
	mat[7] = t[1];
	mat[8] = q13 - q02;
	mat[9] = q23 + q01;
	mat[10] = 1.0 - q11 - q22;
	mat[11] = t[2];
	mat[12] = mat[13] = mat[14] = 0.0;
	mat[15] = 1.0;
}

void ToolRegi::ArrangeTri(vtkPoints * src, vtkPoints * out)
//通过对边长的比较，确保输出的点顺序是根据边的长度关系决定的
{
	std::vector<double> l1, l2;
	double p1[3], p2[3], p3[3];
	src->GetPoint(0, p1);
	src->GetPoint(1, p2);
	src->GetPoint(2, p3);
	l1.push_back(EucDist(p1, p2));
	l1.push_back(EucDist(p2, p3));
	l1.push_back(EucDist(p1, p3));
	int a[3];
	if (l1[1] > l1[0] && l1[2] > l1[0])
	{
		a[0] = 2;
		if (l1[2] > l1[1])
		{
			a[1] = 0;
			a[2] = 1;
		}
		else
		{
			a[1] = 1;
			a[2] = 0;
		}
	}
	else
	{
		if (l1[2] > l1[1])
		{
			a[0] = 0;
			if (l1[0] > l1[2])
			{
				a[1] = 1;
				a[2] = 2;
			}
			else
			{
				a[1] = 2;
				a[2] = 1;
			}
		}
		else
		{
			a[0] = 1;
			if (l1[0] > l1[1])
			{
				a[1] = 0;
				a[2] = 2;
			}
			else
			{
				a[1] = 2;
				a[2] = 0;
			}
		}
	}

	out->InsertNextPoint(src->GetPoint(a[0]));
	out->InsertNextPoint(src->GetPoint(a[1]));
	out->InsertNextPoint(src->GetPoint(a[2]));

}

bool ToolRegi::TriMatch(vtkPoints * a, vtkPoints * b)
{
	std::vector<double> l1, l2;
	double p1[3], p2[3], p3[3];
	a->GetPoint(0, p1);
	a->GetPoint(1, p2);
	a->GetPoint(2, p3);
	l1.push_back(EucDist(p1, p2));
	l1.push_back(EucDist(p2, p3));
	l1.push_back(EucDist(p1, p3));

	b->GetPoint(0, p1);
	b->GetPoint(1, p2);
	b->GetPoint(2, p3);
	l2.push_back(EucDist(p1, p2));
	l2.push_back(EucDist(p2, p3));
	l2.push_back(EucDist(p1, p3));

	return (abs(l1[0] - l2[0]) < 1.0 && abs(l1[1] - l2[1]) < 1.0 &&abs(l1[2] - l2[2]) < 1.0);
}

double ToolRegi::GetFRE(vtkLandmarkTransform * filter)
{
	return this->FRE;
}

void ToolRegi::Mat2Quat(vtkMatrix4x4 * mat, double * q, double *t)
//旋转矩阵  ——>   四元数+位移
{
	double tr = mat->GetElement(0, 0) + mat->GetElement(1, 1) + mat->GetElement(2, 2);
	double temp = 0.0;
	if (tr > 0.0)
	{
		temp = 0.5f / sqrtf(tr + 1);
		q[0] = 0.25f / temp;
		//有可能是反的
		q[1] = (mat->GetElement(2, 1) - mat->GetElement(1, 2)) * temp;
		q[2] = (mat->GetElement(0, 2) - mat->GetElement(2, 0)) * temp;
		q[3] = (mat->GetElement(1, 0) - mat->GetElement(0, 1)) * temp;
	}
	else
	{
		if (mat->GetElement(0, 0) > mat->GetElement(1, 1) && mat->GetElement(0, 0) > mat->GetElement(2, 2))
		{
			temp = 2.0f * sqrtf(1.0f + mat->GetElement(0, 0) - mat->GetElement(1, 1) - mat->GetElement(2, 2));
			q[0] = (mat->GetElement(2, 1) - mat->GetElement(1, 2)) / temp;
			q[1] = 0.25f * temp;
			q[2] = (mat->GetElement(0, 1) + mat->GetElement(1, 0)) / temp;
			q[3] = (mat->GetElement(0, 2) + mat->GetElement(2, 0)) / temp;
		}
		else if (mat->GetElement(1, 1) > mat->GetElement(2, 2))
		{
			temp = 2.0f * sqrtf(1.0f + mat->GetElement(1, 1) - mat->GetElement(0, 0) - mat->GetElement(2, 2));
			q[0] = (mat->GetElement(0, 2) - mat->GetElement(2, 0)) / temp;
			q[1] = (mat->GetElement(0, 1) + mat->GetElement(1, 0)) / temp;
			q[2] = 0.25f * temp;
			q[3] = (mat->GetElement(1, 2) + mat->GetElement(2, 1)) / temp;
		}
		else
		{
			temp = 2.0f * sqrtf(1.0f + mat->GetElement(2, 2) - mat->GetElement(0, 0) - mat->GetElement(1, 1));
			q[0] = (mat->GetElement(1, 0) - mat->GetElement(0, 1)) / temp;
			q[1] = (mat->GetElement(0, 2) + mat->GetElement(2, 0)) / temp;
			q[2] = (mat->GetElement(1, 2) + mat->GetElement(2, 1)) / temp;
			q[3] = 0.25f * temp;
		}
	}

	t[0] = mat->GetElement(0, 3);
	t[1] = mat->GetElement(1, 3);
	t[2] = mat->GetElement(2, 3);
}

void ToolRegi::Mat2Quat2(vtkMatrix4x4 * mat, double * q, double *t)
{
	float q0abs = 0.5f * sqrt(1 + mat->GetElement(0, 0) + mat->GetElement(1, 1) + mat->GetElement(2, 2));
	float q1abs = 0.5f * sqrt(1 + mat->GetElement(0, 0) - mat->GetElement(1, 1) - mat->GetElement(2, 2));
	float q2abs = 0.5f * sqrt(1 - mat->GetElement(0, 0) + mat->GetElement(1, 1) - mat->GetElement(2, 2));
	float q3abs = 0.5f * sqrt(1 - mat->GetElement(0, 0) - mat->GetElement(1, 1) + mat->GetElement(2, 2));

	float q1sign = mat->GetElement(1, 2) - mat->GetElement(2, 1) > 0 ? 1 : -1;
	float q2sign = mat->GetElement(2, 0) - mat->GetElement(0, 2) > 0 ? 1 : -1;
	float q3sign = mat->GetElement(0, 1) - mat->GetElement(1, 0) > 0 ? 1 : -1;

	q[0] = q0abs;
	q[1] = q1sign * q1abs;
	q[2] = q2sign * q2abs;
	q[3] = q3sign * q3abs;

	t[0] = mat->GetElement(0, 3);
	t[1] = mat->GetElement(1, 3);
	t[2] = mat->GetElement(2, 3);
}
