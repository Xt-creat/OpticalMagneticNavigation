#include "DisplayWidget.h"

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include <vtkSphereSource.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkSTLReader.h>
#include<vtkAssembly.h>
#include<vtkPolygon.h>
#include <vtkProperty.h>
#include<vtkTransform.h>

DisplayWidget::DisplayWidget(QWidget* parent)
    : QWidget(parent)
{
    ui.setupUi(this);

    // 1️创建 VTK 渲染窗口
    renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    ui.openGLWidget->setRenderWindow(renderWindow);


    //创建渲染器
    renderer = vtkSmartPointer<vtkRenderer>::New();
    //renderer->SetBackground(0.1, 0.2, 0.4); // 深蓝背景
    renderer->SetBackground(0, 0, 0); 
    renderWindow->AddRenderer(renderer);

    //在世界原点添加一个坐标轴
    addSceneAxes();

    //在左下角添加一个 OrientationMarkerWidget 坐标系
    addCornerAxes();

    addTrackingSideWalls(scaleFactor);

    //  定时器
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &DisplayWidget::updateConePose);

    modelTransform = vtkSmartPointer<vtkTransform>::New();

}


DisplayWidget::~DisplayWidget()
{
    
}

vtkSmartPointer<vtkRenderer> DisplayWidget::getRenderer()
{
    return renderer;
}

void DisplayWidget::clearScene()
{
    renderer->RemoveAllViewProps();
    renderWindow->Render();
}


void DisplayWidget::addSceneAxes()
{
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(30.0, 30.0, 30.0); // X/Y/Z 轴长度
    axes->SetShaftTypeToCylinder();
    axes->SetCylinderRadius(0.01);

    axes->SetAxisLabels(false);

    renderer->AddActor(axes);
}

void DisplayWidget::addCornerAxes()
{
    //创建坐标轴
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(50, 50, 50);

    //创建 OrientationMarkerWidget
    orientationMarkerWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    orientationMarkerWidget->SetOrientationMarker(axes);

    // 关键
    //orientationMarkerWidget->SetInteractor(renderWindow->GetInteractor());
    vtkRenderWindowInteractor* interactor = ui.openGLWidget->interactor();
    orientationMarkerWidget->SetInteractor(interactor);

    // 左下角 30% 区域
    orientationMarkerWidget->SetViewport(0.0, 0.0, 0.2, 0.2);

    orientationMarkerWidget->SetEnabled(1);
    orientationMarkerWidget->InteractiveOff();
}

void DisplayWidget::startRealtimeAnimation()
{
    timer->start(30); // 每30ms刷新一次（33fps）
}

void DisplayWidget::stopRealtimeAnimation()
{
    timer->stop();
}

//  外部实时传感器更新位姿（线程安全可加互斥锁）
void DisplayWidget::updateExternalPose(const Pose& p)
{
    latestPose = p;
    cout << "---------updateExternalPose--------" << endl;
    cout << "rx:" << p.rx << endl;
    cout << "ry:" << p.ry << endl;
    cout << "rz:" << p.rz << endl;
}

void DisplayWidget::updateExternalTran(const vtkSmartPointer<vtkTransform>& p)
{
    std::lock_guard<std::mutex>lock(m_ndiTransformMutex);
    m_ndiTransform = p;
}

//  每帧刷新模型位姿
void DisplayWidget::updateConePose()
{
    if (!coneActor) return;  // 保险

    vtkSmartPointer<vtkTransform> transform;
    {
        std::lock_guard<std::mutex>lock(m_ndiTransformMutex);
        transform = m_ndiTransform;
    }
    //modelAssembly->SetPosition(latestPose.x * scaleFactor, latestPose.y * scaleFactor, latestPose.z * scaleFactor);
    //modelAssembly->SetOrientation(latestPose.rx, latestPose.ry, latestPose.rz);

    modelAssembly->SetUserTransform(transform);

    renderWindow->Render();
}


void DisplayWidget::loadSTLModel(const std::string& filePath)
{
    // 读取 STL
    auto reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filePath.c_str());
    reader->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    coneActor = vtkSmartPointer<vtkActor>::New();
    coneActor->SetMapper(mapper);

    //  如果 STL 太大/太小/方向错，可以修正：
    coneActor->SetScale(1);            // 比例（可调，比如0.1缩小10倍）

    //coneActor->RotateY(-90);          //调整局部坐标轴与8700339.rom一致
    //coneActor->SetPosition(142, -19.7, 168.9);     // 局部坐标系相对于世界坐标系

	//coneActor->RotateZ(-90);          //调整局部坐标轴与Otool1.rom一致
 //   coneActor->RotateX(90);
	//coneActor->SetPosition(135.4, 23.9, -20.9);

  

    // 创建局部坐标系（跟着 STL 模型动）
    localAxes = vtkSmartPointer<vtkAxesActor>::New();
    localAxes->SetTotalLength(15.0, 15.0, 15.0);  // XYZ 轴长度
    localAxes->SetShaftTypeToCylinder();
    localAxes->SetCylinderRadius(0.01);

    //组合：STL 模型 + 局部坐标系
    modelAssembly = vtkSmartPointer<vtkAssembly>::New();
    modelAssembly->AddPart(coneActor);  // 模型
    modelAssembly->AddPart(localAxes);   // 局部坐标系
    modelAssembly->SetScale(scaleFactor);

    //float x0 = -280 * scaleFactor, y0 = -280*scaleFactor, z0 = -1800*scaleFactor;
    //modelAssembly->SetPosition(x0, y0, z0);      //(STL 模型 + 局部坐标系)相对于世界坐标系的初始位置

    // 加到渲染器
    renderer->AddActor(modelAssembly);

    renderer->ResetCamera();
    renderWindow->Render(); 

    //  初始化 transform

}



void DisplayWidget::addTrackingSideWalls(double scaleFactor)
{
    
    double xMin = -783 * scaleFactor;
    double xMax = 783 * scaleFactor;
    double yMin = -656 * scaleFactor;
    double yMax = 656 * scaleFactor;
    double zMin = -2400 * scaleFactor;
    double zMax = -950 * scaleFactor;

    double pts[8][3] = {
        { 262 * scaleFactor,  zMax,  310 * scaleFactor},    // p1 (yMax_small, zMax, xMax_small)
        { 656 * scaleFactor,  zMin,  783 * scaleFactor},    // p2 (yMax_large, zMin, xMax_large)
        { 262 * scaleFactor,  zMax, -310 * scaleFactor},    // p3 (yMax_small, zMax, xMin_small)
        { 656 * scaleFactor,  zMin, -783 * scaleFactor},    // p4 (yMax_large, zMin, xMin_large)
        {-262 * scaleFactor,  zMax,  310 * scaleFactor},    // p5 (yMin_small, zMax, xMax_small)
        {-656 * scaleFactor,  zMin,  783 * scaleFactor},    // p6 (yMin_large, zMin, xMax_large)
        {-262 * scaleFactor,  zMax, -310 * scaleFactor},    // p7 (yMin_small, zMax, xMin_small)
        {-656 * scaleFactor,  zMin, -783 * scaleFactor}     // p8 (yMin_large, zMin, xMin_large)
    };

    auto points = vtkSmartPointer<vtkPoints>::New();
    for (int i = 0; i < 8; ++i)
        points->InsertNextPoint(pts[i]);


    auto polys = vtkSmartPointer<vtkCellArray>::New();


    auto addQuad = [&](int a, int b, int c, int d) {
        auto polygon = vtkSmartPointer<vtkPolygon>::New();
        polygon->GetPointIds()->SetNumberOfIds(4);
        polygon->GetPointIds()->SetId(0, a);
        polygon->GetPointIds()->SetId(1, b);
        polygon->GetPointIds()->SetId(2, c);
        polygon->GetPointIds()->SetId(3, d);
        polys->InsertNextCell(polygon);
        };

    addQuad(0, 1, 5, 4); // 前面 (yMax)
    addQuad(2, 3, 7, 6); // 后面 (yMin)
    addQuad(0, 2, 6, 4); // 左面 (xMax)
    addQuad(1, 5, 7, 3); // 右面 (xMin)

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(polys);

    // 5. Mapper + Actor
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    auto wallsActor = vtkSmartPointer<vtkActor>::New();
    wallsActor->SetMapper(mapper);

    wallsActor->GetProperty()->SetColor(0.0, 0.5, 1.0);  // 蓝色
    wallsActor->GetProperty()->SetOpacity(0.2);          // 半透明

    // 6. 加到渲染器
    renderer->AddActor(wallsActor);
}



void DisplayWidget::applyQuaternionTransform(double tx, double ty, double tz,
    double qw, double qx, double qy, double qz)
{
    if (!modelAssembly) return;

    // 1. 归一化四元数，防止累计误差
    double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm < 1e-8) return;
    qw /= norm; qx /= norm; qy /= norm; qz /= norm;

    // 2. 计算旋转矩阵 (右手系)
    double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    double wx = qw * qx, wy = qw * qy, wz = qw * qz;

    double R[3][3] = {
        {1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)},
        {2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)},
        {2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)}
    };

    // 3. VTK 列优先矩阵
    double M[16] = {
        R[0][0], R[1][0], R[2][0], 0,
        R[0][1], R[1][1], R[2][1], 0,
        R[0][2], R[1][2], R[2][2], 0,
        tx * scaleFactor, ty * scaleFactor, tz * scaleFactor, 1
    };

    modelTransform->SetMatrix(M);
    modelTransform->Modified();  //  保证 VTK 刷新

}



