#ifndef DISPLAYWIDGET_H
#define DISPLAYWIDGET_H

#include <QWidget>
#include <QTimer>
#include "ui_display.h"
#include <vtkSmartPointer.h>
#include <vtkAxesActor.h>
#include<vtkAssembly.h>
#include<vtkOrientationMarkerWidget.h>
#include<vtkTransform.h>
#include <mutex>

class vtkRenderer;
class vtkGenericOpenGLRenderWindow;
class vtkActor;

struct Pose {
    double x, y, z;        // 平移
    double rx, ry, rz;     // 旋转（欧拉角，单位度）
};



class DisplayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DisplayWidget(QWidget* parent = nullptr);
    ~DisplayWidget();

    void clearScene();                      // 清空场景
    vtkSmartPointer<vtkRenderer> getRenderer();

    void startRealtimeAnimation();          //  启动实时动画
    void stopRealtimeAnimation();           //  停止动画

    //void updateExternalPose(const Pose& p); //  外部传感器更新位姿

    vtkSmartPointer<vtkAssembly> dynamicAssembly;   // 运行时实时更新的组合体

    void DisplayWidget::loadSTLModel(const std::string& filePath);
    void DisplayWidget::addTrackingSideWalls(double scaleFactor);

    void applyQuaternionTransform(double tx, double ty, double tz,
        double qw, double qx, double qy, double qz);

    void updateExternalPose(const Pose& p);
    void updateExternalTran(const vtkSmartPointer<vtkTransform>& p);

private slots:
    void updateConePose(); //  QTimer定时刷新
    


private:
    Ui::Form ui;

    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkActor> coneActor;
    vtkSmartPointer<vtkAxesActor> localAxes;
    vtkSmartPointer<vtkAssembly> modelAssembly;  // STL + 坐标轴组合体

    vtkSmartPointer<vtkTransform> modelTransform;   //最新的位姿--旋转矩阵

    vtkSmartPointer<vtkOrientationMarkerWidget> orientationMarkerWidget;

    float scaleFactor = 1;            //缩放因子
    

    QTimer* timer = nullptr;

    Pose latestPose;  // 最新位姿（线程安全可再加锁）

    void addSceneAxes();     //  在场景原点放大坐标轴
    void addCornerAxes();    //  在窗口角落放小坐标系

    std::mutex m_ndiTransformMutex;
    vtkSmartPointer<vtkTransform> m_ndiTransform;
};

#endif // DISPLAYWIDGET_H
