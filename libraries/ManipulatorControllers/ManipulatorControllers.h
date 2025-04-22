#ifndef MANIPULATORCONTROLLERS_H
#define MANIPULATORCONTROLLERS_H

#include <ManipulatorProtocol.h>
// #include <avr_stl.h>
// #include <ArduinoSTL.h>
// #include <vector>

using namespace std;

struct Controller
{
  int enPin;
  int stepPin;
  int dirPin;
};


struct Point
{
    int positions[6];
};

struct Trajectory
{
    Point points[128];
    int index;
    int size;
};


class ManipulatorControllers
{
public:
    ManipulatorControllers(int degrees_);
    virtual ~ManipulatorControllers();

    void manipulatorRun();      //机械臂运动
    void init();        //针脚功能设置
    void poweron();     //使能端开启
    void poweroff();    //使能端关闭

    void addController(int seq, Controller & controller);

    void setPlusePeriod(int p_ = 20);

    void loadNextPoint();       //加载下一个点

    bool isArrived();           //feedback是否达到goal的要求
	
    Trajectory trajectory;      //路径
    StepMotor * goalList;       //当前期望的机械臂的信息
    StepMotor * feedBackList;   //当前机械臂的信息

private:
    void controllerRun(int seq , int plu , int duration);    //控制器运动
    int getMaxPlu();    //获取数组最大值
    int pointIndex;     //当前点的序号
    int degrees;        //自由度
    int plusePeriod;       //节拍周期设置
    Controller * controllersList;      //控制器列表
    int * currentPlu;        //contrllerRun函数中，已经执行的节拍数
    int * expectPlu;         //contrllerRun函数中，需要执行的节拍数

};

#endif //MANIPULATORCONTROLLERS_H
