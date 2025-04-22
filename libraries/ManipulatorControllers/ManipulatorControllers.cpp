#include "ManipulatorControllers.h"
    
ManipulatorControllers::ManipulatorControllers(int degrees_)
{
    degrees = degrees_;
    goalList = new StepMotor[degrees_];
    feedBackList = new StepMotor[degrees_];
    controllersList = new Controller[degrees_];
    currentPlu = new int[degrees_];
    expectPlu = new int[degrees_];
    StepMotor temp = {0,0,0};
    for (size_t i = 0; i < degrees_; i++)
    {
        goalList[i] = temp;
        feedBackList[i] = temp;
        currentPlu[i] = 0;
        expectPlu[i] = 0;
    }
    trajectory.index = 0;
    trajectory.size = 0;
}

ManipulatorControllers::~ManipulatorControllers()
{
    poweroff();
    delete [] goalList;
    delete [] feedBackList;
    delete [] controllersList;
    delete [] currentPlu;
    delete [] expectPlu;
    goalList = NULL;
    feedBackList = NULL;
    controllersList = NULL;
    currentPlu = NULL;
    expectPlu = NULL;
    degrees = 0;
}

void ManipulatorControllers::manipulatorRun()
{//机械臂运动
    for (size_t seq = 0; seq < degrees; seq ++)
    {
        currentPlu[seq] = 0;
        expectPlu[seq] = goalList[seq].position - feedBackList[seq].position;
        //Serial.print(expectPlu[seq]);Serial.print("  ");
    }
    int maxPlu = getMaxPlu();
    for (size_t i = 1; i <= maxPlu; i ++)
    {//获取节拍最大值，作为循环基本周期
        for (size_t seq = 0; seq < degrees; seq++)
        {//循环基本周期内，每个电机都要执行运动
            double tempExcPlu = (double)expectPlu[seq] * i / maxPlu;
            //本基本周期内电机期望的节拍为 (最终期望节拍) * 当前周期数 / 最大周期数
            int addPlu = (int)tempExcPlu - currentPlu[seq];
            //而将要执行的节拍为  本基本周期内电机期望的节拍 - 当前已经执行的周期数
            controllerRun(seq, addPlu, plusePeriod );
            currentPlu[seq] += addPlu;
            feedBackList[seq].position += addPlu;
        }
    }
    /*
    for (size_t seq = 0; seq < degrees; seq ++)
    {
        feedBackList[seq].position += currentPlu[seq];   //当前机械臂的信息
    }*/
}      

void ManipulatorControllers::init()
{//初始化设置
    for (size_t i = 0; i < degrees; i ++)
    {
        pinMode( controllersList[i].enPin, OUTPUT);
        pinMode( controllersList[i].stepPin, OUTPUT);
        pinMode( controllersList[i].dirPin, OUTPUT);
    }
}

void ManipulatorControllers::poweron()
{//使能端开启
    for (int i = 0; i < degrees; i ++)
    {
        digitalWrite( controllersList[i].enPin, LOW);
    }
}     

void ManipulatorControllers::poweroff()
{//使能端关闭
    for (size_t i = 0; i < degrees; i ++)
    {
        digitalWrite( controllersList[i].enPin, HIGH);
    }
}

void ManipulatorControllers::addController(int seq, Controller & controller)
{//添加控制器
    controllersList[seq] = controller;
}

void ManipulatorControllers::setPlusePeriod( int p_ )
{//节拍周期设置
    plusePeriod = p_;
}

void ManipulatorControllers::loadNextPoint()
{//加载下一个点
    // if (trajectory.points.size() > 0)
    if (trajectory.size > 0)
    {
        if (isArrived())
        {
            for (size_t seq = 0; seq < degrees; seq ++)
            {
                goalList[seq].position = trajectory.points[pointIndex].positions[seq];
            }
            // if ( trajectory.points.size() - 1 > pointIndex )
            if ( trajectory.size - 1 > pointIndex )
            {
                pointIndex ++;      //索引指向下一个点
            }
            else
            {//清理
                pointIndex = 0;

                trajectory.index = 0;
                trajectory.size = 0;
                //trajectory.points.clear();
            }    
        }
    }
}


bool ManipulatorControllers::isArrived()
{
    for (size_t seq = 0; seq < degrees; seq ++)
    {
        if ( goalList[seq].position != feedBackList[seq].position)
        {   
            //cout << "第"<< i + 1 << "号关节没有到位\n";
            return false;
        }
    }
    return true;
}



void ManipulatorControllers::controllerRun(int seq , int plu , int duration)    
{//控制器运动
    if ( plu > 0)
    {
        digitalWrite(controllersList[seq].dirPin, HIGH);
    }
    else
    {
        digitalWrite(controllersList[seq].dirPin, LOW);
    }

    for (int i = 0 ; i < abs(plu) ; i ++)
    {
        digitalWrite(controllersList[seq].stepPin, HIGH);
        delayMicroseconds(duration);
        digitalWrite(controllersList[seq].stepPin, LOW);
        delayMicroseconds(duration);
    }
}


int ManipulatorControllers::getMaxPlu()
{
    int max = abs(expectPlu[0]);
    for (size_t seq = 1; seq < degrees; seq++)
    {
        if (abs(expectPlu[seq - 1]) < abs(expectPlu[seq]))
        {
            max = abs(expectPlu[seq]);
        }
    }
    return max;
}
