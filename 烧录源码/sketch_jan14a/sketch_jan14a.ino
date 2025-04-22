#include <TimerOne.h>
#include <LiquidCrystal_I2C.h>
#include <ManipulatorControllers.h>

ManipulatorProtocol * manipulatorProtocolPtr = new ManipulatorProtocol();
ManipulatorControllers * MCP = new ManipulatorControllers(6);
int eventFlag = 0;     //0:init    1:manipulator move    2~40:read;   

int upload()
{
  for (size_t seq = 0; seq < manipulatorProtocolPtr->writeManipulator.degrees; seq ++)
  {
    //每次步进电机运动，都要更新数据
    manipulatorProtocolPtr->writeManipulator.stepMotorList[seq].position = MCP->feedBackList[seq].position;
  }
  return manipulatorProtocolPtr->write();
}

bool download()
{
  //Serial.println("\ndownload");
  if (manipulatorProtocolPtr->read())
  {
    Point point;
    for (size_t seq = 0; seq < manipulatorProtocolPtr->readManipulator.degrees; seq ++)
    {
      //MCP->goalList[seq].position = manipulatorProtocolPtr->readManipulator.stepMotorList[seq].position;
      //point.positions.push_back( manipulatorProtocolPtr->readManipulator.stepMotorList[seq].position);
      MCP->trajectory.points[MCP->trajectory.size].positions[seq] = manipulatorProtocolPtr->readManipulator.stepMotorList[seq].position;
    }
    
    MCP->trajectory.size += 1;
    //MCP->trajectory.points.push_back(point);
    return true;
  }
  return false;
}

void timerEvent()
{
  if (download())
  {
    eventFlag = 0;      //有数据
  }
  else
  {
    if (eventFlag <= 10 )
    {//等待，看看是否还有数据接收到
      eventFlag ++;
    }
    else
    {//当在 10 * 25000  0.25s内没有获得数据时，开始进行机械臂运动
      upload();
    }
  }
}

//配置
void setup()
{
  Serial.begin(115200);
  Timer1.initialize(25000);      //0.025s
  Timer1.attachInterrupt( timerEvent );
  MCP->setPlusePeriod(50);
}

int main()
{
  init();
  setup();

  //2004LCD

  //Protocol
  manipulatorProtocolPtr->manipulatorInit(6);

  //Controllers
  Controller joint_1 = { 32 , 39, 37 };
  Controller joint_2 = { 32 , 43, 41 };
  Controller joint_3 = { 32 , 47, 45 };
  Controller joint_4 = { A8 , 46, 48 };
  Controller joint_5 = { A2 , A6, A7 };
  Controller joint_6 = { 38 , A0 , A1 };
  MCP->addController(0 , joint_1);
  MCP->addController(1 , joint_2);
  MCP->addController(2 , joint_3);
  MCP->addController(3 , joint_4);
  MCP->addController(4 , joint_5);
  MCP->addController(5 , joint_6);
  MCP->init();
  MCP->poweron();     //使能端开启


  interrupts();
  while (true)
  {
    //运动
    if (eventFlag > 10 )
    {
      MCP->manipulatorRun();
      MCP->loadNextPoint();
    }
    delay(1);
  }
  return 0;
}
