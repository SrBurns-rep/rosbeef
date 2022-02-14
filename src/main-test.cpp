/*#include "rosbeef.h"
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;
std_msgs::String msg;
ros::Publisher pub("motor-response", &msg);


void runMotor();
void sendMessage();
void emergencyBrake();

task_t tasks[] = {
                  {0, 100, runMotor},
                  {0, 200, sendMessage},
                  {0, 50, emergencyBrake}
                 };

rosbeef taskManager(sizeof(tasks)/sizeof(task_t));

void motor_cb(const std_msgs::UInt16& input)
{
    //do something
}

ros::Subscriber<std_msgs::UInt16> sub("motor-input", motor_cb);

SIGNAL(TIMER0_COMPA_vect)
{
    taskManager.runTaskTimerCore();
}

void setup()
{
    taskManager.rosbeefInit();
    // pin modes
    nh.initNode();
    // advertise subscriber??
}

void loop()
{
    taskManager.runTasks();
}

void runMotor()
{

}

void sendMessage()
{

}

void emergencyBrake()
{   

}

void ci74HC595Write(byte pino, bool estado) {
  static byte ciBuffer = 0;

  bitWrite(ciBuffer, pino % 8, estado);
  
  digitalWrite(12, LOW); //Inicia a Transmissão
  
  digitalWrite(8, LOW);    //Apaga Tudo para Preparar Transmissão
  digitalWrite(4, LOW);

  for (int nC = 0; nC >= 0; nC--) {
      for (int nB = 7; nB >= 0; nB--) {
  
          digitalWrite(4, LOW);  //Baixa o Clock      
          
          digitalWrite(8,  bitRead(ciBuffer, nB) );     //Escreve o BIT
          
          digitalWrite(4, HIGH); //Eleva o Clock
          digitalWrite(8, LOW);     //Baixa o Data para Previnir Vazamento      
      }  
  }
  
  digitalWrite(12, HIGH);  //Finaliza a Transmissão
}*/