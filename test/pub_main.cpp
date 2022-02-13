#include <rosbeef.h>

#define MESSAGE_INTERVAL 1000

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("motor-out", &str_msg);   /*chatter("chatter", &str_msg);*/

char msg[] = "Hello world!";

void sendMessage();
void toggleLED();

task_t tasks[] = {{0, 1000, sendMessage}, {0, 100, toggleLED}};


rosbeef taskManager(sizeof(tasks) / sizeof(task_t));	

SIGNAL(TIMER0_COMPA_vect) 
{
	taskManager.runTaskTimerCore();
}

void setup()
{
	taskManager.rosbeefInit();
	pinMode(LED_BUILTIN, OUTPUT);

	nh.initNode();
	nh.advertise(chatter);

}

void loop()
{
	taskManager.runTasks();
}

void sendMessage()
{
	str_msg.data = msg;
	chatter.publish( &str_msg );
	nh.spinOnce();
}

void toggleLED()
{
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}