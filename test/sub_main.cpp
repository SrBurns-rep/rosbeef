/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

unsigned long tspin;
bool lock = true;
char msg[32] = {0};

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg); 

void messageCb( const std_msgs::Empty& toggle_msg){
    digitalWrite(LED_BUILTIN, !digitalRead(13));   // blink the led
    lock = false;
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(chatter);
}

void loop()
{
    static unsigned long t0 = 0;
    static unsigned long t1 = 0;
    static unsigned long timer = 0;
    
    if(millis() - timer >= 1000){
        sprintf(msg, "Time result (off): %luus", tspin);
        str_msg.data = msg;
	    chatter.publish( &str_msg );
        timer = millis();
    }

    if(!lock){
        sprintf(msg, "Time to spin result: %luus", tspin);
        str_msg.data = msg;
	    chatter.publish( &str_msg );
        lock = true;
    }

    t0 = micros();
    nh.spinOnce();
    t1 = micros();
    tspin = t1 - t0;
}