/*********************************************************************
 * @file main.cpp
 * @author Caio Burns Lessa
 * @date 2022/03/11
 * @brief Arduino main process source file
*********************************************************************/
// doxygen style header comment.

// Includes
#include <Arduino.h>
#include <std_msgs/UInt32.h>
#include "rosbeef.h"

// Start of defines
    // function pins
#define pinCLK    4   // Clock pin   DIR_CLK
#define pinLTCH   12  // Latch pin   DIR_LATCH
#define pinDS     8   // Data pin    DIR_SER
#define pinEnable 7   // Enable pin  DIR_EN

    // Shift register pins position
#define M1A 2
#define M1B 3
#define M2A 1
#define M2B 4
#define M3A 5
#define M3B 7
#define M4A 0
#define M4B 6

    // Motor pwm pins position
#define pinMotor1PWM 11
#define pinMotor2PWM 3
#define pinMotor3PWM 5
#define pinMotor4PWM 6

    // Motor direction masks        CC E
#define TURN_LEFT   0b10010101  //  10 0
#define TURN_RIGHT  0b01101010  //  01 0
#define FORWARD     0b00100111  //  11 0
#define BACKWARD    0b11011000  //  00 0

    // stop   forward   backward    keep
    // 00     01        10          11
#define MSTOP       0
#define MFORWARDS   1
#define MBACKWARDS  2
#define MKEEP       3

    // Minimun interval for a running motor in milliseconds
#define MIN_INTERVAL    50
// End of defines

// Start of function prototypes
/**
 * @fn void runMotorSM()
 * @brief executes the motor state machine
 * @param null
 * @return null
*/
void runMotorSM();

/**
 * @fn void sendMotorResponse()
 * @brief sends motor response to the response node
 * @param void
 * @return void
*/
void sendMotorResponse();

/**
 * @fn void getMotorParams()
 * @brief decodes the message comming from the computer
 * @param void
 * @return void
*/
void getMotorParams();

/**
 * @fn motorShieldWrite(byte ctrlByte)
 * @param ctrlByte motor control byte that contains the shield's output state
 * @return void
*/
void motorShieldWrite(byte ctrlByte);

/**
 * @fn void motorCallback(const std_msgs::UInt32& input)
 * @param input Receives the 32bit message from the Subscriber node
 * @return void
*/
void motorCallback(const std_msgs::UInt32& input);
// End of function prototypes

// Task array
task_t tasks[] = 
{
    {0, 100, runMotorSM},
    {0, 50, sendMotorResponse},
    {0, 25, getMotorParams}
};

// control messaging
bool gotCtrlMessage = false;

// Ros nodes
ros::NodeHandle nh;
std_msgs::String msg;
ros::Publisher pub("motor-response", &msg);
ros::Subscriber<std_msgs::UInt32> sub("motor-input", motorCallback);

// Task system object
rosbeef taskManager(sizeof(tasks)/sizeof(*tasks));

u32 motorTimer[4] = {0};
u8 motorIntervals[4] = {0};

// Timer interrupt vector function (Needed for the task system!)
SIGNAL(TIMER0_COMPA_vect)
{
    taskManager.runTaskTimerCore();
}

// Functions

void runMotorSM()
{

}

void sendMotorResponse()
{

}

void getMotorParams()
{

}

void motorShieldWrite(byte ctrlByte)
{
    // Set latch low, data pin low and clock pin low to start transmission
    PORTB &= ~(0b00010001); // LATCH and DATA low
    bitClear(PORTD, PD4);

    for(int i = 7; i >= 0; i--){
        //set clock low
        bitClear(PORTD, PD4);

        // set port if bit is set else clear port
        if(bit_is_set(ctrlByte, i)){
            bitSet(PORTB, PB0);
        }else{
            bitClear(PORTB, PB0);
        }

        //set clock high and clear data pin
        bitSet(PORTD, PD4);
        bitClear(PORTB, PB0);
    }
    // set latch high
    bitSet(PORTB, PB4);
}

void motorCallback(const std_msgs::UInt32& input)
{
    const u8 channelA[] = {M1A, M2A, M3A, M4A};
    const u8 channelB[] = {M1B, M2B, M3B, M4B};
    static byte motorCtrl = 0;
    u32 data = input.data;
    //m1 m2 m3 m4 t1 t2 t3 t4 i1 i2 i3 i4
    // 2  2  2  2  2  2  2  2  4  4  4  4
    // stop   forward   backward    maintain
    // 00     01        10          11
    for(int i = 0; i < 4; i++)
    {
        u8 c = (data >> (30 - 2*i)) & 0x03;
        u8 gain = (data >> (12 - 4*i)) & 0x0f;
        u8 interval = (data >> (22 - 2*i)) & 0x02;

        motorIntervals[i] = (interval == 0)*1
                          + (interval == 1)*5
                          + (interval == 2)*50
                          + (interval == 3)*200;

        switch(c)
        {
            case(MSTOP):;
                bitClear(motorCtrl, channelA[i]);
                bitClear(motorCtrl, channelB[i]);
                break;
            case(MFORWARDS):;
                bitSet(motorCtrl, channelA[i]);
                bitClear(motorCtrl, channelB[i]);
                break;
            case(MBACKWARDS):;
                bitClear(motorCtrl, channelA[i]);
                bitSet(motorCtrl, channelB[i]);
                break;
            case(MKEEP):;
                break;
        }
    }
}

// Arduino block
void setup(){

}

void loop()
{
    taskManager.runTasks();
}