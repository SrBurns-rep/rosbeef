/*********************************************************************
 * @file    main.cpp
 * @author  SrBurns <caiobl.25@gmail.com>
 * @date    2022/03/11
 * @brief   Arduino main process source file
*********************************************************************/
// doxygen style header comment.

// Includes
#include <Arduino.h>
#include <std_msgs/UInt32.h>
#include "rosbeef.h"

// Start of defines
    // function pins
#define PIN_CLK    4   // Clock pin   DIR_CLK
#define PIN_LTCH   12  // Latch pin   DIR_LATCH
#define PIN_DS     8   // Data pin    DIR_SER
#define PIN_ENABLE 7   // Enable pin  DIR_EN

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
#define M1PWM 11
#define M2PWM 3
#define M3PWM 5
#define M4PWM 6

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

    // Minimun interval for running a motor in milliseconds
#define MIN_INTERVAL    50
// End of defines

// Start of function prototypes
void timeoutMotors();
void sendResponse();
void shieldWrite(byte ctrlByte);
void callback(const std_msgs::UInt32& input);
byte motorDecode(uint32_t data);
// End of function prototypes

task_t tasks[] = 
{
    {0, MIN_INTERVAL, timeoutMotors},
    {0, 200, sendResponse}
};

// control messaging
bool gotCtrlMessage = false;

// Ros nodes
ros::NodeHandle nh;
std_msgs::String msg;
ros::Publisher pub("motor-response", &msg);
ros::Subscriber<std_msgs::UInt32> sub("motor-input", callback);

// Task system object
rosbeef taskManager(sizeof(tasks)/sizeof(*tasks));

u32 motorTimer[4] = {0};
u16 motorTimeleft[4] = {0};
u8 motorIntervals[4] = {0};
u8 motorIntensities[4] = {0};

const u8 channelA[] = {M1A, M2A, M3A, M4A};
const u8 channelB[] = {M1B, M2B, M3B, M4B};
char buffer[256] = {0};

// Timer interrupt vector function (Needed for the task system!)
SIGNAL(TIMER0_COMPA_vect)
{
    taskManager.runTaskTimerCore();
}

// Common block
void setup()
{
    pinMode(PIN_CLK, OUTPUT);
    pinMode(PIN_LTCH, OUTPUT);
    pinMode(PIN_DS, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);

    pinMode(M1PWM, OUTPUT);
    pinMode(M2PWM, OUTPUT);
    pinMode(M3PWM, OUTPUT);
    pinMode(M4PWM, OUTPUT);

    digitalWrite(PIN_ENABLE, LOW);

    analogWrite(M1PWM, 0);
    analogWrite(M2PWM, 0);
    analogWrite(M3PWM, 0);
    analogWrite(M4PWM, 0);

    taskManager.rosbeefInit();

    nh.initNode();
    nh.advertise(pub);  // register the subscriber
    nh.subscribe(sub);  // register the publisher
}

void loop()
{
    taskManager.runTasks();
}

// Functions
void timeoutMotors()
{
    bool hasTimeout = false;
    uint32_t code = 0xff000000;
    uint32_t current = millis();
    for(int i = 0; i < 4; i++){

        if(motorTimer[i] > current) motorTimer[i] = 0; // timer overflow occurred

        if((u32)(current - motorTimer[i]) >= (u32)(MIN_INTERVAL * motorIntervals[i])) {
            motorTimer[i] = current;
            motorTimeleft[i] = 0;
            code &= (uint32_t)(~(0x03 << (30 - 2*i)));
            hasTimeout = true;
        } else {
            motorTimeleft[i] = (u16)(current - motorTimer[i]);
        }
    }
    if(hasTimeout){
        shieldWrite(motorDecode(code));
    }
}

void sendResponse()
{
    if(gotCtrlMessage){
        snprintf(buffer, sizeof buffer, "%lu,%u,%u,%u,%u", millis(),
        motorIntensities[0], motorIntensities[1], motorIntensities[2], motorIntensities[3]);
        // ROS deve ter coisa pra isso
        // Ver no firefox: ROS / rosserial
        msg.data = buffer;
        pub.publish(&msg);
        nh.spinOnce();

        gotCtrlMessage = false;
    }
}

void shieldWrite(byte ctrlByte)
{
    // Disable interrupts
    cli();
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
    // enable interrupts
    sei();
}

void callback(const std_msgs::UInt32& input)
{
    byte ctrl = motorDecode(input.data);
    shieldWrite(ctrl);
    gotCtrlMessage = true;
}

byte motorDecode(uint32_t data){
    const u8 pwms[] = {M1PWM, M2PWM, M3PWM, M4PWM};
    static byte motorCtrl = 0;  //maintain data

    //m1 m2 m3 m4 t1 t2 t3 t4 i1 i2 i3 i4
    // 2  2  2  2  2  2  2  2  4  4  4  4
    // stop   forward   backward    keep
    // 00     01        10          11

    for(int i = 0; i < 4; i++)
    {
        u8 c = (data >> (30 - 2*i)) & 0x03;
        u8 gain = map((data >> (12 - 4*i)) & 0x0f, 0, 15, 0, 255);
        u8 interval = (data >> (22 - 2*i)) & 0x02;

        motorIntervals[i] = (interval == 0)*1
                          + (interval == 1)*5
                          + (interval == 2)*50
                          + (interval == 3)*200;
        motorIntensities[i] = gain;

        if(c != MKEEP){
            digitalWrite(pwms[i], gain);
        }

        switch(c)
        {
            case(MSTOP):;
                bitClear(motorCtrl, channelA[i]);
                bitClear(motorCtrl, channelB[i]);
                digitalWrite(pwms[i], 0);  // stop motor pwm
                break;
            case(MFORWARDS):;
                bitSet(motorCtrl, channelA[i]);
                bitClear(motorCtrl, channelB[i]);
                motorTimer[i] = millis();
                break;
            case(MBACKWARDS):;
                bitClear(motorCtrl, channelA[i]);
                bitSet(motorCtrl, channelB[i]);
                motorTimer[i] = millis();
                break;
            case(MKEEP):;
                break;
        }
    }
    return motorCtrl;
}