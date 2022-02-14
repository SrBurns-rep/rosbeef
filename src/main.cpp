#include <Arduino.h>

#define pinCLK    4   //Pino Clock  DIR_CLK
#define pinLTCH   12  //Pino Latch  DIR_LATCH
#define pinDS     8   //Pino Data   DIR_SER
#define pinEnable 7   //Pino Enable DIR_EN

#define bitMotor1A 2
#define bitMotor1B 3
#define bitMotor2A 1
#define bitMotor2B 4
#define bitMotor3A 5
#define bitMotor3B 7
#define bitMotor4A 0
#define bitMotor4B 6

#define pinMotor1PWM 11
#define pinMotor2PWM 3
#define pinMotor3PWM 5
#define pinMotor4PWM 6
                                //CC E
#define TURN_LEFT   0b10010101  //10 0
#define TURN_RIGHT  0b01101010  //01 0
#define FORWARD     0b00100111  //11 0
#define BACKWARD    0b11011000  //00 0

// Motor states
#define MS_STOP     0
#define MS_LEFT     1
#define MS_RIGHT    2
#define MS_FORWARD  3
#define MS_BACKWARD 4
// internal
#define MS_DEFAULT  5
#define MS_SETINVAL 6
#define MS_READCMD  8

uint16_t gCMD = 0;
bool gCMD_FLAG = false;

void motorShieldWrite(byte msb){
    cli();
    digitalWrite(pinLTCH, LOW); //Inicia transmissão
    digitalWrite(pinDS, LOW);
    digitalWrite(pinCLK, LOW);

    for(int i = 7; i >= 0; i--){
        digitalWrite(pinCLK, LOW);
        digitalWrite(pinDS, !!(msb & (1 << i))); //pega o valor do bit atual
        digitalWrite(pinCLK, HIGH);
        digitalWrite(pinDS, LOW);
    }
    digitalWrite(pinLTCH, HIGH);
    sei();
}

void runStates(){
    static int state = MS_STOP;
    static uint32_t timer = 0;
    static uint32_t interval = 0;

    if(gCMD){
        state = MS_READCMD;
    }

    switch (state)
    {
    case MS_READCMD:
        if(gCMD){
            state = gCMD >> 13;
        }
        break;
    case MS_STOP:
        motorShieldWrite(MS_STOP);
        gCMD_FLAG = false;
        state = MS_READCMD;
        break;
    case MS_LEFT:
        motorShieldWrite(TURN_LEFT);
        state = MS_SETINVAL;
        break;
    case MS_RIGHT:
        motorShieldWrite(TURN_RIGHT);
        state = MS_SETINVAL;
        break;
    case MS_FORWARD:
        motorShieldWrite(FORWARD);
        state = MS_SETINVAL;
        break;
    case MS_BACKWARD:
        motorShieldWrite(BACKWARD);
        state = MS_SETINVAL;
        break;
    case MS_SETINVAL:
        interval = (uint32_t)(gCMD & 0x00FF) * 10;
        timer = millis();
        int intensity = map((gCMD >> 8) & 0x1F, 0, 31, 0, 255);
        gCMD = 0;   // Clearing command
        analogWrite(pinMotor1PWM, intensity);
        analogWrite(pinMotor2PWM, intensity);
        analogWrite(pinMotor3PWM, intensity);
        analogWrite(pinMotor4PWM, intensity);
        state = MS_DEFAULT;
        break;
    case MS_DEFAULT:
        if(millis() < timer){
            timer = 0;  // just in case of an overflow
        }

        if(millis() - timer > interval){
            gCMD_FLAG = true;
            state = MS_STOP;
        }
        break;
    default:
        break;
    }
}

void setup(){
    pinMode(pinCLK, OUTPUT);
    pinMode(pinLTCH, OUTPUT);
    pinMode(pinDS, OUTPUT);
    pinMode(pinEnable, OUTPUT);

    pinMode(pinMotor1PWM, OUTPUT);
    pinMode(pinMotor2PWM, OUTPUT);
    pinMode(pinMotor3PWM, OUTPUT);
    pinMode(pinMotor4PWM, OUTPUT);

    digitalWrite(pinEnable, LOW);

    Serial.begin(9600);
}

void loop(){
    static bool lock = false;
    if(!lock){
        gCMD_FLAG = true;

        lock = true;
    }

    runStates();
}