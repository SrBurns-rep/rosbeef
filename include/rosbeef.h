#ifndef ROSBEEF_H
#define ROSBEEF_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <bitmacros.h>

#define MAX_TASK_COUNT 64
#define enableInterrupts()  sei()
#define disableInterrupts() cli()

typedef struct task_t
{
    unsigned long timer;
    const unsigned long interval;
    void (*function)();
}task_t;

extern task_t tasks[];

class rosbeef
{
    public:
    void rosbeefInit(size_t);
    void rosbeefInit();
    void runTasks();
    void setFlag(uint8_t);
    void clearFlag(uint8_t);
    uint8_t getTaskInProcess();
    void runTaskTimerCore();

    uint8_t taskCount;

    rosbeef(uint8_t numTasks){
        taskCount = numTasks;
    }

    private:
    volatile uint64_t flags;
    volatile uint8_t currentTask;
};

#endif