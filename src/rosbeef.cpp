#include <rosbeef.h>

void rosbeef::runTaskTimerCore()
{
    unsigned long currentMillis = millis();

    if((currentTask >= MAX_TASK_COUNT) || (currentTask >= taskCount))currentTask = 0;

    if(currentMillis - tasks[currentTask].timer >= tasks[currentTask].interval){
        SETBIT(flags, currentTask);
        tasks[currentTask].timer = currentMillis;
    }

    ++currentTask;
}

uint8_t rosbeef::getTaskInProcess(){
    return currentTask;
}

void rosbeef::rosbeefInit(size_t size)
{
    size_t aux = size / sizeof(task_t);

    taskCount = 0;

    if(aux < MAX_TASK_COUNT)
    {
        OCR0A = 0xAF;               // add comparator
        TIMSK0 |= _BV(OCIE0A);      // add interruption

        taskCount = aux;
        return;
    }

    return;
}

void rosbeef::rosbeefInit()
{
    OCR0A = 0xAF;               // add comparator
    TIMSK0 |= _BV(OCIE0A);      // add interruption
    return;
}

void rosbeef::runTasks()
{
    for(u8 i = 0; (i < taskCount) && (i < MAX_TASK_COUNT); i++)
    {
        if(CHECKBIT(flags, i))
        {
            tasks[i].function();
            CLEARBIT(flags, i);
        }
    }
}

void rosbeef::setFlag(uint8_t flag)
{
    if(flag >= MAX_TASK_COUNT)return;
    disableInterrupts();
    SETBIT(flags, flag);
    enableInterrupts();
    return;
}

void rosbeef::clearFlag(uint8_t flag)
{
    if(flag >= MAX_TASK_COUNT)return;
    disableInterrupts();
    CLEARBIT(flags, flag);
    enableInterrupts();
    return;
}