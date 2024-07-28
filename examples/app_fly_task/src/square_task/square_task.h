#ifndef SQUARETASK_H_
#define SQUARETASK_H_


typedef enum{
    idle,
    flying,
    landing
} State;

typedef enum{
    onGround,
    stage_one,
    stage_two,
    stage_three,
    stage_four,
    next_task
} Stage;



void square_task(void *param);
void land();
void take_off();

#endif