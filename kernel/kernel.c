
#include <platform.h>
#include <hal.h>
#include "kernel.h"

static task_object_t *task_obj;
static unsigned char task_obj_num;

void task_time_isr(void)
{
	unsigned char i = 0;
	
	for(i = 0;i < task_obj_num;i++)
	{
		if(task_obj[i].timer > 0)
		{
			task_obj[i].timer--;
			if(task_obj[i].timer == 0)
			{
				task_obj[i].run = 1;
				task_obj[i].timer = task_obj[i].cycle;
			}
		}
	}
}

void task_schedule(void)
{
	unsigned char i = 0;
	
	for(i = 0;i < task_obj_num;i++)
	{
		if(task_obj[i].run == 1)
		{
			task_obj[i].callback();
			task_obj[i].run = 0;
		}
	}
}


int task_register(task_object_t *task,const unsigned char num)
{
	task_obj = task;
	if(task != NULL && num <= 20)
	{
		task_obj_num = num;
	}
	else
	{
		task_obj_num = 0;
	}
	return 0;
}

int kernel_init(void)
{
	int i;
	
	task_obj = NULL;
	task_obj_num = 0;
	
	return 0;
}

int main(void)
{
	kernel_init();
	return 0;
}
