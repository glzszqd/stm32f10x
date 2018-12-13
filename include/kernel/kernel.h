#ifndef _KERNEL_H_
#define _KERNEL_H_

#define TASK_MAX_NUM		（10）



typedef struct
{
		unsigned char run;
		unsigned int timer;
		unsigned int cycle;
		void (*callback)(void);
}task_object_t;


extern void task_time_isr(void);
extern void task_schedule(void);
extern int task_register(task_object_t *task,const unsigned char num);
extern int kernel_init(void);

#endif
