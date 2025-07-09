/*
 * task_timer.h
 *
 *  Created on: Sep 9, 2020
 *      Author: agarcia
 */

#ifndef INC_TASK_TIMER_H_
#define INC_TASK_TIMER_H_

/* Define task timer tick function and ticks by us constant */

#define TMR_TICK_US	1		// Timer ticks per microsecond
#define TaskTimerGetTick()	GetTick_us()	// 1000 ticks -> 1ms

#define TASK_TIMER_US(x)	((TMR_TICK_US * x) - 1)
#define TASK_TIMER_MS(x)	((1e3 * TMR_TICK_US * x) - 1)
#define TASK_TIMER_SEC(x)	((1e6 * TMR_TICK_US * x) - 1)

#define TMR_CONTINUOUS	0

typedef struct tag_periodic_task
{
	struct
	{
		uint16_t n_times:12;	/* Number of repetitions, max = 4095 */
		uint16_t loop:1;
		uint16_t intialized:1;	/* Set to true on create and false if delete task */
		uint16_t :1;
		uint16_t enable:1;		/* Time enable bit */
	};
	int16_t f_res;				/* Function result variable */
	uint32_t period;		/* Timer expiration time */
	uint32_t start;			/* Internal operation start variable */
	void *p_var;				/* Input function variable pointer */
	int32_t (*pfunc)(void *);	/* task callback function */
	void *p_next;
}timer_task;

void task_timer_runtime(void);
void task_timer_create(timer_task *, int32_t (*func)(void *), void *, uint32_t);
void task_timer_set(timer_task *, int32_t (*func)(void *), void *, uint32_t );
void task_timer_init(timer_task *);
void task_timer_start(timer_task *p_task, uint32_t n_cycles);
void task_timer_stop(timer_task *);
#endif /* INC_TASK_TIMER_H_ */
