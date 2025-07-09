/*
 * task_timer.c
 *
 *  Created on: Sep 9, 2020
 *      Author: agarcia
 */

#include "user.h"

static timer_task *p_first_task = NULL, *p_last_task = NULL;
static uint32_t n_task = 0;

/* Runtime periodic tasks processing */

void task_timer_runtime(void)
{
	timer_task *p_task = p_first_task;

	while (p_task != NULL){
		if ((p_task->enable) && (p_task->pfunc != NULL)
			&& (TimeDiff(p_task->start, TaskTimerGetTick()) > p_task->period)){
			/* If loop bit is enabled, periodic task
			 * is re-enabled else task is disabled */
			p_task->n_times -= (p_task->n_times) ? 1 : 0;
			p_task->enable = (p_task->n_times || p_task->loop);
			p_task->start = TaskTimerGetTick();
			p_task->f_res = p_task->pfunc(p_task->p_var);
		}

		p_task = p_task->p_next;

		/* If p_task points to first task break from while
		 * to avoid infinite loop */
		if (p_task == p_first_task)
			break;
	}
}

/* Create a new task and add it to process list:
 * Input:
 *		p_task,		pointer to task definition structure
 *		func, 		pointer to task function
 *		var, 		variable pointer passed to task function
 *  	expiration, timeout ticks to call task function
 *  	cycles, 	number of functions calls before stop, if 0 infinite calling
 *  Output:
 *  	None.
 */

void task_timer_create(timer_task *p_task, int32_t (*func)(void *),
		void *var, uint32_t expiration)
{
	timer_task task = {
			.pfunc = func,
			.p_var = var,
			.period = expiration,
			.enable = 0,
			.n_times = 0,
			.loop = 0,
			.start = TaskTimerGetTick()};

	//memcpy(p_task, &task, sizeof(timer_task));
	*p_task = task;

	/*If this is the first task add to head pointer */
	if (p_first_task == NULL){
		p_first_task = p_last_task = p_task;
	} else {
		/* update p_last_task->next to point this task */
		/* Update p_last_task with this new pointer */

		p_last_task->p_next = p_task;
		p_last_task = p_task;
	}

	/* created task is the last task and must be NULL*/
	p_task->p_next = NULL;
	p_task->intialized = 1;
	n_task++;
}

/* Delete task from task calling chain
 * input:
 * 		del_task,	pointer to task to delete
 * 	output:
 * 		none.
 */

void task_timer_delete(timer_task *del_task)
{
	timer_task *p_task = p_first_task;

	while ((p_task->p_next != NULL) && (p_task->p_next != p_first_task)){
		if (p_task->p_next == del_task){
			p_task->p_next = del_task->p_next;
			p_task->intialized = 0;
			if(n_task)
				n_task--;

			break;
		}
		p_task = p_task->p_next;
	}
}

/* Modifies timer settings and parameters
 * input:
 * 		del_task,	pointer to task to delete
 * 		expiration,	timeout time in ticks
 *  	cycles, 	number of functions calls before stop, if 0 infinite calling
 *  Output:
 *  	None.
 */

void task_timer_set(timer_task *p_task, int32_t (*func)(void *),
		void *var, uint32_t expiration)
{
	if (func != NULL)
		p_task->pfunc = func;
	if (var != NULL)
		p_task->p_var = var;

	p_task->period = expiration;
}


void task_timer_start(timer_task *p_task, uint32_t n_cycles)
{
	p_task->start = TaskTimerGetTick();
	p_task->n_times = n_cycles;
	p_task->loop = !n_cycles;
	p_task->enable = 1;
}

void task_timer_stop(timer_task *p_task)
{
	p_task->start = TaskTimerGetTick();
	p_task->enable = p_task->n_times = 0;
}
