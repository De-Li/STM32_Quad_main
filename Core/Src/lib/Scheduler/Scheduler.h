#pragma once
/*
 *  main loop scheduler for APM
 *
 *
 */
#include "../HAL/functor.h"
#include "../HAL/ftype.h"
#include <cstdint>

/*
  useful macro for creating scheduler task table
 */
#define SCHEDULER_NAME_INITIALIZER(_clazz,_name) .name = #_clazz "::" #_name,
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros, _priority) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    SCHEDULER_NAME_INITIALIZER(classname, func)\
    .rate_hz = _rate_hz,\
    .max_time_micros = _max_time_micros,        \
    .priority = _priority \
}


/*
  useful macro for creating the fastloop task table
 */
#define FAST_NAME_INITIALIZER(_clazz,_name) .name = #_clazz "::" #_name "*",
#define FAST_TASK_CLASS(classname, classptr, func) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    FAST_NAME_INITIALIZER(classname, func)\
    .rate_hz = 0,\
    .max_time_micros = 0,\
    .priority = Scheduler::FAST_TASK_PRI0 \
}

class Scheduler {
public:
	Scheduler();

	FUNCTOR_TYPEDEF(task_fn_t, void);

	struct Task{
		task_fn_t function;
        const char *name;
        float rate_hz;
        uint16_t max_time_micros;
        uint8_t priority; // task priority
	};

    enum FastTaskPriorities {
        FAST_TASK_PRI0 = 0,
        FAST_TASK_PRI1 = 1,
        FAST_TASK_PRI2 = 2,
        MAX_FAST_TASK_PRIORITIES = 3
    };

    // initialise scheduler
	void init(const Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit);

    // called by main loop - which should be the only thing that function does
	void loop();

    // run the tasks. Call this once per 'tick'.
    // time_available is the amount of time available to run tasks in microseconds
    void run(uint32_t time_available);

    // call when one tick has passed
    void tick(void);

    // return current tick counter
    uint16_t ticks() const { return _tick_counter; }

    // get the time-allowed-per-loop in microseconds
    uint32_t get_loop_period_us() {
        if (_loop_period_us == 0) {
            _loop_period_us = 125000 / _loop_rate_hz;
        }
        return _loop_period_us;
    }

private:
    // overall scheduling rate in Hz
    int16_t _loop_rate_hz;

    // calculated loop period in usec
    uint16_t _loop_period_us;

    // number of microseconds allowed for the current task
    uint32_t _task_time_allowed;

    // list of tasks to run
    const struct Task *_vehicle_tasks;
    uint8_t _num_vehicle_tasks;

    // list of common tasks to run
    const struct Task *_common_tasks;
    uint8_t _num_common_tasks;

    // total number of tasks in _tasks and _common_tasks list
    uint8_t _num_tasks;

    // tick counter at the time we last ran each task
    uint32_t *_last_run;

    // tick() has been called
    uint32_t _tick_counter;

    // maximum task slowdown compared to desired task rate before we
    // start giving extra time per loop
    const uint8_t max_task_slowdown = 4;

    // counters to handle dynamically adjusting extra loop time to
    // cope with low CPU conditions
    uint32_t task_not_achieved;
    uint32_t task_all_achieved;

    // the time in microseconds when the task started
    uint32_t _task_time_started;

    // extra time available for each loop - used to dynamically adjust
    // the loop rate in case we are well over budget
    uint32_t extra_loop_us;

    // semaphore that is held while not waiting for ins samples
    //Semaphore _rsem;
};
