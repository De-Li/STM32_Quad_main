/*
 *  main loop scheduler for APM
 *
 *
 */
#include "stm32f4xx_hal.h"
#include "Scheduler.h"

extern TIM_HandleTypeDef htim6;

// constructor
Scheduler::Scheduler()
{

}

// initialise the scheduler
void Scheduler::init(const Scheduler::Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit)
{
    // grab the semaphore before we start anything
    //_rsem.take_blocking();

    // only allow 50 to 2000 Hz
    /*if (_loop_rate_hz < 400) {
        _loop_rate_hz = 400;
    } else if (_loop_rate_hz > 2000) {
        _loop_rate_hz = 2000;
    }*/
    _loop_rate_hz = 50;
    get_loop_period_us();

    _vehicle_tasks = tasks;
    _num_vehicle_tasks = num_tasks;

    /*AP_Vehicle* vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        vehicle->get_common_scheduler_tasks(_common_tasks, _num_common_tasks);
    }*/

    _num_tasks = _num_vehicle_tasks + _num_common_tasks;

   _last_run = new uint32_t[_num_tasks];
    _tick_counter = 0;
    extra_loop_us = 0;

    HAL_TIM_Base_Start_IT(&htim6);
}

// one tick has passed
void Scheduler::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void Scheduler::run(uint32_t time_available)
{
    uint32_t now = htim6.Instance->CNT;
    uint8_t vehicle_tasks_offset = 0;
    uint8_t common_tasks_offset = 0;

    for (uint8_t i = 0; i < _num_tasks; i++) {
        // determine which of the common task / vehicle task to run
        bool run_vehicle_task = false;
        if (vehicle_tasks_offset < _num_vehicle_tasks &&
            common_tasks_offset < _num_common_tasks) {
            // still have entries on both lists; compare the priorities.
        	// In case of a tie the vehicle-specific entry wins.
            const Task &vehicle_task = _vehicle_tasks[vehicle_tasks_offset];
            const Task &common_task = _common_tasks[common_tasks_offset];
            if (vehicle_task.priority <= common_task.priority) {
                run_vehicle_task = true;
            }
        } else if (vehicle_tasks_offset < _num_vehicle_tasks) {
            // out of common tasks to run
            run_vehicle_task = true;
        } else if (common_tasks_offset < _num_common_tasks) {
            // out of vehicle tasks to run
            //run_vehicle_task = false;
        } else {
            // this is an error; the outside loop should have terminated
            //INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }

        const Scheduler::Task &task = run_vehicle_task ? _vehicle_tasks[vehicle_tasks_offset] : _common_tasks[common_tasks_offset];
        if (run_vehicle_task) {
            vehicle_tasks_offset++;
        } else {
            common_tasks_offset++;
        }

        if (task.priority > MAX_FAST_TASK_PRIORITIES) {
            const uint32_t dt = _tick_counter - _last_run[i];
            // we allow 0 to mean loop rate
            uint32_t interval_ticks = (is_zero(task.rate_hz) ? 1 : _loop_rate_hz / task.rate_hz);
            if (interval_ticks < 1) {
                interval_ticks = 1;
            }

            if (dt < interval_ticks) {
                // this task is not yet scheduled to run again
                continue;
            }
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = task.max_time_micros;

            if (dt >= interval_ticks*max_task_slowdown) {
                // we are going beyond the maximum slowdown factor for a
                // task. This will trigger increasing the time budget
                task_not_achieved++;
            }

            if (_task_time_allowed > time_available) {
                // not enough time to run this task.  Continue loop -
                // maybe another task will fit into time remaining
                continue;
            }
        } else {
            _task_time_allowed = _loop_period_us;
        }

        // run it
        _task_time_started = now;

        //持久化數據結構。 如果有看門狗重置，則此數據會在啟動時恢復。
        //hal.util->persistent_data.scheduler_task = i;
        task.function();
        //hal.util->persistent_data.scheduler_task = -1;

        // record the tick counter when we ran. This drives
        // when we next run the event
        _last_run[i] = _tick_counter;

        // work out how long the event actually took
        now = htim6.Instance->CNT;
        uint32_t time_taken = now - _task_time_started;

        /*bool overrun = false;
        if (time_taken > _task_time_allowed) {
            overrun = true;
            // the event overran!
            debug(3, "Scheduler overrun task[%u-%s] (%u/%u)\n",
                  (unsigned)i,
                  task.name,
                  (unsigned)time_taken,
                  (unsigned)_task_time_allowed);
        }
        perf_info.update_task_info(i, time_taken, overrun);*/
        if (time_taken >= time_available || time_taken < 0) {
            time_available = 0;
            break;
        }
        time_available -= time_taken;
    }

    //剩下的時間-發呆
	while(htim6.Instance->CNT < _loop_period_us){

	}
}

void Scheduler::loop()
{
    // wait for an INS sample
    //hal.util->persistent_data.scheduler_task = -3;
	/*慣性測量(未來要先做)
    _rsem.give();
    AP::ins().wait_for_sample();
    _rsem.take_blocking();
    */
    //hal.util->persistent_data.scheduler_task = -1;

    //const uint32_t sample_time_us = sche_count;

    // tell the scheduler one tick has passed
    //tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    //const uint32_t loop_us = get_loop_period_us();
    //uint32_t now = sche_count;
    //uint32_t time_available = 0;
    /*const uint32_t loop_tick_us = now - sample_time_us;
    if (loop_tick_us < loop_us) {
        // get remaining time available for this loop
        time_available = loop_us - loop_tick_us;
    }*/
    // add in extra loop time determined by not achieving scheduler tasks
    //time_available += extra_loop_us;
	__HAL_TIM_SET_COUNTER(&htim6, 0);

    tick();

    uint32_t time_available = _loop_period_us;
    // run the tasks
    run(time_available);
    /*if (task_not_achieved > 0) {
        // add some extra time to the budget
        extra_loop_us = MIN(extra_loop_us+100U, 5000U);
        task_not_achieved = 0;
        task_all_achieved = 0;
    } else if (extra_loop_us > 0) {
        task_all_achieved++;
        if (task_all_achieved > 50) {
            // we have gone through 50 loops without a task taking too
            // long. CPU pressure has eased, so drop the extra time we're
            // giving each loop
            task_all_achieved = 0;
            // we are achieving all tasks, slowly lower the extra loop time
            extra_loop_us = MAX(0U, extra_loop_us-50U);
        }
    }*/
}
