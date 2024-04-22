#include "Copter.h"
//#include "../lib/AttitudeControl/AttitudeControl.h"

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _prio) SCHED_TASK_CLASS(HexCopter, &copter, func, _interval_ticks, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(HexCopter, &copter, func)

const Scheduler::Task HexCopter::scheduler_tasks[] = {
    FAST_TASK_CLASS(SensorHandler, &copter.ins, update),
	FAST_TASK_CLASS(PilotCommand, &copter.pilotcommand, read_rc_input),
    FAST_TASK(run_rate_controller),
	FAST_TASK(motors_output),
	//SCHED_TASK(prearm_check,            10,    130,  10),
	SCHED_TASK(motor_arm_check,           10,    130,  15),
	//SCHED_TASK(hz_check,              1,    130,  5),
};

void HexCopter::get_scheduler_tasks(const Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

void HexCopter::setup(){
    // initialise the main loop scheduler
    const Scheduler::Task *tasks;
    uint8_t task_count;
    uint32_t log_bit;
    get_scheduler_tasks(tasks, task_count, log_bit);
    scheduler.init(tasks, task_count, log_bit);
    ins.init();
    pilotcommand.init();
    control.init(&motors, &pilotcommand, &ins);
    motors.init();
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
}

void HexCopter::loop(){
	scheduler.loop();
}

void HexCopter::read_AHRS()
{
    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    //ahrs.update(true);
	//char MSG[2];
	//sprintf(MSG, "1");
	//HAL_UART_Transmit(&huart3, (uint8_t*)MSG, sizeof(MSG),100);
}

void HexCopter::run_rate_controller()
{
    control.update();
}

void HexCopter::motors_output()
{
	motors.motors_output();
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void HexCopter::hz_check()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
	//uint8_t MSG = 0xFF;
	//uint8_t buf = 0x00;
	//HAL_UART_Transmit(&huart3, &MSG, sizeof(MSG),100);
	//HAL_UART_Receive(&huart3, &buf, 1, 100);
	printf("%ld\n", HAL_GetTick());
	//copter.ins.update();
}

void HexCopter::prearm_check()
{
	armingcheck.prearm_check();
}

void HexCopter::motor_arm_check()
{
	armingcheck.arm_check();
}
