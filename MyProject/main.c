#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "sensors/proximity.h"
#include "motors.h"

#include "epuck1x/uart/e_uart_char.h"
#include "stdio.h"
#include "serial_comm.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	serial_start();

	proximity_start();
    calibrate_ir();
    motors_init();

    char str[100];
    int str_length0;
    int str_length1;



    /* Infinite loop. */
    int counter = 0;
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(50);
        int prox0 = get_calibrated_prox(0);
        int prox1 = get_calibrated_prox(1);
        int prox2 = get_calibrated_prox(2);
        int prox3 = get_calibrated_prox(3);
        int prox4 = get_calibrated_prox(4);
        int prox5 = get_calibrated_prox(5);
        int prox6 = get_calibrated_prox(6);
        int prox7 = get_calibrated_prox(7);

        if (counter >= 4) {
        	counter = 0;
        	str_length0 = sprintf(str, "prox6 %d prox7 %d prox0 %d prox1 %d\n",prox6, prox7, prox0, prox1);
        	e_send_uart1_char(str, str_length0);
        }
        //str_length1 = sprintf(str, "prox1 %d!\n",prox1);
        //e_send_uart1_char(str, str_length1);

//        if (prox0>50 && prox1>50 && prox2>50){
        if (prox0>40 && prox7>40){
        	// obstacle in the front
        	left_motor_set_speed(-500);
        	right_motor_set_speed(500);
        } else if (prox0>40 || prox1>40){
        	left_motor_set_speed(-500);
        	right_motor_set_speed(500);
        }
//        else if (prox7>50 && prox6>50 && prox5>50){
        else if (prox7>40 || prox6>40){
        	left_motor_set_speed(500);
        	right_motor_set_speed(-500);
        }
        else {
        	left_motor_set_speed(500);
        	right_motor_set_speed(500);
        }
        counter++;
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
