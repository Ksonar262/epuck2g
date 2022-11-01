/*
 * task1.c
 *
 *  Created on: 31-Oct-2022
 *      Author: ksona
 */


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

int motor_speed = 700;
void move_forward(int motor_speed);
void turn_left( int motor_speed);
void turn_right(int motor_speed);

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
    int counter = 0;
    int proxy[8];
    int INDEX = 0;
    int SUM = 0;
    int WINDOW_SIZE=5;
    int READINGS[WINDOW_SIZE];


    /* Infinite loop. */

    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(25);

        for(int i=0; i<=7; i++){
        	SUM = SUM - READINGS[INDEX];
        	READINGS[INDEX] = get_calibrated_prox(i);
        	SUM = SUM + get_calibrated_prox(i);
        	INDEX = (INDEX+1)%WINDOW_SIZE;
        	proxy[i] = SUM/WINDOW_SIZE;
        }
        if (counter >= 4) {
        	counter = 0;
        	str_length0 = sprintf(str, "prox6 %d prox7 %d prox0 %d prox1 %d\n",proxy[6], proxy[7], proxy[0], proxy[1]);
        	e_send_uart1_char(str, str_length0);
        }

        if (proxy[0]>40 && proxy[7]>40){
        	// obstacle in the front
        	turn_right(motor_speed);
        }

        else if (proxy[0]>40 || proxy[1]>40 || proxy[2]>30){
        	turn_left(motor_speed);
        }
        else if (proxy[7]>40 || proxy[6]>40 || proxy[5]>30){
        	turn_right(motor_speed);
        }
        else {
        	move_forward(motor_speed);
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

void turn_left( int motor_speed)
{
		left_motor_set_speed(-motor_speed);
		right_motor_set_speed(motor_speed);
}

void turn_right(int motor_speed)
{
		left_motor_set_speed(motor_speed);
		right_motor_set_speed(-motor_speed);
}

void move_forward(int motor_speed)
{
	left_motor_set_speed(motor_speed);
	right_motor_set_speed(motor_speed);
}






