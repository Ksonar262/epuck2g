/*
 * task2.c
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

#include "sensors/VL53L0X/VL53L0X.h"


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int motor_speed = 700;
void move_forward(int motor_speed);
void turn_left( int motor_speed);
void turn_right(int motor_speed);
void move_backward(int motor_speed);
void stop(void);

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
    VL53L0X_start();

    char str[100];
    int str_length0;
    int counter = 0;
    int proxy[8];
    int INDEX[8];
    int SUM[8];
    int WINDOW_SIZE=5;
    int READINGS[8][WINDOW_SIZE];
    int distance=0;
    int distance1=0;

    /* Infinite loop. */

    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(30);

        for(int i=0; i<=7; i++){
        	SUM[i] = SUM[i] - READINGS[i][INDEX[i]];
        	READINGS[i][INDEX[i]] = get_calibrated_prox(i);
        	SUM[i] = SUM[i] + get_calibrated_prox(i);
        	INDEX[i] = (INDEX[i]+1)%WINDOW_SIZE;
        	proxy[i] = SUM[i]/WINDOW_SIZE;
        }

        if (counter >= 4) {
        	counter = 0;
        	str_length0 = sprintf(str, "prox6 %d prox7 %d prox0 %d prox1 %d\n",proxy[6], proxy[7], proxy[0], proxy[1]);
        	e_send_uart1_char(str, str_length0);
        }


        if (proxy[6]>40 || proxy[5]>40 || proxy[4]>40){
        	turn_left(motor_speed);
        }
        else if (proxy[1]>40 || proxy[2]>40 || proxy[3]>40){
        	turn_right(motor_speed);
        }

        else if (proxy[7]>40 && proxy[0]>40){
        	distance= VL53L0X_get_dist_mm();
        	        if ((1.1*distance)<distance1){
        	        	move_forward(motor_speed);
        	        }
        	        else if ((1.1*distance)>distance1){
        	        	move_backward(motor_speed);
        	        }
        	        else{
        	        	stop();
        	        }

        	        distance1=distance;
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

void move_backward(int motor_speed)
{
	left_motor_set_speed(-motor_speed);
	right_motor_set_speed(-motor_speed);
}

void stop(void)
{
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}








