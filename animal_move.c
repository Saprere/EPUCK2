#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <fft.h>
#include <audio_processing.h>
#include <pi_regulator.h>
#include <sensors/VL53L0X/VL53L0X.h>

static THD_WORKING_AREA(waAnimal, 128); //A OPTIMIZER
static THD_FUNCTION(Animal, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time; 

    int16_t speed_correction = 0;
    uint16_t distance_TOF = 0;

	int8_t mode = 0;

    while(1){

        time = chVTGetSystemTime(); 

        mode = get_mode();

        //distance_TOF is updated by VL53L0X TOF library
        distance_TOF = VL53L0X_get_dist_mm();

        //computes a correction factor to let the robot rotate to be aligned with the sound source
        speed_correction = pi_regulator((float)get_angle(), GOAL_ANGLE); 

        //if the angle variation is neglectable, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

        switch(mode){

	        //The robot moves towards the prey
	        case 1:

				if(distance_TOF >= DIST_GOAL){
					right_motor_set_speed(MOTOR_SPEED_CRUISE - ROTATION_COEFF * speed_correction);
					left_motor_set_speed(MOTOR_SPEED_CRUISE + ROTATION_COEFF * speed_correction);
				}
				
				else{
					right_motor_set_speed(MOTOR_SPEED_LIMIT);
					left_motor_set_speed(MOTOR_SPEED_LIMIT);
				}


	        	break;

			//The robot starts playing
	        case 2:

	        	if(distance_TOF >= DIST_TRESHOLD_L && distance_TOF <= DIST_TRESHOLD_H){
	        		right_motor_set_speed(0);
					left_motor_set_speed(0);
				 }

	        	else if(distance_TOF >= DIST_GOAL){
	        		right_motor_set_speed(MOTOR_SPEED_CRUISE - ROTATION_COEFF * speed_correction);
					left_motor_set_speed(MOTOR_SPEED_CRUISE + ROTATION_COEFF * speed_correction);

				 }

				 else if(distance_TOF <= DIST_GOAL){
					 right_motor_set_speed(-MOTOR_SPEED_CRUISE - ROTATION_COEFF * speed_correction);
					 left_motor_set_speed(-MOTOR_SPEED_CRUISE + ROTATION_COEFF * speed_correction);
				 }

	        	break;

	        //The robot panics
	        case 3:

	        	left_motor_set_speed(MOTOR_SPEED_LIMIT);


	        	break;

	        //Idle mode
	        default:

	         	left_motor_set_speed(0);
				right_motor_set_speed(0);

	        	break;
	    }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void animal_start(void){
	chThdCreateStatic(waAnimal, sizeof(waAnimal), NORMALPRIO, Animal, NULL);
}
