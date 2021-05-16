#include "ch.h"
#include "hal.h"
#include <main.h>
// #include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <motors.h>
#include <fft.h>
#include <audio_processing.h>
// #include <pi_regulator.h>
#include <sensors/VL53L0X/VL53L0X.h>

int16_t pi_regulator_angle(float angle, float goal){

    static float  error = 0;

    static float  speed = 0;

    static float sum_error = 0;

    error = angle - goal;

    //disables the PI regulator if the error is to small
    //this avoids to always move as we cannot exactly be where we want and 
    if(fabs(error) < ERROR_THRESHOLD/10){
        return 0;
    }

    sum_error += error;

    //we set a maximum and a minimum for the sum to avoid an uncontrolled growth
    if(sum_error > MAX_SUM_ERROR){
        sum_error = MAX_SUM_ERROR;
    }else if(sum_error < -MAX_SUM_ERROR){
        sum_error = -MAX_SUM_ERROR;
    }

    speed = KP * error + KI * sum_error;
		
    return (int16_t)speed;
}


static THD_WORKING_AREA(waMove, 128);
static THD_FUNCTION(Move, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time; 

    int16_t speed_correction = 0;
    uint16_t distance_TOF = 0;

	int8_t mode = 0;

	double angle = 0; 

    while(1){

        time = chVTGetSystemTime(); 

        mode = get_mode();

        //distance_TOF is updated by VL53L0X TOF library
        distance_TOF = VL53L0X_get_dist_mm();

        angle = get_angle();

        //computes a correction factor to let the robot rotate to be aligned with the sound source
        speed_correction = -pi_regulator_angle((float)angle, GOAL_ANGLE); 

  //       if(speed_correction == 0){
	 //        right_motor_set_speed(MOTOR_SPEED_LIMIT);
		// 	left_motor_set_speed(MOTOR_SPEED_LIMIT);
		// }	

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

void move_start(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}
