#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <sensors/VL53L0X/VL53L0X.h>

//simple PI regulator implementation
int16_t pi_regulator_distance(float distance, float goal){

	float  error = 0;
	float  speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
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

int16_t pi_regulator_angle(float distance, float goal){

    float  error = 0;
    float  speed = 0;

    static float sum_error = 0;

    error = distance - goal;

    //disables the PI regulator if the error is to small
    //this avoids to always move as we cannot exactly be where we want and 
    //the camera is a bit noisy
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

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //get_dist_mm is modified by the TOF thread
        speed = pi_regulator((float)VL53L0X_get_dist_mm()*CM, DIST_PLAY*CM);
        //computes a correction factor to let the robot rotate to be aligned with the sound source
        speed_correction = 6; //get_angle(); // TRIANGULATION POUR LA CORRECTION

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
