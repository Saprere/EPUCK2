#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <fft.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <pi_regulator.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_PREY		16	//250 Hz frequence at witch the robot hunts
#define FREQ_PLAY		19	//296 Hz frequence at wich the robot plays
#define FREQ_PANIC		23	//359 Hz frequence at wich the robot panics
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_PREY_L			(FREQ_PREY-1)
#define FREQ_PREY_H			(FREQ_PREY+1)
#define FREQ_PLAY_L			(FREQ_PLAY-1)
#define FREQ_PLAY_H			(FREQ_PLAY+1)
#define FREQ_PANIC_L		(FREQ_PANIC-1)
#define FREQ_PANIC_H		(FREQ_PANIC+1)

#define COLLISION_THRESHOLD	65

bool collision_detection(void){

	if(get_calibrated_prox(0) > COLLISION_THRESHOLD){
		return 1;
	}else if(get_calibrated_prox(1) >COLLISION_THRESHOLD){
		return 1;
	}else if(get_calibrated_prox(2) >COLLISION_THRESHOLD){
		return 1;
	}else if(get_calibrated_prox(3) >COLLISION_THRESHOLD){
		return 1;
	}else if(get_calibrated_prox(4) >COLLISION_THRESHOLD){
		return 1;
	}else if(get_calibrated_prox(5) >COLLISION_THRESHOLD){
		return 1;
	}else if(get_calibrated_prox(6) >COLLISION_THRESHOLD){
		return 1;
	}else if(get_calibrated_prox(7) >COLLISION_THRESHOLD){
		return 1;
	}else{
		return 0;
	}
}

static THD_WORKING_AREA(waAnimal, 256); //A OPTIMIZER
static THD_FUNCTION(Animal, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time; 

    int16_t speed = 0;
    int16_t speed_correction = 0;

    uint16_t distance_TOF = 0;

    float max_norm = MIN_VALUE_THRESHOLD;

    //permet de calibrer les IR avec la lumière ambiante
    calibrate_ir();
    //MODIFIER
	int8_t f_mode = 0;

	bool obstacle = 0;

    while(1){


    	bool collision = collision_detection();

    
        time = chVTGetSystemTime(); 


        //MODIFIER
        f_mode = get_f_mode();

        //distance_TOF is updated by VL53L0X TOF library
        distance_TOF = VL53L0X_get_dist_mm();

        //computes the speed to give to the motors
        speed = pi_regulator_distance((float)distance_TOF*CM, DIST_PLAY*CM);

        //computes a correction factor to let the robot rotate to be aligned with the sound source
        speed_correction = pi_regulator_angle((float)get_angle(), 0); // regulation angle POUR LA CORRECTION

        //if the angle variation is neglectable, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

        if(collision){
    		f_mode = 0;
    		obstable = 1;
    	}

        switch(f_mode){

	        //The robot moves towards the prey
	        case 1:

				if(distance_TOF >= DIST_PREY){
					left_motor_set_speed(MOTOR_SPEED_CRUISE - ROTATION_COEFF * speed_correction); 
					right_motor_set_speed(MOTOR_SPEED_CRUISE + ROTATION_COEFF * speed_correction);
				}
				
				else{
					left_motor_set_speed(MOTOR_SPEED_LIMIT);
					right_motor_set_speed(MOTOR_SPEED_LIMIT);
				}

	        	break;

			//The robot starts playing
	        case 2:
	        
				right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(speed  + ROTATION_COEFF * speed_correction);

	        	break;

	        //The robot panics
	        case 3:

	        	left_motor_set_speed(MOTOR_SPEED_LIMIT);

	        	break;

	        //Idle mode
	        default:

	         	left_motor_set_speed(0);
				right_motor_set_speed(0);

				if(obstable){
					wait(10);
					obstacle = 0;
					while(distance_TOF <= DIST_PLAY){
						left_motor_set_speed(-MOTOR_SPEED_CRUISE - ROTATION_COEFF * speed_correction); 
						ight_motor_set_speed(-MOTOR_SPEED_CRUISE + ROTATION_COEFF * speed_correction);
					}
				}

	        	break;
	    }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void animal_start(void){
	chThdCreateStatic(waAnimal, sizeof(waAnimal), NORMALPRIO, Animal, NULL);
}
