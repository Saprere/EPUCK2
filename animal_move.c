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

static float angle_sonore;

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
// #define FREQ_PREY		25	//~400Hz frequence at witch the robot hunts
// #define FREQ_PLAY		35	//frequence at wich the robot plays
// #define FREQ_PANIC		40	//frequence at wich the robot panics
// #define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing
#define FREQ_PREY		16	//frequence at witch the robot hunts
#define FREQ_PLAY		19	//frequence at wich the robot plays
#define FREQ_PANIC		23	//frequence at wich the robot panics
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_PREY_L			(FREQ_PREY-1)
#define FREQ_PREY_H			(FREQ_PREY+1)
#define FREQ_PLAY_L			(FREQ_PLAY-1)
#define FREQ_PLAY_H			(FREQ_PLAY+1)
#define FREQ_PANIC_L		(FREQ_PANIC-1)
#define FREQ_PANIC_H		(FREQ_PANIC+1)

#define DIST_PREY			100
#define DIST_PLAY			50
#define SIGNAL_FREQ //DEFINIR AL FREQUENCE OU CREER UNE VARIABLE POUR LA FONCTION animal

//define for the cm convertor
#define CM						(10)
#define MOTOR_SPEED_CRUISE		(600)

static THD_WORKING_AREA(waAnimal, 256);
static THD_FUNCTION(Animal, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time; ///// UTILE ?????

    int16_t speed = 0;
    int16_t speed_correction = 0;

    uint16_t distance_TOF = 0;

    float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

    while(1){
        time = chVTGetSystemTime(); ///// ??????????

		//distance_TOF is updated by VL53L0X TOF library
		distance_TOF = VL53L0X_get_dist_mm();

        max_norm_index = get_sound_animal_var();

        //computes the speed to give to the motors
        speed = pi_regulator((float)distance_TOF*CM, DIST_PLAY);

        //computes a correction factor to let the robot rotate to be aligned with the sound source
        speed_correction = 6; // regulation angle POUR LA CORRECTION

        //if the angle variation is neglectable, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

		//The robot moves towards the prey
		if(max_norm_index >= FREQ_PREY_L && max_norm_index <= FREQ_PREY_H){
			if(distance_TOF <= DIST_PREY){
				left_motor_set_speed(MOTOR_SPEED_CRUISE - ROTATION_COEFF * speed_correction); 
				right_motor_set_speed(MOTOR_SPEED_CRUISE + ROTATION_COEFF * speed_correction);
				}
				
			else{
				left_motor_set_speed(MOTOR_SPEED_LIMIT);
				right_motor_set_speed(MOTOR_SPEED_LIMIT);
			}
		}
		//The robot starts playing
		else if(max_norm_index >= FREQ_PLAY_L && max_norm_index <= FREQ_PLAY_H){
			// right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
			// left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
			left_motor_set_speed(-MOTOR_SPEED_LIMIT);
        	right_motor_set_speed(-MOTOR_SPEED_LIMIT);
		}
		//The robot panics
		else if(max_norm_index >= FREQ_PANIC_L && max_norm_index <= FREQ_PANIC_H){

			left_motor_set_speed(MOTOR_SPEED_LIMIT);
			//right_motor_set_speed(-MOTOR_SPEED_CRUISE);
		}

		else{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void animal_start(void){
	chThdCreateStatic(waAnimal, sizeof(waAnimal), NORMALPRIO, Animal, NULL);
}
