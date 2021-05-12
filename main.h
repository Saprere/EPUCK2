#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/* LEDs that can be used in EPUCK2
  LED1 			: GPIOD pin 5
  LED3 			: GPIOD pin 6
  LED5 			: GPIOD pin 10
  LED7 			: GPIOD pin 11
  FRONT_LED 	: GPIOD pin 14
WARNING : Not on the same port !!
  BODY_LED		: GPIOB pin 2
*/
//#define LED1     	GPIOD, 5
//#define LED3     	GPIOD, 6
//#define LED5     	GPIOD, 10
//#define LED7     	GPIOD, 11
//#define FRONT_LED	GPIOD, 14
//#define BODY_LED	GPIOB, 2

//constants for the differents parts of the project
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define GOAL_DISTANCE 			10.0f //REDEFINE DISTANCE GOAL
#define MAX_DISTANCE 			25.0f //REDEFINE DISTANCE GOAL
#define ERROR_THRESHOLD			2.5f	//MODIFY IF NOISE COMING FROM TOF
#define KP						80000.0f
#define KI 						350.0f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define DIST_PLAY				400


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
