#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define ROTATION_THRESHOLD		50
#define ROTATION_COEFF			2.0f
#define	GOAL_ANGLE				0.0f
#define ERROR_THRESHOLD			0.1f	//MODIFY IF NOISE COMING FROM TOF
#define KP						160.0f
#define KI 						0.05f	//must not be zero
#define MAX_SUM_ERROR			(MOTOR_SPEED_LIMIT/10)
#define MOTOR_SPEED_CRUISE		500
#define DIST_GOAL				200
#define DIST_TRESHOLD			20
#define DIST_TRESHOLD_H			(DIST_GOAL + DIST_TRESHOLD)
#define DIST_TRESHOLD_L			(DIST_GOAL - DIST_TRESHOLD)

#define LED_FRONT				0
#define LED_RIGHT				2
#define LED_LEFT				6
#define LED_BACK				4 

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
