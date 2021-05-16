#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//constants for the differents parts of the project
#define ROTATION_THRESHOLD		50
#define ROTATION_COEFF			0.7f
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

int16_t pi_regulator_angle(float angle, float goal);

#endif /* PI_REGULATOR_H */