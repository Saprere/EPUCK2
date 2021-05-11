#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <math.h>

#include <pi_regulator.h>
#include <sensors/VL53L0X/VL53L0X.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

static float angle_sonore;
static int16_t sound_animal_var;


#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
// #define FREQ_FORWARD	16	//250Hz
// #define FREQ_LEFT		19	//296Hz
// #define FREQ_RIGHT		23	//359HZ
// #define FREQ_BACKWARD	26	//406Hz
// #define FREQ_PREY		16	//frequence at witch the robot hunts
// #define FREQ_PLAY		19	//fr�quence at wich the robot plays
// #define FREQ_PANIC		23	//frequence at wich the robot panics
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

// #define FREQ_FORWARD_L		(FREQ_FORWARD-1)
// #define FREQ_FORWARD_H		(FREQ_FORWARD+1)
// #define FREQ_LEFT_L			(FREQ_LEFT-1)
// #define FREQ_LEFT_H			(FREQ_LEFT+1)
// #define FREQ_RIGHT_L		(FREQ_RIGHT-1)
// #define FREQ_RIGHT_H		(FREQ_RIGHT+1)
// #define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
// #define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)
#define FREQ_PREY_L			(FREQ_PREY-1)
#define FREQ_PREY_H			(FREQ_PREY+1)
#define FREQ_PLAY_L			(FREQ_PLAY-1)
#define FREQ_PLAY_H			(FREQ_PLAY+1)
#define FREQ_PANIC_L		(FREQ_PANIC-1)
#define FREQ_PANIC_H		(FREQ_PANIC+1)

#define DIST_PREY			50
#define SIGNAL_FREQ 		16//DEFINIR AL FREQUENCE OU CREER UNE VARIABLE POUR LA FONCTION animal


/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/


//YOU KAN KILL
int16_t sound_animal(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	return(max_norm_index);	
}

// void sound_animal(float* data){
// 	float max_norm = MIN_VALUE_THRESHOLD;
// 	int16_t max_norm_index = -1; 

// 	//search for the highest peak
// 	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
// 		if(data[i] > max_norm){
// 			max_norm = data[i];
// 			max_norm_index = i;
// 		}
// 	}

// 	uint16_t distance_TOF = VL53L0X_get_dist_mm();

// 	//The robot moves towards the prey 
// 	if(max_norm_index >= FREQ_PREY_L && max_norm_index <= FREQ_PREY_H){
// 		if(distance_TOF >= DIST_PREY){
// 			left_motor_set_speed(600);
// 			right_motor_set_speed(600);
			
// 		}
// 		else{
// 			left_motor_set_speed(MOTOR_SPEED_LIMIT);
// 			right_motor_set_speed(MOTOR_SPEED_LIMIT);

// 		}
// 	}
// 	//The robot starts playing
// 	else if(max_norm_index >= FREQ_PLAY_L && max_norm_index <= FREQ_PLAY_H){
// 		left_motor_set_speed(-600);
// 		right_motor_set_speed(600);
// 	}
// 	//The robot panics
// 	else if(max_norm_index >= FREQ_PANIC_L && max_norm_index <= FREQ_PANIC_H){

// 		left_motor_set_speed(600);
// 		//right_motor_set_speed(-600);
// 	}

// 	else{
// 		left_motor_set_speed(0);
// 		right_motor_set_speed(0);
// 	}
	
// }


void sound_remote(){
	uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
	//front led
	if(angle_sonore >= (M_PI/2) && angle_sonore <= (M_PI)){
		led5 = 1;
	}
	//turn front
	else if(angle_sonore >= (0) && angle_sonore <= (M_PI/2)){
		led7 = 1;
	}
	//go right
	else if(angle_sonore >= (-M_PI/2) && angle_sonore <= (0)){
		led1 = 1;
	}
	else{
		led3 = 1;
	}

	palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);

}

bool frequency_calcul(float* data1,float* data2){

	float max_norm1 = MIN_VALUE_THRESHOLD;
	float max_norm2 = MIN_VALUE_THRESHOLD;

	int16_t max_norm_index1 = -1;
	int16_t max_norm_index2 = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data1[i] > max_norm1){
			max_norm1 = data1[i];
			max_norm_index1 = i;
		}
	}
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data2[i] > max_norm2){
			max_norm2 = data2[i];
			max_norm_index2 = i;
		}
	}

//	if(max_norm1 > MIN_VALUE_THRESHOLD && max_norm_index1 == SIGNAL_FREQ){
//		if(max_norm2 > MIN_VALUE_THRESHOLD && max_norm_index2 == SIGNAL_FREQ){
//			return 1;
//		}else{
//			return 0;
//		}
//	}else{
//		return 0;
//	}
}


void angle_calculus(){
	double phase_right = -1 ;
	double phase_left = 1 ;

	// PROBLEME ICI!
	phase_right = atan2(micRight_cmplx_input[SIGNAL_FREQ*2 + 1],micRight_cmplx_input[SIGNAL_FREQ*2]);


	phase_left = atan2(micLeft_cmplx_input[SIGNAL_FREQ*2 + 1], micLeft_cmplx_input[SIGNAL_FREQ*2]);

	angle_sonore = phase_left-phase_right;
	//angle_sonore = angle_sonore + M_PI/4;
	//if(angle_sonore > M_PI){
	//	angle_sonore = -2 * M_PI + angle_sonore;
	//}
	double angle = angle_sonore * (360*2/M_PI);

//	chprintf((BaseSequentialStream *)&SD3,"f1 = %lf \n",micRight_cmplx_input[SIGNAL_FREQ + 1]);
//	chprintf((BaseSequentialStream *)&SD3,"f2 = %lf \n",micRight_cmplx_input[SIGNAL_FREQ] );
	chprintf((BaseSequentialStream *)&SD3,"f3 = %lf \n",angle );

}

void processAudioData(int16_t *data, uint16_t num_samples){

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){

		// TROUVER UN MOYEN DE NE PAS UTILISER LES MIC AVANT ET ARRIERE

		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		//micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		//micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		//micBack_cmplx_input[nb_samples] = 0;
		//micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function.
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

		// avant et arri�re � enlever
		//doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		// avant et arri�re � enlever
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3

		nb_samples = 0;

		int16_t f_left = 0;
		int16_t f_right = 0;

			//v�rifier la fonction --> elle ne renvoie pas la frequence pour l'insta
		// probl�me avec valid angle

		bool f_valid = frequency_calcul(micLeft_output,micRight_output);


		if(f_valid == 1){
			//Ici left et right pas les meme
			angle_calculus();
		}else{
			angle_sonore = 0;
		}

		sound_animal_var = sound_animal(micLeft_output);
			// Faire une moyenne angulaire sur t_ech pour stabiliser le signal?

	}

}

int16_t get_sound_animal_var(void){
	return sound_animal_var;
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
