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

#include <pi_regulator.h>
#include <sensors/VL53L0X/VL53L0X.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];

static float audio_angle;
static float audio_angle_old;
static int8_t mode;

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing

// CASE 1
#define FREQ_PREY		16	//frequence at witch the robot hunts
//CASE 2
#define FREQ_PLAY		19	//frequence at wich the robot plays
//CASE 3
#define FREQ_PANIC		23	//frequence at wich the robot panic

#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define MIN_VALUE_THRESHOLD	15000

// angle en radian
#define ANGLE_THRESHOLD 1
//cte de conversion
#define ANGLE_CONVERT 2.85
//cte de lissage exponentiel
#define ALPHA 0.65

#define INVALID_COUNT 2

#define FREQ_PREY_L			(FREQ_PREY-1)
#define FREQ_PREY_H			(FREQ_PREY+1)
#define FREQ_PLAY_L			(FREQ_PLAY-1)
#define FREQ_PLAY_H			(FREQ_PLAY+1)
#define FREQ_PANIC_L		(FREQ_PANIC-1)
#define FREQ_PANIC_H		(FREQ_PANIC+1)

//PRIVATE FUNCTIONS =======================================================


uint16_t frequency_processing(float* data1,float* data2){

	//permet de respecter la dernière condition
	float max_norm1 = MIN_VALUE_THRESHOLD ;
	float max_norm2 = MIN_VALUE_THRESHOLD ;

	int16_t max_norm_index1 = 0;
	int16_t max_norm_index2 = 0;

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


	if(max_norm1 > MIN_VALUE_THRESHOLD && max_norm2 > MIN_VALUE_THRESHOLD
	   && max_norm_index1 == max_norm_index2){
		return max_norm_index1;
	}else{
		return 0;
	}

}



void angle_calculator(uint16_t signal_freq_index){
	chprintf((BaseSequentialStream *)&SD3,"2 = %lf \n",audio_angle*(180/3.14) );
	double phase_right = 0 ;
	double phase_left = 0 ;

	phase_right = atan2(micRight_cmplx_input[signal_freq_index * 2 + 1],micRight_cmplx_input[signal_freq_index * 2]);
	phase_left = atan2(micLeft_cmplx_input[signal_freq_index * 2 + 1], micLeft_cmplx_input[signal_freq_index * 2]);
	audio_angle = (phase_left - phase_right) * ANGLE_CONVERT;


	//Permet de filtrer les valeurs hors limites
	if(audio_angle > audio_angle_old + ANGLE_THRESHOLD || audio_angle < audio_angle_old - ANGLE_THRESHOLD){
		audio_angle = audio_angle_old;
	}else{
		audio_angle = audio_angle * ALPHA + (1 - ALPHA) * audio_angle_old;
		audio_angle_old = audio_angle;
	}

}

uint8_t mode_selector(uint16_t f){
	if(f >= FREQ_PREY_L && f <= FREQ_PREY_H){
		return 1;
	}else if(f >= FREQ_PLAY_L && f <= FREQ_PLAY_H){
		return 2;
	}else if(f >= FREQ_PANIC_L && f <= FREQ_PANIC_H	){
		return 3;
	}else{
		return 0;
	}
}

//PUBLIC FUNCTIONS ========================

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
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else{
		return NULL;
	}
}

void audio_init(){
	audio_angle_old = 0;
	audio_angle = 0;
	mode = 0;
}

void processAudioData(int16_t *data, uint16_t num_samples){
	static uint16_t nb_samples = 0;
	uint16_t f_index;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){

		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		//	FFT proccessing

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);


		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3

		nb_samples = 0;

		f_index = frequency_processing(micLeft_output,micRight_output);
		
		mode = mode_selector(f_index);

		if(mode == 1 || mode == 2){
			angle_calculator(f_index);
		}
		else{

			audio_angle = 0;
		}
	}
}

double get_angle(){
	return audio_angle;
}

int8_t get_mode(){
	return mode;
}


