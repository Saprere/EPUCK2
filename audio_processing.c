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

<<<<<<< HEAD
static float angle_sonore;
static int16_t sound_animal_var;
=======
static float audio_angle;
static float audio_angle_old;
>>>>>>> audio



#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define SIGNAL_FREQ 	25
#define MIN_SIGNAL_FREQ SIGNAL_FREQ - 1
#define MAX_SIGNAL_FREQ SIGNAL_FREQ + 1
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing
#define MIN_VALUE_THRESHOLD	10000
<<<<<<< HEAD
// angle en radian +- 20ï¿½
#define ANGLE_THRESHOLD 0.35
//cte de conversion A TROUVER!!!
#define ANGLE_CONVERT 3
//cte de lissage exponentiel
#define ALPHA 0.6
#define FREQ_PREY_L			(FREQ_PREY-1)
#define FREQ_PREY_H			(FREQ_PREY+1)
#define FREQ_PLAY_L			(FREQ_PLAY-1)
#define FREQ_PLAY_H			(FREQ_PLAY+1)
#define FREQ_PANIC_L		(FREQ_PANIC-1)
#define FREQ_PANIC_H		(FREQ_PANIC+1)
#define DIST_PREY			50
=======

// angle en radian +- 20°
#define ANGLE_THRESHOLD 0.35
//cte de conversion A TROUVER!!!
#define ANGLE_CONVERT 3

//cte de lissage exponentiel
#define ALPHA 0.6
>>>>>>> audio
/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/


void sound_remote(){
	uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
	//front led
	if(audio_angle >= (M_PI/2) && audio_angle <= (M_PI)){
		led5 = 1;
	}
	//turn front
	else if(audio_angle >= (0) && audio_angle <= (M_PI/2)){
		led7 = 1;
	}
	//go right
	else if(audio_angle >= (-M_PI/2) && audio_angle <= (0)){
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

bool frequency_check(float* data1,float* data2){

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

<<<<<<< HEAD
//	if(max_norm1 > MIN_VALUE_THRESHOLD && max_norm_index1 == SIGNAL_FREQ){
//		if(max_norm2 > MIN_VALUE_THRESHOLD && max_norm_index2 == SIGNAL_FREQ){
//			return 1;
//		}else{
//			return 0;
//		}
//	}else{
//		return 0;
//	}
=======
	if(max_norm1 > MIN_VALUE_THRESHOLD && max_norm_index1 == SIGNAL_FREQ){

		if(max_norm2 > MIN_VALUE_THRESHOLD && max_norm_index2 == SIGNAL_FREQ){
			return 1;
		}else{
			return 0;
		}
	}else{
		return 0;
	}
>>>>>>> audio
}


void audio_init(){

<<<<<<< HEAD
	audio_angle = 0;

}
	audio_angle_old = 0;
=======
void audio_init(){

	audio_angle_old = 0;
	audio_angle = 0;

}
>>>>>>> audio

void angle_calculator(){

<<<<<<< HEAD
	double phase_right = -1 ;
	double phase_left = 1 ;
=======
void angle_calculator(){

	double phase_right = -1 ;
	double phase_left = 1 ;

>>>>>>> audio
	phase_right = atan2(micRight_cmplx_input[SIGNAL_FREQ * 2 + 1],micRight_cmplx_input[SIGNAL_FREQ * 2]);
	phase_left = atan2(micLeft_cmplx_input[SIGNAL_FREQ * 2 + 1], micLeft_cmplx_input[SIGNAL_FREQ * 2]);
	audio_angle = phase_left-phase_right;

<<<<<<< HEAD
	// PROBLEME ICI!
	phase_right = atan2(micRight_cmplx_input[SIGNAL_FREQ*2 + 1],micRight_cmplx_input[SIGNAL_FREQ*2]);
=======
	double angle = audio_angle * ( 180*ANGLE_CONVERT /M_PI);

	//Permet de filtrer les valeurs hors limites
	if(audio_angle > audio_angle_old + ANGLE_THRESHOLD || audio_angle < audio_angle_old - ANGLE_THRESHOLD){

		audio_angle = audio_angle_old;
>>>>>>> audio

	}else{

<<<<<<< HEAD
	phase_left = atan2(micLeft_cmplx_input[SIGNAL_FREQ*2 + 1], micLeft_cmplx_input[SIGNAL_FREQ*2]);

	angle_sonore = phase_left-phase_right;
	//angle_sonore = angle_sonore + M_PI/4;
	//if(angle_sonore > M_PI){
	//	angle_sonore = -2 * M_PI + angle_sonore;
	//}
	double angle = angle_sonore * (360*2/M_PI);
=======
		//Méthode de lissage exponentielle
		audio_angle = audio_angle * ALPHA + (1 - ALPHA) * audio_angle_old;
		audio_angle_old = audio_angle;

	}
>>>>>>> audio

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
		//	FFT proccessing

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

<<<<<<< HEAD
		// avant et arriï¿½re ï¿½ enlever
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

		// avant et arriï¿½re ï¿½ enlever
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3

=======
		//	Magnitude processing

		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

>>>>>>> audio
		nb_samples = 0;

		int16_t f_left = 0;
		int16_t f_right = 0;

<<<<<<< HEAD
			//vï¿½rifier la fonction --> elle ne renvoie pas la frequence pour l'insta
		// problï¿½me avec valid angle

		bool f_valid = frequency_calcul(micLeft_output,micRight_output);
=======
		bool f_valid = frequency_check(micLeft_output,micRight_output);
>>>>>>> audio


		if(f_valid == 1){
			//Ici left et right pas les meme
			angle_calculator();
		}else{
			audio_angle = audio_angle_old;
		}

<<<<<<< HEAD
		// FAIRE SELECTEUR DE CASE AVEC UINT4_T
		sound_animal_var = sound_animal(micLeft_output);
			// Faire une moyenne angulaire sur t_ech pour stabiliser le signal?

=======
		chprintf((BaseSequentialStream *)&SD3,"f3 = %lf \n",angle );
		sound_remote();
>>>>>>> audio
	}
}


<<<<<<< HEAD
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// A GARDER MAIS CHANGER LE NOM
int16_t get_sound_animal_var(void){
	return sound_animal_var;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
=======
double get_angle(){
	return audio_angle;
}

/* A vérifier si on peut enlever
>>>>>>> audio

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
*/

