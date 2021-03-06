#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

// A DEFINIR VOIR TP OU COMMENTAIRE CODE 15hz
#define F_THRESHOLD 1

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

//calculates de index of the frequency
int16_t sound_animal(float* data);

//calculates the angle from the phase of the FFT
void angle_calculator(void);

//return the current angle of the sound direction
double get_angle(void);

//verifies the imput signal has the right frequency of both microphone
bool frequency_check(float* data1,float* data2);

//gets the signal, does FFT, calculate the amplitude of FFT, calls the other functions
void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);


void sound_remote(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
