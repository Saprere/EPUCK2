#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define FFT_SIZE 	1024

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



//gets the signal, does FFT, calculate the amplitude of FFT, calls the other functions
void processAudioData(int16_t *data, uint16_t num_samples);

// returns the functionning mode corresponding to predefined working frequencies
int8_t get_f_mode(void);

// initialiazes the file
void audio_init(void);

//return the current angle of the sound direction
double get_angle(void);

void wait_send_to_computer(void);

float* get_audio_buffer_ptr(BUFFER_NAME_t name);


#endif /* AUDIO_PROCESSING_H */
