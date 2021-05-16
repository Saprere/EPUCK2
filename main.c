#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <mode_selector.h>
#include <fft.h>
#include <arm_math.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define SEND_FROM_MIC

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

   /* System init */
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();

    //inits the motors
    motors_init();

    //starts the TOF 
    VL53L0X_start();

    //initialise the audio angle 
    audio_init();

    //stars the threads for the animal move regulator
    move_start();

#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {
        //waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
