#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include "selector.h"
#include <adventure.h>
#include <process_image.h>
#include <sensors/proximity.h>
#include <leds.h>
#include "spi_comm.h"
#include <audio/play_melody.h>
#include <audio/audio_thread.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
*/
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

    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	spi_comm_start();
	//init and calibration IR sensors
	proximity_start();
	calibrate_ir();
	 //init the molody thread
	 dac_start();
	 playMelodyStart();

    //init LEDs RGB
	set_rgb_led(LED2, 0, 0, 0);
    set_rgb_led(LED4, 0, 0, 0);
    set_rgb_led(LED6, 0, 0, 0);
    set_rgb_led(LED8, 0, 0, 0);

    chprintf((BaseSequentialStream *) &SD3, "adventure Start\n \r");
	adventure_start();
	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	process_image_start();

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
