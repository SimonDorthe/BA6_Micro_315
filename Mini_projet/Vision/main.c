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
#include <gpio.h>
#include <selector.h>
#include <timer.h>

//#include <pi_regulator.h>
//#include <process_image.h>
#include <"adventure.h">

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    // Enable GPIOB and GPIOD peripheral clock for the LEDs
	RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;

	// BODY_LED init
	gpio_set(BODY_LED);
	gpio_config_output_pushpull(BODY_LED);


    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();

	init_selector();

	//inserer ici code pour controler que toutes les inits se sont bien passées

	//if tout c'est bien passé lancer le mode adventure
	adventure_start();
	// si un device ne répond pas, lancer le mode ERROR

	//stars the threads for the pi regulator and the processing of the image
//	pi_regulator_start();
//	process_image_start();

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
