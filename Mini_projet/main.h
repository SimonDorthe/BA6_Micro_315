#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the detection of the ways
#define IMAGE_BUFFER_SIZE		640
#define DELTA_ROTATION			5
#define WIDTH_SLOPE				5
#define MAX_LINE_WIDTH			300
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PERIMETER_EPUCK			(3.1415926536f * 5.35f)
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

//constants for the lEDS

/*
#define LED1		GPIOD, 5
#define LED3     	GPIOD, 6
#define LED5     	GPIOD, 10
#define LED7     	GPIOD, 11
*/
#define FRONT_LED	GPIOD, 14
#define BODY_LED	GPIOB, 2


//constants for the motors
#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1



/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
