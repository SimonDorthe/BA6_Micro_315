/*
 * Adventure.c
 *
 *  Created on: 26 avr. 2022
 *      Author: simon
 */
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include "adventure.h"
#include <main.h>
#include <gpio.h>
#include <selector.h>
#include <timer.h>
#include <motors.h>
#include <process_image.h>



static uint8_t colorToFollow = NO_COLOR;  //Couleur du chemin que le robot doit suivre
static uint8_t actualState = ERREUR;
static uint8_t sequence_pos = 0;
static const uint8_t seq[8][4] = {
    {0, 0, 0, 1},	// ON1
	{0, 0, 1, 1},	// ON3
    {0, 1, 1, 1},	// ON5
	{1, 1, 1, 1},	// ON7
    {1, 1, 1, 0},	// OFF1
	{1, 1, 0, 0},	// OFF3
    {1, 0, 0, 0},	// OFF5
	{0, 0, 0, 0},	// OFF7
};

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

void actualize_state(void){

	uint8_t newState = 0;
	switch (actualState){
		case ATTENTE_COULEUR:
			if(colorToFollow == RED || colorToFollow == GREEN || colorToFollow == BLUE){
				newState = RECHERCHE_COULEUR;
			}else{
				colorToFollow == NO_COLOR;
				newState = actualState;
			}
			break;
		case RECHERCHE_COULEUR:

			if(compare_color_viewed()){//si la couleur vue est effectivement la bonne
				newState = BON_CHEMIN_TROUVE;
				motor_stop();
			}else{
				newState = MAUVAIS_CHEMIN_TROUVE;
	    		right_motor_set_speed(-2);
    			left_motor_set_speed(2);
			}

			break;
		case BON_CHEMIN_TROUVE:
			newState = SUIVRE_CHEMIN;
        	sequence_pos = 0;
        	/*
			gpio_set(LED1);
			gpio_set(LED3);
			gpio_set(LED5);
			gpio_set(LED7);
			*/
			break;
		case MAUVAIS_CHEMIN_TROUVE:
			newState = RECHERCHE_COULEUR;//continuer à chercher le bon chemin
        	sequence_pos = 0;
        	/*
			gpio_set(LED1);
			gpio_set(LED3);
			gpio_set(LED5);
			gpio_set(LED7);
			*/
			break;
		case SUIVRE_CHEMIN:
			//si la ligne est toujours la : rester dans le chemin
			if(get_line_position()>= 5 && get_line_position() <= (IMAGE_BUFFER_SIZE-10)){
				newState = actualState;
			}
			//si obstacle détecté : passage à obstacle

			// si la ligne est perdu : passer à Chemin perdu
			else if(get_line_position() < 5){
				newState = CHEMIN_PERDU;
			}
			// Si l'arrivée est détectée : passer à arrivé
			else if(get_line_width() >= MAX_LINE_WIDTH){
				newState = ARRIVE;
			}
			newState = ERREUR;
			break;
		case CHEMIN_PERDU:
			//si selector remis à 0 : Passer à attendre couleur
			if(get_selector()==0){
				newState = ATTENTE_COULEUR;
			}else {
				newState = actualState;// sinon rester dans chemin perdu
			}
			break;
		case OBSTACLE:
			//si obstacle toujours la : rester
			// sinon passer à suivre chemin
			break;
		case PAS_DE_CHEMIN_TROUVE:
			newState = ATTENTE_COULEUR;
			break;
		case ARRIVE:
			//Il faut bouger le selecteur pour arrêter d'être en mode arrivé
			if(get_selector()!=colorToFollow){
				newState = ATTENTE_COULEUR;
			}else{
			newState = actualState;
			}
			break;
		case ERREUR:
			//Il faut remettre le selecteur à 0 pour arrêter d'être en mode erreur
			if(get_selector() == NO_COLOR){
				newState = ATTENTE_COULEUR;
			}else{
			newState = actualState;
			}
			break;
		default:
		break;
		}
	actualState = newState;

}

static THD_WORKING_AREA(waAdventure, 256);
static THD_FUNCTION(Adventure, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
         time = chVTGetSystemTime();

         switch (actualState){
			case ATTENTE_COULEUR:
				//appel au selecteur pour savoir quelle couleur il faut chercher
				colorToFollow = get_selector();
				actualize_state();
				break;
			case RECHERCHE_COULEUR:
				//mise en route des moteur pour tourner dans un sens
				//clockwise rotation of 360Â°
				motor_set_position(PERIMETER_EPUCK*1.5f, PERIMETER_EPUCK*1.5f, -2, 2);
				do{
					// détecter si le robot voit une ligne/un chemin de couleur
					if(get_line_position() >= (IMAGE_BUFFER_SIZE/2)-DELTA_ROTATION && get_line_position() <= (IMAGE_BUFFER_SIZE/2)+DELTA_ROTATION){
						//Stop the 2 motors
						right_motor_set_speed(0);
						left_motor_set_speed(0);
						actialize_state();
					}
				}while(motor_position_reached() != POSITION_REACHED);

				// Si le robot a fait > à 360° , alors la couleur n'existe pas
				actualState = PAS_DE_CHEMIN_TROUVE;
				break;
			case BON_CHEMIN_TROUVE:
				//faire clignoter la led du corps correspondant à la couleur trouvée et allumer la led frontale en Vert
				LEDs_update(seq[sequence_pos]);
				sequence_pos++;
				sequence_pos %= 8;
				//waits before moving to let us position the robot before it moves

				chThdSleepMilliseconds(1500);
				//pendant 2 secondes
				actualize_state();
				break;
			case MAUVAIS_CHEMIN_TROUVE:
				//faire clignoter la led du corps correspondant à la couleur trouvée et allumer la led frontale en Vert
				//pendant 2 secondes
				actualize_state();

				break;
			case SUIVRE_CHEMIN:
		         //computes a correction factor to let the robot rotate to be in front of the line
		         speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

		         //if the line is nearly in front of the camera, don't rotate
		         if(abs(speed_correction) < ROTATION_THRESHOLD){
		         	speed_correction = 0;
		         }

		         //applies the speed from the PI regulator and the correction for the rotation

		 		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		 		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

				break;
			case CHEMIN_PERDU:
				//allumer les leds du corps qui tournent en rond en séquence.
				//allumer la front led en rouge pendant 3 secondes
				actualize_state();
				break;
			case OBSTACLE:
				//si obsatacle toujours la : rester
				// sinon passer à suivre chemin
				break;
			case PAS_DE_CHEMIN_TROUVE:

				break;
			case ARRIVE:
				motor_stop();

				break;
			case ERREUR:
				motor_stop();
				break;
			default:
			break;

         //50Hz
         chThdSleepUntilWindowed(time, time + MS2ST(20));
         }
     }
}


uint8_t get_colorToFollow(void){
	return colorToFollow;
}

void adventure_start(void){
	chThdCreateStatic(waAdventure, sizeof(waAdventure), NORMALPRIO, Adventure, NULL);
}
