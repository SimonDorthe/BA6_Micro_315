/*
 * Adventure.c
 *
 *  Created on: 26 avr. 2022
 *      Author: simon
 */


#include "adventure.h"
#include <main.h>
#include <gpio.h>
#include <selector.h>
#include <timer.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>




static uint8_t colorToFollow = NO_COLOR;  //Couleur du chemin que le robot doit suivre
static uint8_t actualState = ERROR;
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

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
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

			if(compare_color_viewed(colorToFollow)){//si la couleur vue est effectivement la bonne
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
			gpio_set(LED1);
			gpio_set(LED3);
			gpio_set(LED5);
			gpio_set(LED7);
			break;
		case MAUVAIS_CHEMIN_TROUVE:
			newState = RECHERCHE_COULEUR;//continuer à chercher le bon chemin
        	sequence_pos = 0;
			gpio_set(LED1);
			gpio_set(LED3);
			gpio_set(LED5);
			gpio_set(LED7);
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
			newState = ERROR;
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
		case ERROR:
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

void continue_adventure(void){

	switch (actualState){
		case ATTENTE_COULEUR:
			//appel au selecteur pour savoir quelle couleur il faut chercher
			colorToFollow = get_selector();
			actualize_state();
			break;
		case RECHERCHE_COULEUR:
			//mise en route des moteur pour tourner dans un sens
			//clockwise rotation of 360Â°
			motor_set_position(PERIMETER_EPUCK, PERIMETER_EPUCK, -2, 2);
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
		    delay(SystemCoreClock/32);
			//pendant 2 secondes
			actualize_state();
			break;
		case MAUVAIS_CHEMIN_TROUVE:
			//faire clignoter la led du corps correspondant à la couleur trouvée et allumer la led frontale en Vert
			//pendant 2 secondes
			actualize_state();

			break;
		case SUIVRE_CHEMIN:
			//lancer le PI_Regulator, et le traitement d'image pour la couleur du chemin,
			right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
			left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
			//actualisez l'état à 10Hz

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

			break;
		default:
		break;
		}
}

uint8_t get_colorToFollow(void){
	return colorToFollow;
}


