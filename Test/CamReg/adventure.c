/*
 * Adventure.c
 *
 *  Created on: 26 avr. 2022
 *      Author: simon
 */
#include "ch.h"
#include "hal.h"
#include "motors.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include "adventure.h"
#include <main.h>
#include <selector.h>
#include <process_image.h>
#include <sensors/proximity.h>
#include <leds.h>
#include "spi_comm.h"
#include <audio/play_melody.h>
#include <sensors/proximity.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>



static uint8_t colorToFollow = NO_COLOR;  //Couleur du chemin que le robot doit suivre
static uint8_t actualState = ERREUR;
static uint8_t sequence_pos = 0;
static uint32_t left_motor_act_position = MOTOR_POSITION_BASE;
static uint32_t right_motor_act_position = MOTOR_POSITION_BASE;

/*--------actualisation de l'état de l'aventure----------------------
 *
 * Cette fonction à pour but de gérer les transitions d'états de la FSM gérant le mode aventure.
 * Elle est systématiquement appelée à la fin d'un tour de thread adventure afin que l'état soit changé si nécessaire
 * au prochain appel du thread.
 *
 */

void actualize_state(void){

	uint8_t newState = 0;
	switch (actualState){
		case ATTENTE_COULEUR:
			if(colorToFollow == RED || colorToFollow == GREEN || colorToFollow == BLUE){
				newState = RECHERCHE_COULEUR;
				if(colorToFollow == RED){
					set_rgb_led(LED2, 255, 0, 0);

				}else if(colorToFollow == GREEN){
					set_rgb_led(LED2, 0, 255, 0);

				}else if(colorToFollow == BLUE){
					set_rgb_led(LED2, 0, 0, 255);

				}
				set_rgb_led(LED4, 0, 0, 0);
				set_rgb_led(LED6, 0, 0, 0);
				set_rgb_led(LED8, 0, 0, 0);
				left_motor_act_position = MOTOR_POSITION_BASE;
				right_motor_act_position = MOTOR_POSITION_BASE;
				//left_motor_set_pos(left_motor_act_position);
				//right_motor_set_pos(right_motor_act_position);
				chprintf((BaseSequentialStream *) &SD3, "from ATTENTE_COULEUR to RECHERCHE_COULEUR\n \r");
			}else{
				chprintf((BaseSequentialStream *) &SD3, "from ATTENTE_COULEUR to NO_COLOR\n \r");
				colorToFollow == NO_COLOR;
				newState = actualState;
			}
			break;
		case RECHERCHE_COULEUR:
			left_motor_act_position = left_motor_get_pos();
			right_motor_act_position = right_motor_get_pos();
			chprintf((BaseSequentialStream *) &SD3, "moteur position = %x \n\r", left_motor_act_position );
			if(left_motor_get_pos()>=3000){
			// Si le robot a fait > à 360° , alors la couleur n'existe pas
			newState = PAS_DE_CHEMIN_TROUVE;
			chprintf((BaseSequentialStream *) &SD3, "FROM RECHERCHE_COULEUR to PAS_DE_CHEMIN_TROUVE \n\r");
			}
			// détecter si le robot voit une ligne/un chemin de couleur
			else if(get_line_position() >= (IMAGE_BUFFER_SIZE/2)-DELTA_ROTATION && get_line_position() <= (IMAGE_BUFFER_SIZE/2)+DELTA_ROTATION){
				chprintf((BaseSequentialStream *) &SD3, "from RECHERCHE_COULEUR to LIGNE_VUE\n\r");
				newState = CHEMIN_VU;
			}else{
				newState = actualState;
				chprintf((BaseSequentialStream *) &SD3, "from RECHERCHE_COULEUR to RECHERCHE_COULEUR\n\r");
			}
			chprintf((BaseSequentialStream *) &SD3, "get line position =%x \n\r", get_line_position());
			chprintf((BaseSequentialStream *) &SD3, "Compare_color_view =%x \n\r", compare_color_viewed());
			break;
		case CHEMIN_VU:
			left_motor_act_position = left_motor_get_pos();
			right_motor_act_position = right_motor_get_pos();
			if(compare_color_viewed()){//si la couleur vue est effectivement la bonne
				newState = BON_CHEMIN_TROUVE;
				chprintf((BaseSequentialStream *) &SD3, "from CHEMIN_VU to BON_CHEMIN_TROUVE\n\r");

			}else{
				newState = MAUVAIS_CHEMIN_TROUVE;
				chprintf((BaseSequentialStream *) &SD3, "from CHEMIN_VU to MAUVAIS_CHEMIN_TROUVE\n\r");
			}

			break;
		case BON_CHEMIN_TROUVE:
			newState = SUIVRE_CHEMIN;
			chprintf((BaseSequentialStream *) &SD3, "from BON_CHEMIN_TROUVE to SUIVRE_CHEMIN\n\r");

//insert led sequence
			break;
		case MAUVAIS_CHEMIN_TROUVE:
			left_motor_set_pos(left_motor_act_position);
			right_motor_set_pos(right_motor_act_position);
			newState = RECHERCHE_COULEUR;//continuer à chercher le bon chemin
			chprintf((BaseSequentialStream *) &SD3, "from MAUVAIS_CHEMIN_TROUVE to RECHERCHE_COULEUR\n\r");

// insert led sequence
			break;
		case SUIVRE_CHEMIN:
			chprintf((BaseSequentialStream *) &SD3, "line position = %x \n\r", get_line_position() );
			chprintf((BaseSequentialStream *) &SD3, "line width = %x \n\r", get_line_width() );
			// Si un obstacle est détecté par les capteurs de proximité IR : passer en mode obstacle
		   if(get_prox(4)> DISTANCE || get_prox(5)> DISTANCE){
				set_rgb_led(LED4, 80, 0, 80);
				newState = OBSTACLE;
			}// Si l'arrivée est détectée : passer à arrivé
		   else if(get_line_width() >= MAX_LINE_WIDTH){
				newState = ARRIVE;
				chprintf((BaseSequentialStream *) &SD3, "from SUIVRE_CSHEMIN to ARRIVE\n\r");
			}
			// si la ligne est perdu : passer à Chemin perdu
			else if(get_line_position() < 20){
				newState = CHEMIN_PERDU;
				chprintf((BaseSequentialStream *) &SD3, "from SUIVRE_CHEMIN to CHEMIN_PERDU\n\r");
			}
			//si la ligne est toujours la : rester dans le chemin
			else if(get_line_position()>= 5 && get_line_position() <= (IMAGE_BUFFER_SIZE-10)){
				chprintf((BaseSequentialStream *) &SD3, "from SUIVRE_CHEMIN to SUIVRE_CHEMIN\n\r");
				newState = actualState;
			}
			//si obstacle détecté : passage à obstacle

			// si aucuns cas prévu n'apparait : erreur pas sécurité
			else{

				newState = ERREUR;
				chprintf((BaseSequentialStream *) &SD3, "from SUIVRE_CHEMIN to ERREUR\n\r");
			}

			break;
		case CHEMIN_PERDU:
			//si selector remis à 0 : Passer à attendre couleur
			if(get_selector()==0){
				newState = ATTENTE_COULEUR;
				chprintf((BaseSequentialStream *) &SD3, "from CHEMIN_PERDU to ATTENTE_COULEUR\n\r");
			}else {
				newState = actualState;// sinon rester dans chemin perdu
				chprintf((BaseSequentialStream *) &SD3, "from CHEMIN_PERDU to CHEMIN_PERDU\n\r");
			}
			break;
		case OBSTACLE:
			//si obstacle toujours la : rester
			   if(get_prox(4)> DISTANCE || get_prox(5)> DISTANCE){
					newState = actualState;
				}else {
					newState = SUIVRE_CHEMIN;
				}
			// sinon passer à suivre chemin
			chprintf((BaseSequentialStream *) &SD3, "from OBSTACLE to SUIVRE_CHEMIN\n\r");
			break;
		case PAS_DE_CHEMIN_TROUVE:
			if(get_selector() == NO_COLOR){
				newState = ATTENTE_COULEUR;
				chprintf((BaseSequentialStream *) &SD3, "from PAS_DE_CHEMIN_TROUVE to ATTENTE_COULEUR\n\r");
			}
			break;
		case ARRIVE:
			//Il faut bouger le selecteur pour arrêter d'être en mode arrivé
			if(get_selector()!=colorToFollow){
				newState = ATTENTE_COULEUR;
				stopCurrentMelody();
				chprintf((BaseSequentialStream *) &SD3, "from ARRIVE to ATTENTE_COULEUR\n\r");
			}else{
			newState = actualState;
			chprintf((BaseSequentialStream *) &SD3, "from ARRIVE to ARRIVE\n\r");
			}
			break;
		case ERREUR:
			//Il faut remettre le selecteur à 0 pour arrêter d'être en mode erreur
			if(get_selector() == NO_COLOR){
				newState = ATTENTE_COULEUR;
				chprintf((BaseSequentialStream *) &SD3, "from ERREUR to ATTENTE_COULEUR\n\r");
			}else{
			newState = actualState;
			chprintf((BaseSequentialStream *) &SD3, "from ERREUR to ERREUR\r");
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
				set_rgb_led(LED2, 255, 255, 255);
				set_rgb_led(LED6, 255, 255, 255);
				set_rgb_led(LED8, 255, 255, 255);
				set_rgb_led(LED4, 0, 0, 0); //extinction de la LED 4 pour indiquer qu'on ne cherche pas de chemin
				//appel au selecteur pour savoir quelle couleur il faut chercher
				colorToFollow = get_selector();
				left_motor_set_pos(0);
				right_motor_set_pos(0);
				chprintf((BaseSequentialStream *) &SD3, "ATTENTE_COULEUR\n\r");
				chThdSleepMilliseconds(1000);
				break;
			case RECHERCHE_COULEUR:
				chprintf((BaseSequentialStream *) &SD3, "RECHERCHE_COULEUR\n\r");
				toggle_rgb_led(LED4,NUM_COLOR_LED, 255);//LED 4 s'allume en blanc pour indiquer que le Epuck cherche un chemin

				//Start both the 2 motors
				right_motor_set_speed(-60);
				left_motor_set_speed(60);

				break;
			case CHEMIN_VU:
				set_rgb_led(LED4, 0, 0, 255); //LED 4 s'allume en bleu pour indiquer qu'un chemin est vu
				//Stop the 2 motors
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				//chprintf((BaseSequentialStream *) &SD3, "CHEMIN_VU \n\r");
				chprintf((BaseSequentialStream *) &SD3, "CHEMIN_VUE\n\r");
				chThdSleepMilliseconds(1000);
				break;
			case BON_CHEMIN_TROUVE:
				set_rgb_led(LED4, 0, 255, 0); //LED 4 s'allume en vert pour indiquer que le bon chemin est vu
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				chprintf((BaseSequentialStream *) &SD3, "BON_CHEMIN_TROUVE\n");
				//faire clignoter la led du corps correspondant à la couleur trouvée et allumer la led frontale en Vert
				//LEDs_update(seq[sequence_pos]);

				//chprintf((BaseSequentialStream *) &SD3, "BON_CHEMIN_TROUVE \n\r");
				chThdSleepMilliseconds(1500);
				//pendant 2 secondes

				break;
			case MAUVAIS_CHEMIN_TROUVE:
				set_rgb_led(LED4, 255, 0, 0); //LED 4 s'allume en vert pour indiquer que le mauvais chemin est vu
				left_motor_set_pos(left_motor_act_position);
				right_motor_set_pos(right_motor_act_position);
				right_motor_set_speed(MOTOR_SPEED_BACKWARD);
				left_motor_set_speed(MOTOR_SPEED_FORWARD);
				//faire clignoter la led du corps correspondant à la couleur trouvée et allumer la led frontale en Vert
				//pendant 2 secondes
				chprintf((BaseSequentialStream *) &SD3, "MAUVAIS_CHEMIN_TROUVE\n\r");
				chThdSleepMilliseconds(1500);

				break;
			case SUIVRE_CHEMIN:
				set_rgb_led(LED6, 0, 255, 0); // Activation de la LED6 en vert pour signaler que le suivi du chemin est ok
		         //computes a correction factor to let the robot rotate to be in front of the line
		         speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

		         //if the line is nearly in front of the camera, don't rotate
		         if(abs(speed_correction) < ROTATION_THRESHOLD){
		         	speed_correction = 0;
		         }

		         //applies the speed from the PI regulator and the correction for the rotation

		 		right_motor_set_speed(MOTOR_SPEED_FORWARD - ROTATION_COEFF * speed_correction);
		 		left_motor_set_speed(MOTOR_SPEED_FORWARD + ROTATION_COEFF * speed_correction);
		 		chprintf((BaseSequentialStream *) &SD3, "SUIVRE_CHEMIN\n\r");
				break;
			case CHEMIN_PERDU:
				set_rgb_led(LED6, 0, 0, 0); //extinction de la LED6  pour signaler que le epuck est à perdu le chemin
				chprintf((BaseSequentialStream *) &SD3, "CHEMIN_PERDU\n\r");
				//allumer les leds du corps qui tournent en rond en séquence.
				//allumer la front led en rouge pendant 3 secondes

				break;
			case OBSTACLE:
				set_rgb_led(LED6, 80, 80, 0);
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				//si obsatacle toujours la : rester
				// sinon passer à suivre chemin

				chprintf((BaseSequentialStream *) &SD3, "OBSTACLE\n\r");

				break;
			case PAS_DE_CHEMIN_TROUVE:
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				 toggle_rgb_led(LED4,RED_LED, 255);
				chprintf((BaseSequentialStream *) &SD3, "PAS_DE_CHEMIN_TROUVE\n\r");
				chThdSleepMilliseconds(1500);

				break;
			case ARRIVE:
				set_rgb_led(LED6, 0, 0, 255); //Activation de la LED6 en bleu pour signaler que le epuck est arrivé à la fin
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				chprintf((BaseSequentialStream *) &SD3, "ARRIVE\n\r");
				playMelody(SANDSTORMS, ML_SIMPLE_PLAY, NULL);

				break;
			case ERREUR:
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				chprintf((BaseSequentialStream *) &SD3, "ERREUR\n\r");

				break;
			default:
			break;

         }
         actualize_state();
         //50Hz
         chThdSleepUntilWindowed(time, time + MS2ST(30));
     }
}



uint8_t get_colorToFollow(void){
	return colorToFollow;
}

void adventure_start(void){
	chThdCreateStatic(waAdventure, sizeof(waAdventure), NORMALPRIO, Adventure, NULL);
}
