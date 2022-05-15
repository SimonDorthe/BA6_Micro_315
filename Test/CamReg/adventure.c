/*
 * Adventure.c
 *
 *  Created on: 26 avr. 2022
 *      Author: Simon Dorthe & Julien Dibiaggio
 *
 *
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


//d�claration de variables utiles dans plusieurs fonction du mode aventure
static uint8_t colorToFollow = NO_COLOR;  //Couleur du chemin que le robot doit suivre
static uint8_t actualState = ERREUR;
static uint32_t left_motor_act_position = MOTOR_POSITION_BASE;
static uint32_t right_motor_act_position = MOTOR_POSITION_BASE;

/*--------actualisation de l'�tat de l'aventure----------------------
 *
 * Cette fonction � pour but de g�rer les transitions d'�tats de la FSM g�rant le mode aventure.
 * Elle est syst�matiquement appel�e � la fin d'un tour de thread adventure afin que l'�tat soit chang� si n�cessaire
 * au prochain appel du thread.
 *
 */

void actualize_state(void){

	uint8_t newState = 0;
	switch (actualState){
	/* Mode :
	 * ATTENTE_COULEUR -> RECHERCHE_COULEUR : si une couleur est selectionn�
	 * ATTENTE_COULEUR -> ATTENTE_COULEUR : si le selecteur n'est pas sur une des postions correspondante
	 */
		case ATTENTE_COULEUR:
			if(colorToFollow == RED || colorToFollow == GREEN || colorToFollow == BLUE){
				newState = RECHERCHE_COULEUR;
				chprintf((BaseSequentialStream *) &SD3, "from ATTENTE_COULEUR to RECHERCHE_COULEUR\n \r");
			}else{
				chprintf((BaseSequentialStream *) &SD3, "from ATTENTE_COULEUR to NO_COLOR\n \r");
				colorToFollow == NO_COLOR;
				newState = actualState;
			}
			break;
	/* Mode :
	 * RECHERCHE_COULEUR -> PAS_DE_CHEMIN_TROUVE : Si le robot ne trouve pas de chemin de la bonne couleur apr�s un certain nbr de tours
	 * RECHERCHE_COULEUR -> LIGNE_VUE : si le robot voit un chemin ( ne sais pas encore si c'est la bonne couleur
	 * RECHERCHE_COULEUR -> RECHERCHE_COULEUR : Si le robot ne voit pas de ligne pour le moment
	 */
		case RECHERCHE_COULEUR:
			//chprintf((BaseSequentialStream *) &SD3, "moteur position = %x \n\r", left_motor_act_position );//utile pour debug
			if(left_motor_get_pos()>=MAX_ROTATION){// Si le robot a fait > � 360� sans trouver le bon chemin alors la couleur n'existe pas
			newState = PAS_DE_CHEMIN_TROUVE;
			chprintf((BaseSequentialStream *) &SD3, "FROM RECHERCHE_COULEUR to PAS_DE_CHEMIN_TROUVE \n\r");
			}
			// d�tecter si le robot voit une ligne/un chemin de couleur
			else if(get_line_position() >= (IMAGE_BUFFER_SIZE/2)-DELTA_ROTATION && get_line_position() <= (IMAGE_BUFFER_SIZE/2)+DELTA_ROTATION){
				chprintf((BaseSequentialStream *) &SD3, "from RECHERCHE_COULEUR to LIGNE_VUE\n\r");
				newState = CHEMIN_VU;
			}else{
				newState = actualState;
				chprintf((BaseSequentialStream *) &SD3, "from RECHERCHE_COULEUR to RECHERCHE_COULEUR\n\r");
			}
			//chprintf((BaseSequentialStream *) &SD3, "get line position =%x \n\r", get_line_position()); //utile pour le debug
			//chprintf((BaseSequentialStream *) &SD3, "Compare_color_view =%x \n\r", compare_color_viewed()); //utile pour le debug
			break;
	/* Mode :
	 * CHEMIN_VUE -> BON:CHEMIN_VU : si le ligne vue correspond � la couleur du selecteur
	 * CHEMIN_VUE -> MAUVAIS_CHEMIN_VU : si le ligne vue ne correspond pas � la couleur du selecteur
	 */
		case CHEMIN_VU:
			if(compare_color_viewed()){//si la couleur vue est effectivement la bonne
				newState = BON_CHEMIN_TROUVE;
				chprintf((BaseSequentialStream *) &SD3, "from CHEMIN_VU to BON_CHEMIN_TROUVE\n\r");

			}else{
				newState = MAUVAIS_CHEMIN_TROUVE;
				chprintf((BaseSequentialStream *) &SD3, "from CHEMIN_VU to MAUVAIS_CHEMIN_TROUVE\n\r");
			}

			break;
	/* Mode :
	 * BON_CHEMIN_TROUVE -> SUIVRE_CHEMIN : chemin inconditionnel
	 *
	 */
		case BON_CHEMIN_TROUVE:
			newState = SUIVRE_CHEMIN;
			chprintf((BaseSequentialStream *) &SD3, "from BON_CHEMIN_TROUVE to SUIVRE_CHEMIN\n\r");

			break;
	/* Mode :
	 * MAUVAIS_CHEMIN_TROUVE -> RECHERCHE_COULEUR : chemin inconditionnel
	 *
	 */
		case MAUVAIS_CHEMIN_TROUVE:
			newState = RECHERCHE_COULEUR;//continuer � chercher le bon chemin
			chprintf((BaseSequentialStream *) &SD3, "from MAUVAIS_CHEMIN_TROUVE to RECHERCHE_COULEUR\n\r");

			break;
	/* Mode :
	 * SUIVRE_CHEMIN -> SUIVRE_CHEMIN : Si pas de probl�me et le robot pas encore arriv�
	 * SUIVRE_CHEMIN -> OBSTACLE : Si le robot d�tecte un obstacle
	 * SUIVRE_CHEMIN -> ARRIVEE : Si le robot voit l'arriv�e
	 * SUIVRE_CHEMIN -> CHEMIN_PERDU : Si le robot ne voit plus la ligne
	 * SUIVRE_CHEMIN -> ERREUR : en cas de comportement non pr�vu
	 */
		case SUIVRE_CHEMIN:
			//chprintf((BaseSequentialStream *) &SD3, "line position = %x \n\r", get_line_position() ); //utile pour le debug
			//chprintf((BaseSequentialStream *) &SD3, "line width = %x \n\r", get_line_width() );		//utile pour le debug
			// Si un obstacle est d�tect� par les capteurs de proximit� IR : passer en mode obstacle
		   if(get_prox(4)> DISTANCE || get_prox(5)> DISTANCE){
				set_rgb_led(LED4, 80, 0, 80);
				newState = OBSTACLE;
			}// Si l'arriv�e est d�tect�e : passer � arriv�
		   else if(get_line_width() >= MAX_LINE_WIDTH){
				newState = ARRIVE;
				chprintf((BaseSequentialStream *) &SD3, "from SUIVRE_CSHEMIN to ARRIVE\n\r");
			}
			// si la ligne est perdu : passer � Chemin perdu
			else if(get_line_position() < 20){
				newState = CHEMIN_PERDU;
				chprintf((BaseSequentialStream *) &SD3, "from SUIVRE_CHEMIN to CHEMIN_PERDU\n\r");
			}
			//si la ligne est toujours la : rester dans le chemin
			else if(get_line_position()>= 5 && get_line_position() <= (IMAGE_BUFFER_SIZE-10)){
				chprintf((BaseSequentialStream *) &SD3, "from SUIVRE_CHEMIN to SUIVRE_CHEMIN\n\r");
				newState = actualState;
			}
			// si aucuns cas pr�vu n'apparait : erreur pas s�curit�
			else{

				newState = ERREUR;
				chprintf((BaseSequentialStream *) &SD3, "from SUIVRE_CHEMIN to ERREUR\n\r");
			}

			break;
	/* Mode :
	 * CHEMIN_PERDU -> CHEMIN_PERDU : Reste dans le mode tant que le selecteur n'est pas remis � z�ro
	 * CHEMIN_PERDU -> ATTENTE_COULEUR : retourne � l'�tat attente_couleur une fois le syst�me remis � z�ro
	 */
		case CHEMIN_PERDU:
			//si selector remis � 0 : Passer � attendre couleur
			if(get_selector()==NO_COLOR){
				newState = ATTENTE_COULEUR;
				chprintf((BaseSequentialStream *) &SD3, "from CHEMIN_PERDU to ATTENTE_COULEUR\n\r");
			}else {
				newState = actualState;// sinon rester dans chemin perdu
				chprintf((BaseSequentialStream *) &SD3, "from CHEMIN_PERDU to CHEMIN_PERDU\n\r");
			}
			break;
	/* Mode :
	 * OBSTACLE -> OBSTACLE : Reste dans ce mode tant que le robot d�tecte un obstacle
	 * OBSTACLE -> SUIVRE_CHEMIN : Une fois qu'il n'y a plus d'obstacle on peut reprendre le chemin
	 */
		case OBSTACLE:
			//si obstacle toujours la : rester
			   if(get_prox(4)> DISTANCE || get_prox(5)> DISTANCE){
					newState = actualState;
				}else {
					newState = SUIVRE_CHEMIN;
				}
			// sinon passer � suivre chemin
			chprintf((BaseSequentialStream *) &SD3, "from OBSTACLE to SUIVRE_CHEMIN\n\r");
			break;
	/* Mode :
	 * PAS_DE_CHEMIN_TROUVE ->  PAS_DE_CHEMIN_TROUVE : reste dans le mode tant que le selecteur reste sur la couleur inexistante
	 * PAS_DE_CHEMIN_TROUVE -> ATTENTE_COULEUR : une fois le syst�me remis � 0, repart en mode attente de couleur
	 */
		case PAS_DE_CHEMIN_TROUVE:
			if(get_selector()!=colorToFollow){
				newState = ATTENTE_COULEUR;
				chprintf((BaseSequentialStream *) &SD3, "from PAS_DE_CHEMIN_TROUVE to ATTENTE_COULEUR\n\r");
			}else {
				newState = actualState;
			}
			break;
	/* Mode :
	 * ARRIVEE ->  ARRIVEE : reste dans le mode tant que le selecteur reste sur la m�me couleur
	 * ARRIVEE -> ATTENTE_COULEUR : une fois le syst�me remis � 0, repart en mode attente de couleur
	 */
		case ARRIVE:
			//Il faut bouger le selecteur pour arr�ter d'�tre en mode arriv�
			if(get_selector()!=colorToFollow){
				newState = ATTENTE_COULEUR;
				stopCurrentMelody();
				chprintf((BaseSequentialStream *) &SD3, "from ARRIVE to ATTENTE_COULEUR\n\r");
			}else{
			newState = actualState;
			chprintf((BaseSequentialStream *) &SD3, "from ARRIVE to ARRIVE\n\r");
			}
			break;
	/* Mode :
	 * ERREUR ->  ERREUR : reste dans le mode tant que le selecteur n'est pas remis � 0
	 * ERREUR -> ATTENTE_COULEUR : une fois le syst�me remis � 0, repart en mode attente de couleur
	 */
		case ERREUR:
			//Il faut remettre le selecteur � 0 pour arr�ter d'�tre en mode erreur
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
	//mise � jour de l'�tat
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
     	/* Mode : ATTENTE_COULEUR
     	 *  Ce mode est le mode de d�marrage, dans ce mode le robot attend une commande du selecteur. les moteurs sont arr�t�s
     	 * on n'utilise pas d'autres devices dans ce mode.
     	 *
     	 */
			case ATTENTE_COULEUR:
				chprintf((BaseSequentialStream *) &SD3, "ATTENTE_COULEUR\n\r");//Affichage de l'entr�e dans le mode
				//allumage en blanc des LEDs pour montrer que le robot est pr�t � recevoir une couleur et commencer
				set_rgb_led(LED2, 255, 255, 255);
				set_rgb_led(LED6, 255, 255, 255);
				set_rgb_led(LED8, 255, 255, 255);
				set_rgb_led(LED4, 0, 0, 0); //extinction de la LED 4 pour indiquer qu'on ne cherche pas de chemin
				//appel au selecteur pour savoir quelle couleur il faut chercher
				colorToFollow = get_selector();
				//remise � 0 des moteur et des positions
				left_motor_act_position = MOTOR_POSITION_BASE;
				right_motor_act_position = MOTOR_POSITION_BASE;
				left_motor_set_pos(MOTOR_POSITION_BASE);
				right_motor_set_pos(MOTOR_POSITION_BASE);
				//Attente de 1.5s pour laisser le temps de choisir la couleur avec le selecteur
				chThdSleepMilliseconds(1500);
				break;
		/* Mode : RECHERCHE_COULEUR
		 *  Ce mode sert � chercher le chemin de la couleur correspondante. Dans ce mode le robot tourne sur lui-m�me, la cam�ra
		 *  est active et analyse l'image vue pour d�tecter une ligne. La detection de ligne est bas�e sur le TP4 du cours
		 */
			case RECHERCHE_COULEUR:
				chprintf((BaseSequentialStream *) &SD3, "RECHERCHE_COULEUR\n\r");//Affichage de l'entr�e dans le mode
				//Gestion des LED
				set_rgb_led(LED6, 0, 0, 0);
				set_rgb_led(LED8, 0, 0, 0);
				toggle_rgb_led(LED4,NUM_COLOR_LED, 255);//LED 4 s'allume en blanc pour indiquer que le Epuck cherche un chemin
				if(colorToFollow == RED){ //allumage de la lED 2 (LED "couleur � suivre") de la couleur du chemin voulu
					set_rgb_led(LED2, 255, 0, 0);

				}else if(colorToFollow == GREEN){
					set_rgb_led(LED2, 0, 255, 0);

				}else if(colorToFollow == BLUE){
					set_rgb_led(LED2, 0, 0, 255);

				}
				//enregistrement de la position acutelle des moteurs
				left_motor_act_position = left_motor_get_pos();
				right_motor_act_position = right_motor_get_pos();
				//Start both the 2 motors
				right_motor_set_speed(MOTOR_SPEED_BACKWARD/3);
				left_motor_set_speed(MOTOR_SPEED_FORWARD/3);

				break;
		/* Mode : CHEMIN_VU
		 *  Ce mode arrive d�s que le robot rep�re une ligne. � ce stade on ne sait pas encore s'il s'agit de la bonne couleur.
		 *  Ce mode arr�te les moteurs pour prendre le temps d'analyser ce que le robot voit. on change le filtre sur la cam�ra pour
		 *  pour confirmer qui'il s'agit de la bonne couleur ou non.
		 */
			case CHEMIN_VU:
				chprintf((BaseSequentialStream *) &SD3, "CHEMIN_VUE\n\r");//Affichage de l'entr�e dans le mode
				set_rgb_led(LED4, 0, 0, 255); //LED 4 s'allume en bleu pour indiquer qu'un chemin est vu
				//enregistrement de la position acutelle des moteurs
				left_motor_act_position = left_motor_get_pos();
				right_motor_act_position = right_motor_get_pos();
				//Stop the 2 motors
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				//chprintf((BaseSequentialStream *) &SD3, "CHEMIN_VU \n\r");
				chThdSleepMilliseconds(1000); // attente de 1s pour rentre le changement visuel pour l'utilisateur
				break;
		/* Mode : BON_CHEMIN_TROUVE
		 *  Ce mode arrive si la ligne vue par CHEMIN_VUE etait de la bonne couleur. Dans ce cas on indique l'indique par une led
		 * verte et le robot va pouvoir commencer � suivre le chemin.
		 */
			case BON_CHEMIN_TROUVE:
				chprintf((BaseSequentialStream *) &SD3, "BON_CHEMIN_TROUVE\n");//Affichage de l'entr�e dans le mode
				set_rgb_led(LED4, 0, 255, 0); //LED 4 s'allume en vert pour indiquer que le bon chemin est vu
				//Arr�t des moteurs
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				chThdSleepMilliseconds(1000);//attente pendant 1 secondes pour rentre le changement visuel pour l'utilisateur
				break;
		/* Mode : MAUVAIS_CHEMIN_TROUVE
		 *  Ce mode arrive si la ligne vue par CHEMIN_VUE etait de la mauvaise couleur. Dans ce cas on indique l'indique par une led
		 * rouge et le robot va devoir continuer � chercher une autre ligne. On remet en marche les moteurs pur que le robot tourne sur lui-m�me
		 */
			case MAUVAIS_CHEMIN_TROUVE:
				chprintf((BaseSequentialStream *) &SD3, "MAUVAIS_CHEMIN_TROUVE\n\r");//Affichage de l'entr�e dans le mode
				set_rgb_led(LED4, 255, 0, 0); //LED 4 s'allume en vert pour indiquer que le mauvais chemin est vu
				//Remise de la position des moteurs avant l'arr�t
				left_motor_set_pos(left_motor_act_position);
				right_motor_set_pos(right_motor_act_position);
				right_motor_set_speed(MOTOR_SPEED_BACKWARD);
				left_motor_set_speed(MOTOR_SPEED_FORWARD);
				chThdSleepMilliseconds(1000);//attente pendant 1 secondes pour rentre le changement visuel pour l'utilisateur
				break;
		/* Mode : SUIVRE_CHEMIN
		 *  Ce mode permet au robot de suivre la ligne de la couleur voulue. Dans mode les moteurs avance � une vitesse de base
		 *  qu'on vient l�g�rement augmenter ou diminuer pour corriger la trajectoir. Cette partie est bas�e sur le TP4 du cours
		 *  Plusieurs cas de figures sont pris en compte dans les transitions d'�tats comme la d�tection d'obstacle, l'arriv�e, la perte de vue du chemin
		 */
			case SUIVRE_CHEMIN:
				chprintf((BaseSequentialStream *) &SD3, "SUIVRE_CHEMIN\n\r");//Affichage de l'entr�e dans le mode
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
				break;
		/* Mode : CHEMIN_PERDU
		 * Ce mode arrive si le robot perd de vu le chemin qu'il �tait en train de suivre. Dans ce cas on arr�te les moteurs
		 * et on indique avec une led que le robot a perdu le chemin.
		 */
			case CHEMIN_PERDU:
				chprintf((BaseSequentialStream *) &SD3, "CHEMIN_PERDU\n\r");
				set_rgb_led(LED6, 0, 0, 0); //extinction de la LED6  pour signaler que le epuck est � perdu le chemin
				//Arr�t des moteurs
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);

				break;
		/* Mode : OBSTACLE
		 * Ce mode arrive si le robot rencontre un obstacle alors qu'il �tait en train de suivre un chemin. Dans ce cas on arr�te les moteurs
		 * et on indique avec une led que le robot rencontre un obstacle. Il pourra repartir aussi t�t que les capteur IR ne verront plus l�bstacle
		 */
			case OBSTACLE:
				chprintf((BaseSequentialStream *) &SD3, "OBSTACLE\n\r");//Affichage de l'entr�e dans le mode
				set_rgb_led(LED6, 80, 80, 0); // allumage de la LED de gestion chemin en jaune pour signaler un obstacle
				//arr�t des moteurs
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				//si obsatacle toujours la : rester
				// sinon passer � suivre chemin

				break;
		/* Mode : PAS_DE_CHEMIN_TROUVE
		 * Ce mode arrive si le robot n'a pas trouv� de ligne de la couleur correcpondante � ce qu'il cherchait apprait avoir fait un certain nbr de tours
		 * Dans ce mode on arr�te les 2 moteurs et on indique qu'aucuns chemins n'est rouv� par une led qui clignote en rouge
		 */
			case PAS_DE_CHEMIN_TROUVE:
				chprintf((BaseSequentialStream *) &SD3, "PAS_DE_CHEMIN_TROUVE\n\r");//Affichage de l'entr�e dans le mode
				//arr�t des 2 moteurs
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				 toggle_rgb_led(LED4,RED_LED, 255);
				chThdSleepMilliseconds(1500);

				break;
		/* Mode : ARRIVE
		 * Ce mode arrive si le robot arrive au bout du chemin qu'il �tait en train de suivre. Dans ce cas on arr�te les moteurs
		 * et on indique avec une led que le robot est arriv�. � ce moment un musique est �galement jou� sur le buzzeur.
		 */
			case ARRIVE:
				chprintf((BaseSequentialStream *) &SD3, "ARRIVE\n\r"); //Affichage de l'entr�e dans le mode
				set_rgb_led(LED6, 0, 0, 255); //Activation de la LED6 en bleu pour signaler que le epuck est arriv� � la fin
				//Arr�t des moteurs
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				//jouer une m�lodie de victoire (� commenter apr�s 22h, c'est pas cool pour les colocs)
				playMelody(SANDSTORMS, ML_SIMPLE_PLAY, NULL);

				break;
		/* Mode : ERREUR
		 * Ce mode arrive si le robot rencontre un cas de figure impr�vu ou un probl�me inconnu. Dans ce cas on arr�te les moteurs
		 * et on indique qu'il y'a une erreur en allumant toutes les leds en rouge.
		 */
			case ERREUR:
				chprintf((BaseSequentialStream *) &SD3, "ERREUR\n\r");//Affichage de l'entr�e dans le mode
				//Arr�t des moteurs
				right_motor_set_speed(MOTOR_SPEED_STOP);
				left_motor_set_speed(MOTOR_SPEED_STOP);
				//allumer toutes les LED RGB en rouge pour signaler une erreur
				set_rgb_led(LED2, 255, 0, 0);
				set_rgb_led(LED4, 255, 0, 0);
				set_rgb_led(LED6, 255, 0, 0);
				set_rgb_led(LED8, 255, 0, 0);

				break;
			default:
			break;

         }
         actualize_state(); //actualise l'�tat � la fin de du traitement de celui-ci
         //50Hz
         chThdSleepUntilWindowed(time, time + MS2ST(20));
     }
}


//donne acc�s � la couleur cherch� par le robot � l'ext�rieur de ce mode
uint8_t get_colorToFollow(void){
	return colorToFollow;
}
//d�marrage du thread
void adventure_start(void){
	chThdCreateStatic(waAdventure, sizeof(waAdventure), NORMALPRIO, Adventure, NULL);
}
