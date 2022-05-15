/*
 * Adventure.h
 *
 *  Created on: 26 avr. 2022
 *      Author: simon
 */

#ifndef ADVENTURE_H_
#define ADVENTURE_H_

enum AdventureState {
	ERREUR,
	ATTENTE_COULEUR,
	RECHERCHE_COULEUR,
	CHEMIN_VU,
	BON_CHEMIN_TROUVE,
	MAUVAIS_CHEMIN_TROUVE,
	PAS_DE_CHEMIN_TROUVE,
	SUIVRE_CHEMIN,
	CHEMIN_PERDU,
	OBSTACLE,
	ARRIVE,

};
enum Color {
	NO_COLOR,
	RED,
	GREEN,
	BLUE,
};

#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1
#define MOTOR_POSITION_BASE		0
#define MOTOR_SPEED_FORWARD		150
#define MOTOR_SPEED_BACKWARD	-100
#define MOTOR_SPEED_STOP		0
#define DISTANCE 				50

int16_t pi_regulator(float distance, float goal);
void actualize_state(void);
void continue_adventure(void);
uint8_t get_colorToFollow(void);
void adventure_start(void);


#endif /* ADVENTURE_H_ */
