/*
 * Adventure.h
 *
 *  Created on: 26 avr. 2022
 *      Author: simon
 */

#ifndef ADVENTURE_H_
#define ADVENTURE_H_

enum AdventureState {
	ERROR,
	ATTENTE_COULEUR,
	RECHERCHE_COULEUR,
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



void adventure_start(void);
uint8_t get_colorToFollow(void);



#endif /* ADVENTURE_H_ */
