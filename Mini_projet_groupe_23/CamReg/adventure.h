/*
 * Adventure.h
 *
 *  Created on: 26 avr. 2022
 *      Author: simon Dorthe & Julien Dibiaggio
 *
 *      Ce fichier regroupe la partie de notre project gérant le mode principal de notre projet avec notamment la
 *      FSM permettant la gestion et transition des diférentes parties.
 *      Il s'agit d'un thread démarré dans la focntion main du projet et qui tournera jusqu'à son arrêt.
 */

#ifndef ADVENTURE_H_
#define ADVENTURE_H_

//déclaration des différents états atteignable par le robot dans ce mode
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
//déclaration des couleurs de chemin possible à suivre
enum Color {
	NO_COLOR,
	RED,
	GREEN,
	BLUE,
};

//définition des constantes utiles dans ce mode
#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1
#define MOTOR_POSITION_BASE		0
#define MOTOR_SPEED_FORWARD		180
#define MOTOR_SPEED_BACKWARD	-180
#define MOTOR_SPEED_STOP		0
#define DISTANCE 				50
#define MAX_ROTATION			2500

//fonctions publiques disponibles à l'exterieur
void actualize_state(void);
uint8_t get_colorToFollow(void);
void adventure_start(void);


#endif /* ADVENTURE_H_ */
