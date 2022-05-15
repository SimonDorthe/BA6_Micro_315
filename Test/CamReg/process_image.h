/*
 * process_image.h
 *
 *  Based on an importation of EPFL MICRO-315 TP4
 *
 *      Modified by . Simon Dorthe & Julien Dibiaggio
 *
 *  Ce document permet de gérer la capture d'image et le traiment de celles-ci via la caméra de l'e-puck
 */
#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define DELTA_PIXELS		20


uint16_t get_line_position(void);
uint16_t get_line_width(void);
bool compare_color_viewed(void); //cette fonction permet de comparer si la couleur vue est celle que le robot doit voir
void process_image_start(void);



#endif /* PROCESS_IMAGE_H */
