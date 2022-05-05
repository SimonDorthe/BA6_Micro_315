#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define GREEN_THRESHOLD 	100 //à adapter
#define RED_THRESHOLD 		100 //à adapter
#define BLUE_THRESHOLD 		100 //à adapter
#define DELTA_PIXELS		10 //à adapter --> remplacerait les thresholds

float get_distance_cm(void);
uint16_t get_line_position(void);
bool compare_color_viewed(void);
void process_image_start(void);


#endif /* PROCESS_IMAGE_H */
