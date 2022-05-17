/*
 * process_image.c
 *
 *  Based on an importation of EPFL MICRO-315 TP4
 *
 *      Modified by . Simon Dorthe & Julien Dibiaggio
 *
 *
 */

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <adventure.h>
#include <process_image.h>




static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint8_t *img_buff_ptr;
static uint8_t image[IMAGE_BUFFER_SIZE] = {0};
static uint16_t lineWidth = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}



	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

//cette fonction permet d'appliquer un filttre ne montrant que les pixels rouges d'une image
void extract_red_pixels(void){

	//Extracts only the red pixels
	for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
		image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8; //0xF8 = 0b11111000
	}
}
//cette fonction permet d'appliquer un filttre ne montrant que les pixels verts d'une image
void extract_green_pixels(void){

	uint8_t byte_RG = 0;
	uint8_t byte_GB = 0;
	//Extracts only the green pixels
	for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
		byte_RG = (uint8_t)img_buff_ptr[i]&0x07;
		byte_RG = (byte_RG << 5)&0xE0;
		byte_GB = (uint8_t)img_buff_ptr[i+1]&0xC0;
		byte_GB = (byte_GB >> 3)&0x1C;
		image[i/2] = (byte_RG | byte_GB)&0xFC;
	}
}
//cette fonction permet d'appliquer un filttre ne montrant que les pixels bleus d'une image
void extract_blue_pixels(void){

	//Extracts only the blue pixels
	for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
		image[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F) << 3; //0x1F = 0b00011111, shifted by 3 bits : 0b1111100
	}
}

//cette fonction compare si la couleur vue par la caméra du robot à un moment précis correspont à la couleur que le robot doit détecter
// si oui -> retour true si ce n'est pas le cas : retourne false
bool compare_color_viewed(void){
	//stockage de la position de la ligne avec l'ancien filtre
	uint16_t old_line_position = get_line_position();
	//waits until an image has been captured
    chBSemWait(&image_ready_sem);
	//gets the pointer to the array filled with the last image in RGB565
	img_buff_ptr = dcmi_get_last_image_ptr();

	//applique le nouveau filtre à l'image en fonction de la couleur voulue
	switch(get_colorToFollow())
	{
		case RED:
			extract_green_pixels();
			break;
		case GREEN:
			extract_blue_pixels();
			break;
		case BLUE:
			extract_green_pixels();
			break;
		default :
			return false;
	}
	//traitement d'image
	lineWidth = extract_line_width(image);
	//chprintf((BaseSequentialStream *) &SD3, "get line position =%x \n\r", get_line_position()); //utile pour le débug
	if (get_line_position()>(old_line_position-5)&&get_line_position()<(old_line_position+5)){ // if line still existing after changing color filter
		return true;
	} else return false;
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		switch(get_colorToFollow()){
			case GREEN:
			case BLUE:
				extract_red_pixels();
				break;
			case RED:
				extract_blue_pixels();
				break;
			default:
				break;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);
    }
}

// retourne la position de la ligne
uint16_t get_line_position(void){
	return line_position ;
}
// retourne la largeur de la ligne
uint16_t get_line_width(void){
	return lineWidth ;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
