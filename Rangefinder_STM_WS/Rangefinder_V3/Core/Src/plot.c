/*
 * plot.c
 *
 *  Created on: Oct 29, 2023
 *      Author: alan1
 */

#include "plot.h"
#include "main.h"
#include "cmsis_os.h"
#include "ssd1306.h"

extern osMutexId_t OledLock;

int pop(buffer_t *buf/*, uint8_t *data*/){
	int next;

	// No data
	if (buf->head == buf->tail)
		return 0;

	next = buf->tail + 1;
	if(next >= buf->maxlen)
		next = 0;

//	*data = buf->distances[buf->tail];
	buf->tail = next;
	return 1;
}

int pushback(buffer_t *buf, uint8_t data){
	int next;

	// Head pts to next after this write
	next = buf->head + 1;
	if(next >= buf->maxlen)
		next = 0;

	// Buffer full
	if(next == buf->tail)
		return 0;

	buf->distances[buf->head] = data;
	buf->head = next;
	return 1;
}

// Convert distance from float to pixel value
// Float should range from 0.0 to 8.0
uint8_t float_to_pixel(float val){
	// Multiply and ceil number
	return (uint8_t)ceil(val * 3);
}


void draw_axes(plot_t *plot){
	// x-axis
	ssd1306_Line(plot->oled_plot_x - 1, plot->oled_plot_y + 2,
			plot->oled_plot_x + plot->x_axis_length,
			plot->oled_plot_y + 2,
			White);

	// y-axis
	ssd1306_Line(plot->oled_plot_x - 2, plot->oled_plot_y + 1,
				plot->oled_plot_x - 2,
				plot->oled_plot_y - plot->y_axis_length,
				White);
}


// Use within Oled Locks
void clear_graph(plot_t *plot){
	ssd1306_FillRectangle(plot->oled_plot_x, plot->oled_plot_y,
						  plot->oled_plot_x + plot->x_axis_length,
						  plot->oled_plot_y - plot->y_axis_length - 4,
						  Black);
}

// Calculates and draws pixels in oled memory
int calculate_graph(buffer_t *buf, plot_t *plot){
	osMutexAcquire(OledLock, osWaitForever);

	// clears plot area
	clear_graph(plot);

	for(int i = 0; i < MAX_PLOTTED_VALUES; i++){
//		if(buf->distances[i] > 0){
		uint8_t x_coord = i + plot->oled_plot_x;
		uint8_t y_coord = plot->oled_plot_y - buf->distances[i] ;

		ssd1306_DrawPixel(x_coord, y_coord, White);
//		}
	}
	osMutexRelease(OledLock);
	return 0;
}

