/*
 * plot.h
 *
 *  Created on: Oct 29, 2023
 *      Author: alan1
 */

#ifndef INC_PLOT_H_
#define INC_PLOT_H_

#include <string.h>
#include <math.h>
#include <stdint.h>

#define MAX_PLOTTED_VALUES 60


// Graph Distance on a scrolling plot
// Use a fifo circular buffer

typedef struct {
	// Define origin of plot on Oled screen
	uint8_t oled_plot_x;
	uint8_t oled_plot_y;
	uint8_t x_axis_length;
	uint8_t y_axis_length;
} plot_t;

// Circular buffer
// Inspired by https://embedjournal.com/implementing-circular-buffer-embedded-c/
typedef struct {
	uint8_t * const distances;
	int head;
	int tail;
	const int maxlen;
} buffer_t;

// removes value from end of buffer
// Returns 1 on success 0 on failure
int pop(buffer_t *buf/*, uint8_t *data*/);

// Adds value to buffer
// Returns 1 on success, 0 on failure
int pushback(buffer_t *buf, uint8_t data);

// Convert distance from float to pixel value
// Float should range from 0.0 to 8.0
uint8_t float_to_pixel(float val);

void draw_axes(plot_t *plot);

// Use within Oled Locks
void clear_graph(plot_t *plot);

// Calculates and draws pixels in oled memory
int calculate_graph(buffer_t *buf, plot_t *plot);

#endif /* INC_PLOT_H_ */
