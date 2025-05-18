
#include <stdbool.h>

void robot_controller_init();

void stop();

void move_forward();

void rotate_right();

void rotate_left();

void rotate_left_in_degrees(float degrees);

bool * get_sensors_data();

void print_sensor_values();