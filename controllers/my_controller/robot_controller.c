#include "robot_controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>


static WbDeviceTag left_motor, right_motor;

/* e-puck angular speed (rad/s) */
#define MAX_SPEED 6.28

/* distance sensor */
#define NUMBER_OF_DISTANCE_SENSORS 8
static WbDeviceTag distance_sensors[NUMBER_OF_DISTANCE_SENSORS];
static const char *distance_sensors_names[NUMBER_OF_DISTANCE_SENSORS] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

/* higher threshold accounting for noise*/
#define SENSOR_VALUE_DETECTION_THRESHOLD 280

/* speed of robot to spinning in place (in degrees per second) */
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.588111888

static int TIME_STEP;

void robot_controller_init(int time_step)
{
TIME_STEP = time_step;

    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
  

	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
			distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
			wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
	}
}

static float calculate_rotation_time(float degrees)
{
	return abs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
}

void motor_stop() {
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

void move_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void rotate_right() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

void rotate_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void rotate_left_in_degrees(float degrees) {
	rotate_left();
	
	float duration = calculate_rotation_time(degrees);
	float start_time = wb_robot_get_time();
	do
	{
		wb_robot_step(TIME_STEP);
	} while (wb_robot_get_time() < start_time + duration);
	
	motor_stop();
}

/* function to get sensors condition */
bool *get_sensors_data()
{
	static bool sensor_data[NUMBER_OF_DISTANCE_SENSORS] = {false};
	
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
		if (wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_VALUE_DETECTION_THRESHOLD) {
			sensor_data[i] = true;
		} else {
			sensor_data[i] = false;
		}
	}
	
	return sensor_data;
}

/* function to print sensors values for debugging
 * */
void print_sensor_values() {
	printf("%s sensor values: ", wb_robot_get_name());
	
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
		printf("%d:%.3f ", i, wb_distance_sensor_get_value(distance_sensors[i]));
	}
	
	printf("\n");
}