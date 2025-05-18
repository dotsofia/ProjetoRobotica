#include <stdio.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "robot_controller.h"

WbNodeRef caixa;
int TIME_STEP;


static void init() {
  wb_robot_init();
	TIME_STEP = (int)wb_robot_get_basic_time_step();
	robot_controller_init(TIME_STEP);	
}

/* main function */
int main(int argc, char **argv) {
  init();
  caixa = wb_supervisor_node_get_from_def("CAIXA");
  const double PosicaoCaixaX = wb_supervisor_node_get_position(caixa)[0];
  const double PosicaoCaixaY = wb_supervisor_node_get_position(caixa)[1];
  
  while (wb_robot_step(TIME_STEP) != -1) {		
    if (!(fabs(PosicaoCaixaX - wb_supervisor_node_get_position(caixa)[0]) < 0.001 &&
          fabs(PosicaoCaixaY - wb_supervisor_node_get_position(caixa)[1]) < 0.001))
    {
        while (wb_robot_step(TIME_STEP) != -1)
        {
            rotate_left();
        }
    }

		bool *is_sensors_active = get_sensors_data();
		
		if (is_sensors_active[1] && is_sensors_active[6]) 
    {
			rotate_left_in_degrees(180);
		} 
    else if (is_sensors_active[0] || is_sensors_active[1]) 
    {
			rotate_left();
		} 
    else if (is_sensors_active[7] || is_sensors_active[6])
    {
			rotate_right();
		} 
    else 
    {
			move_forward();
		}
  };

  wb_robot_cleanup();

  return 0;
}