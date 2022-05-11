#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

int main(int argc, char *argv[])
{
    wb_robot_init();

    const WbNodeRef root_node = wb_supervisor_node_get_root();
    const WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
    double timeStep = wb_robot_get_basic_time_step();

    WbNodeRef door1, door2;
    WbFieldRef field1, field2;
    door1 = wb_supervisor_field_get_mf_node(root_children_field, 6);
    field1 = wb_supervisor_node_get_field(door1, "position");
    door2 = wb_supervisor_field_get_mf_node(root_children_field, 7);
    field2 = wb_supervisor_node_get_field(door2, "position");

    int cnt = 0;
    while (wb_robot_step(timeStep) != -1)
    {
        if (cnt < 150)
        {
            wb_supervisor_field_set_sf_float(field1, -1.57); // door1开
            wb_supervisor_field_set_sf_float(field2, 0);     // door2关
        }
        else
        {
            wb_supervisor_field_set_sf_float(field1, 0);    // door1关
            wb_supervisor_field_set_sf_float(field2, 1.57); // door2开
        }
        cnt++;
        if (cnt > 300)
            cnt = cnt % 300;
    }

    wb_robot_cleanup();
    return 0;
}
