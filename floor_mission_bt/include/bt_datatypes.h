#include "rmf_task_msgs/msg/api_request.hpp"

#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"


#include "ant_fleet_interfaces/srv/register_robot.hpp"
#include "ant_fleet_interfaces/srv/check_if_selected_for_floor_mission.hpp"
#include "ant_fleet_interfaces/srv/check_drone_idle.hpp"
#include "ant_fleet_interfaces/srv/mission_heartbeat_srv.hpp"
#include "ant_fleet_interfaces/srv/last_known_end_waypoint_name.hpp"
#include "ant_fleet_interfaces/srv/suspend_rmf_pathing.hpp"
#include "ant_fleet_interfaces/srv/move_out.hpp"
#include "ant_fleet_interfaces/srv/mission_success.hpp"
#include "ant_fleet_interfaces/srv/check_if_come_out_triggered.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"