#include "rmf_task_msgs/msg/api_request.hpp"

#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"


#include "ant_queen_interfaces/srv/register_robot.hpp"
#include "ant_queen_interfaces/srv/check_if_floor_mission_triggered.hpp"
#include "ant_queen_interfaces/srv/last_known_end_waypoint_name.hpp"
#include "ant_queen_interfaces/srv/mission_success.hpp"
#include "ant_queen_interfaces/srv/come_out.hpp"
#include "ant_queen_interfaces/srv/dropoff_pos.hpp"

#include "antdrone_interfaces/srv/suspend_release_rmf_pathing.hpp"
#include "antdrone_interfaces/srv/check_rmf_client_idle.hpp"
#include "antdrone_interfaces/srv/mission_heartbeat_srv.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"