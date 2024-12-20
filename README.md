# Ant drone

## SLAM Startup - Simulation

1. Run `xhost +local:docker`.
2. Plugin the `Microsoft Corp. Xbox Wireless Controller (model 1914)` to the computer. 
3. Run `docker compose -f docker-compose.sim.slam_demo.yaml up`. The first time will take longer for the containers to build.
4.  RVIZ2 will open a view of the robot, including its base frame, laser scans, and the map being generated.

![image](https://github.com/user-attachments/assets/46f9765e-67b2-4c94-81f8-cf27c6a01253)

6. Drive the robot around with the teleop terminal or xbox controller.
7. The map will expand as the robot drives around. The robot should stay well-localized to the map, and the scans should stay aligned to the map.
8. To save a map, run `docker exec -it antdrone0_slam` and `ros2 run nav2_map_server map_saver_cli -f /home/humble_ws/src/antdrone_slam/maps/`.
9. Be sure the robot is in the map bounds or the map will not save.
10. You can view your map in the [map folder](/antdrone_slam/maps/).
11. For compatability with RMF traffic maps, edit the map's yaml file to have its coordinates in the upper-left hand corner of the image.
12. Gimp can be used to cleanup any noise or moveable obstalces in the map, and rotate the map to be aligned with it's image coordinates.
13. Follow [this tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html) to add keepout zones to the map.



## Startup Instructions

1. Run `xhost +local:docker` to allow RVIZ to display from Docker containers.
<!-- 2. In `.env`, set _SLAM_OR_NAV_ to either _NAV_ or _SLAM_. -->
3. (Optional) Plug in a Microsoft Xbox controller (Model 1914) to USB.
4. Run `docker compose build`.
5. Run `docker compose up`. It may take 1-2 minutes for Gazebo to load. A `teleop twist` terminal window will open for you to drive the robot.
6. If running NAV, set the robot's starting position in RVIZ2 with the "2D pose estimate" button. Spin the robot a few times until the scans and map line up in RVIZ2.

## Expected Results

- Gazebo will display a view of the robot and the world, including all the lidar scans.
  
  ![Gazebo View](https://github.com/user-attachments/assets/fb57d7c8-b000-4d7e-bef2-c3fcf2c7c44f)

- SLAM: RVIZ will open with the robot's view of the world and should begin auto-generating a map using SLAM.
  
  ![RVIZ Map View](https://github.com/user-attachments/assets/4be799ff-4d3a-48c9-b58a-216c756eebee)

- NAV: RVIZ will open the robots view of the world with the pre-generated map.

![image](https://github.com/user-attachments/assets/ab75c0cb-3a92-4946-96fa-c2bac89121b6)

- NAV: Nav2 goal sent using RVIZ
![image](https://github.com/user-attachments/assets/3130b8f3-9c7c-4197-aa68-da44a7812d5c)



Drone mermaid diagram

```mermaid
flowchart TD
    nav2["nav2"] -- cmd_vel --> gz_domain_bridge["gz_domain_bridge"]
    gz_domain_bridge -- odom --> nav2
    gz_domain_bridge -- TF: odom to base link --> nav2
    queen_client -- navigate to pose --> nav2
    queen_client -- robot rmf state --> queen_domain_bridge
    gz_domain_bridge -- pointclouds --> pointcloud_filtering["pointcloud filtering"]
    pointcloud_filtering -- scan --> nav2
    queen_domain_bridge -- "Mission vertices" --> queen_client
    bt <-- Mission coordination --> queen_domain_bridge
    queen_domain_bridge -- Mission requests --> bt
```
