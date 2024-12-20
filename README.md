# Ant drone

## SLAM Startup - Simulation

1. Run `xhost +local:docker`.
2. Plugin the `Microsoft Corp. Xbox Wireless Controller (model 1914)` to the computer. 
3. Run `docker compose -f docker-compose.sim.slam_demo.yaml up`. The first time will take longer for the containers to build.
4. RVIZ2 will open a view of the robot, including its base frame, laser scans, and the map being generated.

![image](https://github.com/user-attachments/assets/46f9765e-67b2-4c94-81f8-cf27c6a01253)

6. Drive the robot around with the teleop terminal or xbox controller.
7. The map will expand as the robot drives around. The robot should stay well-localized to the map, and the scans should stay aligned to the map.
8. To save a map, run `docker exec -it antdrone0_slam` and `ros2 run nav2_map_server map_saver_cli -f /home/humble_ws/src/antdrone_slam/maps/`.
9. Be sure the robot is in the map bounds or the map will not save.
10. You can view your map in the [map folder](/antdrone_slam/maps/).
11. For compatability with RMF traffic maps, edit the map's yaml file to have its coordinates in the upper-left hand corner of the image.
12. Gimp can be used to cleanup any noise or moveable obstalces in the map, and rotate the map to be aligned with it's image coordinates.
13. Follow [this tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html) to add keepout zones to the map.

## Navigation Startup - Simulation

1. Run `xhost +local:docker`.
2. Plugin the `Microsoft Corp. Xbox Wireless Controller (model 1914)` to the computer. 
3. Run `docker compose -f docker-compose.sim.nav_demo.yaml up`. The first time will take longer for the containers to build.
4. RVIZ2 will open a view of the robot, including its base frame, laser scans, and the map being generated.
5. Drive the robot around with the xbox controller. The robot should stay well-localizaed and the scans should stay aligned with the map.
6. Send a nav2 goal with rviz. The robot should automatically drive itself to the goal position.

![image](https://github.com/user-attachments/assets/0fc7edd3-e9e9-4f7a-86c6-1e48aa4bf5e2)


## High-level interaction with Queen

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

See further [documentation of the behavior tree](docs/bt.md).
