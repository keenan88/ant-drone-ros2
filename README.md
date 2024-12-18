# Ant drone

# SLAM (map generation)

1. (first time only) Run `gitmodule init` from the repo root.
2. (first time only) Run `docker compose -f docker-compose.slam.yaml build`


4. Run `docker compose -f docker-compose.slam.yaml up`.
5. RVIZ2 will open a view of the robot, including its base frame, laser scans, and the map being generated.

![image](https://github.com/user-attachments/assets/9cd74e48-2e57-49d6-a1e9-6bf72832615f)

6. Drive the robot around with the teleop terminal or xbox controller.
7. The map will expand as the robot drives around. The robot should stay well-localized to the map, and the scans should stay aligned to the map.

![image](https://github.com/user-attachments/assets/aad7d61e-92ad-4c15-9a90-82a8ed1b8f70)


9. Be sure the robot is in the map bounds or the map will not save.

10. You can view your map in the [map folder](/antdrone_navigation/maps/)  



## Map Cleanup

1. Install gimp: `apt install gimp`
2. Open gimp and open the .pgm map image file generated by nav2

![image](https://github.com/user-attachments/assets/29c1b3a7-fff2-427a-a17e-4ad77640a56c)


4. Remove any dynamic obstacles that should not be in the static costmap with the `Eraser Tool`.

![image](https://github.com/user-attachments/assets/99ea09cf-31ab-47cd-91a0-d3493cb7a703)

5. Select `Tools -> Transform tools -> Rotate`, and rotate the image until the pipe rail lengths look horizontal. Rotation will induce a blur.

![image](https://github.com/user-attachments/assets/1ed3dcb3-8f28-4c4d-81e8-2b742947e1cb)
_The edges of a rotated pipe rail should not slant from one row of pixels up or down, but be perfectly level._

7. Select `Image -> Canvas Size` and change the canvas to fit the rotated image.

![image](https://github.com/user-attachments/assets/d1e0ef54-adbd-407a-9984-4a9f8d6ecb36)
   
9. Select `Filters -> Enhance -> Sharpen`. Change the "Amount" parameter until the pipe rails show up black.

![image](https://github.com/user-attachments/assets/7f10b7c1-bdbf-4456-a169-a070bb039571)
_The edges of a rotated & sharpened pipe rail should not slant from one row of pixels up or down, but be perfectly level._

9. Use the `Pencil Tool` with white ink to remove any undesired dark pixels added in the sharpening.

![image](https://github.com/user-attachments/assets/e2a3d587-ec4e-4b31-a7a2-a6a5bf216daf)
    
11. Press `Ctrl + Shift + E` and export the new map to .pgm format (use default export settings).
12. Press `Ctrl + Shift + E` and export the new map to .png format, if you want to use it in RMF traffic editor (use default export settings).

## Adding keepout zones to map

1. Follow [this](https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html) tutorial to create the .pgm and .yaml files that represent a keepout mask.
2. For compatbility with RMF traffic editor, set the map origin in the yaml file to `[0.0, y, 0.0]`, where `y = -1 * resolution * map height in px` (You can find map height in the .pgm file's properties).
3. Edit [filter_mask_server.yaml](/antdrone_navigation/config/filter_mask_server.yaml) -> `filter_mask_server -> yaml_filename` to point to the yaml file that lists your keepout filter .pgm image.
4. Run ___-compose.yaml. The keepout filter can be viewed in foxglove or rviz2.



https://github.com/open-rmf/rmf_demos/tree/humble


RMF traffic: If a vertex is not connected to a lane, IT WILL NOT BE INCLUDED IN THE NAV GRAPH!!

Nav2 critic scale: https://robotics.stackexchange.com/questions/105749/dwb-planner-in-nav2-does-not-properly-set-scale-for-critics

To get RMF fleet manager to recognize completed task, robot state must have empty path, and mode set to idle or charging


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



