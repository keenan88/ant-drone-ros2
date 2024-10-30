# linorobot2

## Nvidia Setup

1. Follow the instructions in [Nvidia IsaacSim Installation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to: 
 - Install Nvidia drivers [compatable with IsaacSim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html) and [your Nvidia graphics card](https://www.nvidia.com/download/index.aspx). You can check your graphics driver version with the command `nvidia-smi`, if you already have Nvidia graphics drivers installed.
 - Install docker engine (not desktop!)
 - Download Nvidia's IsaacSim Docker image. You will need to generate an [NGC Api key](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#:~:text=Generate%20your%20NGC%20API%20Key) to obtain the IsaacSim docker image.
 - Install the Nvidia container toolkit.
2. Run the IsaacSim container as detailed in the tutorial above. Close any other compute-intense applications before running the IsaacSim container for the first time, it can take a few minutes to loadup the first time. Close the IsaacSim container before you move on with the following instructions.
3. Download the Omniverse Launcher (not SDK) from [Nvidia's website](https://www.nvidia.com/en-us/omniverse/download/). To run the .AppImage file, apt install `libfuse2`, **not** `fuse`.

## IsaacSim (running robot in simulation)

1. Complete steps in `Nvidia Setup`
2. Run `docker compose -f docker-compose.isaacsim.yaml build`. This could take a few minutes the first time.
3. Run `docker compose -f docker-compose.isaacsim.yaml up`. This can take 20-30s for IsaacSim server to start.

5. Download the Nvidia [omniverse launcher](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage).  
   
7. Start the omniverse launcher by navigating to your downloads folder and running `chmod +x omniverse-launcher-linux.AppImage` and `./omniverse-launcher-linux.AppImage`. This will open the Omniverse Launcher:

![image](https://github.com/keenan88/isaacsim_ros2_greenhouse/assets/45887966/91172e36-8f79-4857-a11f-f74e619535fe)

6. Download the _Omniverse Streaming Client_ from the launcher's "Exchange" tab. Launch the streaming client from the Omniverse Launcher _Library_ tab. Enter the IP address of the IsaacSim container when prompted (127.0.0.1). This should open a view of an empty sim in IsaacSim. 

![image](https://github.com/keenan88/ant-worker-ros2/assets/45887966/a3239330-28c2-475c-ab42-52be73b52d69)

![image](https://github.com/user-attachments/assets/8a0bacd1-4dfb-41bb-bfa4-cf695f2ecea9)

7. `Ctrl + O` to open `/isaac-sim/humble_ws/src/antworker_isaacsim_world/world/ant_drone_greenhouse.usd` and press the play button to start the simulation.

![image](https://github.com/user-attachments/assets/05cc0a20-a319-42a8-a21b-98e4a5914660)

8. Run `docker compose -f docker-compose.teleop.yaml` and use the teleop terminal the move the robot. **Scale down velocity to approx 0.3 m/s and 0.6 rad/s, the simulated robot is unstable at faster velocities!**

![image](https://github.com/user-attachments/assets/1f1a7e7d-c77f-438c-a864-d243dbb46fc9)


# SLAM (map generation)




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
3. Edit [filter_mask_server.yaml](/linorobot2_navigation/config/filter_mask_server.yaml) -> `filter_mask_server -> yaml_filename` to point to the yaml file that lists your keepout filter .pgm image.
4. Run ___-compose.yaml. The keepout filter can be viewed in foxglove or rviz2.



https://github.com/open-rmf/rmf_demos/tree/humble


RMF traffic: If a vertex is not connected to a lane, IT WILL NOT BE INCLUDED IN THE NAV GRAPH!!

Nav2 critic scale: https://robotics.stackexchange.com/questions/105749/dwb-planner-in-nav2-does-not-properly-set-scale-for-critics

To get RMF fleet manager to recognize completed task, robot state must have empty path, and mode set to idle or charging


Forked from [linorobot2](https://github.com/linorobot/linorobot2).

## Startup Instructions

1. Run `xhost +local:docker` to allow RVIZ to display from Docker containers.
2. In `.env`, set _SLAM_OR_NAV_ to either _NAV_ or _SLAM_.
3. (Optional) Plug in a Microsoft Xbox controller (Model 1914) to USB.
4. Run `docker compose build`.
5. Run `docker compose up`. It may take 1-2 minutes for Gazebo to load. A `teleop twist` terminal window will open for you to drive the robot.
6. If running NAV, set the robot's starting position in RVIZ2 with the "2D pose estimate" button. Spin the robot a few times until the scans and map line up in RVIZ2.
7. Drive the robot using:
   - Xbox controller (Hold the right bumper and toggle the right joystick to move), **or**
   - Run `docker exec -it linorobot2-nav-slam bash` and `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to control the robot via keyboard.

## Expected Results

- Gazebo will display a view of the robot and the world, including all the lidar scans.
  
  ![Gazebo View](https://github.com/user-attachments/assets/fb57d7c8-b000-4d7e-bef2-c3fcf2c7c44f)

- SLAM: RVIZ will open with the robot's view of the world and should begin auto-generating a map using SLAM.
  
  ![RVIZ Map View](https://github.com/user-attachments/assets/4be799ff-4d3a-48c9-b58a-216c756eebee)

- NAV: RVIZ will open the robots view of the world with the pre-generated map.

![image](https://github.com/user-attachments/assets/ab75c0cb-3a92-4946-96fa-c2bac89121b6)

- NAV: Nav2 goal sent using RVIZ
![image](https://github.com/user-attachments/assets/3130b8f3-9c7c-4197-aa68-da44a7812d5c)



