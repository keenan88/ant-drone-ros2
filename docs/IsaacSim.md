# linorobot2

## Nvidia Setup

1. Follow the instructions in [Nvidia IsaacSim Installation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to: 
 - Install Nvidia drivers [compatable with IsaacSim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html) and [your Nvidia graphics card](https://www.nvidia.com/download/index.aspx). You can check your graphics driver version with the command `nvidia-smi`, if you already have Nvidia graphics drivers installed.
 - Install docker engine (not desktop!)
 - Download Nvidia's IsaacSim Docker image. You will need to generate an [NGC Api key](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#:~:text=Generate%20your%20NGC%20API%20Key) to obtain the IsaacSim docker image.
 - Install the Nvidia container toolkit.
2. Run the IsaacSim container as detailed in the tutorial above. Close any other compute-intense applications before running the IsaacSim container for the first time, it can take a few minutes to loadup the first time. Close the IsaacSim container before you move on with the following instructions.
3. Download the Omniverse Launcher (not SDK) from [Nvidia's website](https://www.nvidia.com/en-us/omniverse/download/). To run the .AppImage file, apt install `libfuse2`, **not** `fuse`.

## Omniverse Content (Optional)

1. Follow Nvidia Setup above
2. Open omniverse launcher and go to the `Echange tab`

![image](https://github.com/user-attachments/assets/b69f20b2-0f75-48c1-9529-ee4849df4c8a)

3. Filter for Content. There is a `Warehouse 3D Models Pack` and `Industrial 3D models pack`.

![image](https://github.com/user-attachments/assets/6ed0422b-d54b-4fdc-9c3a-804e4e2321e1)

4. Copy and unzip downloaded content to [simulation/models](simulation/models). The models will be available in IsaacSim when you launch the container (launch instructions below).

## IsaacSim (running robot in simulation)

1. (first time only) Complete steps in `Nvidia Setup`.
2. (first time only) Run `docker compose -f docker-compose.isaacsim.yaml build`.
3. Run `docker compose -f docker-compose.isaacsim.yaml up`. This can take 20-30s for IsaacSim server to start.

5. Download the Nvidia [omniverse launcher](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage).  
   
7. Start the omniverse launcher by navigating to your downloads folder and running `chmod +x omniverse-launcher-linux.AppImage` and `./omniverse-launcher-linux.AppImage`. This will open the Omniverse Launcher:

![image](https://github.com/keenan88/isaacsim_ros2_greenhouse/assets/45887966/91172e36-8f79-4857-a11f-f74e619535fe)

6. Download the _Omniverse Streaming Client_ from the launcher's "Exchange" tab. Launch the streaming client from the Omniverse Launcher _Library_ tab. Enter the IP address of the IsaacSim container when prompted (127.0.0.1). This should open a view of an empty sim in IsaacSim. 

![image](https://github.com/keenan88/ant-worker-ros2/assets/45887966/a3239330-28c2-475c-ab42-52be73b52d69)

![image](https://github.com/user-attachments/assets/8a0bacd1-4dfb-41bb-bfa4-cf695f2ecea9)

7. `Ctrl + O` to open `/isaac-sim/humble_ws/src/antworker_isaacsim_world/world/ant_drone_greenhouse.usd` and press the play button to start the simulation.

![image](https://github.com/user-attachments/assets/05cc0a20-a319-42a8-a21b-98e4a5914660)

8. Optional: Plugin a Microsoft Xbox Controller (Model 1914) via USB. `X` button is deadman switch for rotation-only movement, `right bumper` is deadman switch for linear/rotational movement. `Right joystick` causes movement.
   
10. Run `docker compose -f docker-compose.teleop.yaml` and use the teleop terminal or xbox controller the move the robot. **Scale down velocity to approx 0.3 m/s and 0.6 rad/s, the simulated robot is unstable at faster velocities!**

![image](https://github.com/user-attachments/assets/1f1a7e7d-c77f-438c-a864-d243dbb46fc9)



