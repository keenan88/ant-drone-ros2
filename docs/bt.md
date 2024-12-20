# Drone Behavior Tree

The drone uses a behavior tree to coordinate its behavior and its interactions with the [Ant Queen](https://github.com/keenan88/ant-queen-ros2/tree/main). You can download [Groot2](https://www.behaviortree.dev/groot/) to view the tree in a GUI.

## Main tree
![alt text](docs/images/1_main_bt.png)

Registers the drone with the queen once, then continually executes missions from the queen.

## Execute missions
![alt text](docs/images/2_execute_missions.png)

Continually checks if selected for a mission by the queen, and if so, starts the mission.

## Floor mission
![alt text](docs/images/3_floor_mission.png)

Goes to the worker, picks it up, goes to the target location, drops off worker.

## Go To Waypoint With Feedback
![alt text](docs/images/4_go_to_vertex_with_feedback.png)

Publishes a task request to RMF for RMF to send a set of waypoints necessary for the robot to reach the target RMF waypoint. 
RMF then sends the waypoints 1 at a time to the robot for nav2 to execute.
Once RMF's robot handle sees drone made it to commanded vertex, return success.

## Pickup Sequence
![alt text](docs/images/5_pickup_sequence.png)

Suspends RMF path requests to avoid interrupting robot movement during pickup.
If worker is in row, sends a request for worker to come out of row.
Triggers pickup movements, updates drone's footprint since worker is now mounted. Clear costmap of obstacles since worker is no longer an obstacle.
Accept RMF path requests again.

## Dropoff Sequence
![alt text](docs/images/6_dropoff_sequence.png)

Suspend RMF pathing to avoid interrupting robot movement during dropoff.
Send the drone's current position to the worker so the worker can re-localize itself in the map frame, at the start of the row.
Lower the worker, update the drone footprint, and un-suspend RMF pathing so RMF can send post-dropoff location to worker.

## Repeat Idle Check
![alt text](docs/images/7_idle_check.png)
Repeatedly checks if RMF client on drone is currently executing a path.
