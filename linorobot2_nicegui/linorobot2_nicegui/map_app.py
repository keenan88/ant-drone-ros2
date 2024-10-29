from nicegui import ui

# Define the HTML and JavaScript code as a string
ros2d_viewer_html = '''
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />

<script src="https://cdn.jsdelivr.net/npm/easeljs@1/lib/easeljs.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/eventemitter2@6/lib/eventemitter2.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/ros2d/build/ros2d.js"></script>

<script>
  function init() {
    // Connect to ROS 2 WebSocket bridge
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    // Create the main viewer
    var viewer = new ROS2D.Viewer({
      divID : 'map',
      width : 640,
      height : 480
    });

    // Setup the map client
    var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene,
      // Use ROS 2 map topic; in ROS 2, the default map topic is typically `/map`
      topic : '/map'
    });

    // Scale the canvas to fit the map
    gridClient.on('change', function() {
      viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
      viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
  }
</script>
</head>

<body onload="init()">
  <h1>ROS 2 Map Example</h1>
  <div id="map"></div>
</body>
</html>
'''

# Embed the HTML code in NiceGUI
with ui.html():
    ui.add_body_html(ros2d_viewer_html)

# Run the app
ui.run()
