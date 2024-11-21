# SMART diffbot workshop

## Launch simulation with full navigation stack

```bash
ros2 launch smart_diffbot_bringup main.launch.py 
```

## Monitor with RViz
```bash
ros2 launch smart_diffbot_navigation rviz.launch.py 
```

## Send docking command
```bash
ros2 run smart_diffbot_clients docking_client 
```
