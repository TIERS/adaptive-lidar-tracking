# Adaptive lidar tracking

Tracking MAVs from ground robots with adaptive lidar scan integration

## Summary

TO DO

## Installation

TO DO

## Run it

### Tello Motion Start
tello_motion node is to let tello drone do a certain shape of motion like a circle which is currently supported.

```
roslaunch adaptive_lidar_tracking tello_motion.launch
```
parameters setting includes:
```
position_topic: "/dwm1001/tag/2A24/position"         
drone_cmd_topic: "/tello/cmd_vel" 
takeoff_height: 1.0 
drone_speed: 0.6
```

## Contact

For any questions, write to `jopequ@utu.fi`, `qingqli@utu.fi`, or `xianjia.yu@utu.fi`.

Visit us at [https://tiers.utu.fi](https://tiers.utu.fi)
