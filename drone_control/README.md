## Drone Manual

Designed for use with Marno Nel's [drone_control package](https://github.com/Marnonel6/ROS2_offboard_drone_control/tree/main) and [custom 15" PX4-Based Quadcopter](https://marnonel6.github.io/projects/0-autonomous-px4-drone).

### Contents

1. [Manual Flight](#manual-flight)
2. [Autonomous Flight](#autonomous-flight)
3. [Other](#other)  
   * [Important Information](#important-information)
   * [Pre-Flight Checklist](#pre-flight-checklist)
   * [Post-Flight Checklist](#post-flight-checklist)
   * [QGroundControl](#qgroundcontrol)
   * [Simulation](#simulation)


### Manual Flight

The drone can be flown manually with the RadioMaster TX16 Controller (RC). This does not require the Raspberry Pi to be powered.

**Before flight, follow the [Pre-Flight Checklist](#pre-flight-checklist)**

1. Ensure correct drone configuration, including RC's button and stick mapping.  
   * See [QGroundControl](#qgroundcontrol) for more information.
2. Power the Pixhawk and motors with a 6s LiPo battery.  
   * ESCs should beep. If not, check connections.
3. ARM the drone by moving the RC sticks to the ARM configuration (L-->BR; R-->BL).  
   * ESCs should beep again. If not, connect to QGroundControl and see error.
   * Once armed, the drone is ready for flight.
4. Slowly increase the throttle to lift off smoothly.
5. Once landed, DISARM the drone (L-->BL; R-->BR).


### Autonomous Flight

Nodes can be written and loaded onto the Raspberry PI.


### Other

#### Important Information

* To fly the drone, you must first pass [The Recreational UAS Safety Test (TRUST)](https://www.faa.gov/uas/recreational_flyers/knowledge_test_updates).
* To fly the drone on Northwestern property, you must follow the University's [official guidance](https://www.faa.gov/uas/recreational_flyers/knowledge_test_updates), including flying in the designated UAS zones.

#### Pre-Flight Checklist

1. Mission Planning  
   * Use the B4UFLY app to verify your flight location.  
   * Ensure appropriate weather conditions.
2. Hardware Check  
   * Ensure batteries are charged; check for damage, swelling.
   * With batteries disconnected, check propellers are secured.
   * Check frame and body for any damage or cracks.
3. Connect Pixhawk to QGroundControl.  
   * Check for errors on main screen.
   * Ensure GPS can find satellites.
   * Ensure failsafe settings are configured correctly.
4. Takeoff  
   * Ensure the takeoff area is clear and flat.

#### Post-Flight Checklist

#### QGroundControl



