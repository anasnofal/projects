# **Technical Reflection: TurtleBot3 Digital Twin**

This document provides a technical reflection on the development of a digital twin system for the TurtleBot3 robot. It outlines what was achieved, the challenges faced, what was not completed, and potential future work across the three core areas of the project.

username: team13
password:12345678

## **1. Bidirectional Communication**

#### **What Was Achieved**

To establish a clear communication channel, we cloned and renamed the official TurtleBot3 packages to create a unique ROS 2 namespace for the virtual twin. This foundational step prevented topic collisions and allowed for distinct control of each robot. Our application's intelligence was divided across several nodes: a `BinSensorMockNode` to simulate bin fill levels, and a `DecisionNode` acting as a state machine to dispatch the robot. The `cmd_vel_relay` node was crucial, broadcasting velocity commands from Nav2 to both the primary and virtual robots.

A key aspect of our bidirectional communication was the hybrid use of data from both the real and virtual robots. The low-level navigation, managed by the Nav2 stack, relied entirely on the **real robot's sensor data** for obstacle avoidance and path execution. However, the high-level task assignment in our `DecisionNode` subscribed to and used the digital twin's odometry and the topic from the Bin simulation Node and the navigation node that navigates the Real robot. This means the decision of which bin was 'closest' was made from the digital twin's perspective, creating a sophisticated data flow where the real robot handled physical navigation while the digital twin's state influenced mission strategy.

#### **Bottlenecks or Challenges**

A significant challenge was managing the complexity and timing of our multi-node architecture. The system's logic was distributed across the `DecisionNode`, `NavigationExecutorNode`, and `BinSensorMockNode`, which communicated asynchronously. Debugging issues related to timing, message delivery, or state transitions between these nodes was complex. For example, ensuring the `DecisionNode` correctly interpreted the status messages from the `NavigationExecutorNode` without race conditions required careful design and testing. This distributed nature meant that a failure in one node could silently impact the entire system, making it difficult to pinpoint the root cause. so, having a complex code and many states will negatively affect the robot's automated behavior. 

#### **What Was Not Completed**

While our communication system worked for the core task in simulation, we did not achieve a fully **robust, production-ready communication link.** The intermittent failures of the `cmd_vel_relay` node, which would occasionally stop working and require a system restart, meant the system lacked the reliability needed for long-duration, autonomous operation without supervision.

#### **Future Improvements**

1.  **Develop a Hybrid Synchronization System:** We would move beyond a purely open-loop or closed-loop system to a more advanced hybrid model. The primary synchronization would remain the real-time, open-loop `cmd_vel_relay` for maximum responsiveness. In parallel, we would develop a low-frequency, closed-loop "corrector" node. This node would run periodically (e.g., every 5 seconds) to compare the odometry of the real and virtual robots and publish a small, corrective adjustment to fix any drift or mismatch, combining the benefits of real-time control and long-term accuracy.
2.  **Synchronize Real-World Obstacles to Gazebo:** We would create a perception node to detect dynamic obstacles in the real world using the robot's Lidar scan. This node would then use a Gazebo service call to spawn a corresponding virtual object in the simulation, ensuring the virtual environment accurately mirrors the real one.
  
3.  We would also implement a communication channel for internal hardware states. For example, a RobotStatusNode on the real robot could publish its battery voltage,which is act similar to real truck fuel. A corresponding node in the digital twin environment would subscribe to this, allowing us to visualize the twin's battery draining in sync with the real robot. This fulfills the requirement to mirror not just pose, but also internal operational conditions to help make better decisions regarding the route. 

## **2. State Synchronization**

#### **What Was Achieved**

Our strategy for state synchronization was **command-based mirroring**. We focused on synchronizing the robot's movement by using the `cmd_vel_relay` node. This approach broadcasted the exact same velocity commands from the Nav2 stack to both robots simultaneously. In a controlled simulation environment, this open-loop method resulted in the two robots successfully mirroring each other's paths and maintaining a synchronized pose.

#### **Bottlenecks or Challenges**

The command-based mirroring approach is fundamentally **open-loop and not robust**. Its greatest challenge is that it assumes both robots will react identically to identical commands, which is rarely true in practice. It cannot account for critical physical differences such as:
* inaccurate odometry data from wheel encoders,
* wheel slippage on different surfaces,
* minor sensor calibration differences, 
* the navigation map shifting or the robot becoming lost due to noisy real-world sensor data.

All of these factors lead to inevitable state divergence over time. Our initial attempt to build a more robust, closed-loop `OdomToGazeboPoseNode` to solve these issues was a significant challenge that we were unable to overcome.Our key insight from this challenge was that synchronizing commands is fundamentally different from synchronizing the actual measured state.

#### **What Was Not Completed**

We did not achieve robust state synchronization when integrating with the physical TurtleBot3. The open-loop nature of our system was exposed by a real-world localization problem. Due to the symmetrical features of our map, the physical robot could not determine its correct initial orientation. This caused a complete **state divergence**: when given a goal, the physical robot would move towards one corner while the virtual twin, with a different understanding of its pose, would move towards another. This failure was the ultimate outcome of an open-loop synchronization strategy that lacked a feedback mechanism to correct for real-world uncertainty. and a clear starting point and orientation.



#### **Future Improvements **

1.  **Implement Closed-Loop State Synchronization:** The most critical future work is to replace the open-loop `cmd_vel_relay` node with a closed-loop feedback controller. We already designed the `PoseControllerNode` for this purpose. This node would:
    * Subscribe to the real robot's `/odom` topic to get its measured state.
    * Subscribe to the virtual robot's `/my_tb3/odom` topic to get its current state.
    * Continuously calculate the error between the two poses and publish corrective `cmd_vel` commands to the virtual robot to minimize this error.
    This would create a high-fidelity mirror of the real robot's *actual measured pose*, directly solving the state divergence issue we observed.

2.  **Add a Localization Sanity Check:** To prevent the kind of state divergence we saw when the Digital robot was poorly localized, we would add a "Sanity Check" node. This node would monitor the TF transforms for both the real robot (`map` -> `base_footprint`) and the virtual robot (`map` -> `my_tb3/base_footprint`). If the distance between the two robots ever exceeds a certain threshold , it would publish a system-wide "PAUSE" or "WARNING" signal, preventing the `DecisionNode` from dispatching new tasks until the digital robot's localization is corrected.

## **3. Environmental & Object Interaction**

#### **What Was Achieved**

We successfully implemented environmental interaction on the **real robot**, which acted as the leader in our twin system. The real robot ran a full Nav2 stack, using its own sensor data to autonomously navigate our test environment and perform all dynamic obstacle avoidance. The digital twin did not run its own navigation; its interaction was limited to passively mirroring the movements of the real robot. On a higher level, a successful **logical interaction** was achieved through our "smart waste collection" application, which intelligently dispatched the leader robot to various goals based on simulated sensor data.

#### **Bottlenecks or Challenges**

The main challenge of this leader-follower architecture was the complete **lack of environmental awareness in the digital twin**. Because the twin was only mirroring the leader's movements and not performing its own navigation, it was effectively "blind" to the Gazebo simulation environment. This meant that if an obstacle existed in the simulation that was not present in the real world, the twin would drive straight through it, breaking the validity of the simulation. This one-way interaction model made it impossible to test or simulate how the system would react to dangers or opportunities that only appeared in the virtual space.


#### **Future Improvements**

1.  **Create a High-Fidelity "Smart Bin" Model:** Instead of the simple `BinSensorMockNode`, we would develop a proper URDF model for the smart bin. This model would include its own plugins to simulate sensor readings and behavior, making the virtual object a much more realistic mirror of a potential real-world counterpart.
2.  **Establish Shared Environmental Awareness:** We would implement a system for true, synchronized interaction. This would involve creating a node that processes DT robot's sensor data (e.g., from `/scan`) to detect an obstacle and then dynamically adds that obstacle to the Nav2 costmap of the Real robot,by transforming the coordinates of the obstacles into the real robot's map frame, and publish them as a PointCloud2. The real robot's Nav2 costmap would be configured to use this point cloud as an additional obstacle source, causing the physical robot to plan paths around purely virtual objects. This would cause both robots to react to the same environmental feature, even if only one can directly perceive it.
