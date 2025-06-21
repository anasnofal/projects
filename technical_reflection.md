# **Technical Reflection: TurtleBot3 Digital Twin**

This document provides a technical reflection on the development of a digital twin system for the TurtleBot3 robot. It outlines what was achieved, the challenges faced, what was not completed, and potential future work across the three core areas of the project.

## **1. Bidirectional Communication**

#### **What Was Achieved**

To establish a clear communication channel, we cloned and renamed the official TurtleBot3 packages to create a unique ROS 2 namespace for the virtual twin. This foundational step prevented topic collisions and allowed for distinct control of each robot. Our application's intelligence was divided across several nodes: a `BinSensorMockNode` to simulate bin fill levels, and a `DecisionNode` acting as a state machine to dispatch the robot. The `cmd_vel_relay` node was crucial, broadcasting velocity commands from Nav2 to both the primary and virtual robots.

A key aspect of our bidirectional communication was the **hybrid use of data from both the real and virtual robots**. The low-level navigation, managed by the Nav2 stack, relied entirely on the **real robot's sensor data** for obstacle avoidance and path execution. However, the high-level task assignment in our `DecisionNode` subscribed to and used the **digital twin's odometry**. This means the decision of which bin was 'closest' was made from the digital twin's perspective, creating a sophisticated data flow where the real robot handled physical navigation while the digital twin's state influenced mission strategy.

#### **Bottlenecks or Challenges**

A significant challenge was managing the complexity and timing of our multi-node architecture. The system's logic was distributed across the `DecisionNode`, `NavigationExecutorNode`, and `BinSensorMockNode`, which communicated asynchronously. Debugging issues related to timing, message delivery, or state transitions between these nodes was complex. For example, ensuring the `DecisionNode` correctly interpreted the status messages from the `NavigationExecutorNode` without race conditions required careful design and testing. This distributed nature meant that a failure in one node could silently impact the entire system, making it difficult to pinpoint the root cause.

#### **What Was Not Completed**

While our communication system worked for the core task in simulation, we did not achieve a fully **robust, production-ready communication link.** The intermittent failures of the `cmd_vel_relay` node, which would occasionally stop working and require a system restart, meant the system lacked the reliability needed for long-duration, autonomous operation without supervision.

#### **Future Improvements (5-8 Weeks)**

1.  **Develop a Hybrid Synchronization System:** We would move beyond a purely open-loop or closed-loop system to a more advanced hybrid model. The primary synchronization would remain the real-time, open-loop `cmd_vel_relay` for maximum responsiveness. In parallel, we would develop a low-frequency, closed-loop "corrector" node. This node would run periodically (e.g., every 5 seconds) to compare the odometry of the real and virtual robots and publish a small, corrective adjustment to fix any drift or mismatch, combining the benefits of real-time control and long-term accuracy.
2.  **Synchronize Real-World Obstacles to Gazebo:** We would create a perception node to detect dynamic obstacles in the real world using the robot's Lidar scan. This node would then use a Gazebo service call to spawn a corresponding virtual object in the simulation, ensuring the virtual environment accurately mirrors the real one.
3.  **Formalize the Twin Interface Protocol:** Instead of relying on multiple, separate topics (`/cmd_vel`, `/odom`, status topics), we would define a custom ROS 2 action or service specifically for twin management. This would create a single, robust API for controlling and querying the twin's state, making the communication more organized, maintainable, and easier to debug.

## **2. State Synchronization**

#### **What Was Achieved**

Our strategy for state synchronization was **command-based mirroring**. We focused on synchronizing the robot's movement by using the `cmd_vel_relay` node. This approach broadcasted the exact same velocity commands from the Nav2 stack to both robots simultaneously. In a controlled simulation environment, this open-loop method resulted in the two robots successfully mirroring each other's paths and maintaining a synchronized pose.

#### **Bottlenecks or Challenges**

The command-based mirroring approach is fundamentally **open-loop and not robust**. Its greatest challenge is that it assumes both robots will react identically to identical commands, which is rarely true in practice. It cannot account for physical differences like wheel slippage or minor sensor calibration differences, leading to inevitable state divergence over time. Our initial attempt to build a more robust, closed-loop `OdomToGazeboPoseNode` was a significant challenge that we were unable to overcome.

#### **What Was Not Completed**

We did not achieve robust state synchronization **when integrating with the physical TurtleBot3.** The open-loop nature of our system was exposed by a real-world localization problem. Due to the symmetrical features of our map, the physical robot could not determine its correct initial orientation. This caused a complete **state divergence**: when given a goal, the physical robot would move towards one corner while the virtual twin, with a different understanding of its pose, would move towards another. This failure was the ultimate outcome of an open-loop synchronization strategy that lacked a feedback mechanism to correct for real-world uncertainty.

## **3. Environmental & Object Interaction**

#### **What Was Achieved**

We successfully implemented two forms of environmental interaction. First, a high-level **logical interaction** was achieved via our "smart waste collection" application, where the robot would be dispatched to a bin's location based on its simulated state. Second, a basic **physical interaction** was achieved through **obstacle avoidance**. Each robot, using its own Nav2 stack and sensor data, could successfully detect and navigate around obstacles within its own environment.

#### **Bottlenecks or Challenges**

The main challenge was the lack of **shared environmental awareness**. While both robots could avoid obstacles, they did so in isolation. An obstacle present in the real world would not appear in the simulation, and vice-versa. This meant that an avoidance maneuver by one robot would not be mirrored by the other, leading to state divergence and compromising any coordinated task. Furthermore, the "smart bin" interaction was purely logical and did not involve any physical manipulation.

#### **What Was Not Completed**

We did not achieve fully synchronized interaction. The obstacle avoidance was performed independently, not as a shared, mirrored behavior. Furthermore, direct physical manipulation of objects, such as pushing a bin, was not implemented.

#### **Future Improvements (5-8 Weeks)**

1.  **Create a High-Fidelity "Smart Bin" Model:** Instead of the simple `BinSensorMockNode`, we would develop a proper URDF model for the smart bin. This model would include its own plugins to simulate sensor readings and behavior, making the virtual object a much more realistic mirror of a potential real-world counterpart.
2.  **Establish Shared Environmental Awareness:** We would implement a system for true, synchronized interaction. This would involve creating a node that processes one robot's sensor data (e.g., from `/scan`) to detect an obstacle and then dynamically adds that obstacle to the Nav2 costmap of the other robot. This would cause both robots to react to the same environmental feature, even if only one can directly perceive it.