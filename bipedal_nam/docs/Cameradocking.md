# **MobiDock: Design and Control of A Modular Self Reconfigurable Bimanual Mobile Manipulator via Robotic Docking** 

Xuan Thuan Nguyen, Khac Nam Nguyen, Ngoc Duy Tran, Thi Thoa Mac, Anh Nguyen, Hoang Hiep Ly, Tung D. Ta* 

**_Abstract_ — Multi-robot systems, particularly mobile manipulators, face inherent challenges in coordination and dynamic stability during collaborative tasks. Rather than focusing on complex software-based coordination algorithms, this study proposes MobiDock, a modular reconfigurable system that introduces a physical docking mode as a structural solution to these challenges. By allowing two independent mobile manipulators to physically connect via a vision-based autonomous docking strategy and a threaded screw-lock mechanism, the system transforms into a unified bimanual platform. This hardware-level reconfiguration simplifies the control problem by treating the multi-robot assembly as a single kinematic entity, thereby augmenting the operational range of multi-robot teams. Experimental results demonstrate that this additional docked mode significantly outperforms traditional independent cooperation in dynamic stability and efficiency. Specifically, the unified configuration achieves lower Root Mean Square Acceleration (RMSA) and Jerk values, better angular precision, and faster task completion times. These findings confirm that integrating a physical docking mode is a powerful design principle that complements existing multi-robot systems, offering a highly stable and efficient alternative for complex tasks in real-world environments.** 

## I. INTRODUCTION 

Robotics is rapidly evolving toward highly flexible, adaptable systems. One promising direction is the development of reconfigurable robots that can change their morphology to match different tasks and environments [1]. This ability to adapt both form and function is a major advantage, improving robustness and versatility in situations where fixedform robots are less effective. They have been applied across diverse application domains, including industrial manipulation [2], autonomous cleaning robot [3], search and rescue operation [4], and space exploration [5]. 

Among reconfigurable robot architectures, modular selfreconfigurable robots (MSRRs) are an increasingly prominent and fast-growing direction. These systems include multiple independent robotic modules that can be interconnected to form various configurations [6], [7]. This modularity offers several advantages, including increased flexibility and the ability to overcome the physical limitations of a single module, such as restricted reach or payload capacity [8], [9]. MSRRs are particularly valuable for tasks that require a change in form to navigate different environments, for example, SolderCubes [10], SMORES-EP [11], RSModCubes [12], M-TRAN [13], and FireAnt [14] demonstrating these capabilities in many applications. 

Besides the development of MSRRs, mobile manipulator robots have long been recognized for their dexterity and flexibility, combining the mobility of a wheeled base with 



Fig. 1: Key components of the proposed MobiDock system and its experimental applications: (a) the post-reconfigured bimanual robot system, (b-c) the independent mobile manipulator modules before docking, (d) the mechanical docking mechanism, (e) the system undergoes a stability test while lifting a box, (f) a cooperative manipulation experiment demonstrates the system’s performance by picking up trash and putting it in a bin. 

the precision of a robotic arm [15], [16]. This integration has opened new possibilities for tackling complex tasks in unstructured environments [17], [18], [19]. However, flexibility comes with a considerable increase in control complexity and a lack of dynamic stability, particularly when two or more mobile manipulators must coordinate to perform a shared task [20], [21], [22]. The main challenge is the strong kinematic and dynamic coupling between robots, which demands advanced control strategies to ensure formation stability, collision avoidance, and minimal internal forces. Several approaches have been proposed to address this challenge, including distributed control, adaptive algorithms, and reinforcement learning, each demonstrating encouraging results. [23], [24], [25], [26]. In practice, these approaches typically rely on high communication bandwidth, fast computation, and accurate state estimation, requirements that are challenging to meet in chaotic real-world environments. Consequently, many solutions perform well in laboratory settings but remain difficult to scale in real-world deployment. This study addresses a critical gap: while MSRRs provide high structural adaptability and mobile manipulators offer superior dexterity, a system that seamlessly transitions between independent, cooperative, and unified states remains an open challenge. We propose MobiDock, a modular reconfigurable mobile manipulator system designed to augment the operational repertoire of multi-robot teams by introducing a high- 

stability docking mode. The central concept of MobiDock is to allow independent mobile manipulators to physically dock, transforming a complex multi-robot coordination problem into the control of a single, unified bimanual platform. Crucially, this approach is not intended to replace traditional software-based cooperative strategies; rather, it provides an additional, hardware-enabled operational mode specifically optimized for tasks requiring high load capacity and dynamic stability. By establishing a rigid physical link, we effectively “collapse” the coordination complexity, bypassing the synchronization bottlenecks and communication latencies inherent in multi-robot teams while preserving the individual flexibility of each module. 

The primary contributions of this paper, visually summarized in Fig. 1, are as follows: 

- 1) The design and realization of **MobiDock** , a system that enables a multi-modal operational framework, allowing robots to switch between separated mobile manipulators for independent tasks execution and a unified, rigid bi-manual configuration. 

- 2) An integrated control strategy that manages the entire lifecycle of reconfiguration, including a robust visionbased docking procedure and a unified control law for the post-docking phase. 

- 3) A comprehensive experimental validation demonstrating that the proposed docking mode provides a superior alternative for stability-critical tasks compared to traditional independent coordination. 

## II. PRELIMINARIES 

## _A. System Overview_ 

This study introduces a reconfigurable mobile manipulator system designed to dynamically form new configurations by docking multiple modules. This allows the system to adjust its kinematic structure and payload capacity for tasks ranging from cooperative transport to complex multi-arm manipulation, providing a flexible solution to limitations inherent in fixed-configuration robots. 

The system’s basic module, based on the LeKiwi<sup>1</sup> design, features a 6-DOF manipulator mounted on a three-wheel omnidirectional mobile base. This triangular omni-wheel configuration provides the maneuverability required for precise docking, allowing for lateral and rotational adjustments without complex maneuvers while the manipulator executes diverse tasks. 

Integrating modules via a base-level coupling mechanism presents a distinct control challenge compared to arm-based docking. However, the omnidirectional platform mitigates this by enabling precise pose attainment and compensation for external disturbances. Furthermore, this design simplifies the kinematic control of the reconfigured system by decoupling translational and rotational motions, ensuring an efficient transition into a unified multi-arm platform. 

## _B. Kinematic Modeling_ 

The mobile base consists of a rigid body with three omnidirectional wheels, each placed at a 120<sup>_◦_</sup> interval from the others. We define the robot’s pose in the global frame as a vector **q** = [ _x, y, θ_ ]<sup>_T_</sup> , where ( _x, y_ ) are the coordinates of the robot’s center and _θ_ is its orientation. The robot’s linear and angular velocities are represented by **˙q** = [ ˙ _x, y,_ ˙ _θ_<sup>˙</sup> ]<sup>_T_</sup> . As shown in Fig. 2, each wheel, with a radius _r_ , has a rotational velocity of _ϕ_<sup>˙</sup> _i_ where _i ∈{_ 1 _,_ 2 _,_ 3 _}_ . 

The relationship between the robot’s velocity and the rotational velocities of its wheels is given by the following equation: 



where _L_ is the distance from the robot’s central point to the center of each wheel. The mounting angles of the three wheels are defined as _α_ 1 _, α_ 2 _,_ and _α_ 3, which are set to 150<sup>_◦_</sup> , _−_ 90<sup>_◦_</sup> and 30<sup>_◦_</sup> respectively in our system. 



Fig. 2: A 3-wheeled omnidirectional robot model. 

## III. RECONFIGURABLE MOBILE MANIPULATOR SYSTEM 

To enhance the versatility of robotic systems in dynamic environments, we propose a reconfigurable mobile manipulator capable of adapting its morphology to task-specific requirements. This modular approach allows individual units to dock physically, creating a unified platform with augmented capabilities. The following subsections detail the system’s core components: **(A)** Mechanical Design, **(B)** Docking Strategy, and **(C)** Post-Reconfigured System Analysis. 

## _A. Mechanical Design_ 

This section presents the mechanical design of the modular docking mechanism, integrated directly into the side of each module’s omnidirectional wheel. To ensure a compact and reliable connection, the design meets three core requirements: (i) independence from wheel rotation, (ii) utilization 

1https://huggingface.co/docs/lerobot/lekiwi 

of existing wheel motor torque for engagement, and (iii) provision of a flat surface for visual perception. 

Based on these criteria, we propose a threaded screw-lock mechanism (specifications in Table I). The design features a large triangular cross-section and beveled leading edges (Fig. 3a) to maximize the capture envelope and facilitate self-alignment. 

The proposed screw-lock mechanism provides a strategic balance between structural rigidity and power efficiency. Unlike phase-change systems such as SolderCubes [10], which require high thermal power ( _≈_ 15W) and long operational cycles ( _≈_ 30s), MobiDock minimizes energy overhead. By leveraging an ”actuator-sharing” strategy with locomotion motors, the system eliminates auxiliary actuators while its self-locking geometry ensures zero power consumption in the holding state. 

As summarized in Table II, our threaded engagement generates substantial axial clamping force (pre-loading), outperforming magnetic systems (e.g., SMORES-EP [11]) that lack shear resistance, or mechanical latches (e.g., FireAnt [14]) prone to backlash. This pre-loading effect ensures a rigid, single-body configuration (Fig. 3b) and a higher alignment tolerance (±5 mm) than most mechanical alternatives. Thus, MobiDock represents a power-autonomous and robust solution for heavy-duty modular manipulation. 

## _B. Docking Strategies_ 

Successful physical connection requires a robust visionbased strategy to detect and engage the docking mechanisms accurately. We utilize AprilTag markers [27] and an armmounted camera to execute a three-phase maneuver (Fig. 4). 



<!-- Start of picture text -->
{Phase 1 ! | Phase2 {| Phase3 H<br>Stat i (eke) |<br>it YE) Constraints H<br>U"emerDowwart) if (“retold |! ay anise) |<br>| if i HH for 3 wheels H<br>Rotational Search|! | Center Camera) if | :<br>( LE | on Apnttag 11 | Engagement | |<br><!-- End of picture text -->

Fig. 4: A schematic overview of the reconfiguration procedure between robotic modules, highlighting the main stages leading to physical connection. 

TABLE I: Technical specifications of the proposed mechanical design 

|**Parameter**|**Value**|
|---|---|
|Thread type|ISO metric thread|
|Designation|M56_×_5_._5|
|Nominal diameter, _d_|56 mm|
|Pitch, _P_<br>|5_._5 mm|
|Major diameter, _dmax_<br>|56_._000 mm|
|Pitch diameter, _d_2|53_._917 mm|
|Minor diameter, _d_1|51_._835 mm|
|Theoretical thread height, _H_|4_._763 mm|
|Effective thread height, _h_|3_._372 mm|
|Flank angle|60<sup>_◦_</sup>|
|Tolerance class|6g/6H|





<!-- Start of picture text -->
‘Omnidirectional a<br>Wheel?<br>Beveled Guild<br>Housing<br>Male Thread<br><!-- End of picture text -->



Fig. 3: The key components and a final configuration of the docking mechanism: (a) exploded view of the proposed screw-based docking mechanical design. The docking hubs are attached to one of the wheels of the two robots. During the docking process, the female hub remains stationary while the male hub rotates to fasten the docking. (b) Overview of the two modules fully docked, forming a single, rigid reconfigured system. 

_1) Phase 1:_ The system first reconfigures the manipulator arm to align with the docking wheel while directing the camera downward. The mobile base performs a rotational scan to detect the target AprilTag within the surrounding environment (Fig. 5). 



<!-- Start of picture text -->
@) 2 |<br>Parallelwsarm<br>Tag, \)<br><!-- End of picture text -->



<!-- Start of picture text -->
.<br>”. Via<br>=?.<br>[Docking part<br><!-- End of picture text -->

Fig. 5: The initial operational phase of a MobiDock module, (a) the robot rotates to find the target tag, with its manipulator parallel to the docking mechanism, (b) the camera is then rotated downward to precisely detect and localize the tag. 

_2) Phase 2:_ Upon detection, the robot centers the camera on the tag and approaches until a predefined depth is reached. Precision alignment is maintained by minimizing two key parameters: the lateral offset _X_ offset and depth offset _Z_ offset, guiding the robot into the final docking pose (Fig. 6). 

TABLE II: Comparison of different docking mechanisms 

|**Docking design**|**Representative**|**Holding force**|**Auxiliary Actuators**|**Holding Power**|**Alignment Tolerance**|**Docking time**|
|---|---|---|---|---|---|---|
|Phase-change|SolderCubes [10]|High|1 (Heater)|No|_±_2 mm|_≈_30s|
|Magnetic|SMORES-EP [11]|Low|0|Yes|_±_10 mm|_≈_1s|
|Mechanical latch|FireAnt [14], M-TRAN [13]|Medium|1 (Motor)|No|_±_2 mm|_≈_5s|
|**Screw-lock**|**MobiDock (Ours)**|**Medium**|**0**|**No**|_±_**5 mm**|_≈_**15s**|















Fig. 6: Illustrations of AprilTag and camera axes. Lateral _X_ offset and depth _Z_ offset offsets guide the robot to center the camera on the tag, ensuring precise alignment during approach. 

_3) Phase 3:_ The final phase executes the locking maneuver by combining forward motion with controlled torsional force to engage the threads. To counteract the omnidirectional base’s tendency to follow an arcing path—which hinders engagement—we implemented a kinematic control law with specific constraints: zero lateral velocity ( ˙ _y_ = 0), constant forward velocity ( ˙ _x_ = A), and a constant rotational velocity for the docking wheel ( _ϕ_<sup>˙</sup> 3) to provide necessary screwing torque. Based on the kinematics in (1), the velocities of the remaining wheels are calculated as: 



where _θ_<sup>˙</sup> = _ϕ_ ˙3 (the constant angular velocity of the docking wheel). The motion plan is visually described in Fig. 7. Based on this control method, we can ensure both the necessary translational force for a secure press and a synchronized rotational torque for thread engagement. This approach facilitates the robust and seamless locking of the threaded mechanism, thereby ensuring a successful docking process. 

## _C. Post-Reconfigured System Analysis_ 

The modularity of our design ensures that fundamental physical principles remain applicable to any _n_ -module docked system ( _n ≥_ 2). Upon docking, the unified structure possesses a total mass _M_ and a global center of mass **r** CM determined by: 



where _mi_ and **r** _i_ are the mass and center of mass vector of the _i_ -th module, respectively. This redistribution ensures the center of mass shifts to maintain balance along the primary axis of the multi-module chain. 









Fig. 7: Visualization of the kinematic law for locking, where zero lateral motion and controlled forward/rotational velocities coordinate wheel motion to achieve proper thread engagement during docking. 

Similarly, the system’s total moment of inertia _I_ total is calculated via the Parallel Axis Theorem: 



where _Ii_ is the individual moment of inertia and _di_ is the distance to the new collective center of mass. The cumulative increase in _I_ total, alongside an expanded stability polygon, significantly enhances resistance to external disturbances and tilting moments during heavy payload manipulation. 

Beyond these physical shifts, reconfiguration achieves structural rigidity by reducing redundant degrees of freedom (DOF). While _n_ independent modules possess 9 _n_ DOF, the docking mechanism imposes kinematic constraints that merge the individual bases into a single rigid platform. Consequently, the system’s total DOF is simplified to: 



Eliminating 3( _n −_ 1) redundant base DOFs through rigid coupling transforms the independent modules into a unified _x, y, θ_ platform with enhanced multi-arm dexterity. Unlike software-centric coordination for closed-chain constraints [25], [26], MobiDock’s hardware-level approach collapses the high-dimensional state space into a single centralized model. This bypasses complex internal force modeling and high-frequency peer-to-peer synchronization, significantly reducing both computational overhead and communication bandwidth requirements. 

To coordinate translational maneuvers for the n-modules system, a common system axis is defined parallel or perpendicular to a radial wheel vector (as illustrated in Fig. 8). Each 

module _i_ synchronizes its heading by applying a rotation matrix _R_ ( _βi_ ) to its local frame, ensuring all modules move in unison as a single rigid platform. The resulting wheel velocity mapping for the _i_ -th module is expressed as: 



where the rotation matrix aligns the local coordinate frame with the unified system axis: 



This alignment ensures that all modules move in unison as a rigid platform. Conversely, for rotational control, the wheels located at the docking interfaces are held stationary to serve as rigid pivots, while the remaining peripheral wheels are driven in a coordinated direction to generate a moment around the system’s collective center of mass, thereby ensuring stability and preventing internal kinematic conflicts within the docked chain. 

## IV. EXPERIMENTS AND EVALUATIONS 

This section evaluates the MobiDock system through four experiments validating our theoretical model. The platform utilizes two LeKiwi modules, each powered by three 12V Feetech ST3215 servos and a Raspberry Pi 5 (8GB) with dual cameras for vision-based alignment. These trials progress from assessing the mechanical reliability of docking (Section IV-A) to analyzing post-reconfiguration stability (Section IV-B, IV-C), finally demonstrating the operational efficiency of the unified bimanual platform in a real-world task (Section IV-D). 

## _A. Autonomous Reconfiguration Validation_ 

This experiment identifies the operational boundaries of our vision-based docking strategy, focusing on AprilTag detection limits under varying environmental factors. Establishing this reliability is a prerequisite for all subsequent stability and task-performance trials. 

We conducted 15 docking trials across four lighting conditions (Fig. 9). The system achieved 93 _._ 3% success rate, with failures occurring only in total darkness where the camera could not resolve the tag. This demonstrates that the MobiDock system is robust to most indoor lighting fluctuations. 

Additionally, we evaluated the detection limit by varying the _angular deviation_ —the angle between the AprilTag’s z- axis and the modules’ line-of-sight (Fig. 10). Trials revealed a reliable detection threshold of _±_ 60<sup>_◦_</sup> , beyond which perspective distortion prevents accurate pose estimation. 

Crucially, while the vision system handles the long-range approach, the screw-lock mechanism provides a mechanical self-alignment effect during final contact. This capability to neutralize minor residual errors ensures a successful 

connection and establishes the zero-backlash foundation required for the high-precision mobility and stability analyzed experiments. 

## _B. Post-Reconfigured Motion Experiment_ 

Once the physical link is established, the evaluation shifts to the control stability of the unified kinematic entity. To rigorously quantify performance, the experimental procedure follows a three-stage tracking protocol: (1) reference mapping, where a static Vive Tracker v3.0 establishes a highresolution L-shaped spatial reference (Fig. 11); (2) real-time feedback control, utilizing an on-robot tracker to minimize instantaneous cross-track and orientation errors [ _x, y, θ_ ] at 10 Hz; and (3) error cross-referencing between the navigated and desired paths. 

Benchmarking the docked system against a single module (Fig. 12a) confirms the consistency of our centralized control law. Despite coordinating six independent wheels, the docked platform demonstrated kinematic behavior equivalent to a single robot, maintaining a steady heading during forward (0 _._ 1 m _/_ s) and lateral (0 _._ 05 m _/_ s) maneuvers. This validates that the unified model (Section III-C) effectively synchronizes distributed actuators without introducing control lag or path deviation. 

Quantitatively, the coordinate standard deviation of the docked system remained comparable to that of a single module (Fig. 12b). This validates that our control law successfully manages the expanded footprint and actuator redundancy without introducing positional instability. These findings prove that the reconfigured system functions as a reliable, integrated unit, providing the predictable motion baseline necessary for further experiments. 

## _C. Dynamic Stability Experiment_ 

Based on the experimental setup illustrated in Fig. 13, we benchmark the unified MobiDock platform against a dual-robot cooperative team during a heavy-payload task using the L-shaped trajectory from Section IV-B. Stability was quantified via a multi-modal approach combining highfrequency inertial data and precise spatial tracking. We evaluated performance through four metrics: (1) **RMSA** (Root Mean Square Acceleration) and (2) **Jerk** (Derivative of acceleration), derived from the payload-mounted MPU6050 gyroscope to quantify overall motion smoothness and highfrequency perturbations; (3) **Angular Standard Deviation** ( _σω_ ), extracted from Vive Tracker signals to indicate pathfollowing accuracy; and (4) **Average Transport Time** for overall operational efficiency. 

TABLE III: Comparative Analysis of Dynamic Stability and Performance (lower is better) 

|**Mode**|**RMSA** (m s<sup>_−_2</sup>)|**Jerk** (m s<sup>_−_3</sup>)|_σω_ (<sup>_◦_</sup>)|**Avg. Time**|(s)|
|---|---|---|---|---|---|
|Cooperating|0.6068|2.038|1.10||84|
|**Docking**|**0.2226**|**1.710**|**0.56**||**76**|



Results averaged over 10 trials (Table III) and visualized in Fig. 14 show the docked configuration reduced RMSA and Jerk by 63 _._ 3 % and 16 _._ 1 %, respectively. This gain is 



<!-- Start of picture text -->
y1<br>(a) (b) Ih x1' 1 β1 x1 (c)<br>O<br>x3 x0 (System’s axis) x4<br>2 y1' 3<br>x2' 3 x3' 4<br>a x2 i x3' S , Sy) x3 2 x4' &<br>β2 β3 3 β3 β4<br>y2 y3 y0 y3 y4<br>x4<br>y2' O y3' y1 y2 x2 y3' O y4'<br>x1' x0 (System’s axis) y4 x2' β1 x2' x0 (System’s axis)<br>1 β1 y1 x4' β4 4 x1' 1 β2 2<br>x1 x2 y2 x1<br>y0 y0<br>y1' y4' y1' y2'<br>β2<br>y2'<br>2<br>x3<br>x4<br>β3 y3<br>x4'<br>β4<br>y4<br>y4'<br>4<br><!-- End of picture text -->

Fig. 8: Examples of different docking configurations with 4 mobile manipulators. The solid axes ( _Oxiyi_ ) denote the original robot control frames and initial robot arm orientation ( _Oyi_ ). After docking, the frames are rotated so the dashed axes ( _Oxi_<sup>_′y_</sup> _i_<sup>_′_)alignwiththesystem’s</sup> <u>axis, enabling coordinated motion around the new center of mass.</u> 







<!-- Start of picture text -->
os tight wee |<br><!-- End of picture text -->



<!-- Start of picture text -->
@ Dark<br><!-- End of picture text -->

Fig. 9: The different lighting conditions of the experiment environment used for the docking trials. 



<!-- Start of picture text -->
a S<br>co<br>FZ deviation<br>Ztag<br>X<br><!-- End of picture text -->

Fig. 10: Angular deviation, the angle between the AprilTag’s z-axis and the module’s connection line, defining the maximum viewing angle for reliable tag detection. 

a direct consequence of collapsing the system’s state space from a high-dimensional 9n DOF problem into a single rigid frame. By physically locking the modules, we eliminate redundant degrees of freedom and internal force conflicts that occur when independent actuators ”fight” for positioning. Furthermore, the rigid coupling bypasses the communication latencies inherent in wireless coordination, which typically cause asynchronous motor responses. In the docked state, 



<!-- Start of picture text -->
[ ><br><!-- End of picture text -->

Fig. 11: Experimental setup for the Post-Reconfigured Motion Experiment, the real-world environment with corresponding dimen- ~~sional pa~~ rameters. 

these impulses are absorbed as structural stress, effectively acting as a mechanical low-pass filter. Consequently, the system achieves a 49 % improvement in angular precision ( _σω_ ), proving that MobiDock’s rigid architecture is superior ~~for payload transport where maximum stability and compu-~~ tational efficiency are paramount. 

## _D. Task Performance Evaluation_ 

Physical reconfiguration provides a critical advantage for dual-arm collaborative tasks by transforming a multi-agent coordination problem into a single-base manipulation task. To evaluate this, we conducted a comparative teleoperation study involving two human operators performing a synchronized ”trash collection” task. As shown in Fig. 15(a-c), one module holds a bin while the other picks and places 10 debris items. This task requires continuous spatial synchronization to maintain the relative pose between the manipulator and the receptacle. 

The experiment was conducted five times per system within a 2 _._ 4 m _×_ 3 _._ 6 m workspace. In the cooperative mode, operators must constantly compensate for the independent drift and relative motion between the two mobile bases, leading to high cognitive load and frequent re-alignment pauses. Conversely, the docked system eliminates relative 



Fig. 12: Comparison result between the Single and Docked Robot, (a) visualization of the robot’s position and orientation tracking, showing the actual path followed (solid lines) relative to the desired trajectory, (b) the mean standard deviation of the robot’s position and angle provides a quantitative measure of its tracking accuracy. 





Fig. 13: Experimental setup for the Stability Experiment, (a) docking robot, (b) cooperation between two independent robots. 

base motion, allowing operators to focus exclusively on endeffector precision. 

As illustrated in Fig. 15d, the docked configuration achieved a significant reduction in execution time. These results confirm that physical docking simplifies the control strategy by removing the need for active base-to-base synchronization. Furthermore, this success highlights a broader potential: the docking paradigm can effectively resolve complex manipulation tasks that are otherwise bottlenecked by the coordination overhead and communication constraints of multi-robot systems, offering a more robust and scalable alternative for high-precision collaborative robotics. 

## V. DISCUSSION & FUTURE WORKS 

The experimental results demonstrate that physical reconfiguration via MobiDock offers significant advantages in stability and control simplicity. However, it is important to 



Fig. 14: Comparative Analysis of Dynamic Stability Docked and Cooperating Robot Systems, (a) the statistical results of RMS Acceleration and Angular Standard Deviation from 10 runs for the two systems under comparison, (b-c) the corresponding RMS Acceleration and Angular Standard Deviation results from a single representative run for the two systems. 

clarify that this docking paradigm is intended to complement, not replace, traditional multi-robot coordination. Cooperative systems remain essential for tasks requiring high flexibility, such as transporting oversized objects or navigating environments where modules must remain spatially separated. Our proposed system introduces a versatile hybrid morphology, allowing a multi-robot team to switch between three operational modes: independent, cooperative, and docked. This multi-modal capability significantly expands the functional scope of mobile manipulators in dynamic environments. 

Despite these strengths, certain limitations persist. The current docking and alignment mechanism is optimized for planar 2D surfaces. In its present form, uneven terrain could hinder the precision of the screw-lock engagement. Additionally, the reliance on teleoperation for task evaluation introduces operator-dependent variability. 

In future work, we will focus on: 

- 3D Docking Mechanisms: Developing advanced alignment strategies to support reconfiguration on non-planar surfaces and multi-level environments. 

- Autonomous Coordination: Implementing Reinforcement Learning (RL) to transition from teleoperation 





Fig. 15: Experimental setup for the Task Performance Evaluation, (a) Docking robot, (b) Cooperation between two independent robots, (c) The real-world environment, (d) Comparative analysis of task completion time for the docked system and two cooperating robots. 

to fully autonomous task execution, ensuring more objective performance benchmarks. 

- System Scaling: Researching control strategies for scaling beyond two modules, focusing on force distribution and communication protocols for large-scale reconfigurable swarms. 

## VI. CONCLUSIONS 

In this research, we introduced MobiDock, a modular reconfigurable mobile manipulator system designed to bridge the gap between multi-robot flexibility and single-platform stability. We presented an integrated framework encompassing an innovative mechanical docking design, a visionbased alignment strategy, and a unified control law. Our experiments validate that the docked configuration achieves kinematic equivalence to a single robot while providing superior dynamic stability (reduced RMSA and Jerk) and operational efficiency compared to independent cooperative teams. 

The findings confirm that establishing a rigid physical link effectively collapses the control complexity and eliminates the synchronization bottlenecks typical of multi-agent systems. By offering a new morphological “mode” for heavyduty and high-precision tasks, MobiDock provides a robust foundation for the next generation of collaborative robots in complex, real-world applications. 

## REFERENCES 

- [1] M. Yim, W. M. Shen, B. Salemi, D. Rus, M. Moll, H. Lipson, E. Klavins, and G. S. Chirikjian, “Modular self-reconfigurable robot systems [grand challenges of robotics],” _IEEE Robotics and Automation Magazine_ , vol. 14, pp. 43–52, Mar. 2007. 

- [2] R. P. Mohamed, F. Xi, and Y. Lin, “A combinatorial search method for the quasi-static payload capacity of serial modular reconfigurable robots,” _Mechanism and Machine Theory_ , vol. 92, pp. 240–256, Oct. 2015. 

- [3] P. Veerajagadheswar, S. Yuyao, P. Kandasamy, M. R. Elara, and A. A. Hayat, “S-sacrr: A staircase and slope accessing reconfigurable cleaning robot and its validation,” _IEEE Robotics and Automation Letters_ , vol. 7, pp. 4558–4565, Apr. 2022. 

- [4] T. P. Cordie, T. Bandyopadhyay, J. Roberts, M. Dunbabin, K. Greenop, R. Dungavell, and R. Steindl, “Modular field robot deployment for inspection of dilapidated buildings,” _Journal of Field Robotics_ , vol. 36, pp. 641–655, Jun. 2019. 

- [5] Y. Li, Z. Xu, X. Yang, Z. Zhao, L. Zhuang, J. Zhao, and H. Liu, “Mrsrs: Modular reconfigurable space robotic system for future space exploration and its standard interface,” _IEEE/ASME Transactions on Mechatronics_ , 2025. 

- [6] G. Liang, D. Wu, Y. Tu, and T. L. Lam, “Decoding modular reconfigurable robots: A survey on mechanisms and design,” _International Journal of Robotics Research_ , vol. 44, pp. 740–767, Oct. 2023. 

- [7] H. Dokuyucu and N. G. Ozmen,<sup>¨</sup> “Achievements and future directions in self-reconfigurable modular robotic systems,” _Journal of Field Robotics_ , vol. 40, pp. 701–746, May 2023. 

- [8] Y. Li, H. Huang, and B. Li, “Design of a deployable continuum robot using elastic kirigami-origami,” _IEEE Robotics and Automation Letters_ , vol. 8, pp. 8382–8389, Dec. 2023. 

- [9] K. L. Walker, A. J. Partridge, H. Y. Chen, R. R. Ramachandran, A. A. Stokes, K. Tadakuma, L. C. D. Silva, and F. Giorgio-Serchi, “A modular, tendon driven variable stiffness manipulator with internal routing for improved stability and increased payload capacity,” _Proceedings - IEEE International Conference on Robotics and Automation_ , pp. 3030–3035, May 2024. 

- [10] J. Neubert, A. R. Wagner, and R. Gross, “Soldercubes: A selfreconfiguring modular robot system,” _Autonomous Robots_ , vol. 40, no. 1, pp. 139–158, Jan. 2016. 

- [11] C. Liu, Q. Lin, H. Kim, and M. Yim, “Smores-ep, a modular robot with parallel self-assembly,” _Autonomous Robots_ , vol. 47, pp. 211– 228, Jan. 2023. 

- [12] J. Zheng, G. Dai, B. He, Z. Mu, Z. Meng, T. Zhang, W. Zhi, and D. Fan, “Rs-modcubes: Self-reconfigurable, scalable, modular cubic robots for underwater operations,” _IEEE Robotics and Automation Letters_ , vol. 10, pp. 3534–3541, Apr. 2025. 

- [13] S. Murata, E. Yoshida, A. Kamimura, H. Kurokawa, K. Tomita, and S. Kokaji, “M-tran: Self-reconfigurable modular robotic system,” _IEEE/ASME Transactions on Mechatronics_ , vol. 7, pp. 431–441, Dec. 2002. 

- [14] P. Swissler and M. Rubenstein, “Fireant3d: A 3d self-climbing robot towards non-latticed robotic self-assembly,” _IEEE International Conference on Intelligent Robots and Systems_ , pp. 3340–3347, Oct. 2020. 

- [15] C. Pu, C. Yang, J. Pu, and R. B. Fisher, “A general mobile manipulator automation framework for flexible tasks in controlled environments,” _Advanced Engineering Informatics_ , vol. 57, p. 102062, Aug. 2023. 

- [16] Q. Yu, G. Wang, X. Hua, S. Zhang, L. Song, J. Zhang, and K. Chen, “Base position optimization for mobile painting robot manipulators with multiple constraints,” _Robotics and Computer-Integrated Manufacturing_ , vol. 54, pp. 56–64, Dec. 2018. 

- [17] Ruchika and N. Kumar, “Control of coordinated multiple mobile manipulators with neural network-based fast terminal sliding mode control,” _International Journal of Dynamics and Control_ , vol. 12, pp. 796–812, Mar. 2024. 

- [18] Y. Zhang, “A coordinated planning and control framework for mobile dual-arm robots with manipulability optimization and force tracking,” _Journal of Intelligent and Robotic Systems: Theory and Applications_ , vol. 111, Jun. 2025. 

- [19] Y. Lian, X. Xiao, J. Zhang, L. Jin, J. Yu, and Z. Sun, “Neural dynamics for cooperative motion control of omnidirectional mobile manipulators in the presence of noises: A distributed approach,” _IEEE/CAA Journal of Automatica Sinica_ , vol. 11, pp. 1605–1620, Jul. 2024. 

- [20] Z. Sun, S. Tang, J. Zhang, and J. Yu, “Nonconvex noise-tolerant neural model for repetitive motion of omnidirectional mobile manipulators,” _IEEE/CAA Journal of Automatica Sinica_ , vol. 10, pp. 1766–1768, Aug. 2023. 

- [21] A. Zhai, H. Zhang, J. Wang, G. Lu, J. Li, and S. Chen, “Adaptive neural synchronized impedance control for cooperative manipulators processing under uncertain environments,” _Robotics and ComputerIntegrated Manufacturing_ , vol. 75, p. 102291, Jun. 2022. 

- [22] J. Chen and S. Kai, “Cooperative transportation control of multiple mobile manipulators through distributed optimization,” _Science China Information Sciences_ , vol. 61, pp. 1–17, Dec. 2018. 

- [23] Y. Ren, S. Sosnowski, and S. Hirche, “Fully distributed cooperation for networked uncertain mobile manipulators,” _IEEE Transactions on Robotics_ , vol. 36, pp. 984–1003, Aug. 2020. 

- [24] C. Wu, H. Fang, Q. Yang, X. Zeng, Y. Wei, and J. Chen, “Distributed cooperative control of redundant mobile manipulators with safety constraints,” _IEEE Transactions on Cybernetics_ , vol. 53, no. 2, p. 1195–1207, Feb. 2023. 

- [25] A. Marino, “Distributed adaptive control of networked cooperative mobile manipulators,” _IEEE Transactions on Control Systems Technology_ , vol. 26, pp. 1646–1660, Sep. 2018. 

- [26] P. Xu, Y. Cui, Y. Shen, W. Zhu, Y. Zhang, B. Wang, and Q. Tang, “Reinforcement learning compensated coordination control of multiple mobile manipulators for tight cooperation,” _Engineering Applications of Artificial Intelligence_ , vol. 123, Aug. 2023. 

- [27] E. Olson, “Apriltag: A robust and flexible visual fiducial system,” in _2011 IEEE international conference on robotics and automation_ . IEEE, 2011, pp. 3400–3407. 

