# SDCND_Path_planning_project


<p align="center">
<img align="center" width="500"  src="https://user-images.githubusercontent.com/41348711/83053927-d71cb780-a051-11ea-8c60-b58350fbf443.JPG">

### Code sequence

#### 1. Load map 

**File: Map.cpp**; 
**Function: load_map()**;

**Highway_map.csv** file provides the x and y coordinates for each waypoint. In addition to these coordiantes, the file gives the coordinates of the unit vector (dx,dy) that defines the *d* direcction in Frenet coordinates for each Waypoint (orange arrow in the next picture - counterclokwise positive); 

This unit vector cannot be used to generate a continous channel , but calculating theta as *tetha=atan(dx/dy)*  we can generate a continous channel . Just have to consider that the code needs to be modified to deal with the offset produced at PI ahgle(180 degrees-see plts below).

Picture below shows the Waypoints coordinates in blue color, the direction of travel marked as a white arrow and two samples of this theta angle (marked in orange colour)

<p align="center">
<img width="450"  src="https://user-images.githubusercontent.com/41348711/83170868-81114800-a115-11ea-986b-57d48134f2ce.png">

The aim of these process is get x,y and tetha as a function of s frenet corrdinate:
* *x=x(s)*;
* *y=y(s)*;
* *tetha=tetha(s)*;

The splines are generated using *spline.h* function

<p align="center">
<img width="800" src="https://user-images.githubusercontent.com/41348711/83174951-75288480-a11b-11ea-9cba-e839f5e0d070.png">
 
  
<p align="center"> 
<img width="800" alt="tetha   tetha corrected" src="https://user-images.githubusercontent.com/41348711/83437765-9c939000-a440-11ea-8f3d-87e18f6bafc0.png">


#### 2.	Finite State Machine (FSM)

**File: fsm.h**; 

FSM is programmed using the code from Michael Egli (MIT). The schema below summarizes the states and the possible transitions.

<p align="center"> 
<img width="400" src="https://user-images.githubusercontent.com/41348711/83362376-1704d700-a391-11ea-8468-2095ecb82e89.JPG">

The table below summarizes the triggers that promote the state change.


| From State       		|To State	     					|Trigger      |Guard(condition)                                | 
|:------------------:|:-----------------:|:-----------:| :---------------------------------------------:| 
| Keep Lane         	|Keep Lane   							|Clear        | TRUE                                           | 
| Keep Lane         	|Lane change Left 		|Vehicle ahead| is posible to change to left lane |                                 
| Keep Lane         	|Lane change Right  |Vehicle ahead| is posible to change to right lane  |                                         
| Keep Lane         	|Follow Vehicle   		|Vehicle ahead| no possibility to change lane      | 
| Lane change Left  	|Keep Lane          |Clear        | TRUE  |                                         
| Lane change Left 	 |Follow Vehicle   		|Vehicle ahead| TRUE      |
| Lane change Right  |Keep Lane          |Clear        | TRUE  |                                         
| Lane change Right 	|Follow Vehicle   		|Vehicle ahead| TRUE      |
| Follow Vehicle 	   |Follow Vehicle   		|Vehicle ahead| no possibility to change lane      |
| Follow Vehicle  	  |Keep Lane          |Clear        | TRUE  |                                         
| Follow Vehicle 	   |Lane change Left 		|Vehicle ahead| is posible to change to left lane      |
| Follow Vehicle 	   |Lane change Right 	|Vehicle ahead| is posible to change to right lane      |


#### 3.	Initial and final states 

**File: main.cpp**; 
Prior to the trajectory generation, we have to define the initial and the final states of the next trajectory.
For the first iteration we use static vehicle conditions as initial state, but once ego vehicle starts moving , the final state of the previous path is used as an initial state for the next state. My code generates 50 points paths, and uses remaining points from previous path.

* Final d state {d,d´,d´´}:

Once the data is received from telemetry,the environment of the ego vehicle is checked searching any vehicle in front of us. In case a vehicle is detected at 50 meters or closer (tunable parameter through variables.h file), we start checking left and right lanes in anticipation to a lane change.

Then, FSM is triggered using the vehicle_ahead variable, taking into account the conditions mentioned in table above. As an output 
we receive the intended lane in the next update loop (d).

To simplify the code, final d velocity (d´) and d acceleration (d´´) are set to zero.

<p align="center"> 
<img width="500" src="https://user-images.githubusercontent.com/41348711/83552111-8ac9ef80-a509-11ea-96e4-a0484215cdb7.png">


To calculate the final s state a switch case piece of code is used. Depending on the 
  
* Final s state {s,s´,s´´}:
 
Final s state is calculated as a function of the final state in the previous path, considering speed and acceleration limits.

#### 3.	Trajectory generation

**File: trajectoy_planner.cpp**; 


Once initial and final states are stated in frenet coordinates, the code under *trajectory_planner.cpp*:

1) Calculates Jerk minimizing s and d trajectories
2) Uses the funcions x(s), y(s),tetha(s) (mentioned in point #1) to transform Frenet coordinates to 0.02 equispaced cartesian coordinates

Finally cartesian trajectory points are loaded into the ego vehicle

#### 	Rubric points:


* [X] The car is able to drive at least 4.32 miles without incident:

    The car is able to driver aroung 10 miles, although there is still singular situations that cause collision with other vehicles.      This shall be a future improvement 
    
* [X] The car drives according to the speed limit:

    Setting a maximum speed of 85% of the maximum alowed speed, ego vehicle keeps inside the speed limit
    
* [X] Max Acceleration and Jerk are not Exceeded :

    The code inside the swicth case function guarantees neither jerk, nor acceleration are exceed.
    
* [X] Car does not have collisions:

    The car is able to detect vehicles in front and sideways. Actually the vehicle is not able to avoid a collision if any surrounding    vehicle intersecs an actual driving path. This is also an improvement to be performed.
    
* [X] The car stays in its lane, except for the time between changing lanes:

   The car stays in lanes except for lane changing
   
* [X] The car is able to change lanes:

   FSM is working propertly
