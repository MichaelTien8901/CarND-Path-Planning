# CarND-Path-Planning-Project
## Behavior Planning
### Finite State Machines
   Use finite state machines to determine behavior of car in highway.  The type STATE_PATH_PLANNING defines all states.
```cpp
   typedef enum {
  STATE_START = 0, 
  STATE_KEEP_LANE = 1, 
  STATE_PREPARE_LANE_CHANGE_LEFT = 2, 
  STATE_LANE_CHANGE_LEFT = 3,
  STATE_PREPARE_LANE_CHANGE_RIGHT = 4, 
  STATE_LANE_CHANGE_RIGHT= 5
} STATE_PATH_PLANNING;
```
### Cost Functions
*Speed cost* encourage speed near the speed limit but not exceed the speed limit.  
*Distance Cost* encourage long distance for the front car in the same lane.  
*Change Lane Cost* prevent too much change lane activity unless necessary.
*Safety Cost* prevent unsafe driving behavior, like collision, or out of lane.

### State Transition Table
The next_state_table define the transitions and cost functions of each state.  
```cpp
std::map<STATE_PATH_PLANNING, vector<NextStateFunc>> next_state_table = {
  { STATE_START,
    { { STATE_KEEP_LANE, &PathPlanner::cost_default , 1},
    }
  },
  { STATE_KEEP_LANE,
     { { STATE_KEEP_LANE, &PathPlanner::cost_kl, 1.},
     { STATE_PREPARE_LANE_CHANGE_LEFT, &PathPlanner::cost_kl_plcl, 1.0 },
     { STATE_PREPARE_LANE_CHANGE_RIGHT, &PathPlanner::cost_kl_plcr, 1.0 },
     }
  },
  { STATE_PREPARE_LANE_CHANGE_LEFT,
     { { STATE_PREPARE_LANE_CHANGE_LEFT, &PathPlanner::cost_plcl, 1. },
     { STATE_LANE_CHANGE_LEFT, &PathPlanner::cost_plcl_lcl, 1. },
     { STATE_KEEP_LANE, &PathPlanner::cost_plcl_kl, 1.05 },
     } 
  },
  { STATE_LANE_CHANGE_LEFT,
     { { STATE_KEEP_LANE, &PathPlanner::cost_kl, 1. },
       // { STATE_LANE_CHANGE_LEFT, &PathPlanner::cost_lcl} // in transition
     } 
  },
  { STATE_PREPARE_LANE_CHANGE_RIGHT,
     { {STATE_PREPARE_LANE_CHANGE_RIGHT, &PathPlanner::cost_plcr, 1. },
       {STATE_LANE_CHANGE_RIGHT, &PathPlanner::cost_plcr_lcr, 1. },
     { STATE_KEEP_LANE, &PathPlanner::cost_plcr_kl, 1.0 },
     } 
  },
  { STATE_LANE_CHANGE_RIGHT,
    { { STATE_KEEP_LANE, &PathPlanner::cost_kl, 1 },
       // { STATE_LANE_CHANGE_RIGHT, &PathPlanner::cost_lcr} // in transition
    }
  }
};  
```

## Trajectory Generation
The method used for smooth trajectory is spline method. The spline must be applied in the global coordinate because of the "Frenet Problem" mentioned below.  
In order to minimize jerk, several method are used.  
* State change only happened after a period of time of previous changes.  The prevent too much changes in trajectory generation.
* State change only happened after current state is stable.  For example, the ego car reach the target lane.

## Problems

*Frenet Coordinate*
The conversion between Frenet Coordinate and global coordinate is not very accurate. If the smooth trajectory is generated in Frenet coordinate, the smoothness after conversion to global coordinate is not guaranteed.  

*Frenet Coordinate s problem*
Since the simulator use a circular model, the s value of Frenet coordinate could reset to zero during driving.  This will cause problem to decide the position of cars.  A special function s_distance is implemented to decide the relative s distance between cars.

*Speed Exceed Limit*
I set the target speed very near to the speed limit.  But during a curve turn, the trajectory will exceed the speed limit.  The problem is the generation of x based on a straight line. To overcome this problem, a segment speed between two trajectory points is calculated to lower speed if exceed speed limit.

*Too many change Lanes*

At first, the ego car keep changing lanes, even there is no cars in front of lanes.  This is because the cost for many lanes are the same and only a little randon value changes cause car change lanes.  The way to stop this behavior is to add a little cost to change lane activity.

*Spline Error*

When car target speed is zero, the estimate first and second points used to generate spline are the same.  This will cause spline function error.  The way to stop this error is add a very small value to speed to prevent zero target speed. 

*Safety Check In Trajectory*

Even I implement collision check in trajectory generation, the delay of lane changes not only cause "Out Of Lane" error, and sometimes cause more harm because of delay of action.

## Ways to Improve

* Smart Behavior
Currently my implementation is very conservative driving behavior. Car change lanes only in safe position and speed.  But no adjust position or speed for next behavior changes.  In order to achieve higher performance, maybe more states are need to adjust location and speed to change lanes, or even consider all drivable lanes in the highway.  

* JMT Trajectory Generation
JMT supposed to minimize jerk in trajectory generation.  It also can be used to specify the target velocity and acceleration in the trajectory.  

* Safety Behavior Planning
In case of collision in trajectory, more planning are need to minimize problem. 
