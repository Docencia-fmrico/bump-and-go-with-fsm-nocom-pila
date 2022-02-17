[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6870053&assignment_repo_type=AssignmentRepo)
# fsm_bump_go

## Nocom-pila's bump and go versions

We've made three files with different code. One corresponds to each version of the practice
1. Bump and go basic version (with only one spin)
2. Bump and go advanced (with two spin possibilities)
3. Bump and go laser (pro version using the base and the laser)
  
  
### 1. Bump and go basic
This version has only the code to make the kobuki spin to the left following a very simple states-machine
The possible states are:
- GOING_FORWARD
- GOING_BACK
- TURNING_LEFT  
The program makes the kobuki start with the "GOING_FORWARD" state, and it changes it when the bumper of the base is pressed by any direction possible. The next state is always "GOING_BACK" and after a few seconds, with no other external stimoulation needed, it changes to "TURNING_LEFT". Then, the program waits another few seconds and switches the state to "GOING_FORWARD" again.
  
### 2. Bump and go advanced 
This version has the code done with inheritance, and uses certain methods from the basic version, some of them modified or with additions. The states-machine is a little bit more complex and has a new state wich permits the kobuki decide the side it's going to spin to.
The possible states are:
- GOING_FORWARD
- GOING_BACK
- TURNING_LEFT
- TURNING_RIGHT  
The "advanced" program also makes the kobuki start with the "GOING_FORWARD" state, but now when the bumper is pressed it can follow two different routes:
If the left bumper is pressed, it changes now to "GOING_BACK" state, then a few seconds later to the "TURNING_RIGHT" and finally, some seconds later, it returns to the "GOING_FORWARD" state.
If the right or front bumper is pressed, the state changes to "GOING_BACK" just as the other case, but after that now it goes through the "TURNING_LEFT" state and a few seconds later, finally returns to the initial state.
  
### 3. Bump and go laser
This version has the code done with inheritance too, using certain methods from the basic version modified or with added lines. This version detects events from the bumper and the laser and can spin to the left and to the right to avoid the obstacles found. It also has a secutity addition that detects other possible objects found on it's way when the kobuki is going back.
The possible states are
- GOING_FORWARD
- GOING_BACK
- TURNING_LEFT
- TURNING_RIGHT
This "pro" version of the program starts, as the others, in the "GOING_FORWARD" state. When any obstacle is detected by the laser closer than 0,3m, the kobuki switches it's state to "GOING_BACK" and a few seconds later to "TURNING_LEFT" or "TURNING_RIGHT" just as the previous version does. Furthermore, when the kobuki is going back, it can switch before the common time has finished if it detects another obstacle on the way it's moving backward.
If any bumper of the base is pressed by an object that hadn't been detected by the laser, it also changes the kobuki's current state, identically as the previous way.  
  
All the versions described above include extra-code that turns on a led while the kobuki is turning to any side. If it's turning to the left it's turned on the led 1, and if it's to the right is the led 2

