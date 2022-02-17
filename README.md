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
The "aadvanced" program also makes the kobuki start with the "GOING_FORWARD" state, but now when the bumper is pressed it can follow two different routes:
If the left bumper is pressed, it changes now to "GOING_BACK" state, then a few seconds later to the "TURNING_RIGHT" and finally, some seconds later, it returns to the "GOING_FORWARD" state.
If the right or front bumper is pressed, the state changes to "GOING_BACK" just as the other case, but after that now it goes through the "TURNING_LEFT" state and a few seconds later, finally returns to the initial state.

