Code for gait transitions, made to run in Ubuntu WSL and uses C++ and Python3

### Dependencies
- Eigen: `sudo apt install libeigen3-dev`
- Boost: `sudo apt install libboost-all-dev`
- [Bezier](https://github.com/minhn02/Bezier): clone repo, use CMake, and `sudo make install`
- Pybind11: `sudo apt install python3-pybind11`
- sshkeyboard `sudo pip3 install sshkeyboard`

I've included the apt commands, but they can also be installed from source

### Code

**src/control** holds C++ code for calculating optimal transition trajectories
- *transition_generator.cpp* is where you can edit the weights, I've added variables to edit at the top of the file

**src/python/asterix_state_machine.py** is the main file to run experiments
- on line 20, there's a variable that determines the telemetry file name, this should be changed every run

After changing code in C++ files, run `sudo python3 setup.py build` in top directory and `sudo python3 setup.py install` to remake the python module that is imported the python scripts as `stm_state_machine`.

### Usage

Keybinds:

| Key         | Functionality|
|--------------|-------------|
| q | transition to idle |
| w | transition to squirming|
| e | transition to wheel walking|
| a | naive transition strategy |
| s | bezier switching strategy |
| d | linear waypoint transition strategy |
| f | spline transition strategy |
| z | start transition test sequence |

To run experiments with the Bezier strategy, you would start the script with

`sudo python3 asterix_state_machine.py` (sudo necessary to write telemetry file)

on the onboard Pi. Then press *s* switch the robot into bezier transition mode and *z* to start the test sequence.

If running experiments, make sure to sync the timestamp of the onboard pi with **util/sync_epoch.sh** and simultaneously begin recording Vicon motion capture data.

The telemetry will be saved to **/home/pi/minh/logs/{DATE}/filename** which will need to be transferred off the Pi. Be careful to rename the telemetry file at the top of the main script every time or else it will be overwritten.

When the transitions are done, the terminal should continually output `TRANSITIONS DONE`.

You can `Ctrl + C` any time to stop the script and telemetry will be preserved. Sometimes the wheels don't stop correctly and you need to type `stop`.