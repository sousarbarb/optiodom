# Synchonization

Only use the scripts implemented in this folder if you do not have avaliable synchonized odometry and ground-truth (only works with OptiTrack data) data.

1. Open `main_*.m` MATLAB program depending on which steering geometry used in the experiments
2. Copy the data files to the [data/](https://github.com/sousarbarb/odometry-calibration/data) folder
3. Change the following variables:
   - filenameCenter: OptiTrack data acquisition with all the markers of the robot (including the ones on the wheels)
   - filenameRobot: OptiTrack data acquisition with the markers on top of the robot
   - dataUniqueID
   - RobotParam: set the initial estimation for the robot's kinematic parameters
   - N: total of runs acquired from the experiments
4. Execute the program

**Note:** the metadata file is generaeted automatically.
