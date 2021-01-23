# Sousa et al.

## Execute the Method

1. Open `main.m` MATLAB program
2. Copy the data files to the [data/](https://github.com/sousarbarb/odometry-calibration/tree/main/data) folder
3. Change the following variables:
   - visualize: false (only shows the maximum errors) / true
   - Dataset.filenames: `[directory/fileID]` (any number of datasets)
   - Dataset.metadata: `[directory/fileID_metadata.csv]`
   - Method.options: change the parameters of the optimisation algorithm (Rprop-based) in the `setRpropParameters` function
4. Execute the program
5. Change the kinematic parameters to the ones computed by the calibration method

## Test Different Kinematic Parameters

1. Open `main_testParameters.m` MATLAB program
2. Copy the data files to the [data/](https://github.com/sousarbarb/odometry-calibration/tree/main/data) folder
3. Change the following variables:
   - visualize: false (only shows the maximum errors) / true
   - Dataset.filenames: `[directory/fileID]` (any number of datasets)
   - Dataset.metadata: `[directory/fileID_metadata.csv]`
4. If you desired, change the robot's kinematic parameters:
   - Initial estimation: RobotParam (Section `INITIALIZATION`)
   - Calibrated parameters: RobotEstParam (Section `SET THE TEST PARAMETERS`)
5. Execute the program
6. Execute the `main_visualizationAux.m` MATLAB program to compare on the same plot different parameters
