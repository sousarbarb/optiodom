# Ivanjko et al.

1. Open `main.m` MATLAB program
2. Copy the data files (retrieved when the robot had gone through the path specified by Ivanjko et al.) to the [data/](https://github.com/sousarbarb/odometry-calibration/tree/main/data) folder
   - N/3 first runs must be relative to straight-line motion
   - The next N/3 runs must be relative to the "on-the-spot" 180º rotation in the CW (clockwise) direction
   - The other N/3 runs must be relative to the "on-the-spot" 180º rotation in the CCW (counterclockwise) direction
3. Change the following variables:
   - visualize: false (only shows the maximum errors) / true
   - Dataset.filenames: `[directory/fileID]` (only 1 dataset!)
   - Dataset.metadata: `[directory/fileID_metadata.csv]`
4. Execute the program
5. Change the kinematic parameters to the ones computed by the calibration method
