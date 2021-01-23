# Borenstein and Feng - The University of Michigan Benchmark (UMBmark)

1. Open `main.m` MATLAB program
2. Copy the data files (retrieved when the robot had gone through a squared path) to the [data/](https://github.com/sousarbarb/odometry-calibration/data) folder
   - N/2 first runs must be relative to the CW (clockwise) direction
   - The other N/2 runs must be relative to the CCW (counterclockwise) direction
3. Change the following variables:
   - visualize: false (only shows the maximum errors) / true
   - Dataset.filenames: `[directory/fileID]` (only 1 dataset!)
   - Dataset.metadata: `[directory/fileID_metadata.csv]`
4. Execute the program
5. Change the kinematic parameters to the ones computed by the calibration method
