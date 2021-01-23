# Sousa et al.

1. Open `main.m` MATLAB program
2. Copy the data files to the [data/](https://github.com/sousarbarb/odometry-calibration/tree/main/data) folder
3. Change the following variables:
   - visualize: false (only shows the maximum errors) / true
   - Dataset.filenames: `[directory/fileID]` (any number of datasets)
   - Dataset.metadata: `[directory/fileID_metadata.csv]`
   - Method.options: change the parameters of the optimisation algorithm (Rprop-based)
4. Execute the program
5. Change the kinematic parameters to the ones computed by the calibration method