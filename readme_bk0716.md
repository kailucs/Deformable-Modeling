# Deformable Modling

Note: The "home position" used in this project has the EE frame: 

local x - global -z  local y - global -y  localz - global -x

## Step 1: Calibrate camera

(Needs to be done once in a while.)
Place the box with colored dot on the table.

- run take_calibration_pictures.py... make sure that the box is always in the camera's field of view. This file saves the pictures and pcd.

- run generate_marketr_transform_color.py. This file extracts the centroid 3D positions in the camera frame.

- run calibration_calculation_color.py. This file calculates the hand-eye calibration matrix and saves it in calibrate_camera_xform.txt. 
  Delete the "tab" in the file and copy it to /calibrated_transform ?



## Step 2: Reconstruct the object model

- create a new experiment_data folder
- run data_collection_get_PCD.py to take 5 pcds.
- run data_collection_process_pcd.py to process each of the 5 pcds (e.g. remove the table, remove artifacts, etc.)
- Go to /cpp_ws to run MyProgram to merge the 5 PCDs
- run convert_ply.py to convert the file. Remember to change the NofVertices in the file. Finally, we have TSDF_converted.ply as the raw pcd.
- run data_collection_process_pcd_with_curvatures to downsample this pcd and calculate normal + curvature(this needs to be debugged still).the saved probePCD.txt are the downsampled pcd that will be poked. The originalPcd.txt is the raw pcd augmented with color and normal and curvature(not implemented yet..

## Step3: Probe to get Data 

- run data_collection_probing.py. First run debug and check everything is fine. Then run on physical robot.



Change the folder name experiment_data to something else...