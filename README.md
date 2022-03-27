In this project we used of Nuscenes database to implement an occupancy grid usig Lidar points cloud.
the data used in this project is the mini version of the full dataset (which contains 10 scenes) and can be downloaded from https://www.nuscenes.org/nuscenes#download
the main code is called occupancy_grid_lidar.ipynb and located in the tutorial folder "nuscenes-devkit-lidar-occupancy-grid/python-sdk/tutorials/", and there is another python class for lidar utils can be found in the same location.
there are some auxillary functions in the main code notebook help to visualize the data which  you are working on.
It is important to set the dataroot of the Nuscenes database correctly in line 4 of file lidar_utils.py.

![t](https://user-images.githubusercontent.com/56690379/160300769-2abe9afd-2825-4018-917a-e9ca7221df91.png)
here is an example image show the occupancy grid of scene1

![occupancy](https://user-images.githubusercontent.com/56690379/160300891-6601bf59-2c05-455a-85ad-ad795d212084.jpg)
