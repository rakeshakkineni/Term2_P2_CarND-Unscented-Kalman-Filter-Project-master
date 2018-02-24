## **Unscented Kalman Filter Project**

The goals / steps of this project are the following:
* Modify the code to estimate the vehicle position and velocity using LIDAR and RADAR data using Unscented Kalman Filter


[//]: # (Image References)

[image1]: ./Output/Dataset1.png 
[image2]: ./Output/Dataset2.png 
[image3]: ./Output/RADAR_NIS_Data _Set_1.png
[image4]: ./Output/LIDAR_NIS_Data_Set_1.png
[image5]: ./Output/RADAR_NIS_Data_Set_2.png
[image6]: ./Output/LIDAR_NIS_Data_Set_2.png

---
### Writeup / README
In this project simulated RADAR and LIDAR data is used to estimate Vehicle position and Velocity using Unscented Kalman Filter. Source code provided by "UDACITY CarND Unscented Kalman Filter" was used as base for this project. 

### Modifications
ukf.cpp , ukf.h , tools.cpp are modified to implement unscented kalaman filter and fusion logic. ProcessMeasurement,Prediction , UpdateLidar , UpdateRadar functions were modified to proces RADAR/ LIDAR data and to implement Unscented Kalman Filter. CalculateRMSE function was also modified to calculate RMS value. Modified code can be found [here] ("./Source")

### Code Flow
Following is brief description of the flow of the code.
- Initialize the Unscented Kalaman Filter. 
- Establish the connection with simulator.
- Get the data from the simulator for one scan.
- Perform Perdict the future position and velocity of the vehicle.
- Check if the data is from LIDAR or RADAR, accordingly call UpdateLidar or UpdateRadar functions.
- NIS value is calculated for LIDAR and RADAR saperately and written to a file name "nis_output.txt"
- Calculate the RMSE for each of the parameters and send it to Simulator and print it to a file.

### Output
Unscented Kalman Filter was able to estimate the position of the vehicle. 

-Data Set 1 In the following picture the estimated path followed by the vehicle is in green. 

![alt text][image1]

-Data Set 2 In the following picture the estimated path followed by the vehicle is in green. 

![alt text][image2]

- NIS for each scan is printed in a text file. Using the printed data graphs were ploted to understand the trend of the RMSE for each parameter. 

RADAR Data Set 1

![alt text][image3]

LIDAR Data Set 1

![alt text][image4]

RADAR Data Set 2

![alt text][image5]

LIDAR Data Set 2

![alt text][image5]

## Result:
For both DataSet RMSE values were less than 0.09,0.10,0.40,0.30 as required in rubics. 
