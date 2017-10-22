# Extended Kalman Filter Project

[Uncompleted... To be continued]

[//]: # (Image References)

[image1]: ./ekf_example.png "example"

In this project I will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

![alt text][image1]

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---
