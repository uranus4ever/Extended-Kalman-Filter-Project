# **Object Tracking with Sensor Fusion-based Extended Kalman Filter**

### Objective
Utilize sensor data from both LIDAR and RADAR measurements for object (e.g. pedestrian, vehicles, bicycle, or other moving objects) tracking with the Extended Kalman Filter.


## Instruction
### 1. Dependencies & Environment

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Eigen library

### 2. My project files

* `CMakeLists.txt` is the cmake file.
* `data` folder contains test lidar and radar measurements.
* `Docs` folder contains docments which describe the data.
* `src` folder contains the source code.
 * `main.cpp` - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE.
 * `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function.
 * `kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar.
 * `tools.cpp` - a function to calculate RMSE and the Jacobian matrix.

### 3. How to run the code

1. Clone this repo.
2. Download the Udacity Term-2 [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.0).
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
5. Run the command: `./ExtendedKF`.
6. Run the simulator.


## Result

Threshold: RMSE <= [0.11, 0.11, 0.52, 0.52] 

**Dataset 1**
Accuracy - RMSE: [0.0973, 0.0855, 0.4513, 0.4399]

![dataset1][img1]


**Dataset 2**
Accuracy - RMSE: [0.0726, 0.0967, 0.4579, 0.4966]

![dataset2][img2]


_Note:_
Red Circle - LIDAR measurements
Blue Circle - RADAR measurements
Green triangle - Estimated location

## Knowledge Background

### 1. How does LIDAR measurement look like

![Lidar][img5]

The LIDAR will produce 3D measurement px,py,pz. But for the case of driving on the road, we could simplify the pose of the tracked object as: px,py and one rotation. In other words, we could only use px and px to indicate the position of the object, and one rotation to  indicate the orientation of the object. But in real world where you have very steep road, you have to consider z axis as well. Also in application like airplane and drone, you definitely want to consider pz as well.



### 2. How does RADAR measurement look like

![Radar][img6]

### 3. Comparison of LIDAR, RADAR and Camera

|            Sensor type           |  LIDAR |    RADAR  |   Camera   |
|:--------------------------------:|:------:|:---------:|:----------:|
|            Resolution            | Median |  Low      |  **High**  |
|      Direct velocity measure     |   No   |  **Yes**  |     No     |
|            All-weather           |   Bad  |  **Good** |     Bad    |
|            Sensor size           |  Large | **Small** |  **Small** |
| sense non-line of  sight object  |   No   |  **Yes**  |     No     |


**_Note_**:

* LIDAR wavelength in infrared; RADAR wave length in mm. 
* LIDAR most affected by dirt and small debris.


### 5. How does the Extended Kalman Filter Work

![EKF flow][img3]

### 6. Extended Kalman Filter V.S. Kalman Filter

![EKF vs KF][img4]

**All Kalman filters have the same three steps:**

1. Initialization
2. Prediction
3. Update

## Reflection

1. Playing with three scenarios: tracking with both Lidar and Radar, only Lidar and only Radar. Radar measurements are much more noisy than Lidar measurements. EKF with both Lidar and Radar performs robust as it reduces error / noise.

2. Lesson Learnt for accuracy improvement and debug:
    a. Avoid divide by Zero! Insert a zero judgement before division.
    b. KF expects small angle values between -pi and pi. Add 2pi or subtract 2pi until the angle is within the desired range.
    c. Communication between Windows Host and Linux Virtual Machine (VMware). Follow this [guide](https://jingyan.baidu.com/article/c35dbcb0d1ff248916fcbc0d.html) to set the port mapping.


[//]: # (Image References)
[img1]: ./extra/dataset1.PNG
[img2]: ./extra/dataset2.PNG
[img3]: ./extra/ekf_flow.jpg
[img4]: ./extra/ekf_vs_kf.jpg
[img5]: ./extra/lidar.jpg
[img6]: ./extra/radar.jpg