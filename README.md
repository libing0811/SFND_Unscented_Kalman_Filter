# SFND_Unscented_Kalman_Filter
Sensor Fusion UKF Highway Project 

## The understanding of whole project

###render.h & render.cpp
render files are used for visualizing the result with pcl_visualizer library. 
Including:
- Highway Scene
- Radar Ray & object position
- Lidar Cloud Points & object position
- Car structure definition.
- Cars object and position.
- Cars prediction furture position of UKF
- The ground_truth movement process of the cars.
- Car motion structure: accuation (motion_time, motion_acceleration, motion_steering_angle)

###lidar.h
lidar.h is used for generating lidar cloud points.
But we don't use lidar.h to generate lidar pcd file in runtime. 
The pcd files has been generated and store in data/pcd directory. 
We load the files in data/pcd directory and use them for simulation.  -- see detail in highway.h::void stepHighway() function.

###highway.h
highway.h is very important file.

In construct function Highway(pcl::visualization::PCLVisualizer::Ptr& viewer), we define the following things:
- ego car and three cars initial state (position,size,velocity, angle, )
- ego car and three cars further motion, including (motion_time,motion_acceleration,motion_steering_angle). Note: the motions is ground_truth value for simulation.

In stepHighway function:
- first, we call render functions defined in render.cpp to render the highway environment.
- then, for each car, we call their move function, to simulate their movement according to the timestamp.
        traffic[i].move((double)1/frame_per_sec, timestamp);
- after that, we get the ground_truth value of this timestamp
- then we call tool.h & tool.cpp function to call the UKF predict & measurement process, to get the estimation of this timestamp
- we compare all four cars' ground_truth value and ukf_estimation value, too compute a whole RSME evaluation result of the UKF method, then visualize it.

###tools.h&tools.cpp
tools files main contain the following functions:
- lidarSense function: handle (preprocess) lidar measurement input and add noise into it, then call UKF.ProcessMeasurement function
- radarSense function: handle (preprocess) radar measurement input and add noise into it, then call UKF.ProcessMeasurement function
- ukfResults function: 
    Call UKF.predict function to get further a few step of the car's state.
    Show UKF tracking and also predict future path
        Parameter: double time:: time ahead in the future to predict
        Parameter: int steps:: how many steps to show between present and time and future time
- CalculateRMSE function: calcuate RMSE of all the four cars.
- savePcd & loadPcd function: the function to save and load pcd files.

###ukf.h&ukf.cpp
The main implmentation of UKF method.
It should implement the CTRV model and UKF method (including a lot of parameter, for example: process noise vector and measurement noise vector, we need to define them).
It contains following method:
        void Prediction(double delta_t);
        void ProcessMeasurement(MeasurementPackage meas_package);
        void UpdateLidar(MeasurementPackage meas_package);
        void UpdateRadar(MeasurementPackage meas_package);
