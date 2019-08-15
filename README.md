# SFND_Unscented_Kalman_Filter
Sensor Fusion UKF Highway Project 

## The understanding of whole project

### render.h & render.cpp
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

### lidar.h
lidar.h is used for generating lidar cloud points.
But we don't use lidar.h to generate lidar pcd file in runtime. 
The pcd files has been generated and store in data/pcd directory. 
We load the files in data/pcd directory and use them for simulation.  -- see detail in highway.h::void stepHighway() function.

### highway.h
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
- we compare all traffic cars' ground_truth value and ukf_estimation value, too compute a whole RSME evaluation result of the UKF method, then visualize it.

### tools.h&tools.cpp
tools files main contain the following functions:
- lidarSense function: handle (preprocess) lidar measurement input and add noise into it, then call UKF.ProcessMeasurement function
- radarSense function: handle (preprocess) radar measurement input and add noise into it, then call UKF.ProcessMeasurement function
- ukfResults function: 
    Call UKF.predict function to get further a few step of the car's state.
    Show UKF tracking and also predict future path
        Parameter: double time:: time ahead in the future to predict
        Parameter: int steps:: how many steps to show between present and time and future time
- CalculateRMSE function: calcuate RMSE of all the traffic cars.
- savePcd & loadPcd function: the function to save and load pcd files.

### ukf.h&ukf.cpp
The main implmentation of UKF method.
It should implement the CTRV model and UKF method (including a lot of parameter, for example: process noise vector and measurement noise vector, we need to define them).
It contains following method:

        void Prediction(double delta_t);
        void ProcessMeasurement(MeasurementPackage meas_package);
        void UpdateLidar(MeasurementPackage meas_package);
        void UpdateRadar(MeasurementPackage meas_package);
        
In order to make implement the function, i make a little change and add new functions as follow:

        /*the prediction method. Used in two cases: one case is UKF predict process; another case is to predict the futher path.
        For futher path predict, the s!=NULL, this method won't change member x_, it will return the result to s and P.*/
        void Prediction(double delta_t, Eigen::VectorXd *s=NULL, Eigen::MatrixXd *P=NULL);
        
        /*the update function of lidar measurement. It will use normal KF formula to update the x_ and P_ */
        void UpdateLidar(MeasurementPackage meas_package);
        
        /*the update function of radar measurement. It will use UKF method to update the x_ and P_ */
        void UpdateRadar(MeasurementPackage meas_package);
        
        /*Prediction Process: generate sigma points with noise augment, according to the current state and covariance */
        void GenerateSigmaPointsWithAugment(Eigen::MatrixXd* Xsig_out);
        
        /*Prediction Process: transform the orginal sigma points to the predict sigma points. Use the f(x,mu) function. f(x,mu) is defined by CTRV model*/
        void SigmaPointPredictionWithAugment(const Eigen::MatrixXd & Xsig_aug, double dt, Eigen::MatrixXd* Xsig_predict_out);
        
        /*Prediction Process: use the predicted sigma points to compute the predicted state mean and covariance.*/
        void PredictMeanAndCovarianceWithSigmaPoints(const Eigen::MatrixXd & Xsig_predict, Eigen::VectorXd *s=NULL, Eigen::MatrixXd *P=NULL);
        
        /*Radar Update Process: use the predicted sigma points to compute the measurement predict sigma points. Use the h(x) function. h(x) is defined by Radar itself*/
        void PredictRadarMeasurementWithSigmaPoints(const Eigen::MatrixXd & Xsig_predict, Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd* Z_sigma_points_out);
        
        /*Radar Update Process: use measurement predict sigma points and the real radar measurement to update the state mean x_ and covariance P_*/
        void UpdateStateWithRadar(const Eigen::VectorXd & z_pred, const Eigen::MatrixXd & S, const Eigen::MatrixXd & Z_sigma_points, const Eigen::MatrixXd & Xsig_pred, const Eigen::VectorXd & z_measurement, double * nis_radar);
        
### Parameter choose
In order to make the choice about the process parameter of std_a_ and sta_yawdd, i record all cars lidar and radar nis value in nis_result.txt file. 
Then i choose the parameter std_a_=3.5 and std_yawdd_=M_PI/3.

### Lidar measurement weight and Radar measurement weight
In udacity project, there is a well design UKF sensor fusion framework. It use lidar and radar measurement to fusion, then tracking the object.
But I am wondering lidar and radar must have different belief in different situation(range & weather). But in this project, udacity don't give the method of apply the different belief (or weight) of the measurement.
I have try to use to vector to modify the Kalman_Gain added to the state x_

        Eigen::VectorXd kalman_gain_factor_lidar = VectorXd(n_x_);
        Eigen::VectorXd kalman_gain_factor_radar = VectorXd(n_x_);
        kalman_gain_factor_lidar << 1.2, 1.2,  1,   1,  1;  //i think lidar has more accuray in px,py belief
        kalman_gain_factor_radar << 1,   1,    1.2, 1,  1;  //i think radar has more accuray in v belief
        
        //lidar measurement udpate
        VectorXd _gain=(K * y);
        if(kalman_gain_factor_lidar_use)
          _gain= _gain.cwiseProduct(kalman_gain_factor_lidar);
        x_ = x_ + _gain;
        
        //radar measurement update
        VectorXd _gain=(K * z_diff);
        if(kalman_gain_factor_radar_use)
          _gain= _gain.cwiseProduct(kalman_gain_factor_radar);
        x_ = x_ + _gain;

But the result seems not like what i want. I don't know why now.

### NIS compute result
Here i list the nis result of lidar and radar nis result pictures. (use parameter std_a_=3.5 and std_yawdd_=M_PI/3).
You can find them in nis_result.xlsx and nis_result.txt.
![car1_lidar_nis](https://github.com/libing0811/SFND_Unscented_Kalman_Filter/blob/master/media/car1_lidar_nis.PNG)
![car1_radar_nis](https://github.com/libing0811/SFND_Unscented_Kalman_Filter/blob/master/media/car1_radar_nis.PNG)
![car2_lidar_nis](https://github.com/libing0811/SFND_Unscented_Kalman_Filter/blob/master/media/car2_lidar_nis.PNG)
![car2_radar_nis](https://github.com/libing0811/SFND_Unscented_Kalman_Filter/blob/master/media/car2_radar_nis.PNG)
![car3_lidar_nis](https://github.com/libing0811/SFND_Unscented_Kalman_Filter/blob/master/media/car3_lidar_nis.PNG)
![car3_radar_nis](https://github.com/libing0811/SFND_Unscented_Kalman_Filter/blob/master/media/car3_radar_nis.PNG)
