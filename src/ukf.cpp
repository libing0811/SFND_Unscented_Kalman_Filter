#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  //dimension of state
  n_x_=5; 

  //dimension of augment_state
  n_aug_=n_x_+2;

  //sigma point spread paramter
  lambda_ = 3 - n_x_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.5; //this value need to be fine tuned.

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/3; //this value need to be fine tuned.
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  is_initialized_=false; 

  previous_timestamp_=0;

  Xsig_pred_ = MatrixXd(n_aug_, 2*n_x_+1);

  //time_us_ = ? //how to use time_us? 

  //init weights of sigma points 
  weights_= VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  double weight = 0.5/(lambda_+n_aug_);
  weights_(0) = weight_0;

  for (int i=1; i<2*n_aug_+1; ++i) {  
    weights_(i) = weight;
  }
  
  //
  lidar_nis_list = new std::vector<double>();
  radar_nis_list = new std::vector<double>();
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_) {
    //cout << "Kalman Filter Initialization " << endl;
    
    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // set the state with the initial location and zero velocity
      //LASER
      if(use_laser_==false)return;
      x_ << meas_package.raw_measurements_[0], 
            meas_package.raw_measurements_[1], 
            0, 
            0,
            0;
    }
    else
    {
      //RADAR
      if(use_radar_==false)return;
      double range = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double range_velocity= meas_package.raw_measurements_[2];

      // set the state with the initial location and zero velocity
      x_ << range*sin(phi),
            range*cos(phi),
            0,
            0,
            0;
    }
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(dt);

  if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    if(use_laser_)
      UpdateLidar(meas_package);
  }
  else
  {
    if(use_radar_)
      UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t, VectorXd *x, MatrixXd *P) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  //1.generate sigma points
  MatrixXd Sigma_points;
  GenerateSigmaPointsWithAugment(&Sigma_points);
  //GenerateSigmaPointsWithAugment(Eigen::MatrixXd* Xsig_out);

  //2.predict sigma points and store it in Xsig_pred_
  SigmaPointPredictionWithAugment(Sigma_points, delta_t, &Xsig_pred_);
  //SigmaPointPredictionWithAugment(const Eigen::MatrixXd & Xsig_aug, double dt, Eigen::MatrixXd* Xsig_predict_out);

  //3.compute&update state and covariance --> find Prior
  PredictMeanAndCovarianceWithSigmaPoints(Xsig_pred_, x, P);
  //PredictMeanAndCovarianceWithSigmaPoints(const Eigen::MatrixXd & Xsig_predict);

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  
  //Use Normal Kalman Filter Update Method.

  MatrixXd H_= MatrixXd(2,5);
  H_ << 1,0,0,0,0,
        0,1,0,0,0;

  MatrixXd R_= MatrixXd(2,2);
  R_ << std_laspx_*std_laspx_,0,
        0,std_laspy_*std_laspy_;
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  //NIS_lidar
  double NIS_lidar=  y.transpose()*Si*y;
  
  lidar_nis_list->push_back(NIS_lidar);

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

   //1. Predict Measurement , reuse the predicted sigma points
   VectorXd z_pred;
   MatrixXd S_pred;
   MatrixXd Z_sigma_points;
   double NIS_radar;
   PredictRadarMeasurementWithSigmaPoints(Xsig_pred_, &z_pred, &S_pred, &Z_sigma_points);
   //PredictRadarMeasurementWithSigmaPoints(const Eigen::MatrixXd & Xsig_predict, Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd* Z_sigma_points_out);

   //2. Compute Update (find Postior)
   UpdateStateWithRadar(z_pred, S_pred, Z_sigma_points, Xsig_pred_, meas_package.raw_measurements_, &NIS_radar);
   //UpdateStateWithRadar(const Eigen::VectorXd & z_pred, const Eigen::MatrixXd & S, const Eigen::MatrixXd & Z_sigma_points, const Eigen::MatrixXd & Xsig_pred, const Eigen::VectorXd & z_measurement, double * nis_radar);
  
  radar_nis_list->push_back(NIS_radar);
   
}

void UKF::GenerateSigmaPointsWithAugment(Eigen::MatrixXd* Xsig_out){

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;

  // create augmented sigma points
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  // write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPredictionWithAugment(const Eigen::MatrixXd & Xsig_aug, double dt, Eigen::MatrixXd* Xsig_predict_out)
{
  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  double delta_t = dt; // time diff in sec

  // predict sigma points
  // avoid division by zero
  for(int i=0 ; i< Xsig_aug.cols(); i++)
  {
      VectorXd sigma_point = Xsig_aug.col(i);
      VectorXd predict(n_x_) ;
      
      float px= sigma_point[0];
      float py= sigma_point[1];
      float v= sigma_point[2];
      float yaw= sigma_point[3];
      float yaw_rate= sigma_point[4];
      float mu_acc=sigma_point[5];
      float mu_yaw=sigma_point[6];

      if(yaw_rate==0)
      {
          //yaw rate == 0 
          predict[0] = v*cos(yaw)*delta_t;
          predict[1] = v*sin(yaw)*delta_t;
          predict[2] = 0;
          predict[3] = yaw_rate *delta_t;
          predict[4] = 0;
      }
      else
      {
          //yaw rate != 0
          predict[0] = v*(sin(yaw+yaw_rate*delta_t)-sin(yaw))/yaw_rate;
          predict[1] = v*(-cos(yaw+yaw_rate*delta_t)+cos(yaw))/yaw_rate;
          predict[2] = 0;
          predict[3] = yaw_rate*delta_t;
          predict[4] = 0;

      }

      //add noise value and x_state
      predict[0] += delta_t*delta_t*cos(yaw)*mu_acc*0.5 + px;
      predict[1] += delta_t*delta_t*sin(yaw)*mu_acc*0.5 + py;
      predict[2] += delta_t * mu_acc + v;
      predict[3] += delta_t*delta_t*mu_yaw*0.5 +yaw;
      predict[4] += delta_t * mu_yaw + yaw_rate;

      //
      // write predicted sigma points into right column
      Xsig_pred.col(i) = predict;
  }

  *Xsig_predict_out = Xsig_pred;
}

void UKF::PredictMeanAndCovarianceWithSigmaPoints(const Eigen::MatrixXd & Xsig_predict, VectorXd * _x, MatrixXd * _P)
{
  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  x.fill(0.0);
  // predict state mean
  for(int i=0; i<2*n_aug_+1 ; i++)
  {
      x += weights_(i) * Xsig_predict.col(i);
  }

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  P.fill(0.0);
  // predict state covariance matrix
  for(int i=0; i<2*n_aug_+1; i++)
  {
      VectorXd delta= Xsig_predict.col(i)-x;

      // angle normalization
      while (delta(3)> M_PI) delta(3)-=2.*M_PI;
      while (delta(3)<-M_PI) delta(3)+=2.*M_PI;

        
      P += weights_(i) * (delta)*(delta.transpose());
  }

  // write result to x and P
  if(_x==NULL){
    x_ = x;
    P_ = P;
  }
  else{
    *_x= x;
    *_P= P;
  }
}


void UKF::PredictRadarMeasurementWithSigmaPoints(const Eigen::MatrixXd & Xsig_predict, Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd* Z_sigma_points_out)
{
  int n_z=3; //measurement dimension

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // transform sigma points into measurement space
  for(int i=0; i< Xsig_predict.cols(); i++)
  {
      VectorXd point = Xsig_predict.col(i);
      double px=point(0);
      double py=point(1);
      double v=point(2);
      double yaw=point(3);
    
      double v1 = cos(yaw)*v;
    	double v2 = sin(yaw)*v;

      double len=sqrt(px*px+py*py);

      VectorXd measure_point(n_z);

      measure_point[0] = len;
      measure_point[1] = atan2(py,px);
      measure_point[2] = (px*v1 + py*v2) / len;

      Zsig.col(i) = measure_point;
  }


  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
   }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
   }


  MatrixXd R(3,3);
  R.fill(0.0);
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;

  S += R;
  
  // write result
  *z_out = z_pred;
  *S_out = S;
  *Z_sigma_points_out=Zsig;
}

void UKF::UpdateStateWithRadar(const Eigen::VectorXd & z_pred, const Eigen::MatrixXd & S, const Eigen::MatrixXd & Z_sigma_points, const Eigen::MatrixXd & Xsig_pred, const Eigen::VectorXd & z_measurement, double * nis_radar)
{
  int n_z=3;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for(int i=0; i< Xsig_pred.cols(); i++)
  {
      // residual
      VectorXd z_diff = Z_sigma_points.col(i) - z_pred;
      // angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      // state difference
      VectorXd x_diff = Xsig_pred.col(i) - x_;
      // angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd S_inverse = S.inverse();
  MatrixXd K = Tc * S_inverse;

  // update state mean and covariance matrix
  // residual
  VectorXd z_diff = z_measurement - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ = x_ + K*(z_diff);
  P_ = P_ - K*S*K.transpose();

  //compute NIS
  double nis= z_diff.transpose()* S_inverse*z_diff;

  *nis_radar=nis;
}
