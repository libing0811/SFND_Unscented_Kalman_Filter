#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t, Eigen::VectorXd *s=NULL, Eigen::MatrixXd *P=NULL);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


  //Generate Sigma Points with augment noise vector
  void GenerateSigmaPointsWithAugment(Eigen::MatrixXd* Xsig_out);

  //Predict Sigma Points with augument noise vector
  void SigmaPointPredictionWithAugment(const Eigen::MatrixXd & Xsig_aug, double dt, Eigen::MatrixXd* Xsig_predict_out);

  //Compute predict state x mean and covariance.
  void PredictMeanAndCovarianceWithSigmaPoints(const Eigen::MatrixXd & Xsig_predict, Eigen::VectorXd *s=NULL, Eigen::MatrixXd *P=NULL);

  //Predict Radar Measurement
  void PredictRadarMeasurementWithSigmaPoints(const Eigen::MatrixXd & Xsig_predict, Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd* Z_sigma_points_out);

  //Update state with measurement
  void UpdateStateWithRadar(const Eigen::VectorXd & z_pred, const Eigen::MatrixXd & S, const Eigen::MatrixXd & Z_sigma_points, const Eigen::MatrixXd & Xsig_pred, const Eigen::VectorXd & z_measurement, double * nis_radar);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  //
  long long previous_timestamp_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
  
  std::vector<double> *lidar_nis_list;
  
  std::vector<double> *radar_nis_list;
};

#endif  // UKF_H
