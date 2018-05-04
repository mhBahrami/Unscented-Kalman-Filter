#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {

  ///* Augmented mean vector
  VectorXd x_aug_;
  ///* Augmented state covariance
  MatrixXd P_aug_;
  ///* Square root matrix
  MatrixXd A_aug_;
  ///* Augmented sigma points
  MatrixXd Xsig_aug_;

  VectorXd xk_;
  VectorXd XkDt_;
  VectorXd Nu_;
  VectorXd x_diff_;

  ///* RADAR Sigma points matrix in measurement space
  MatrixXd Zsig_radar_;

  ///* RADAR Mean predicted measurement
  VectorXd z_pred_radar_;

  ///* RADAR Measurement noise covariance matrix R_
  MatrixXd R_radar_;

  ///* RADAR Measurement covariance matrix S_
  MatrixXd S_radar_;

  ///* LIDAR Sigma points matrix in measurement space
  MatrixXd Zsig_lidar_;

  ///* LIDAR Mean predicted measurement
  VectorXd z_pred_lidar_;

  ///* LIDAR Measurement noise covariance matrix R_
  MatrixXd R_lidar_;

  ///* LIDAR Measurement covariance matrix S_
  MatrixXd S_lidar_;


  /**
   * Prediction helper functions
   */
  void GenerateAugmentedSigmaPoints();
  void PredictSigmaPoints(const double &delta_t);
  void PredictMeanAndCovariance();
  /**
   * Update helper functions
   */
  void PredictRadarMeasurement();
  void PredictLidarMeasurement();

  /**
   * Angle normalization to [-Pi, Pi]
   * @param angle_in_rad is angle in Radian.
   */
  void AngleNormalization(double *angle_in_rad);


public:

  ///* Initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* If this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* If this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* State covariance matrix
  MatrixXd P_;

  ///* Predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* Time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma points dimension
  int n_sig_;

  ///* Measurement dimension, RADAR can measure r, phi, and r_dot
  int n_z_radar_;

  ///* Measurement dimension, LIDAR can measure px, py
  int n_z_lidar_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* RADAR NIS
  double NIS_radar_;

  ///* LIDAR NIS
  double NIS_lidar_;


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
  void ProcessMeasurement(const MeasurementPackage &meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(const double &delta_t);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(const MeasurementPackage &meas_package);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const MeasurementPackage &meas_package);

};

#endif /* UKF_H */
