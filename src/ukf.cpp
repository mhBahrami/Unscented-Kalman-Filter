#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>
#define THRESHOLD 0.001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  
  /// DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  /// DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;

  // If this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // If this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Measurement dimension, radar can measure r, phi, and r_dot
  n_z_radar_ = 3;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // Initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // Initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);

  // Initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  // Time when the state is true, in us
  time_us_ = 0;

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Augmented mean vector
  x_aug_ = VectorXd(n_x_);
  // Augmented state covariance
  P_aug_ = MatrixXd(n_x_, n_x_);
  // Square root matrix
  A_aug_ = MatrixXd(n_x_, n_x_);
  // Aaugmented sigma points
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  xk_ = VectorXd(n_x_);
  XkDt_ = VectorXd(n_x_);
  Nu_ = VectorXd(n_x_);
  x_diff_ = VectorXd(n_x_);

  // Sigma points matrix in measurement space
  Zsig_radar_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

  // Mean predicted measurement
  z_pred_radar_ = VectorXd(n_z_radar_);

  // Measurement covariance matrix S_radar_
  S_radar_ = MatrixXd(n_z_radar_, n_z_radar_);

  ///* Tuning parameters
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  ///* Tuning parameters

}

UKF::~UKF() = default;

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
        * Initialize the state x_ with the first measurement.
        * Create the covariance matrix.
        * We need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "UKF: " << endl;
    //x_ = VectorXd(4);
    x_ << 1.0, 1.0, 0, 0, 0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];
      double px = rho*cos(phi);
      double py = rho*sin(phi);
//      double vx = rho_dot*cos(phi);
//      double vy = rho_dot*sin(phi);
      x_ << px, py, 0, 0, 0;
      cout << "[RADAR] Initializing x: " << endl << x_ << endl;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], x_(2), x_(3), x_(4);
      cout << "[LASER] Initializing x: " << endl << x_ << endl;
    }

    // If the initial values are less than THRESHOLD
//    if (fabs(x_(0) < THRESHOLD && fabs(x_(1)) < THRESHOLD) {
//      x_[0] = THRESHOLD;
//      x_[1] = THRESHOLD;
//    }

    P_ = MatrixXd::Identity(n_x_, n_x_);

    // The initial timestamp for dt calculation
    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  /**
     * Time is measured in seconds.
     * Compute the time elapsed between the current and previous measurements
   */
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0f;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    UpdateRadar(meas_package);
  } else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    UpdateRadar(meas_package);
  }

}

void UKF::GenerateAugmentedSigmaPoints() {
  // Create augmented mean state
  x_aug_.fill(0.0);
  x_aug_.head(n_x_) = x_;

  // Create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.block(0, 0, n_x_, n_x_) = P_;
  P_aug_(5, 5) = std_a_ * std_a_;
  P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

  // Create square root matrix
  A_aug_ = P_aug_.llt().matrixL();

  // Create augmented sigma points
  double RSLambda = sqrt(lambda_ + n_aug_);
  Xsig_aug_.col(0) = x_aug_;
  for(int i=0; i < n_aug_; ++i) {
    Xsig_aug_.col(i + 1)          = x_aug_ + RSLambda * A_aug_.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - RSLambda * A_aug_.col(i);
  }
}

void UKF::PredictSigmaPoints(const double &delta_t) {

  double dt22 = delta_t*delta_t/2;
  double nu_ak = 0.0;
  double nu_yawddk = 0.0;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    xk_ = Xsig_aug_.block(0, i, n_x_, 1);
    nu_ak = Xsig_aug_(5, i);
    nu_yawddk = Xsig_aug_(6, i);

    // Avoid division by zero
    XkDt_.fill(0.0);
    if (fabs(xk_(4)) < THRESHOLD) {
      XkDt_(0) = xk_(2) * cos(xk_(3)) * delta_t;
      XkDt_(1) = xk_(2) * sin(xk_(3)) * delta_t;
    } else {
      XkDt_(0) = xk_(2) / xk_(4) * (sin(xk_(3) + xk_(4) * delta_t) - sin(xk_(3)));
      XkDt_(1) = xk_(2) / xk_(4) * (-cos(xk_(3) + xk_(4) * delta_t) + cos(xk_(3)));
    }
    XkDt_(3) = xk_(4) * delta_t;

    Nu_ << dt22 * cos(xk_(3)) * nu_ak,
        dt22 * sin(xk_(3)) * nu_ak,
        delta_t * nu_ak,
        dt22 * nu_yawddk,
        delta_t * nu_yawddk;

    // Write predicted sigma points into right column
    Xsig_pred_.col(i) = xk_ + XkDt_ + Nu_;
  }

}

void UKF::PredictMeanAndCovariance() {
  // Set weights
  double perLamNa2 = 0.5 / (lambda_ + n_aug_);
  weights_.fill(perLamNa2);
  weights_(0) *= 2 * lambda_;

  // Predict state mean
  x_ = Xsig_pred_ * weights_;

  // Predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_diff_ = Xsig_pred_.col(i) - x_;
    while (x_diff_(3) >  M_PI) x_diff_(3) -= 2.0 * M_PI;
    while (x_diff_(3) < -M_PI) x_diff_(3) += 2.0 * M_PI;

    P_ += weights_(i) * x_diff_ * x_diff_.transpose();
  }

}

void UKF::PredictRadarMeasurement() {

  // Transform sigma points into measurement space
  double px;
  double py;
  double v;
  double yaw;

  double rho;
  double phi;
  double rhod;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    px = Xsig_pred_(0, i);
    py = Xsig_pred_(1, i);
    v = Xsig_pred_(2, i);
    yaw = Xsig_pred_(3, i);
    rho = sqrt(px * px + py * py);
    phi = atan2(py, px);
    rhod = (px * cos(yaw) + py * sin(yaw)) * v / rho;

    Zsig_radar_(0, i) = rho;
    Zsig_radar_(1, i) = phi;
    Zsig_radar_(2, i) = rhod;
  }

  // Calculate mean predicted measurement
  z_pred_radar_ = Zsig_radar_ * weights_ ;

  // Calculate innovation covariance matrix S_radar_
  MatrixXd R = MatrixXd(n_z_radar_, n_z_radar_);
  R.fill(0.0);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;

  VectorXd z_diff = VectorXd(n_z_radar_);
  S_radar_ = R;
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_diff = Zsig_radar_.col(i) - z_pred_radar_;

    while (z_diff(1) >  M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    S_radar_ += weights_(i) * z_diff * z_diff.transpose();
  }

}


void UKF::PredictLaserMeasurement() {

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(const double &delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  this->GenerateAugmentedSigmaPoints();
  this->PredictSigmaPoints(delta_t);
  this->PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage &meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  this->PredictRadarMeasurement();

  // Incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;
  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);


  // Calculate cross correlation matrix
  VectorXd z_diff = VectorXd(n_z_radar_);
  VectorXd x_diff = VectorXd(n_x_);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_diff = Xsig_pred_.col(i) - x_;
    z_diff = Zsig_radar_.col(i) - z_pred_radar_;

    while (x_diff(3) >  M_PI) x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
    while (z_diff(1) >  M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Calculate Kalman gain K;
  MatrixXd K = Tc * S_radar_.inverse();

  // Update state mean and covariance matrix
  x_ = x_ + K * (z - z_pred_radar_);
  P_ = P_ - K * S_radar_ * K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage &meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}
