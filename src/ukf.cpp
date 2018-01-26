#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Define macro constants for each sensor
#define N_LASER 2
#define N_RADAR 3

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // // Process noise standard deviation longitudinal acceleration in m/s^2
  // std_a_ = 30;
  //
  // // Process noise standard deviation yaw acceleration in rad/s^2
  // std_yawdd_ = 30;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  //****************************************************************************
  // Complete Initialization
  //****************************************************************************

  // Set initial state to non-initialized
  is_initialized_ = false;

  // Set constants for dimensions and sizes
  n_x_ = 5;
  n_aug_ = 7;
  n_sig_ = 15;

  // Set lambda
  lambda_ = 3 - n_x_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5; // Assuming bicycle changes acceleration at 0.5 m/s^2 vs 3.0 m/s^2 for a car

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 10.0 * M_PI / 180.0; // Assuming bicycle changes turn at 10 degrees/s^2

  // Set x_ to an empty vector of zeroes
  x_.fill(0);

  // Set P_ to identity and fix its values later in state initialization
  P_ = MatrixXd::Identity(n_x_, n_x_); // 5x5

  // Set Xsig_pred_ matrix size
  Xsig_pred_ = MatrixXd(n_x_, n_sig_); // 5x15

  // Initialize sigma point weights
  weights_ = VectorXd(n_sig_);
  weights_.fill( 0.5 / (lambda_ + n_aug_) );
  weights_(0) = lambda_ / (lambda_ + n_aug_); // Only the mean state has a different weight

  // time_us_; // set in state initialization step
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  //****************************************************************************
  // Initialization Step
  //****************************************************************************
  if (!is_initialized_) {

    double init_v       = 3.0;        // assuming bicycle is going at 3.0 m/s
    double init_yaw     = 0.5 * M_PI; // assuming bicycle is going along Y-axis
    double init_yaw_dot = 0.0;        // assuming bicycle is going straight

    if(meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize x_ with raw data and assumptions
      x_ << meas_package.raw_measurements_(0),
            meas_package.raw_measurements_(1),
            init_v,
            init_yaw,
            init_yaw_dot;

      // Initialize P_ position stds with laser std values
      P_(0,0) = std_laspx_*std_laspx_;
      P_(1,1) = std_laspy_*std_laspy_;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Initialize x_ with raw data and assumptions
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      // double rho_dot = meas_package.raw_measurements_(2);

      x_ << (rho * cos(phi)),
            (rho * sin(phi)),
            init_v,
            init_yaw,
            init_yaw_dot;

      // Initialize P_ position stds with radar std values
      double std_radpx_ = std_radr_ * cos(std_radphi_);
      double std_radpy_ = std_radr_ * sin(std_radphi_);
      P_(0,0) = std_radpx_*std_radpx_;
      P_(1,1) = std_radpy_*std_radpy_;
    }

    // Set is_initialized_ to true to avoid re-initialization
    is_initialized_ = true;
  }

  //****************************************************************************
  // Predict and Update Steps Based on Sensor Type
  //****************************************************************************
  else {
    // Determine delta time in seconds
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    Prediction(dt);

    // Run Kalman Filter Pipeline based on selected sensor type
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      UpdateRadar(meas_package);
    }
  }

  //****************************************************************************
  // Update Time
  //****************************************************************************
  time_us_ = meas_package.timestamp_;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  //****************************************************************************
  // Determine Sigma Points
  //****************************************************************************

  MatrixXd Xsig_aug(n_aug_, n_sig_); // 7x15 matrix
  VectorXd x_aug(n_aug_); // 7d vector
  MatrixXd P_aug(n_aug_, n_aug_); // 7x7 matrix
  MatrixXd L; // 7x7 square root matrix based on P_aug

  // Create augmented mean state vector
  x_aug.fill(0);
  x_aug.head(n_x_) = x_;
  // x_aug(5) and x_aug(6), nu_a, and nu_yawdd, have means of 0

  // Set augmented covariance matrix
  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_ * std_yawdd_;

  // Calculate square root matrix
  L = P_aug.llt().matrixL();

  // Calculate sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(1+i)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(1+i+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //****************************************************************************
  // Predict Sigma Points' Next States
  //****************************************************************************

  for(int i = 0; i < n_sig_; ++i) {
    // VectorXd x_aug(n_aug_); // x_aug already declared in previous step
    x_aug = Xsig_aug.col(i);

    //--------------------------------------------------------------------------
    // Get x term
    //--------------------------------------------------------------------------
    VectorXd x = x_aug.head(n_x_);

    //--------------------------------------------------------------------------
    // Get x_dot term
    //--------------------------------------------------------------------------
    double v        = x_aug(2);
    double yaw      = x_aug(3);
    double yaw_dot  = x_aug(4);
    double nu_a     = x_aug(5);
    double nu_yawdd = x_aug(6);

    VectorXd x_dot(n_x_);
    x_dot.fill(0);
    x_dot(3) = yaw_dot * delta_t;

    // Check if yaw_dot is 0 to determine how to set px_dot and py_dot values
    if (fabs(yaw_dot) >= 0.0001) {
      double coeff = v / yaw_dot;
      double trig_term = yaw + yaw_dot * delta_t;

      x_dot(0) = coeff * (sin(trig_term) - sin(yaw));
      x_dot(1) = coeff * (cos(yaw) - cos(trig_term));
    } else {
      x_dot(0) = v * cos(yaw) * delta_t;
      x_dot(1) = v * sin(yaw) * delta_t;
    }

    //--------------------------------------------------------------------------
    // Get noise term
    //--------------------------------------------------------------------------
    double dt2 = delta_t * delta_t;

    VectorXd nu(n_x_);
    nu.fill(0);
    nu << (0.5 * dt2 * cos(yaw) * nu_a),
          (0.5 * dt2 * sin(yaw) * nu_a),
          (delta_t * nu_a),
          (0.5 * dt2 * nu_yawdd),
          (delta_t * nu_yawdd);

    //--------------------------------------------------------------------------
    // Calculate predicted sigma points
    //--------------------------------------------------------------------------
    Xsig_pred_.col(i) = x + x_dot + nu;
  }


  //****************************************************************************
  // Update New State Mean and Covariance
  //****************************************************************************

  // Update x_ prediction
  x_.fill(0);
  for(int i = 0; i < n_sig_; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // Normalize Angles
  while(x_(3) >  M_PI) x_(3) -= 2.0 * M_PI;
  while(x_(3) < -M_PI) x_(3) += 2.0 * M_PI;

  // Update P_ prediction
  P_.fill(0);
  for(int i = 0; i < n_sig_; ++i) {
    // Get diff of point from mean state
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Normalize Angles
    while(x_diff(3) >  M_PI) x_diff(3) -= 2.0 * M_PI;
    while(x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

    // Update P_
    P_ += weights_(i) * (x_diff * x_diff.transpose());
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  //****************************************************************************
  // Predict z for radar
  //****************************************************************************
  MatrixXd Zsig(N_LASER, n_sig_); // 3x15 matrix
  VectorXd z_pred(N_LASER); // 3d vector

  // Map Xsig_pred_ to z state space
  for(int i=0; i < n_sig_; ++i) {
    float px  = Xsig_pred_(0,i);
    float py  = Xsig_pred_(1,i);
    // float v   = Xsig_pred_(2,i);
    // float yaw = Xsig_pred_(3,i);
    // float yaw_dot = Xsig_pred_(4,i);

    Zsig(0,i) = px;
    Zsig(1,i) = py;
  }

  // Calculate mean predicted measurement vector
  z_pred.fill(0);
  for(int i = 0; i < n_sig_; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  //****************************************************************************
  // Calculate S, T and K
  //****************************************************************************
  MatrixXd S(N_LASER, N_LASER); // 3x3 matrix
  MatrixXd T(n_x_, N_LASER); // 5x3 matrix
  MatrixXd K; // 5x3 matrix

  //--------------------------------------------------------------------------
  // Calculate innovation covariance matrix S
  //--------------------------------------------------------------------------
  S.fill(0);
  S(0,0) = std_laspx_ * std_laspx_; // Set S to R Matrix to avoid unnecessary memory allocation for R
  S(1,1) = std_laspy_ * std_laspy_; // Set S to R Matrix to avoid unnecessary memory allocation for R

  for(int i = 0; i < n_sig_; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S += weights_(i) * (z_diff * z_diff.transpose());
  }

  //--------------------------------------------------------------------------
  // Calculate cross correlation matrix T
  //--------------------------------------------------------------------------
  T.fill(0);
  for(int i = 0; i < n_sig_; ++i) {
    T += weights_(i) * (Xsig_pred_.col(i) - x_) * (Zsig.col(i) - z_pred).transpose();
  }

  //--------------------------------------------------------------------------
  // Calculate Kalman gain K
  //--------------------------------------------------------------------------
  K = T * S.inverse();

  //****************************************************************************
  // Update New State Mean and Covariance
  //****************************************************************************
  // Get sensor measurement z
  VectorXd z(N_LASER);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1);

  // Residual
  VectorXd z_diff = z - z_pred;

  // Update x_ and P_
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  //****************************************************************************
  // Calculate NIS
  //****************************************************************************
  VectorXd epsilon = (z_diff).transpose() * S.inverse() * z_diff;
  float NIS = epsilon(0);

  // Print as CSV file in format "LASER_NIS, RADAR_NIS"
  std::cout << NIS << "," << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //****************************************************************************
  // Predict z for radar
  //****************************************************************************
  MatrixXd Zsig(N_RADAR, n_sig_); // 3x15 matrix
  VectorXd z_pred(N_RADAR); // 3d vector

  // Map Xsig_pred_ to z state space
  for(int i=0; i < n_sig_; ++i) {
    float px  = Xsig_pred_(0,i);
    float py  = Xsig_pred_(1,i);
    float v   = Xsig_pred_(2,i);
    float yaw = Xsig_pred_(3,i);
    // float yaw_dot = Xsig_pred_(4,i);

    Zsig(0,i) = sqrt(px*px + py*py);
    Zsig(1,i) = atan2(py, px);
    Zsig(2,i) = (px*cos(yaw)*v + py*sin(yaw)*v) / Zsig(0,i);
  }

  // Calculate mean predicted measurement vector
  z_pred.fill(0);
  for(int i = 0; i < n_sig_; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  //****************************************************************************
  // Calculate S, T and K
  //****************************************************************************
  MatrixXd S(N_RADAR, N_RADAR); // 3x3 matrix
  MatrixXd T(n_x_, N_RADAR); // 5x3 matrix
  MatrixXd K; // 5x3 matrix

  //--------------------------------------------------------------------------
  // Calculate innovation covariance matrix S
  //--------------------------------------------------------------------------
  S.fill(0);
  S(0,0) = std_radr_ * std_radr_;     // Set S to R Matrix to avoid unnecessary memory allocation for R
  S(1,1) = std_radphi_ * std_radphi_; // Set S to R Matrix to avoid unnecessary memory allocation for R
  S(2,2) = std_radrd_ * std_radrd_;   // Set S to R Matrix to avoid unnecessary memory allocation for R

  for(int i = 0; i < n_sig_; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Normalize yaw angle
    while(z_diff(1) >  M_PI) z_diff(1) -= 2.0 * M_PI;
    while(z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    S += weights_(i) * (z_diff * z_diff.transpose());
  }

  //--------------------------------------------------------------------------
  // Calculate cross correlation matrix T
  //--------------------------------------------------------------------------
  T.fill(0);
  for(int i = 0; i < n_sig_; ++i) {

    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // Normalize yaw angle
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    T += weights_(i) * x_diff * z_diff.transpose();

    // T += weights_(i) * (Xsig_pred_.col(i) - x_) * (Zsig.col(i) - z_pred).transpose();
  }

  //--------------------------------------------------------------------------
  // Calculate Kalman gain K
  //--------------------------------------------------------------------------
  K = T * S.inverse();

  //****************************************************************************
  // Update New State Mean and Covariance
  //****************************************************************************
  // Get sensor measurement z
  VectorXd z(N_RADAR);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1),
       meas_package.raw_measurements_(2);

  // Residual
  VectorXd z_diff = z - z_pred;

  // Normalize yaw angle
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // Update x_ and P_
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  //****************************************************************************
  // Calculate NIS
  //****************************************************************************
  VectorXd epsilon = (z_diff).transpose() * S.inverse() * z_diff;
  float NIS = epsilon(0);

  // Print as CSV file in format "LASER_NIS, RADAR_NIS"
  std::cout << "," << NIS << std::endl;
}
