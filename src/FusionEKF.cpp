#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  /* Hj_ here is initialized with incorrect values, need to be recalculated when the radar data is received.
  */
  Hj_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 10, 0, 0, 0,
               0, 10, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho_ = measurement_pack.raw_measurements_(0);
      double theta_ = measurement_pack.raw_measurements_(1);
      double rho_dot_ = measurement_pack.raw_measurements_(2);
      double cos_theta_ = std::cos(theta_);
      double sin_theta_ = std::sin(theta_);

      ekf_.x_(0) = rho_ * cos_theta_;
      ekf_.x_(1) = rho_ * sin_theta_;
      ekf_.x_(2) = rho_dot_ * cos_theta_;
      ekf_.x_(3) = rho_dot_ * sin_theta_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt_ = double(measurement_pack.timestamp_ - previous_timestamp_) * 1e-6;
  previous_timestamp_ = measurement_pack.timestamp_;
  cout << "Time stamp:" << measurement_pack.timestamp_ << endl;
  cout << "dt = " << dt_ << endl;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt_, 0,
             0, 1, 0, dt_,
             0, 0, 1, 0,
             0, 0, 0, 1;

  cout << ekf_.F_ << endl;

  float dt4_ = std::pow(dt_, 4) / 4.0;
	float dt3_ = std::pow(dt_, 3) / 2.0;
	float dt2_ = std::pow(dt_, 2);

  double noise_ax = 9;
  double noise_ay = 9;

	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt4_ * noise_ax, 0, dt3_ * noise_ax, 0,
	           0, dt4_ * noise_ay, 0, dt3_ * noise_ay,
             dt3_ * noise_ax, 0, dt2_ * noise_ax, 0,
             0, dt3_ * noise_ay, 0, dt2_ * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  ekf_.Q_ << 0.1, 0, 0, 0,
             0, 0.1, 0, 0,
             0, 0, 0.1, 0,
             0, 0, 0, 0.1;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    // Radar updates
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
