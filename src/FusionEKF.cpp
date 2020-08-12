#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // --> DONE  

  // H for laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 10, 0, 0, 0,
            0, 10, 0, 0,
            0, 0, 100, 0,
            0, 0, 0, 100;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // DONE

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

      // --> DONE  
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);

      while (phi > M_PI) { 
          phi -= 2.0 * M_PI;
      }

      while (phi < -M_PI) { 
          phi += 2.0 * M_PI;
      }

      ekf_.x_ <<  ro * cos(phi),
                  ro * sin(phi),      
                  ro_dot * cos(phi),
                  ro_dot * sin(phi);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      // TODO: Initialize state 
      // --> DONE  
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;

    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
    
  }



  /**
   * Prediction
   */


  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   */


  // update timestamp
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd::Identity(4,4);

  ekf_.F_(0,2) = ekf_.F_(1,3) = dt;

  /* 
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // done

  float noise_ax = 9, noise_ay = 9;

  float d2, d3 ,d4;

  d2 = dt * dt;
  d3 = d2 * dt;
  d4 = d3 * dt;

  ekf_.Q_ = MatrixXd(4,4);

  ekf_.Q_ << d4/4 * noise_ax, 0, d3/2 * noise_ax, 0,
             0, d4/4* noise_ay,0, d3/2*noise_ay,
             d3/2 * noise_ax, 0, d2 * noise_ax, 0,
             0, d3/2* noise_ay,0, d2 * noise_ay;
             
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    // done

    // choose H and R
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    // update Extended Kalman Filter
    ekf_.UpdateEKF(measurement_pack.raw_measurements_); 

  } else {
    // TODO: Laser updates

    // done

    // update Extended Kalman Filter
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    // update Kalman Filter
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
