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

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);

  //measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //For jacobian calculation
  Tools tools;

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
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float ro_p = measurement_pack.raw_measurements_[2];

      ekf_.x_ << ro*cos(phi), ro*sin(phi), ro_p*cos(phi), ro_p*sin(phi);

      // Considering low uncertainity in the poistion and high certeinity in velocity 
      ekf_.P_ << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

      // Considering high certeinity in velocity in position and maximun uncertainity in velocity
      ekf_.P_ << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1000, 0,
                 0, 0, 0, 1000;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;

    //cout << "x_ = " << endl << ekf_.x_ << endl;
    //cout << "P_ = " << endl << ekf_.P_ << endl;

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
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;


  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  //acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;


  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);

  MatrixXd G(4,2);
  G << dt*dt/2,       0,
         0    , dt*dt/2,
         dt   ,     0,
         0    ,    dt;

  MatrixXd Qv(2,2);
  Qv << noise_ax, 0,
      0     , noise_ay;

  ekf_.Q_ = G*Qv*G.transpose();



  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    //As always Jacobian Matrix have 0 in the right corner I set it to -1 in case
    //a division by zero in its calculation. When that error happen, we didn´t update.
    if (Hj_(0,3)!=-1){
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }else{
    //In case a division by zero, update only with the raw measurement
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float ro_p = measurement_pack.raw_measurements_[2];

      ekf_.x_ << ro*cos(phi), ro*sin(phi), ro_p*cos(phi), ro_p*sin(phi);
    }
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << endl << ekf_.x_ << endl;
  //cout << "P_ = " << endl << ekf_.P_ << endl;
  
}
