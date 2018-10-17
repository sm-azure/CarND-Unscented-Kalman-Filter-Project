#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  P_.setIdentity();


  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
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
  

  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  ///* predicted sigma points matrix
  //MatrixXd Xsig_pred_;

 ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.setZero();;

  count = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  cout << "------------------------------------------------>" << count++ << endl;
  if(!is_initialized_){

    //Set weights vector
    weights_ = VectorXd(2*n_aug_+1);
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {  
      double weight = 0.5/(n_aug_+lambda_);
      weights_(i) = weight;
    }

    cout << "Weights" << weights_ <<endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      float px = measurement_pack.raw_measurements_[0]* cos(measurement_pack.raw_measurements_[1]);
      float py = measurement_pack.raw_measurements_[0]* sin(measurement_pack.raw_measurements_[1]);
      //float rho = measurement_pack.raw_measurements_[0];
      //float phi = measurement_pack.raw_measurements_[1];
      //float rho_dot = measurement_pack.raw_measurements_[2];
      
      cout <<"Radar Init:" <<endl;
      x_ << px,py,0.0,0.0,0.0;
      cout << x_ <<endl;
      cout <<"Radar Init Complete" <<endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      
      cout <<"Lidar Init:" <<endl;
      x_ << px,py,0.0,0.0,0.0;
      cout << x_ <<endl;
      cout <<"Lidar Init Complete" <<endl;
    }

    time_us_ = measurement_pack.timestamp_;
    cout << "t :" << measurement_pack.timestamp_ <<endl;
    is_initialized_ = true;
    return;
  }

  //Check if this type of data is allowed
  if((measurement_pack.sensor_type_ == MeasurementPackage::LASER && !use_laser_) || (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && !use_radar_)){
    return;
  }

  // Calculate delta time 
  cout << "t :" << measurement_pack.timestamp_ <<endl;
  double delta_t = measurement_pack.timestamp_ - time_us_; 
  if(delta_t == 0.0 ){
    return;
  }

  time_us_ = measurement_pack.timestamp_;

  //Predict
  cout << "delta_t (s):"  << delta_t/1000000.0 <<endl;
  Prediction(delta_t/1000000.0);

  //Update based on sensor type
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "Radar" <<endl;
    UpdateRadar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    cout << "Lidar -" <<endl;
    UpdateLidar(measurement_pack);      
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  cout << "In Prediction" <<endl;
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //Create Augemented Sigma Points  --------------------------------------------------->
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(x_.rows()) = x_;
  
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(P_.rows(), P_.cols()) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  
  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  //create augmented sigma points

  double scaling_factor = 3;
  MatrixXd A_Scaled = sqrt(scaling_factor) * A;
  MatrixXd A_Scaled_Add = A_Scaled.colwise() + x_aug;
  MatrixXd A_Scaled_Subtract = (-A_Scaled).colwise() + x_aug;
  
  
  Xsig_aug.fill(0.0);
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.block(0,1,A.rows(),A.cols()) = A_Scaled_Add;
  Xsig_aug.block(0,8,A.rows(),A.cols()) = A_Scaled_Subtract;

  //cout << "XSig" <<endl;
  //cout << Xsig_aug <<endl;
  

  //Predict Sigma Points --------------------------------------------------->
  

  //predict sigma points
  for (int i = 0 ; i<2 * n_aug_ + 1 ;i++ ){

    Xsig_pred_(0, i) = Xsig_aug(0, i) + 0.5 * (delta_t * delta_t)* cos(Xsig_aug(3,i)) * Xsig_aug(5,i);  
    Xsig_pred_(1, i) = Xsig_aug(1, i) + (delta_t * delta_t /2.0)* sin(Xsig_aug(3,i)) * Xsig_aug(5,i);
    Xsig_pred_(2, i) = Xsig_aug(2, i) + delta_t * Xsig_aug(5,i);
    Xsig_pred_(3, i) = Xsig_aug(3, i) + 0.5 * (delta_t * delta_t) * Xsig_aug(6,i) + delta_t * Xsig_aug(4, i);
    Xsig_pred_(4, i) = Xsig_aug(4, i) + delta_t * Xsig_aug(6,i);

    //avoid division by zero
    if(Xsig_aug(4, i) >0.001 ){
      Xsig_pred_(0, i) += (Xsig_aug(2,i) / Xsig_aug(4,i) )* (sin(Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t) - sin(Xsig_aug(3, i)));
      Xsig_pred_(1, i) += (Xsig_aug(2,i) / Xsig_aug(4,i) )* (-cos(Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t) + cos(Xsig_aug(3, i)));
    }else{
      Xsig_pred_(0, i) += Xsig_aug(2,i) * cos(Xsig_aug(3,i)) * delta_t;
      Xsig_pred_(1, i) += Xsig_aug(2,i) * sin(Xsig_aug(3,i)) * delta_t;
    }
  }

  // Predict Mean and Covariance ------------------------------------------->
  //predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //while (x_(3)> M_PI) x_(3)-=2.*M_PI;
  //while (x_(3)<-M_PI) x_(3)+=2.*M_PI;

  cout << "Prediction (x)" << endl;
  cout << x_ << endl;

  //cout << Xsig_pred_ << endl;
    
  //predict state covariance matrix
  P_.fill(0.0);
  
  for (int i = 0; i <2*n_aug_+1 ; i++ ){
      
    VectorXd D = Xsig_pred_.col(i) - x_;
    //cout << i << ")" << D << endl;
    //angle normalization
    while (D(3)> M_PI) D(3)-=2.*M_PI;
    while (D(3)<-M_PI) D(3)+=2.*M_PI;
  
    //cout << D.rows() << "," << D.cols() << endl;
    MatrixXd XX = D * D.transpose();
    P_ = P_ + weights_(i) * XX;
  }
    

  cout << "Prediction complete" << endl;
  //cout << P_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  cout << "In Update Radar" <<endl;
  float rho = measurement_pack.raw_measurements_[0];
  float phi = measurement_pack.raw_measurements_[1];
  float rho_dot = measurement_pack.raw_measurements_[2];
      
  cout << "Raw measurements: " << rho << "," << phi << "," << rho_dot << endl;
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  Zsig.fill(0.0);
  z_pred.fill(0.0);
  S.fill(0.0);
  
  for (int i=0; i<2 * n_aug_ + 1 ;i++ ){
    double rho =  sqrt (Xsig_pred_(0,i)* Xsig_pred_(0,i) + Xsig_pred_(1,i)*Xsig_pred_(1,i));
    double phi = atan2 (Xsig_pred_(1,i), Xsig_pred_(0,i));
    double rho_dot = (Xsig_pred_(0,i)* cos(Xsig_pred_(3,i)) * Xsig_pred_(2,i) + Xsig_pred_(1,i)* sin(Xsig_pred_(3,i)) * Xsig_pred_(2,i))/(rho);
      
    //cout << rho << "," << phi << "," << rho_dot << endl;
    z_pred[0] += weights_[i] * rho;
    z_pred[1] += weights_[i] * phi;
    z_pred[2] += weights_[i] * rho_dot;
    
    Zsig(0,i) = rho;
    Zsig(1,i) = phi;
    Zsig(2,i) = rho_dot;
  }
  cout << "Predicted measurements: " << z_pred[0] << "," << z_pred[1] << "," << z_pred[2] << endl;
  //calculate mean predicted measurement
  //calculate innovation covariance matrix S
  for(int i=0;i < 2* n_aug_+1 ;i++){
      VectorXd z_diff = Zsig.col(i)- z_pred;
      while (z_diff[1]> M_PI) z_diff[1]-=2.*M_PI;
      while (z_diff[1]<-M_PI) z_diff[1]+=2.*M_PI;
      S += weights_[i] * (z_diff * (z_diff).transpose());
  }
  //cout << "After z_diff 1" <<endl;
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  
  S += R;

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  rho = measurement_pack.raw_measurements_[0];
  phi = measurement_pack.raw_measurements_[1];
  rho_dot = measurement_pack.raw_measurements_[2];
  VectorXd z = VectorXd(n_z);
  z << rho, phi, rho_dot;

  
  for (int i=0;i< 2 * n_aug_ + 1; i ++ ){
      VectorXd z_diff = Zsig.col(i)- z_pred;
      while (z_diff[1]> M_PI) z_diff[1]-=2.*M_PI;
      while (z_diff[1]<-M_PI) z_diff[1]+=2.*M_PI;

      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      while (x_diff[1]> M_PI) x_diff[1]-=2.*M_PI;
      while (x_diff[1]<-M_PI) x_diff[1]+=2.*M_PI;

      Tc += weights_[i] * (x_diff) * (z_diff).transpose();
  }
  //cout << "Tc" << endl;
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;
  while (z_diff[1]> M_PI) z_diff[1]-=2.*M_PI;
  while (z_diff[1]<-M_PI) z_diff[1]+=2.*M_PI;
  
  x_ = x_ + K * (z_diff);

  P_ = P_ - K*S*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
 cout << "In Update Lidar" <<endl;
  float x = measurement_pack.raw_measurements_[0];
  float y = measurement_pack.raw_measurements_[1];
      
  cout << "Raw measurements: " << x << "," << y << endl;
  int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  Zsig.fill(0.0);
  z_pred.fill(0.0);
  S.fill(0.0);
  
  for (int i=0; i<2 * n_aug_ + 1 ;i++ ){
    double x =  Xsig_pred_(0,i);
    double y = Xsig_pred_(1,i);
      
    //cout << rho << "," << phi << "," << rho_dot << endl;
    z_pred[0] += weights_[i] * x;
    z_pred[1] += weights_[i] * y;
    
    Zsig(0,i) = x;
    Zsig(1,i) = y;
  }
  cout << "Predicted measurements: " << z_pred[0] << "," << z_pred[1] << endl;
  //calculate mean predicted measurement
  //calculate innovation covariance matrix S
  for(int i=0;i < 2* n_aug_+1 ;i++){
      VectorXd z_diff = Zsig.col(i)- z_pred;
      S += weights_[i] * (z_diff * (z_diff).transpose());
  }
  //cout << "After z_diff 1" <<endl;
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_;  
  S += R;

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  x = measurement_pack.raw_measurements_[0];
  y = measurement_pack.raw_measurements_[1];
  VectorXd z = VectorXd(n_z);
  z << x,y;

  
  for (int i=0;i< 2 * n_aug_ + 1; i ++ ){
      VectorXd z_diff = Zsig.col(i)- z_pred;
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      Tc += weights_[i] * (x_diff) * (z_diff).transpose();
  }
  //cout << "Tc" << endl;
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;
  x_ = x_ + K * (z_diff);

  P_ = P_ - K*S*K.transpose();
}
