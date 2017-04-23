#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

double constrainAngle(double x){
    x = fmod(x + M_PI, 2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5; // a balance between smooth tracking and fast response

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.54;

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
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  //example init P_
  P_ << 0.5, 0, 0, 0, 0,
  0, 0.5, 0, 0, 0,
  0, 0, 10.0, 0, 0,
  0, 0, 0, 4.0, 0,
  0, 0, 0, 0, 4.0;


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
    //cout << "UKF: " << endl;

    x_.fill(0.0);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];

      x_ <<  rho*cos(phi), rho*sin(phi), 0.0, 0.0, 0.0;
      previous_timestamp_ = meas_package.timestamp_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0.0, 0.0, 0.0;
      previous_timestamp_ = meas_package.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "UKF Initialized" << endl;
    return;

    cout << "x0 " << x_ <<endl;
  }

  // Loop here -----
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true) || (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true)) {

    double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    cout << "ps: " << previous_timestamp_ <<endl;
    cout << "ts: " << meas_package.timestamp_ <<endl;
    previous_timestamp_ = meas_package.timestamp_;

    cout << "delta_t: " << delta_t << endl;
    // while (delta_t > 0.1)
    // {
    //   const double dt = 0.05;
    //   Prediction(dt);
    //   delta_t -= dt;
    // }
    Prediction(delta_t);
    // normalize the yaw angle
    x_(3) = constrainAngle(x_(3));
    // check for negative velocity, if so, flip vel and yaw angle
    if (x_(2) < 0){
      x_(2) *= -1.0;
      x_(4) *= -1.0;
      x_(3) = constrainAngle(x_(3)+M_PI);
    }

    cout << "xP " << x_ <<endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true) {
      UpdateRadar(meas_package);
      cout << "xUR " << x_ <<endl;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true) {
      UpdateLidar(meas_package);
      cout << "xUL " << x_ <<endl;
    }
    x_(3) = constrainAngle(x_(3));
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // SIGMA POINTS ----
  //set state dimension
  n_x_ = 5;

  //define spreading parameter
  double lambda = 3 - n_x_;

  /*
  //create sigma point matrix - don't need to do this
  Xsig_ = MatrixXd(n_x_, 2 * n_x_ + 1);


  //calculate square root of P
  A_ = P_.llt().matrixL();

  //calculate sigma points
  //set sigma points as columns of matrix Xsig
  Xsig_.col(0) = x_;
  for (int i=0;i<n_x_;i++){
    Xsig_.col(i+1) = x + sqrt(lambda+n_x_)*A_.col(i);
    Xsig_.col(1+n_x+i) = x - sqrt(lambda+n_x_)*A_.col(i);
  }
  */

  // AUGMENTED SIGMA POINTS ----

  //set augmented dimension
  n_aug_ = 7;

  //create augemented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //define augmented spreading parameter
  double lambda_aug = 3 - n_aug_;

  //create augmented mean vector
  x_aug_ = VectorXd(n_aug_);

  //create augmented state covariance
  P_aug_ = MatrixXd(n_aug_, n_aug_);

  //create augmented mean state
  x_aug_.fill(0.0);
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;
  //cout << "x_aug_ " << x_aug_ <<endl ; //debug
  //create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0, 0, std_yawdd_*std_yawdd_;
  P_aug_.bottomRightCorner(2,2) = Q;

  A_aug_ = P_aug_.llt().matrixL();

  Xsig_aug_.col(0) = x_aug_;

  for (int i=0;i<n_aug_;i++){
    Xsig_aug_.col(i+1) = x_aug_ + sqrt(lambda_aug+n_aug_)*A_aug_.col(i);
    Xsig_aug_.col(1+n_aug_+i) = x_aug_ - sqrt(lambda_aug+n_aug_)*A_aug_.col(i);
  }
  //cout << "Xsig_aug_ " << Xsig_aug_ <<endl ; //debug
  //SIGMA PT PREDICTION ------
  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // RE Implementation - check if something is wrong

  //Predict points
  // for (int i=0;i<2*n_aug_+1;i++){ //iterate through columns
  //   VectorXd x = Xsig_aug_.col(i); // NOT x_ , temporary state, one sigma pt
  //     //std::cout << x << std::endl;
  //   VectorXd A = VectorXd(5); //use this to hold the state transition matrix
  //   VectorXd B = VectorXd(5); // use this to hold the noise influence
  //
  //   float px = x(0);
  //   float py = x(1);
  //   float v = x(2);
  //   float psi = x(3);
  //   float psi_dot = x(4);
  //   float nu_a = x(5);
  //   float nu_psi_dotdot = x(6);
  //
  //   //yaw rate is not zero
  //   if (fabs(psi_dot) > 0.001){
  //       A << v/psi_dot*(sin(psi+psi_dot*delta_t)-sin(psi)), v/psi_dot*(-1.0*cos(psi+psi_dot*delta_t)+cos(psi)),
  //           0, psi_dot*delta_t, 0;
  //       B << 0.5*pow(delta_t,2)*cos(psi)*nu_a, 0.5*pow(delta_t,2)*sin(psi)*nu_a,
  //           delta_t*nu_a, 0.5*pow(delta_t,2)*nu_psi_dotdot, delta_t*nu_psi_dotdot;
  //
  //       VectorXd x_pred = x.head(5) + A + B;
  //       //std::cout << x_pred;
  //       Xsig_pred_.col(i) = x_pred;
  //     }
  //   //yaw rate is zero
  //   else {
  //
  //       A << v*cos(psi)*delta_t, v*sin(psi)*delta_t, 0.0, psi_dot*delta_t, 0.0;
  //
  //       B << 0.5*pow(delta_t,2)*cos(psi)*nu_a, 0.5*pow(delta_t,2)*sin(psi)*nu_a,
  //           delta_t*nu_a, 0.5*pow(delta_t,2)*nu_psi_dotdot, delta_t*nu_psi_dotdot;
  //
  //       VectorXd x_pred = x.head(5) + A + B;
  //       //std::cout << x_pred;
  //       Xsig_pred_.col(i) = x_pred;
  //     }
  // }
  //cout << "Xsig_pred_ " << Xsig_pred_ <<endl;

  // Classroom implementation
  //predict sigma points
   for (int i = 0; i< 2*n_aug_+1; i++)
   {
     //extract values for better readability
     double p_x = Xsig_aug_(0,i);
     double p_y = Xsig_aug_(1,i);
     double v = Xsig_aug_(2,i);
     double yaw = Xsig_aug_(3,i);
     double yawd = Xsig_aug_(4,i);
     double nu_a = Xsig_aug_(5,i);
     double nu_yawdd = Xsig_aug_(6,i);

     //predicted state values
     double px_p, py_p;

     //avoid division by zero
     if (fabs(yawd) > 0.001) {
         px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
         py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
     }
     else {
         px_p = p_x + v*delta_t*cos(yaw);
         py_p = p_y + v*delta_t*sin(yaw);
     }

     double v_p = v;
     double yaw_p = yaw + yawd*delta_t;
     double yawd_p = yawd;

     //add noise
     px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
     py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
     v_p = v_p + nu_a*delta_t;

     yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
     yawd_p = yawd_p + nu_yawdd*delta_t;

     //write predicted sigma point into right column
     Xsig_pred_(0,i) = px_p;
     Xsig_pred_(1,i) = py_p;
     Xsig_pred_(2,i) = v_p;
     Xsig_pred_(3,i) = yaw_p;
     Xsig_pred_(4,i) = yawd_p;
    //  cout << "vp= "<<v_p<<endl;
   }



  //PREDICT MEAN AND COVARIANCE

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  //set weights -- probably don't need to recalculate this every time..
  double weight_0 = lambda_aug/(lambda_aug+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1;i++){
    double weight = 0.5/(n_aug_+lambda_aug);
    weights(i) = weight;
  }


  //predict state mean -- DO WE NEED TO MAKE ANOTHER x_ and P_ .. probably not
  x_.fill(0.0);

  for (int i=0; i<2*n_aug_+1; i++){
      x_ = x_ + weights(i)*Xsig_pred_.col(i);
  }
  //cout << "x_pred_ " << x_ <<endl;
  //predict state covariance matrix
  P_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++){
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      x_diff(3) = constrainAngle(x_diff(3));

      P_ = P_ + weights(i)*x_diff*x_diff.transpose();
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
  //set state dimension -redundant with n_x_
  int n_x = 5;

  //set augmented dimension -redundant with n_aug_
  int n_aug = 7;

  //set measurement dimension, lidar can measure px,py
  int n_z = 2;

  //define spreading parameter
  double lambda = 3 - n_aug_;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);

  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

//mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  MatrixXd R = MatrixXd(n_z,n_z);

  Zsig.fill(0.0);
  //transform sigma points into measurement space -- not much required for a laser
  for (int i=0;i<2*n_aug+1;i++){
    double px = Xsig_pred_.col(i)(0);
    double py = Xsig_pred_.col(i)(1);
    Zsig.col(i)(0) = px;
    Zsig.col(i)(1) = py;
  }
  //std::cout<<Zsig<<std::endl;
  //calculate mean predicted measurement

  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++){
      z_pred = z_pred + weights(i)*Zsig.col(i);
  }
  //calculate measurement covariance matrix S

  R.fill(0.0);
  R(0,0) = pow(std_laspx_,2);
  R(1,1) = pow(std_laspy_,2);


  S.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights(i)*z_diff * z_diff.transpose();
    //S = S + weights(i)*(Zsig.col(i)-z_pred)*(Zsig.col(i)-z_pred).transpose();
  }

  S = S + R;

  // UPDATE-----------------

  //create vector for incoming measurement
  VectorXd z = VectorXd(n_z);

  // load the measurements
  z = meas_package.raw_measurements_;
  //----LOAD measurements

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++){

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = constrainAngle(x_diff(3));

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    //Tc = Tc + weights(i)*(Xsig_pred_.col(i)-x_)*(Zsig.col(i)-z).transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  //std::cout << K << std::endl;

  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;

  x_ = x_ + K*z_diff;

  P_ = P_ - K * S * K.transpose();

  //Calculate NIS
  NIS_laser_ = z_diff.transpose()*S.inverse()*z_diff;
}

/** ----------------------------------------------------------------------------
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //set state dimension -redundant with n_x_
  int n_x = 5;

  //set augmented dimension -redundant with n_aug_
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);

  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  MatrixXd R = MatrixXd(n_z,n_z);

  Zsig.fill(0.0);
  //transform sigma points into measurement space
  for (int i=0;i<2*n_aug+1;i++){
    // float px = Xsig_pred_.col(i)(0);
    // float py = Xsig_pred_.col(i)(1);
    // float v = Xsig_pred_.col(i)(2);
    // float psi = Xsig_pred_.col(i)(3);
    //
    // float rho = sqrt(pow(px,2)+pow(py,2));
    // float phi = atan2(py,px);
    // float rhod = (px*cos(psi)*v + py*sin(psi)*v)/rho;
    // Zsig.col(i)(0) = rho;
    // Zsig.col(i)(1) = phi;
    // Zsig.col(i)(2) = rhod;

    // extract values for better readibility -- Classroom for sanity check
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    if (p_x == 0 && p_y == 0) {
      Zsig(0,i) = 0;
      Zsig(1,i) = 0;
      Zsig(2,i) = 0;
    } else {
      Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
      Zsig(1,i) = atan2(p_y,p_x);                                 //phi
      Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }
  }
  //std::cout<<Zsig<<std::endl;
  //calculate mean predicted measurement

  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++){
      z_pred = z_pred + weights(i)*Zsig.col(i);
  }
  //calculate measurement covariance matrix S

  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  // R.fill(0.0);
  // R(0,0) = pow(std_radr_,2);
  // R(1,1) = pow(std_radphi_,2);
  // R(2,2) = pow(std_radrd_,2);

  S.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = constrainAngle(z_diff(1));

    S = S + weights(i)*z_diff * z_diff.transpose();
  }

  S = S + R;


  // UPDATE-----------------

  //create vector for incoming measurement
  VectorXd z = VectorXd(n_z);

  // load the measurements
  z = meas_package.raw_measurements_;
  //----LOAD measurements

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++){
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = constrainAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = constrainAngle(x_diff(3));

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    //  Tc = Tc + weights(i)*(Xsig_pred_.col(i)-x_)*(Zsig.col(i)-z).transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc  *S.inverse();
  //std::cout << K << std::endl;

  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = constrainAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;

  P_ = P_ - K * S * K.transpose();

  //Calculate NIS
  NIS_radar_ = z_diff.transpose()*S.inverse()*z_diff;
}
