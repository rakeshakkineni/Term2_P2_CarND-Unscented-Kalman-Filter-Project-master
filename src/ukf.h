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
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_pred_;

  ///* state covariance matrix
  MatrixXd P_pred_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
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

  ///* Sigma point spreading parameter
  double lambda_;

  //laser NIS
  double laser_nis_;

  //radar NIS
  double radar_nis_;
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
  void Prediction(double delta_t);

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

  /**
   * Generates the augmeneted sigma points.
   * @param {MatrixXd} Xsig_out : Augmented sigma points
   */
  void AugmentedSigmaPoints(MatrixXd* Xsig_out);

  /**
   * Transforms the augmeneted sigma points using the process equations.
   * @param {MatrixXd*} aug : Augmented sigma points
   * 		  {double }	delta_t: Time difference
   */
  void SigmaPointPrediction(MatrixXd* Xsig_aug, double delta_t);

  /**
   * Calculate the mean and covariance using the augmented sigma points.
   * @param void
   */
  void PredictMeanAndCovariance(void);

  /**
   * Predict the Radar measurement before updating with new measurements.
   * @param {VectorXd*} z_out:mean predicted measurement
   * 		  {MatrixXd*} z_sig:transformed sigma points into measurement space
   * 		  {MatrixXd*} S_out:innovation covariance matrix S
   */
  void PredictRadarMeasurement(VectorXd* z_out,MatrixXd* z_sig, MatrixXd* S_out);

  /**
   * Update Radar measurements with new measurements.
   * @param {VectorXd*} z_pred:mean predicted measurement
   * 		  {MatrixXd*} Zsig:transformed sigma points into measurement space
   * 		  {MatrixXd*} S:innovation covariance matrix S
   * 		  {MeasurementPackage*} meas_package:New measurement points
   */
  void UpdateState(VectorXd &z_pred, MatrixXd &Zsig,MatrixXd &S, MeasurementPackage meas_package);

  /**
   * Predict the Lidar measurement before updating with new measurements.
   * @param {VectorXd*} z_out:mean predicted measurement
   * 		  {MatrixXd*} z_sig:transformed sigma points into measurement space
   * 		  {MatrixXd*} S_out:innovation covariance matrix S
   */
  void PredictLidarMeasurement(VectorXd* z_out,MatrixXd* z_sig, MatrixXd* S_out);


  /**
   * Update Lidar measurements with new measurements.
   * @param {VectorXd*} z_pred:mean predicted measurement
   * 		  {MatrixXd*} Zsig:transformed sigma points into measurement space
   * 		  {MatrixXd*} S:innovation covariance matrix S
   * 		  {MeasurementPackage*} meas_package:New measurement points
   */
  void UpdateStateLidar(VectorXd &z_pred, MatrixXd &Zsig,MatrixXd &S, MeasurementPackage meas_package);
};

#endif /* UKF_H */
