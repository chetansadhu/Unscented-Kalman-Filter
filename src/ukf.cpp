#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>


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

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.25;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.25;
  
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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // It is true after the first measurement
  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_ + 1);
  for (int i = 0; i < 2*n_aug_+1; ++i) {
	  if (i == 0) weights_(i) = lambda_/(lambda_ + n_aug_);
	  else weights_(i) = 0.5/(lambda_ + n_aug_);
  }

  // time when the state is true, in us
  time_us_ = 0;

  // Predicting the sigma points
  Xsig_pred_ = MatrixXd(5, 15);

  // Logger initialization
  const char *filename = "log_file.log";
  logger_ = new Logger(filename);

  logger_->LOG(INFO, __FUNCTION__, "Initialization completed");
#ifdef _DEBUG
  char msg[1024];
  sprintf(msg, "Initializations\nWeights:\n");
  std::stringstream ss;
  ss << weights_;
  sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
  logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif
}

UKF::~UKF() {
	if (logger_) {
		delete logger_;
		logger_ = NULL;
	}
}

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
	if (!is_initialized_) {
		P_ << 1, 0, 0, 0, 0,
				0, 1, 0, 0, 0,
				0, 0, 1, 0, 0,
				0, 0, 0, 1, 0,
				0, 0, 0, 0, 1;
		if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			double rho = meas_package.raw_measurements_[0];
			double phi = meas_package.raw_measurements_[1];
			double rhodot = meas_package.raw_measurements_[2];
			x_ << rho*cos(phi), rho*sin(phi), 0, 0, 0;
		}
		else return;
		time_us_ = meas_package.timestamp_;
		is_initialized_ = true;
		logger_->LOG(INFO, __FUNCTION__, "Initialized the measurement");
#ifdef _DEBUG
		char msg[10240];
		sprintf(msg, "Initialized state matrix\nx_:\n");
		std::stringstream ss;
		ss << x_;
		sprintf(msg + strlen(msg), "%s\n", ss.str().c_str());
		logger_->LOG(DEBUG, __FUNCTION__, msg);
		memset(msg, '\0', sizeof(msg));
		ss.str("");
		sprintf(msg, "Initialized state covariance matrix\nP_:\n");
		ss << P_;
		sprintf(msg + strlen(msg), "%s\n", ss.str().c_str());
		logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif
		return;
	}
	double dt = double(meas_package.timestamp_ - time_us_)/1000000.0;
	time_us_ = meas_package.timestamp_;

	// Prediction
	Prediction(dt);

	// Update step
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		UpdateRadar(meas_package);
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
		UpdateLidar(meas_package);
	}
	return;
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
	// Augmented state vector
	VectorXd x_aug = VectorXd(n_aug_);
	x_aug.head(5) = x_;
	x_aug(5) = x_aug(6) = 0;
	logger_->LOG(INFO, __FUNCTION__, "Created augmented state vector");
#ifdef _DEBUG
	char msg[10240];
	sprintf(msg, "Augmented state vector\nx_aug:\n");
	std::stringstream ss;
	ss << x_aug;
	sprintf(msg + strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Augmented state covariance matrix
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
	P_aug.fill(0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;
	logger_->LOG(INFO, __FUNCTION__, "Created augmented state covariance matrix");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Augmented state covariance matrix\nP_aug:\n");
	ss << P_aug;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif


	// Generating augmented sigma points
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);
	MatrixXd L = P_aug.llt().matrixL();
	Xsig_aug.fill(0);
	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i < n_aug_; ++i) {
		Xsig_aug.col(i + 1) = 			x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(n_aug_ + i + 1) = 	x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
	}
	logger_->LOG(INFO, __FUNCTION__, "Created augmented sigma points");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Augmented sigma points\nXsig_aug:\n");
	ss << Xsig_aug;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Sigma point prediction
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		const double px = Xsig_aug(0, i);
		const double py = Xsig_aug(1, i);
		const double v = Xsig_aug(2, i);
		const double yaw = Xsig_aug(3, i);
		const double yawd = Xsig_aug(4, i);
		const double nu_a = Xsig_aug(5, i);
		const double nu_yawd = Xsig_aug(6, i);
		if (fabs(yawd) > 0.001) {
			Xsig_pred_(0, i) = px + (v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw))) +
					(0.5 * delta_t * delta_t * cos(yaw) * nu_a);
			Xsig_pred_(1, i) = py + (v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t))) +
					(0.5 * delta_t * delta_t * sin(yaw) * nu_a);
		}
		else {
			Xsig_pred_(0, i) = px + (v * cos(yaw) * delta_t) + (0.5 * delta_t * delta_t * cos(yaw) * nu_a);
			Xsig_pred_(1, i) = py + (v * sin(yaw) * delta_t) + (0.5 * delta_t * delta_t * sin(yaw) * nu_a);
		}
		Xsig_pred_(2, i) = v + delta_t * nu_a;
		Xsig_pred_(3, i) = yaw + (yawd * delta_t) + (0.5 * delta_t * delta_t * nu_yawd);
		Xsig_pred_(4, i) = yawd + delta_t * nu_yawd;
	}
	logger_->LOG(INFO, __FUNCTION__, "Predicted sigma points");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Sigma points prediction\nXsig_pred:\n");
	ss << Xsig_pred_;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// State mean prediction
	x_.fill(0);
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		x_ += weights_(i) * Xsig_pred_.col(i);
	}

	logger_->LOG(INFO, __FUNCTION__, "Predicted state mean");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "State mean prediction\nx_{k+1|k}:\n");
	ss << x_;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// State covariance prediction
	P_.fill(0);
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
		while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;
		P_ += weights_(i) * x_diff * x_diff.transpose();
	}
	logger_->LOG(INFO, __FUNCTION__, "Predicted state covariance");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "State covariance prediction\nP_{k+1|k}:\n");
	ss << P_;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif
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

	const int n_z = 2;

	// Sigma points transformation to measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		Zsig(0, i) = Xsig_pred_(0, i);
		Zsig(1, i) = Xsig_pred_(1, i);
	}
	logger_->LOG(INFO, __FUNCTION__, "Transformed sigma points to measurement space");
#ifdef _DEBUG
	char msg[10240];
	memset(msg, '\0', sizeof(msg));
	sprintf(msg, "Sigma points transformation to measurement space\nZsig:\n");
	std::stringstream ss;
	ss << Zsig;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0);
	for (int i = 0; i < 2*n_aug_+1; i++) {
		z_pred += weights_(i) * Zsig.col(i);
	}
	logger_->LOG(INFO, __FUNCTION__, "predicted measurement mean");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Mean predicted measurement\nz_{k+1|k}:\n");
	ss << z_pred;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Innovation covariance matrix
	MatrixXd S = MatrixXd(n_z, n_z);
	MatrixXd R = MatrixXd(n_z, n_z);
	R.fill(0);
	R(0, 0) = std_laspx_ * std_laspx_;
	R(1, 1) = std_laspy_ * std_laspy_;
	logger_->LOG(INFO, __FUNCTION__, "Initialized the measurement noise covariance");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Measurement noise covariance\nR:\n");
	ss << R;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif
	S.fill(0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		S += weights_(i) * z_diff * z_diff.transpose();
	}
	S += R;
	logger_->LOG(INFO, __FUNCTION__, "Calculated the innovation covariance matrix");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Innovation covariance matrix\nS:\n");
	ss << S;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Cross correlation matrix
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		while (x_diff(3) > M_PI)
			x_diff(3) -= 2.*M_PI;
		while (x_diff(3) < -M_PI)
			x_diff(3) += 2.*M_PI;
		VectorXd z_diff = Zsig.col(i) - z_pred;
		Tc += weights_(i) * x_diff * z_diff.transpose();
	}
	logger_->LOG(INFO, __FUNCTION__, "Calculated cross correlation matrix");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Cross correlation matrix\nT_{k+1|k}:\n");
	ss << Tc;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Calculate Kalman Gain
	MatrixXd K = Tc * S.inverse();
	logger_->LOG(INFO, __FUNCTION__, "Calculated kalman gain");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Kalman gain\nK_{k+1|k}:\n");
	ss << K;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	//Update state
	x_ = x_ + K * (meas_package.raw_measurements_ - z_pred);

	// Update state covariance matrix
	P_ = P_ - K * S * K.transpose();

	logger_->LOG(INFO, __FUNCTION__, "Updated state and state covariance matrix");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Updated state\nx_{k+1|k+1}:\n");
	ss << x_;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Updated state covariance matrix\nx_{k+1|k+1}:\n");
	ss << P_;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Building NIS
	VectorXd z_diff = (meas_package.raw_measurements_ - z_pred);
	double e = z_diff.transpose() * S.inverse() * z_diff;
	std::ofstream f("lidar_nis.txt", std::ios_base::app);
	f << e << "\n";
	f.close();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  You'll also need to calculate the radar NIS.
  */

	// Predict measurement
	const int n_z = 3;
	// Sigma points transformation to measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		double px = Xsig_pred_(0, i);
		double py = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		Zsig(0, i) = sqrt(px*px + py*py);
		if (px == 0 && py == 0) {
			logger_->LOG(WARN, __FUNCTION__, "Handled px and py both 0 by setting phi of radar measurement to 0");
			Zsig(1, i) = 0;
		}
		else {
			Zsig(1, i) = atan2(py, px);
		}
		Zsig(2, i) = (px*v*cos(yaw) + py*v*sin(yaw))/Zsig(0, i);
	}

	logger_->LOG(INFO, __FUNCTION__, "Transformed sigma points to measurement space");
#ifdef _DEBUG
	char msg[10240];
	memset(msg, '\0', sizeof(msg));
	sprintf(msg, "Sigma points transformation to measurement space\nZsig:\n");
	std::stringstream ss;
	ss << Zsig;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0);
	for (int i = 0; i < 2*n_aug_+1; i++) {
		z_pred += weights_(i) * Zsig.col(i);
	}
	logger_->LOG(INFO, __FUNCTION__, "predicted measurement mean");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Mean predicted measurement\nz_{k+1|k}:\n");
	ss << z_pred;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Innovation covariance matrix
	MatrixXd S = MatrixXd(n_z, n_z);
	MatrixXd R = MatrixXd(n_z, n_z);
	R.fill(0);
	R(0, 0) = std_radr_ * std_radr_;
	R(1, 1) = std_radphi_ * std_radphi_;
	R(2, 2) = std_radrd_ * std_radrd_;
	logger_->LOG(INFO, __FUNCTION__, "Initialized the measurement noise covariance");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Measurement noise covariance\nR:\n");
	ss << R;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif
	S.fill(0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
		S += weights_(i) * z_diff * z_diff.transpose();
	}
	S += R;
	logger_->LOG(INFO, __FUNCTION__, "Calculated the innovation covariance matrix");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Innovation covariance matrix\nS:\n");
	ss << S;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Cross correlation matrix
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		while (x_diff(3) > M_PI) x_diff(3) -= 2*M_PI;
		while (x_diff(3) < -M_PI) x_diff(3) += 2*M_PI;
		VectorXd z_diff = Zsig.col(i) - z_pred;
		while (z_diff(1) > M_PI) z_diff(1) -= 2*M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2*M_PI;
		Tc += weights_(i) * x_diff * z_diff.transpose();
	}
	logger_->LOG(INFO, __FUNCTION__, "Calculated cross correlation matrix");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Cross correlation matrix\nT_{k+1|k}:\n");
	ss << Tc;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Calculate Kalman Gain
	MatrixXd K = Tc * S.inverse();
	logger_->LOG(INFO, __FUNCTION__, "Calculated kalman gain");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Kalman gain\nK_{k+1|k}:\n");
	ss << K;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	//Update state
	x_ = x_ + K * (meas_package.raw_measurements_ - z_pred);

	// Update state covariance matrix
	P_ = P_ - K * S * K.transpose();

	logger_->LOG(INFO, __FUNCTION__, "Updated state and state covariance matrix");
#ifdef _DEBUG
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Updated state\nx_{k+1|k+1}:\n");
	ss << x_;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
	memset(msg, '\0', sizeof(msg));
	ss.str("");
	sprintf(msg, "Updated state covariance matrix\nx_{k+1|k+1}:\n");
	ss << P_;
	sprintf(msg+strlen(msg), "%s\n", ss.str().c_str());
	logger_->LOG(DEBUG, __FUNCTION__, msg);
#endif

	// Building NIS
	VectorXd z_diff = (meas_package.raw_measurements_ - z_pred);
	double e = z_diff.transpose() * S.inverse() * z_diff;
	std::ofstream f("radar_nis.txt", std::ios_base::app);
	f << e << "\n";
	f.close();
}
