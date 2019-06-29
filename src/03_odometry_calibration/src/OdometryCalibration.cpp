#include <odometry_calibration/OdometryCalibration.h>
#include <cmath>

namespace odometry_calibration {

/**
 * \brief Computes the odometry error function.
 * \param[in] groundTruth The ground truth odometry measured by an external sensor (called u* in the slides).
 * \param[in] observation The odometry measurement observed by the robot itself, e.g., using wheel sensors or joint angles (called u in the slides).
 * \param[in] calibrationMatrix The calibration matrix of the current iteration (called X in the slides).
 * \return The error function vector (called e in the slides).
 */
Eigen::Vector3d OdometryCalibration::errorFunction(const Odometry& groundTruth, const Odometry& observation, const Eigen::Matrix3d& calibrationMatrix) {
	Eigen::Vector3d error;
	//TODO: Compute the error vector.
        Eigen::Vector3d gt( groundTruth.ux, groundTruth.uy, groundTruth.utheta);
        Eigen::Vector3d ob( observation.ux, observation.uy, observation.utheta);
        error = gt - calibrationMatrix * ob; 
	return error;
}

/**
 * \brief Computes the Jacobian (matrix derivative) of the error function for a given odometry measurement.
 * \param[in] measurement The odometry measured by the robot (called u in the slides).
 * \return The Jacobian matrix of the error function.
 */
Eigen::Matrix3Xd OdometryCalibration::jacobian(const Odometry& observation) {
	Eigen::Matrix3Xd jacobian(3, 9);
        jacobian = Eigen::Matrix3Xd::Zero(3,9);
	//TODO: Calculate the 3x9 Jacobian matrix of the error function for the given observation.
	// Formula for jacobian from the observation as per the slides
        jacobian(0,0) = -observation.ux;
        jacobian(0,1) = -observation.uy;
        jacobian(0,2) = -observation.utheta;
        jacobian(1,3) = -observation.ux;
        jacobian(1,4) = -observation.uy;
        jacobian(1,5) = -observation.utheta;
        jacobian(2,6) = -observation.ux;
        jacobian(2,7) = -observation.uy;
        jacobian(2,8) = -observation.utheta;
        return jacobian;
}

/**
 * \brief Calibrates the odometry of a robot.
 * \param[in] measurementData A vector containing ground truth and observed odometry measurements.
 * ŗeturn The calibration matrix that can be used to correct the odometry.
 */
Eigen::Matrix3d OdometryCalibration::calibrateOdometry(const std::vector<MeasurementData>& measurements) {
	Eigen::Matrix3d calibrationMatrix = Eigen::Matrix3d::Identity();
	/** TODO: Calculate the calibration matrix. The steps to do are:
	 * - Start with an arbitrary initial calibration matrix
	 * - Iterate through the calibration data
	 * - Compute the error function and Jacobian for each data set
	 * - Accumulate the linear system components H and b
	 * - Solve the linear system
	 * - Update the calibration matrix
	 */
        
        // Only one iteration
        Eigen::VectorXd bt(9); 
        bt = Eigen::VectorXd::Zero(9);
        Eigen::MatrixXd H(9,9); 
        H = Eigen::MatrixXd::Zero(9,9);
        Eigen::VectorXd delx(9);
        for( std::vector<MeasurementData>::const_iterator itr = measurements.begin(); itr != measurements.end(); ++itr)
        {
            Eigen::Vector3d e = OdometryCalibration::errorFunction(itr->groundTruth, itr->uncalibrated, calibrationMatrix);
            Eigen::Matrix3Xd j = OdometryCalibration::jacobian(itr->uncalibrated);
            bt += e.transpose() * calibrationMatrix * j;
            H  += j.transpose() * calibrationMatrix * j;
        }
        
        // solve the linear equation
        delx = - H.inverse() * bt;
        // update the calibration matrix
        calibrationMatrix(0,0) += delx(0);
        calibrationMatrix(0,1) += delx(1);
        calibrationMatrix(0,2) += delx(2);
        calibrationMatrix(1,0) += delx(3);
        calibrationMatrix(1,1) += delx(4);
        calibrationMatrix(1,2) += delx(5);
        calibrationMatrix(2,0) += delx(6);
        calibrationMatrix(2,1) += delx(7);
        calibrationMatrix(2,2) += delx(8);
	
	return calibrationMatrix;
}

/**
 * \brief Applies the calibration matrix to an odometry measurement in order to get a corrected estimate.
 * \param[in] uncalibratedOdometry An uncalibrated odometry measurement (called u in the slides).
 * \param[in] calibrationMatrix The calibration matrix computed by calibrateOdometry in a previous step (called X in the slides).
 * \return The corrected odometry estimate.
 */
Odometry OdometryCalibration::applyOdometryCorrection(const Odometry& uncalibratedOdometry, const Eigen::Matrix3d& calibrationMatrix) {
	Odometry calibratedOdometry;
	/*TODO: Given the calibration matrix, return the corrected odometry measurement so that the robot has
	 * a better estimate of the location where it is currently.
	 */
        Eigen::Vector3d ob( uncalibratedOdometry.ux, uncalibratedOdometry.uy, uncalibratedOdometry.utheta);
        Eigen::Vector3d calibrated;
        calibrated = calibrationMatrix * ob;
        calibratedOdometry.ux = calibrated(0);
        calibratedOdometry.uy = calibrated(1);
        calibratedOdometry.utheta = calibrated(2);        
	return calibratedOdometry;
}

/**
 * \brief Converts an odometry reading into an affine 2D transformation.
 * \param[in] odometry The odometry reading.
 * \returns The corresponding affine transformation as a 3x3 matrix.
 */
Eigen::Matrix3d OdometryCalibration::odometryToAffineTransformation(const Odometry& odometry) {
	Eigen::Matrix3d transformation;
	//TODO: Convert the odometry measurement to an affine transformation matrix.
        transformation(0,0) = cos(odometry.utheta);
        transformation(0,1) = -sin(odometry.utheta);
        transformation(0,2) = odometry.ux;
        transformation(1,0) = sin(odometry.utheta);
        transformation(1,1) = cos(odometry.utheta);
        transformation(1,2) = odometry.uy;
        transformation(2,0) = 0.0;
        transformation(2,1) = 0.0;
        transformation(2,2) = 1.0;
        
	return transformation;
}

/**
 * \brief Converts an affine 2D transformation matrix into a 2D robot pose (x, y, and theta).
 * \param[in] transformation An affine transformation as a 3x3 matrix.
 * \returns The corresponding 2D pose (x, y, and theta).
 */
Pose2D OdometryCalibration::affineTransformationToPose(const Eigen::Matrix3d& transformation) {
	Pose2D pose;
	/* TODO: replace the following lines by the x and y position and the rotation of the robot.
	 * Hint: x and y can be directly read from the matrix. To get the rotation, use the acos/asin
	 * functions on the rotation matrix and take extra care of the sign, or (better) use the
	 * atan2 function.
	 */
	pose.x = transformation(0,2);
	pose.y = transformation(1,2);
	pose.theta = acos(transformation(1,1));
	return pose;
}

/**
 * \brief Calculate the robot's trajectory in Cartesian coordinates given a list of calibrated odometry measurements.
 * \param[in] calibratedOdometry Odometry measurements that have already been corrected using the applyOdometryCorrection method.
 * \returns A vector of 2D poses in the global coordinate system.
 */
std::vector<Pose2D> OdometryCalibration::calculateTrajectory(const std::vector<Odometry>& calibratedOdometry) {
	std::vector<Pose2D> trajectory;
	/* TODO: Compute the trajectory of the robot.
	 * - Start at the position x = 0, y = 0, theta = 0. (Do not add this point to the trajectory).
	 * - Iterate through the odometry measurements.
	 * - Convert each odometry measurement to an affine transformation using the
	 *   odometryToAffineTransformation method from above.
	 * - Chain the affine transformation to get the next pose.
	 * - Convert the affine transformation back to a 2D pose using the
	 *   affineTransformationToPose method from above.
	 * - Store the pose in the trajectory vector.
	 */    
        Pose2D pose;
        for( std::vector<Odometry>::const_iterator itr = calibratedOdometry.begin(); itr != calibratedOdometry.end(); ++itr)
        {
            Eigen::Vector3d p(pose.x,pose.y,1.0);
            Eigen::Matrix3d t = OdometryCalibration::odometryToAffineTransformation(*itr) ;
            p = t*p;
            Pose2D temp = OdometryCalibration::affineTransformationToPose(t);
        
            pose.x = p(0);
            pose.y = p(1);
            pose.theta = temp.theta;
            trajectory.push_back(pose);

        }
	return trajectory;
}




} /* namespace odometry_calibration */
