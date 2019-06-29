#include <projective_geometry/ProjectiveGeometry.h>
#include <math.h>
#include <iostream>
namespace projective_geometry 
{
const double ProjectiveGeometry::PI = 3.141592654;

/**
 * \brief Converts a 3D Euclidean coordinates to homogeneous coordinates.
 * \param[in] 3D point in Euclidean coordinates.
 * \return 3D point in homogeneous coordinates.
 */
Eigen::Vector4d ProjectiveGeometry::euclideanToHomogeneous(const Eigen::Vector3d& point)
{
    Eigen::Vector4d result = Eigen::Vector4d::Zero();
    //TODO
    result(0) = point(0);
    result(1) = point(1);
    result(2) = point(2);
    result(3) = 1;

    return result;
}
/**
 * \brief Converts a 2D point in homogeneous coordinates into Euclidean coordinates.
 * \param[in] 2D point in homogeneous coordinates.
 * \return 2D point in Euclidean coordinates.
 */
Eigen::Vector2d ProjectiveGeometry::homogeneousToEuclidean(const Eigen::Vector3d& point)
{
    Eigen::Vector2d result = Eigen::Vector2d::Zero();
    //TODO
    result(0) = point(0)/point(2);
    result(1) = point(1)/point(2);
    return result;
}
/**
 * \brief Assigns the values of the camera's extrinsic and intrinsic parameters.
 * \param[in] alpha The camera's current rotation angle.
 * \return a struct 'cameraParameters' which contains the camera parameters.
 */
CameraParameters ProjectiveGeometry::setCameraParameters(const double alpha)
{
    CameraParameters results;
    //TODO
    results.xH = 400;
    results.yH = 300;
    results.m = 0.0025;
    results.c = 550;
   
    Eigen::Vector3d X(0.40,0.0,10.0);
    results.X0 = X;
    results.rotX = 0.0;
    results.rotY = alpha;
    results.rotZ = 0.0;
    return results;
}
/**
 * \brief Computes the calibration matrix based on the camera's intrinsic parameters.
 * \param[in] camera parameters (CameraParameters struct).
 * \return Calibration matrix.
 */
Eigen::Matrix3d ProjectiveGeometry::calibrationMatrix(const CameraParameters& param)
{
    Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
    //TODO
    result(0,0) = param.c;
    result(1,1) = (param.c)*(1+param.m);
    result(2,2) = 1.0;
    result(0,2) = param.xH;
    result(1,2) = param.yH;
    

    return result;
}

/**
 * \brief Computes the projection matrix based on the camera's parameters and the pre-computed calibration matrix.
 * \param[in] Calibration matrix.
 * \param[in] Camera parameters (cameraParameters struct).
 * \return Projection matrix.
 */
Eigen::MatrixXd ProjectiveGeometry::projectionMatrix(const Eigen::Matrix3d& calibrationMatrix, const CameraParameters& param)
{
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 4);
    //TODO
    
    Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(3, 3);
    rot(0,0) = cos(param.rotY);
    rot(0,1) = 0.0;
    rot(0,2) = sin(param.rotY);
    rot(1,0) = 0.0;
    rot(1,1) = 1.0;
    rot(1,2) = 0.0;
    rot(2,0) = -sin(param.rotY);
    rot(2,1) = 0.0;
    rot(2,2) = cos(param.rotY);
    
    Eigen::MatrixXd t = Eigen::MatrixXd::Zero(3, 4);
    t(0,0) = t(1,1) = t(2,2) = 1.0;
    t(0,3) = -param.X0(0);
    t(1,3) = -param.X0(1);
    t(2,3) = -param.X0(2);
    
    result = calibrationMatrix*rot*t;
    return result;
}
/**
 * \brief Applies the pre-computed projection matrix on a 3D point in Euclidean coordinates.
 * \param[in] 3D point in Euclidean coordinates.
 * \param[in] Projection matrix.
 * \return 2D point in Euclidean coordinates.
 */
Eigen::Vector2d ProjectiveGeometry::projectPoint(const Eigen::Vector3d& point, const Eigen::MatrixXd& projectionMatrix)
{
    Eigen::Vector2d result = Eigen::Vector2d::Zero();
    //TODO
    Eigen::Vector4d p = Eigen::Vector4d::Zero();
    p(0) = point(0);
    p(1) = point(1);
    p(2) = point(2);
    p(3) = 1.0;

    Eigen::Vector3d temp = projectionMatrix*p;
    result(0) = temp(0)/temp(2);
    result(1) = temp(1)/temp(2);
    return result;
}

/**
 * \brief Reprojects an image pixel to a 3D point on a given horizontal plane in Euclidean coordinates.
 * \param[in] imagePoint The image point in pixels where a feature (e.g., the corner of a chess board) was detected.
 * \param[in] calibrationMatrix The calibration matrix calculated above.
 * \param[in] param The intrinsic and extrinsic camera parameters.
 * \param[in] tableHeight The height of the table in front of the robot. The reprojected point will lie on that table.
 * \return 3D point in Eucldiean coordinates.
 */
Eigen::Vector3d ProjectiveGeometry::reprojectTo3D(const Eigen::Vector2d& imagePoint, 
                                                  const Eigen::Matrix3d& calibrationMatrix,
                                                  const CameraParameters& param, 
                                                  const double tableHeight)
{
    //Eigen::Vector3d coordinates3D = Eigen::Vector3d::Zero();
    // TODO
    Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(3, 3);
    rot(0,0) = cos(param.rotY);
    rot(0,1) = 0.0;
    rot(0,2) = sin(param.rotY);
    rot(1,0) = 0.0;
    rot(1,1) = 1.0;
    rot(1,2) = 0.0;
    rot(2,0) = -sin(param.rotY);
    rot(2,1) = 0.0;
    rot(2,2) = cos(param.rotY);
    
    
    Eigen::Vector3d x(imagePoint(0),imagePoint(1), 1.0);
    x = (calibrationMatrix*rot).inverse() * x;
    double W = x(2) / ( 1.0 - param.X0(2)/tableHeight );
    double U = x(0) + param.X0(0) *(W/tableHeight);
    double V = x(1) + param.X0(1) *(W/tableHeight);
    double T = W/tableHeight;
    Eigen::Vector3d coordinates3D(U/T,V/T,W/T);
    
    
    return coordinates3D;
}


}  // namespace projective_geometry
