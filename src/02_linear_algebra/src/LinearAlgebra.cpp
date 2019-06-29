#include <linear_algebra/LinearAlgebra.h>
#include <Eigen/Dense>
#include <cmath>

//using Eigen::MatrixXd;

namespace linear_algebra {

/**
 * This function should return the vector (2, 1, 3) as an Eigen vector.
 */
Eigen::Vector3d LinearAlgebra::vectorA() {
	Eigen::Vector3d result(2,1,3);

	// TODO: set the vector "result" to (2, 1, 3).
        return result;
}

/**
 * This function should return the vector (-1, -5, 2) as an Eigen vector.
 */
Eigen::Vector3d LinearAlgebra::vectorB() {
	Eigen::Vector3d result(-1, -5, 2);

	// TODO: set the vector "result" to (-1, -5, 2).
        return result;
}

Eigen::Matrix3d LinearAlgebra::matrixM() {
	Eigen::Matrix3d m;

	// TODO: fill in the matrix elements
         
         m(0,0) = 1;
         m(0,1) = 2;
         m(0,2) = 7;
         m(1,0) = 0;
         m(1,1) = 2;
         m(1,2) = 0;
         m(2,0) = 1;
         m(2,1) = 0;
         m(2,2) = -1;
	return m;
}

Eigen::Matrix3d LinearAlgebra::invMatrixM(const Eigen::Matrix3d& M) {
	Eigen::Matrix3d result = M.inverse();

	// TODO: return the inverse of matrix M

	return result;
}

Eigen::Matrix3d LinearAlgebra::transposeMatrixM(const Eigen::Matrix3d& M) {
	Eigen::Matrix3d result = M.transpose();

	// TODO: return the transpose of matrix M

	return result;
}

double LinearAlgebra::detOfMatrixM(const Eigen::Matrix3d& M)
{
	double result = M.determinant();

	// TODO: return the determinant of matrix M

	return result;
}

double LinearAlgebra::dotProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	double result = a.dot(b);

	// TODO: return the dot product of vectors a and b.

	return result;
}


bool LinearAlgebra::isLinearIndependent(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	bool result = false;

	/* TODO: test if the vectors a and b are linear independent.
	   Return true if they are independent, false if they are dependent.*/
        if((a[0]*b[1] - b[0]*a[1]) != 0 || (a[0]*b[2] - b[0]*a[2]) != 0 || (a[1]*b[2] - b[1]*a[2]) != 0)
        {
           return true;
        }
        else
        {
          return false;
        }
	return result;
        

     //Here the vectors are linearly dependent

}

Eigen::Vector3d LinearAlgebra::solveLinearSystem(const Eigen::Matrix3d& M, const Eigen::Vector3d& a) {
	/* Check why this works?
	 * Eigen::Vector3d res = M * a;
	 * Eigen::Vector3d result = M.inverse()*a;
	*/

	// TODO: Solve Mx = a for x and return x.
        Eigen::Vector3d result1 = M.colPivHouseholderQr().solve(a);

	return result1;
}

}  // namespace linear_algebra
