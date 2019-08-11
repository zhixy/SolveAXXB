#include "conventionalaxxbsvdsolver.h"
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/LU>
#include <Eigen/QR>
#include <stdlib.h>
#include "axxb_utils.h"
#include <fstream>

Pose ConventionalAXXBSVDSolver::SolveX()
{
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(12*A_.size(),12);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(12*A_.size());
  for(int i=0;i<A_.size();i++)
  {
    //extract R,t from homogophy matrix
    Eigen::Matrix3d Ra = A_[i].topLeftCorner(3,3);
    Eigen::Vector3d Ta = A_[i].topRightCorner(3,1);
    Eigen::Matrix3d Rb = B_[i].topLeftCorner(3,3);
    Eigen::Vector3d Tb = B_[i].topRightCorner(3,1);

    m.block<9,9>(12*i,0) = Eigen::MatrixXd::Identity(9,9) - Eigen::kroneckerProduct(Ra,Rb);
    Eigen::Matrix3d Ta_skew = skew(Ta);
    m.block<3,9>(12*i+9,0) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(3,3),Tb.transpose());
    m.block<3,3>(12*i+9,9) = Eigen::MatrixXd::Identity(3,3) - Ra;
    b.block<3,1>(12*i+9,0) = Ta;
  }

  Eigen::Matrix<double, 12, 1> x = m.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  Eigen::Matrix3d R = Eigen::Map<Eigen::Matrix3d>(x.data());

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Pose handeyetransformation = Pose::Identity(4,4);
  handeyetransformation.topLeftCorner(3,3) = svd.matrixU() * svd.matrixV().transpose();
  handeyetransformation.topRightCorner(3,1) = x.block<3,1>(9,0);
  return handeyetransformation;
}
