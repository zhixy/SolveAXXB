#include "extendedaxxbelilambdasvdsolver.h"
#include<Eigen/SVD>
#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/LU>
#include <Eigen/QR>
#include <stdlib.h>
#include<fstream>
#include<cmath>
#include"axxb_utils.h"

Pose ExtendedAXXBEliLambdaSVDSolver::SolveX()
{

  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(12*A_.size(),12);
  for(int i=0;i<A_.size();i++)
  {
    //extract R,t from homogophy matrix
    Eigen::Matrix3d Ra = A_[i].topLeftCorner(3,3);
    Eigen::Vector3d Ta = A_[i].topRightCorner(3,1);
    Eigen::Matrix3d Rb = B_[i].topLeftCorner(3,3);
    Eigen::Vector3d Tb = B_[i].topRightCorner(3,1);

    m.block<9,9>(12*i,0) = Eigen::MatrixXd::Identity(9,9) - Eigen::kroneckerProduct(Ra,Rb);
    Eigen::Matrix3d Ta_skew = skew(Ta);
    m.block<3,9>(12*i+9,0) = Eigen::kroneckerProduct(Ta_skew,Tb.transpose());
    m.block<3,3>(12*i+9,9) = Ta_skew - Ta_skew*Ra;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd( m, Eigen::ComputeFullV | Eigen::ComputeFullU );
  CHECK(svd.computeV())<<"fail to compute V";

  Eigen::Matrix3d R_alpha;
  R_alpha.row(0) = svd.matrixV().block<3,1>(0,11).transpose();
  R_alpha.row(1) = svd.matrixV().block<3,1>(3,11).transpose();
  R_alpha.row(2) = svd.matrixV().block<3,1>(6,11).transpose();
  //double a = std::fabs(R_alpha.determinant());
  //double alpha = R_alpha.determinant()/(pow(std::fabs(R_alpha.determinant()),4./3.));
  double det = R_alpha.determinant();
  double alpha = std::pow(std::abs(det),4./3.)/det;
  Eigen::HouseholderQR<Eigen::Matrix3d> qr(R_alpha/alpha);

  Pose handeyetransformation = Pose::Identity(4,4);
  Eigen::Matrix3d Q = qr.householderQ();
  Eigen::Matrix3d Rwithscale = alpha*Q.transpose()*R_alpha;
  Eigen::Vector3d R_diagonal = Rwithscale.diagonal();
  for(int i=0;i<3;i++)
  {
    handeyetransformation.block<3,1>(0,i) = int(R_diagonal(i)>=0?1:-1)*Q.col(i);
  }

  handeyetransformation.topRightCorner(3,1) = svd.matrixV().block<3,1>(9,11)/alpha;
  return handeyetransformation;
}
