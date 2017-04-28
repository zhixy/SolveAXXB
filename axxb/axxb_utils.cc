#include "axxb_utils.h"

Eigen::Matrix3d skew(Eigen::Vector3d u)
{
  Eigen::Matrix3d u_hat = Eigen::MatrixXd::Zero(3,3);
  u_hat(0,1) = u(2);
  u_hat(1,0) = -u(2);
  u_hat(0,2) = -u(1);
  u_hat(2,0) = u(1);
  u_hat(1,2) = u(0);
  u_hat(2,1) = -u(0);

  return u_hat;
}
