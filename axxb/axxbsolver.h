#ifndef AXXBSOLVER_H
#define AXXBSOLVER_H

#include<Eigen/Core>
#include"type.h"
#include<glog/logging.h>

//used for hand eye calibration
class AXXBSolver
{
public:
  AXXBSolver();
  AXXBSolver(const Poses A, const Poses B):A_(A),B_(B)
  {
    CHECK(A_.size()==B_.size())<<"two sizes should be equal";
    CHECK(A_.size()>=2)<<"at least two motions are needed";
  }

  virtual Pose SolveX()=0;

  Poses A_;
  Poses B_;
};

#endif // AXXBSOLVER_H
