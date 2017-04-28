#ifndef CONVENTIONALAXXBSVDSOLVER_H
#define CONVENTIONALAXXBSVDSOLVER_H
#include"axxbsolver.h"

// This is a SVD based solution of conventional hand-eye calibration problem.
// This is firstly proposed by Andreff et al. in "On-line hand-eye calibration".
class ConventionalAXXBSVDSolver:public AXXBSolver
{
public:
    ConventionalAXXBSVDSolver(const Poses A, const Poses B):AXXBSolver(A,B){}
    Pose SolveX();
};

#endif // CONVENTIONALAXXBSVDSOLVER_H
