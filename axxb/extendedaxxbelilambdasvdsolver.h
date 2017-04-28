#ifndef EXTENDEDAXXBELILAMBDASVDSOLVER_H
#define EXTENDEDAXXBELILAMBDASVDSOLVER_H

#include"axxbsolver.h"
class ExtendedAXXBEliLambdaSVDSolver:public AXXBSolver
{
public:
    ExtendedAXXBEliLambdaSVDSolver(const Poses A, const Poses B):AXXBSolver(A,B){}
    Pose SolveX();
};

#endif // EXTENDEDAXXBELILAMBDASVDSOLVER_H
