//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: p_fun.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 27-Jul-2021 21:06:12
//
#ifndef P_FUN_H
#define P_FUN_H

// Include Files
//#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
// Function Declarations
extern void p_fun(const double in1[4], const double in2[2], double p_FK[3]);

#endif

int main(){
  double q[4] = {0.3, 0.4, 0.3, 0.5};
  double L[2] = {0.125, 0.125};
  double p[3];
  
  p_fun(q,L,p);
  Eigen::VectorXd P_EE = Eigen::VectorXd::Zero(3);
  for (int i = 0; i < 3; i++){
      P_EE(i) = p[i];
    }
  std::cout << P_EE << std::endl;
}
//
// File trailer for p_fun.h
//
// [EOF]
//
