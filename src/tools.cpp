#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse = VectorXd(4);
  rmse << 0,0,0,0; 
  if (estimations.size() != 0 && estimations.size()== ground_truth.size()){
        //accumulate squared residuals
        for(int i=0; i < estimations.size(); ++i){
              rmse(0) = rmse(0) + pow(estimations[i](0) - ground_truth[i](0) ,2);
              rmse(1) = rmse(1) + pow(estimations[i](1)- ground_truth[i](1) ,2);
              rmse(2) = rmse(2) + pow(estimations[i](2) - ground_truth[i](2) ,2);
              rmse(3) = rmse(3) + pow(estimations[i](3) - ground_truth[i](3) ,2);
        }
        //cout <<"Sum:" << rmse <<endl;
      
        //calculate the mean
        rmse(0) = pow (rmse(0)/estimations.size(), 0.5);
        rmse(1) = pow (rmse(1)/estimations.size(), 0.5);
        rmse(2) = pow (rmse(2)/estimations.size(), 0.5);
        rmse(3) = pow (rmse(3)/estimations.size(), 0.5);   
    }
    return rmse;
}