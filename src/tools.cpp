#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

using namespace std;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  VectorXd se(4);
  rmse << 0,0,0,0;
  se << 0,0,0,0;

    // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
      || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
        VectorXd e =(estimations[i] - ground_truth[i]);
        VectorXd ee = e.array()*e.array();
        se += ee;

  }

  //calculate the mean
  VectorXd mean = 1.0/estimations.size() * se.array();

  //calculate the squared root
  // ... your code here
    rmse = mean.array().sqrt();
  //return the result
  return rmse;
}
