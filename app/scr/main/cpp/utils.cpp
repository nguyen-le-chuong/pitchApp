#include "utils.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Core>

double wrapAngle(double angle)
{
	angle = fmod(angle, (2.0*M_PI));
	if (angle <= -M_PI){angle += (2.0*M_PI);}
	else if (angle > M_PI){angle -= (2.0*M_PI);}
	return angle;
}

double calculateMean(const std::vector<double>& dataset)
{
    if (dataset.empty()){return NAN;}
    double sum = std::accumulate(std::begin(dataset), std::end(dataset), 0.0);
    double mean =  sum / dataset.size();
    return mean;
}

double calculateRMSE(const std::vector<double>& dataset)
{
    double accum = 0.0;
    if (dataset.empty()){return 0.0;}
    std::for_each (std::begin(dataset), std::end(dataset), [&](const double d) {accum += (d * d);});
    double rmse = sqrt(accum / (dataset.size()));
    return rmse;
}


