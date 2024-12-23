#ifndef READDATA_H
#define READDATA_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <ctime>
#include <iomanip>
#include <cstdlib>
#include <algorithm>
#include "Eigen/Dense"
// Function declarations
time_t parse_timestamp(const std::string &timestamp);

std::map<double, std::vector<double>> read_data(const std::string &filename);

std::set<double> extract_keys(const std::map<double, std::vector<double>> &data);

std::vector<std::tuple<double, Eigen::VectorXd, Eigen::VectorXd>> merge_data(
    const std::map<double, std::vector<double>> &dataA,
    const std::map<double, std::vector<double>> &dataG);

std::map<time_t, std::vector<double>> read_dataTimeStamp(const std::string &filename);

std::set<time_t> extract_keysTimeStamp(const std::map<time_t, std::vector<double>> &data);

std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merge_dataTimeStamp(
    const std::map<time_t, std::vector<double>> &dataA,
    const std::map<time_t, std::vector<double>> &dataG);

#endif // READDATA_H
