#include "readData.h"

// Placeholder for your timestamp parsing function
// time_t parse_timestamp(const std::string &timestamp) {
//     std::tm t = {};
//     std::istringstream ss(timestamp);
//     ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S"); // Adjust the format as needed
//     return std::mktime(&t);
// }

time_t parse_timestamp(const std::string &timestamp) {
    std::tm t = {}; // Initialize the tm structure to zero
    std::istringstream ss(timestamp); // Create a stringstream from the timestamp string
    ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S"); // Parse the timestamp using the given format
    return std::mktime(&t); // Convert tm structure to time_t
}
Eigen::VectorXd to_eigen_vector(const std::vector<double> &vec) {
    return Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());
}

// Function to read data from a CSV file
std::map<double, std::vector<double>> read_data(const std::string &filename) {
    std::map<double, std::vector<double>> data;
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;
            long double ts;

            bool is_first = true;
            while (std::getline(ss, token, ',')) {
                if (is_first) {
                    ts = std::round(std::stod(token) * 100.0) / 100.0; // Parse timestamp
                    // std::cout << ts << std::endl;
                    is_first = false;
                } else {
                    values.push_back(std::stod(token)); // Convert string to double
                }
            }
            // while (std::getline(ss, token, ',')) {
            // token.erase(remove(token.begin(), token.end(), '\"'), token.end());
            
            // if (is_first) {
            //     // std::cout << token << std::endl;
            //     ts = std::strtold(token.c_str(), nullptr);  // Parse timestamp
            //     is_first = false;
            // } else {
            //     values.push_back(std::stod(token)); // Convert string to double
            // }
            // }
            data[ts] = values;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << "\n";
    }

    return data;
}

std::map<time_t, std::vector<double>> read_dataTimeStamp(const std::string &filename) {
    std::map<time_t, std::vector<double>> data;
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;
            time_t ts;
            bool is_first = true;
            while (std::getline(ss, token, ',')) {
                // std::cout << token << std::endl;
                if (is_first) {
                    // token.erase(remove(token.begin(), token.end(), '\"'), token.end());
                    // std::cout << token << std::endl;
                    ts = static_cast<time_t>(std::stoll(token));; // Parse timestamp
                    // std::cout << ts << std::endl;
                    is_first = false;
                } else {
                    // token.erase(remove(token.begin(), token.end(), '\"'), token.end());
                    values.push_back(std::stod(token)); // Convert string to double
               
                }
            }
            data[ts] = values;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << "\n";
    }

    return data;
}

// Function to extract timestamps from a map of double to vector of doubles
std::set<double> extract_keys(const std::map<double, std::vector<double>> &data) {
    std::set<double> keys;
    for (const auto &entry : data) {
        keys.insert(entry.first);
    }
    return keys;
}

std::set<time_t> extract_keysTimeStamp(const std::map<time_t, std::vector<double>> &data) {
    std::set<time_t> keys;
    for (const auto &entry : data) {
        keys.insert(entry.first);
    }
    return keys;
}

// Function to merge data based on common timestamps
// std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merge_dataTimeStamp(
//     const std::map<time_t, std::vector<double>> &dataA,
//     const std::map<time_t, std::vector<double>> &dataG) {
    
//     std::set<time_t> keysA = extract_keysTimeStamp(dataA);
//     std::set<time_t> keysG = extract_keysTimeStamp(dataG);

//     // Find common timestamps
//     std::set<time_t> common_timestamps;
//     std::set_intersection(keysA.begin(), keysA.end(), keysG.begin(), keysG.end(),
//                           std::inserter(common_timestamps, common_timestamps.begin()));

//     // Merge data based on common timestamps
//     std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merged_data;
//     for (const auto &ts : common_timestamps) {
//         // std::cout << ts << std:: endl;
//         Eigen::VectorXd acc = to_eigen_vector(dataA.at(ts));
//         Eigen::VectorXd gyro = to_eigen_vector(dataG.at(ts));
//         merged_data.push_back(std::make_tuple(ts, acc, gyro));
//     }

//     return merged_data;
// }

std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merge_dataTimeStamp(
    const std::map<time_t, std::vector<double>>& dataA,
    const std::map<time_t, std::vector<double>>& dataG) {

    // Extract timestamps from each dataset
    std::set<time_t> keysA = extract_keysTimeStamp(dataA);
    std::set<time_t> keysG = extract_keysTimeStamp(dataG);

    // Create a vector to store the merged data
    std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merged_data;

    // Iterate through each accelerometer timestamp
    for (const auto& tsA : keysA) {
        // Find the closest gyroscope timestamp within ±10 ms
        auto lower = keysG.lower_bound(tsA - 10);
        auto upper = keysG.upper_bound(tsA + 10);
        // std::cout << tsA << " " <<  *lower << std::endl;
        time_t closest_tsG = -1;
        if (lower != keysG.end() && *lower <= tsA + 10) {
            closest_tsG = *lower;
        } else if (upper != keysG.begin()) {
            closest_tsG = *--upper;
        }
        // std::cout << closest_tsG << std::endl;
        // If a suitable gyroscope timestamp is found, add to merged data
        if (closest_tsG != -1 && std::abs(closest_tsG - tsA) <= 10) {
            Eigen::VectorXd acc = to_eigen_vector(dataA.at(tsA));
            Eigen::VectorXd gyro = to_eigen_vector(dataG.at(closest_tsG));
            // std::cout << "Matrix gyro:\n" << gyro << std::endl;
            
            merged_data.push_back(std::make_tuple(tsA, acc, gyro));
        }
        // break;
    }

    return merged_data;
}


std::vector<std::tuple<double, Eigen::VectorXd, Eigen::VectorXd>> merge_data(
    const std::map<double, std::vector<double>> &dataA,
    const std::map<double, std::vector<double>> &dataG) {
    
    std::set<double> keysA = extract_keys(dataA);
    std::set<double> keysG = extract_keys(dataG);

    // Find common timestamps
    std::set<double> common_timestamps;
    std::set_intersection(keysA.begin(), keysA.end(), keysG.begin(), keysG.end(),
                          std::inserter(common_timestamps, common_timestamps.begin()));

    // Merge data based on common timestamps
    std::vector<std::tuple<double, Eigen::VectorXd, Eigen::VectorXd>> merged_data;
    for (const auto &ts : common_timestamps) {
        // std::cout << ts << std:: endl;
        Eigen::VectorXd acc = to_eigen_vector(dataA.at(ts));
        Eigen::VectorXd gyro = to_eigen_vector(dataG.at(ts));
        merged_data.push_back(std::make_tuple(ts, acc, gyro));
    }

    return merged_data;
}
