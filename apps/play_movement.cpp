#include "3d-soft-trunk/ControllerPCC.h"
#include <iostream>
#include <fstream>
/**
 * @brief play back a recorded csv
 */
int main(){

    srl::State state_ref, state;
    std::string filename;
    std::string line;
    std::cout << "Enter filename you want to play back, located in 3d-soft-trunk\n";
    std::cin >> filename;
    filename = fmt::format("{}/{}", SOFTTRUNK_PROJECT_DIR, filename);
    std::ifstream log_file;
    std::string substr;
    VectorXd bendLab_offset = VectorXd::Zero(4);
    log_file.open(filename);
    
    ControllerPCC cpcc{CurvatureCalculator::SensorType::bend_labs};

    if(log_file){
        //hardcoded: check that number of segments are correct, get offsets
        getline(log_file, line);
        std::stringstream ss(line);

        getline(ss, substr, ',');
        assert (std::stoi (substr) == st_params::num_segments);
        getline(ss, substr);
        assert (std::stoi (substr) == st_params::sections_per_segment);
        
        getline(log_file, line);
        std::stringstream().swap(ss);
        ss << line;
        for (int i = 0; i < 3; i++){
            getline(ss, substr, ',');
            bendLab_offset(i) = std::stod (substr);
        }
        getline(ss, substr);
        bendLab_offset(3) = std::stod(substr);

        srl::Rate r{30};
        while(getline(log_file, line)){
            std::stringstream().swap(ss);
            ss << line;
            for (int i = 0; i < st_params::q_size; i++){
                getline(ss, substr, ',');
                state_ref.q(i) = std::stod (substr);
            }
            getline(ss, substr);
            state_ref.q(st_params::q_size - 1) = std::stod (substr);
            r.sleep();
        }    
    } else {std::cout << "\n Failed to open log file";}

    
    

}
