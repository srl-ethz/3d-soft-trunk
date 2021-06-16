#include "3d-soft-trunk/ControllerPCC.h"
#include <iostream>
#include <fstream>
/**
 * @brief play back a recorded csv
 */
int main(){

    SoftTrunkParameters st_params{};
    st_params.finalize();
    srl::State state_ref = st_params.getBlankState();
    srl::State state = st_params.getBlankState();
    std::string filename;
    std::string line;
    std::cout << "Enter filename you want to play back, located in 3d-soft-trunk\n";
    std::cin >> filename;
    filename = fmt::format("{}/{}", SOFTTRUNK_PROJECT_DIR, filename);
    std::ifstream log_file;
    std::string substr;
    log_file.open(filename);
    
    ControllerPCC cpcc{st_params, CurvatureCalculator::SensorType::qualisys};

    if(log_file){
        //skip the header
        getline(log_file, line);
        std::stringstream ss(line);

        srl::Rate r{30};
        while(getline(log_file, line)){
            std::stringstream().swap(ss);
            ss << line;
            for (int i = 0; i < st_params.q_size; i++){
                getline(ss, substr, ',');
                state_ref.q(i) = std::stod (substr);
            }
            getline(ss, substr);
            state_ref.q(st_params.q_size - 1) = std::stod (substr);
            cpcc.set_ref(state_ref);
            r.sleep();
        }    
    } else {std::cout << "\n Failed to open log file";}

    
    

}
