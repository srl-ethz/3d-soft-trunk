#include "mobilerack-interface/QualisysClient.h"
#include "mobilerack-interface/ValveController.h"

int main(){
    std::vector<int> map = {12, 13, 14, 15};
    std::vector<int> cameras = {9};
    unsigned long long int timestamp;
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    double dt = 0.01;
    double distance;
    std::fstream log_file;
    int max_pressure = 1000;
    //log_file = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, "prisamtic_datagathering_logs.csv");
    
    ValveController vc{"192.168.0.100", map, max_pressure};
    QualisysClient qc{2, cameras};

    log_file.open("prisamtic_datagathering_logs.csv", std::fstream::out);

    srl::Rate r{1./dt};
    for (size_t i = 0; i < max_pressure; i+50)
    {
    vc.setSinglePressure(0,i);
    qc.getData(frames, timestamp);
    distance = (frames[0].translation()-frames[1].translation()).norm();
    r.sleep();
    log_file << distance << "," << timestamp << "," << i << "\n";
    }
    log_file.close();
        


    return 1;
}
