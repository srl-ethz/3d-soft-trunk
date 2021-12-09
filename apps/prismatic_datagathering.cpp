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

    
    ValveController vc{"192.168.0.100", map, 1000};
    QualisysClient qc{2, cameras};

    srl::Rate r{1./dt};
    while(true){
        vc.setSinglePressure(0,100);
        qc.getData(frames, timestamp);
        distance = (frames[0].translation()-frames[1].translation()).norm();
        r.sleep();
    }
    


    return 1;
}
