#include "mobilerack-interface/QualisysClient.h"
#include "mobilerack-interface/ValveController.h"

int main(){
    std::vector<int> map = {0,1,2,3,4,6,7};
    std::vector<int> cameras = {9};
    unsigned long long int timestamp;
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    double dt = 0.01;
    double distance;
    std::fstream log_file;
    int max_pressure = 2000;
    //log_file = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, "prisamtic_datagathering_logs.csv");
    
    ValveController vc{"192.168.0.100", map, max_pressure};
    QualisysClient qc{2, cameras};

    log_file.open("prisamtic_datagathering_logs.csv", std::fstream::out);

    srl::Rate r{1./dt};
   
    for (size_t i = 0; i < max_pressure; i+=20)
        {
  /*         
        vc.setSinglePressure(0,i);       
*/ 

        vc.setSinglePressure(1,i);
        vc.setSinglePressure(2,i);
        vc.setSinglePressure(3,i);
        vc.setSinglePressure(4,i);
        vc.setSinglePressure(5,i);
        vc.setSinglePressure(6,i);

        qc.getData(frames, timestamp);
        distance = (frames[0].translation()-frames[1].translation()).norm();
        r.sleep();
        log_file << distance << "," << timestamp << "," << i << "\n";
        }
    
   
   /* 
    vc.setSinglePressure(0,100);
    qc.getData(frames, timestamp);
    distance = (frames[0].translation()-frames[1].translation()).norm();
    log_file << distance << "," << timestamp << "," << 100 << "\n";
    r.sleep();
    vc.setSinglePressure(0,500);
    qc.getData(frames, timestamp);
    distance = (frames[0].translation()-frames[1].translation()).norm();
    log_file << distance << "," << timestamp << "," << 200 << "\n";
    log_file.close();
    r.sleep();  
 */ 

    return 1;
}
    