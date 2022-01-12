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
    int max_pressure = 500;
    //log_file = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, "prisamtic_datagathering_logs.csv");
    
    ValveController vc{"192.168.0.100", map, max_pressure};
    QualisysClient qc{2, cameras};


    log_file.open("experiment_px_Pis/px_Pis_2,5offsett_001_20_500_3.csv", std::fstream::out);

    srl::Rate r{1./dt};

    p1 =   2.22e-14
    p2 =  -7.215e-11
    p3 =   4.793e-08
    p4 =   -9.85e-06 
    p5 =      0.2005

    //outer loop defines initial position
    for (size_t i = 0; i < max_pressure; i+=100)
    {

        //define pressure/distance function
        double pos = p1*i^4 + p2*i^3 + p3*i^2 + p2*i + p5;

        //set pressure in all McK's
        for (size_t d = 1; d < 8; d++)
        {
            vc.setSinglePressure(d,i);
        }

        srl::sleep(1);
        
        //log data
        qc.getData(frames, timestamp);
        distance = (frames[0].translation()-frames[1].translation()).norm();
        log_file << distance << "," << timestamp << "," << i << pos << (distance-pos).norm() "\n";

        srl::sleep(1);

        /*
        //give pressure step and record average accelration for set initial position
        for (size_t d = 1; d < 8; d++)
        {
            vc.setSinglePressure(d,i+100);
        }
        */
    }
    
    
      /*  
    for (size_t i = 0; i < max_pressure; i+=20)
        {
          
        vc.setSinglePressure(0,i);       

 
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

     */
   

    return 1;
}
    