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
    std::fstream log_file2;
    int max_pressure = 2000;
    //log_file = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, "prisamtic_datagathering_logs.csv");
    ValveController vc{"192.168.0.100", map, max_pressure};
    
    QualisysClient qc{2, cameras};


    log_file.open("experiment_control/dxdp_McK_bottom_001_1__2000_1_withSoPrA.csv", std::fstream::out);
    //log_file2.open("experiment_dadp_McK/dadp_McK_bottom_001_200_100_2000_1.csv", std::fstream::out);
   

    double p1 =   2.22e-14;
    double p2 =  -7.215e-11;
    double p3 =   4.793e-08;
    double p4 =   -9.85e-06;
    double p5 =      0.2005;

/*
    //outer loop defines initial position
    for (size_t i = 400; i < max_pressure; i+=200)
    {

        //define pressure/distance function
        double pos = p1*pow(i,4) + p2*pow(i,3) + p3*pow(i,2) + p4*i + p5;

        //set pressure in all McK's
        for (size_t d = 1; d < 7; d++)
        {
            vc.setSinglePressure(d,i);
        }

        srl::sleep(2);
        
        //log data
        qc.getData(frames, timestamp);
        distance = (frames[0].translation()-frames[1].translation()).norm();
        log_file << distance << "," << pos << "," << (distance-pos) << "," << timestamp << "," << i << "\n";

        srl::sleep(1);

        //qc.getData(frames, timestamp);
        //distance = (frames[0].translation()-frames[1].translation()).norm();
        //log_file2 << distance << "," << timestamp << "," << i << "\n";

        //give pressure step and record position change
        for (size_t f = 1; f < 7; f++)
        {
            vc.setSinglePressure(f,(i+100));
        }

        //log acceleartions and pressure
        srl::Rate r{1./dt};
        for (size_t e = 0; e < 20; e++)
        {


            //compute average of all active valves
            double average = 0;
            int active_valves = 0;

            for(int j = 0; j < vc.sensor_pressures.size(); j++){
                if (vc.sensor_pressures[j] != 0)
                {
                    active_valves++;
                    average += vc.sensor_pressures[j];
                }
            }
            average = average/active_valves;

            //log results
            qc.getData(frames, timestamp);
            distance = (frames[0].translation()-frames[1].translation()).norm();
            log_file2 << distance << "," << timestamp << "," << i+100;
            log_file2 << "," << average << "\n";
            r.sleep();
        }
        
        srl::sleep(1);

    }
    log_file.close();
    log_file2.close();
    */
    //this section tracks the p/x trajectory
        srl::Rate r{1./dt};
    for (size_t i = 0; i < max_pressure; i-=1)
        {
          
        vc.setSinglePressure(0,0);       

 
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

     log_file.close();
   

    return 1;
}
    