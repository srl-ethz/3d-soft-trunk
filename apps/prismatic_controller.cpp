#include "mobilerack-interface/QualisysClient.h"
#include "mobilerack-interface/ValveController.h"


int main(){
    std::vector<int> map = {0,1,2,3,4,6,7};
    std::vector<int> cameras = {9};
    unsigned long long int timestamp;
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    double dt = 0.01;
    double distance; //this is the distance between the two attachment points of the actuator so its between 0.15-0.2m
    std::fstream log_file;
    int max_pressure = 2000;
    //log_file = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, "prisamtic_datagathering_logs.csv");
    ValveController vc{"192.168.0.100", map, max_pressure};
    
    QualisysClient qc{2, cameras};


    log_file.open("experiment_dadp_McK/dxdp_McK_bottom_001_200_100_2000_1.csv", std::fstream::out);
   
    //define pressure/distance relationship for the McKibbens

    double p1 =   2.22e-14;
    double p2 =  -7.215e-11;
    double p3 =   4.793e-08;
    double p4 =   -9.85e-06;
    double p5 =      0.2005;

    //double distance_p = p1*pow(i,4) + p2*pow(i,3) + p3*pow(i,2) + p4*i + p5; //distance as funtion of pressure
    double static_pressure = p1 - distance*p2 -p3 + p4*distance + p5//pressure as funtion of distance

    //controller parameters

    double k_p = 1;
    double k_d = 1;

    //input desired position

    double x_ref = 17; //position will be the offset from the top 
    double delta_x = x_ref - distance;
    double delta_x_dot = 0;


    //PD control loop
    srl::Rate r{1./dt};
    while (true)
    {
        //get data and calculate values
        qc.getData(frames, timestamp);
        distance = (frames[0].translation()-frames[1].translation()).norm();

        //activate piston if actuator needs to move down
        if (delta_x > 0) {
            vc.setSinglePressure(0, k_p*delta_x + k_d*delta_x_dot);
        }
        //activate McKs when its supposed to move up
        else {
            for (size_t f = 1; f < 7; f++)
            {
                vc.setSinglePressure(f, static_pressure + k_p*delta_x + k_d*delta_x_dot);
            }
        }
        r.sleep();
    }
    
    


