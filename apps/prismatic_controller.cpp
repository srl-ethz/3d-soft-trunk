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
    int max_pressure = 2200;
    //log_file = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, "prisamtic_datagathering_logs.csv");
    ValveController vc{"192.168.0.100", map, max_pressure};
    
    QualisysClient qc{2, cameras};


    log_file.open("Push_pull.csv", std::fstream::out);
   
    

    //controller parameters

    double k_p = 10000;
    double k_d = 1;
    double k_i = 100;

    //input desired position

    double x_ref = 0.15; //position will be the offset from the top 
    double delta_x = x_ref - distance;
    double delta_x_dot = 0;
    double integrator = 0;

    //define pressure/distance relationship for the McKibbens

    /*
    double p1 =   2.22e-14;
    double p2 =  -7.215e-11;
    double p3 =   4.793e-08;
    double p4 =   -9.85e-06;
    double p5 =      0.2005;

    double distance_p = p1*pow(i,4) + p2*pow(i,3) + p3*pow(i,2) + p4*i + p5; //distance as funtion of pressure
    */

   //define pressure/distance relationship for the McKibbens


    double p1 =   -181.9;
    double p2 =  -696.4;
    double p3 =   -603.3;
    double p4 =   461.3;
    double p5 =     551.3;
    double p6 =   -115.9;
    double p7 =   -429.9;
    double p8 =   1161;
   

    long double static_pressure = p1*pow(x_ref,7) + p2*pow(x_ref,6) + p3*pow(x_ref,5) + p4*pow(x_ref,4) + p5*pow(x_ref,3) + p6*pow(x_ref,2) + p7*x_ref + p8; //distance as funtion of pressure
   

    //long double static_pressure = 2000 - 1400/0.05*(x_ref-0.15);

    qc.getData(frames, timestamp);
    distance = (frames[0].translation()-frames[1].translation()).norm();
    srl::sleep(1);

    //PD control loop
    srl::Rate r{1./dt};
    for (size_t i = 0; i < 1000; i++)
    {
        
        //define reference trajectory
        x_ref = 0.02*sin(2*M_PI*i/250) + 0.17;
        

        //get data and calculate values
        qc.getData(frames, timestamp);
        distance = (frames[0].translation()-frames[1].translation()).norm();
        delta_x = x_ref - distance;


        //activate piston if actuator needs to move down
        if (delta_x > 0) {
            static_pressure = p1*pow(x_ref,7) + p2*pow(x_ref,6) + p3*pow(x_ref,5) + p4*pow(x_ref,4) + p5*pow(x_ref,3) + p6*pow(x_ref,2) + p7*x_ref + p8;
            for (size_t f = 1; f < 7; f++)
            {
                vc.setSinglePressure(f, static_pressure);
            }
            vc.setSinglePressure(0, k_p*1.5*delta_x + k_d*delta_x_dot);
            //log data
            log_file << x_ref << "," << distance << "," << delta_x << "," << "pushing with" << "," << k_p*1.5*delta_x + k_d*delta_x_dot << "," << vc.sensor_pressures[0] << "," << static_pressure << "," << integrator << "\n";
        }


        //activate McKs when its supposed to move up
        else {
            vc.setSinglePressure(0, 0); //reset piston pressure, otherwise it will keep on pushing
            static_pressure =  p1*pow(x_ref,7) + p2*pow(x_ref,6) + p3*pow(x_ref,5) + p4*pow(x_ref,4) + p5*pow(x_ref,3) + p6*pow(x_ref,2) + p7*x_ref + p8;
            for (size_t f = 1; f < 7; f++)
            {
                vc.setSinglePressure(f, static_pressure + k_p*-delta_x + k_d*delta_x_dot);
            }
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
            //log data
            log_file << x_ref << "," << distance << "," << delta_x << "," << "pulling with" << "," <<  static_pressure + k_p*-delta_x + k_d*delta_x_dot << "," << average << "," << static_pressure << "," << integrator << "\n";
        }

        integrator += delta_x;
        r.sleep();
        
    }
    log_file.close();
    return 1;
}
    
    


