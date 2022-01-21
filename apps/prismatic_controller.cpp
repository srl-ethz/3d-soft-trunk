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


    log_file.open("experiment_control/steps_kp35000_1.csv", std::fstream::out);
   
    

    //controller parameters

    double k_p = 35000;
    double k_d = 1;
    double k_i = 100;

    //input desired position

    double x_ref = 0.21; //position will be the offset from the top 
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

    //old vlaues
    /*
    double p1 =   1.41328141453413e+16;
    double p2 =  -8.45833368929168e+15;
    double p3 =   2.88855935347167e+15;
    double p4 =   -615654455624184;
    double p5 =     83859684251354;
    double p6 =   -7129046976908.42;
    double p7 =   345824233842.199;
    double p8 =   -7328966565.07981;
    double p9 =  -1.03164077889415e+16;
   */

  //new values
    double p1 = 1.22581873479512e+16; 
    double p2 =  -7.26283111947410e+15;
    double p3 =   2.45520680796212e+15;
    double p4 =   -517950463550658;
    double p5 =     69823882217993.7;
    double p6 =   -5873988781448.49;
    double p7 =   281938954326.112 ;
    double p8 =   -5911301273.26948;
    double p9 =  -9.03790616540163e+15;
    

    long double static_pressure = p9*pow(distance,8) + p1*pow(distance,7) + p2*pow(distance,6) + p3*pow(distance,5) + p4*pow(distance,4) + p5*pow(distance,3) + p6*pow(distance,2) + p7*distance + p8 ; //distance as funtion of pressure
    //long double static_pressure =  p9*pow(x_ref,8) + p1*pow(x_ref,7) + p2*pow(x_ref,6) + p3*pow(x_ref,5) + p4*pow(x_ref,4) + p5*pow(x_ref,3) + p6*pow(x_ref,2) + p7*x_ref + p8 ;

    //long double static_pressure = 2000 - 1400/0.05*(x_ref-0.15);

    qc.getData(frames, timestamp);
    distance = (frames[0].translation()-frames[1].translation()).norm();
    srl::sleep(1);

    //PID control loop
    srl::Rate r{1./dt};
    for (size_t i = 0; i < 500; i++)
    {
        
        //define reference trajectory
        //x_ref = 0.025*cos(2*M_PI*i/250) + 0.175;
        x_ref = 0.2;
        //if (i%100==0) x_ref -= 0.01;

        //get data and calculate values
        qc.getData(frames, timestamp);
        distance = (frames[0].translation()-frames[1].translation()).norm();
        delta_x = x_ref - distance;


        //activate piston if actuator needs to move down
        if (delta_x > 0) {
            static_pressure =  p9*pow(distance,8) + p1*pow(distance,7) + p2*pow(distance,6) + p3*pow(distance,5) + p4*pow(distance,4) + p5*pow(distance,3) + p6*pow(distance,2) + p7*distance + p8;
            for (size_t f = 1; f < 7; f++)
            {
                vc.setSinglePressure(f, static_pressure);
            }
            double piston_p = k_p*1.9*delta_x + k_d*delta_x_dot + k_i*integrator;
            if (piston_p > 1200) piston_p = 1200;
            vc.setSinglePressure(0, piston_p);
            //log data
            log_file << x_ref << "," << distance << "," << delta_x << "," << "pushing" << "," << k_p*1.5*delta_x + k_d*delta_x_dot + k_i*integrator << "," << vc.sensor_pressures[0] << "," << static_pressure << "," << integrator << "\n";
        }


        //activate McKs when its supposed to move up
        else {
            vc.setSinglePressure(0, 0); //reset piston pressure, otherwise it will keep on pushing
            static_pressure =  p9*pow(distance,8) + p1*pow(distance,7) + p2*pow(distance,6) + p3*pow(distance,5) + p4*pow(distance,4) + p5*pow(distance,3) + p6*pow(distance,2) + p7*distance + p8;
            double mck_p =  static_pressure -k_p*delta_x + k_d*delta_x_dot + k_i*integrator;
            if (mck_p > 2000) mck_p = 2000;
            for (size_t f = 1; f < 7; f++)
            {
                vc.setSinglePressure(f, mck_p);
            }
             //compute average of all active valves
            double average = 0;
            int active_valves = 0;
            for(int j = 1; j < vc.sensor_pressures.size(); j++){
                if (vc.sensor_pressures[j] != 0)
                {
                    active_valves++;
                    average += vc.sensor_pressures[j];
                }
            }
            average = average/active_valves;
            //log data
            log_file << x_ref << "," << distance << "," << delta_x << "," << "pulling" << "," <<  static_pressure + k_p*-delta_x + k_d*delta_x_dot - k_i*integrator << "," << average << "," << static_pressure << "," << integrator << "\n";
        }

        //integrator += delta_x;
        r.sleep();
        
    }
    log_file.close();
    return 1;
}
    
    


