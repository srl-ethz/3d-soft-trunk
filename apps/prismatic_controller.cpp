#include "mobilerack-interface/QualisysClient.h"
#include "mobilerack-interface/ValveController.h"


int main(){
    std::vector<int> map = {0,1};
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


    log_file.open("experiment_control/playaround.csv", std::fstream::out);
   
   //srl::sleep(4);

   

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
    */
/*
    //parameters with sopra attached druing lifting
    double p1 = 1.27149367844176e+16; 
    double p2 =  -7.89636435403446e+15;
    double p3 =  2.79803705573077e+15;
    double p4 =   -618740107922287;
    double p5 =    87435810376300.4;
    double p6 =   -7710722693852.38;
    double p7 =   387977263373.941;
    double p8 =   -8527820459.58139;
    double p9 =  -8.94404845044271e+15;


    //parameters with sopra attached druing falling
    double p1 = -2.29494316545169e+15; 
    double p2 = 1.64098919415634e+15;
    double p3 =  -661778114487526;
    double p4 =   165006142774963;
    double p5 =     -26091634292548.4;
    double p6 =   2558427050560.87;
    double p7 =   -142374110358.501;
    double p8 =   3445371999.62632;
    double p9 =  1.38147468102622e+15;
*/

  //parameters with sopra attached druing lifting
    double p1 = 2.27286685805334e+16; 
    double p2 =  -1.44742813860741e+16;
    double p3 =  5.26144455255893e+15;
    double p4 =   -1.19403849976615e+15;
    double p5 =   173236603400900;
    double p6 =   -15691774818037.8;
    double p7 =   811336068389.078;
    double p8 =   -18333497313.1710;
    double p9 =  -1.55973266940115e+16;

    //parameters with sopra during falling

    double pa1 = -3.46533160247876e+15; 
    double pa2 =  2.25018172335107e+15;
    double pa3 =  -834656736243444;
    double pa4 =   193435836054681;
    double pa5 =   -28681818851200.6;
    double pa6 =   2657163630691.87;
    double pa7 =   -140622269201.993;
    double pa8 =   3254846129.69969;
    double pa9 =  2.33400234637593e+15;

     //controller parameters

    double k_p = 20000; //gain without SoPrA around 35000, with SoPrA around 8000
    double k_d = 2*sqrt(k_p);
    double k_i = 0;

    //input desired position

    double x_ref = 0.210; //position will be the offset from the top 
    double delta_x = x_ref - distance;
    double x_dot_ref = 0;
    double delta_x_dot = 0;
    double integrator = 0;
    double distance_prev = 0; // used for computation of delta_x_dot


    long double static_pressure = 100*(0.005-delta_x)*(pa9*pow(distance,8) + pa1*pow(distance,7) + pa2*pow(distance,6) + pa3*pow(distance,5) + pa4*pow(distance,4) + pa5*pow(distance,3) + pa6*pow(distance,2) + pa7*distance + pa8) + (1-100*(0.005-delta_x))*(p9*pow(distance,8) + p1*pow(distance,7) + p2*pow(distance,6) + p3*pow(distance,5) + p4*pow(distance,4) + p5*pow(distance,3) + p6*pow(distance,2) + p7*distance + p8);

    long double static_pressure_up  = p9*pow(distance,8) + p1*pow(distance,7) + p2*pow(distance,6) + p3*pow(distance,5) + p4*pow(distance,4) + p5*pow(distance,3) + p6*pow(distance,2) + p7*distance + p8 ; //distance as funtion of pressure
    long double static_pressure_down = pa9*pow(distance,8) + pa1*pow(distance,7) + pa2*pow(distance,6) + pa3*pow(distance,5) + pa4*pow(distance,4) + pa5*pow(distance,3) + pa6*pow(distance,2) + pa7*distance + pa8 ;
    //long double static_pressure =  p9*pow(x_ref,8) + p1*pow(x_ref,7) + p2*pow(x_ref,6) + p3*pow(x_ref,5) + p4*pow(x_ref,4) + p5*pow(x_ref,3) + p6*pow(x_ref,2) + p7*x_ref + p8 ;


    //PID control loop
    log_file << "Input" << "," << "Output" << "," << "error" << "," << "state" << "," << "desired pressure" << "," << "actual pressure" << "," << "static feed forward pressure" << "\n";
    
    qc.getData(frames, timestamp);
    distance = (frames[0].translation()-frames[1].translation()).norm();
    delta_x = x_ref - distance;
    distance_prev = distance;
    srl::Rate r{1./dt};
    
    for (size_t i = 0; i < 1000; i++)
    {
        
        //define reference trajectory
        //if (i < 1000)
      // {
        //    x_ref = 0.022*cos(2*M_PI*i/500) + 0.188;
      //      x_dot_ref = 0.023*sin(2*M_PI*i/500)*2*M_PI/500;
     //  }
      // else {
            x_dot_ref = 0;
            if (i==250) x_ref = 0.175;
            if (i==500) x_ref = 0.195;
      // }

        //get data and calculate values
        qc.getData(frames, timestamp);
        distance = (frames[0].translation()-frames[1].translation()).norm();
        delta_x = x_ref - distance;
        delta_x_dot = x_dot_ref - (distance - distance_prev)/dt;
        if(distance - distance_prev < 0.001) {
            delta_x_dot = 0;
        }
        distance_prev = distance;

        long double static_pressure_up  = (p9*pow(distance,8) + p1*pow(distance,7) + p2*pow(distance,6) + p3*pow(distance,5) + p4*pow(distance,4) + p5*pow(distance,3) + p6*pow(distance,2) + p7*(distance) + p8) ; 
        //if (static_pressure_up > 1800) static_pressure_up;

        long double static_pressure_down = pa9*pow(distance,8) + pa1*pow(distance,7) + pa2*pow(distance,6) + pa3*pow(distance,5) + pa4*pow(distance,4) + pa5*pow(distance,3) + pa6*pow(distance,2) + pa7*(distance) + pa8 ;

        static_pressure =  (1-100*(0.005-delta_x))*static_pressure_down + 100*(0.005-delta_x)*static_pressure_up;
        if (delta_x>0.005) static_pressure = static_pressure_down;
        if (delta_x<-0.005) static_pressure = static_pressure_up;
        

        //activate piston if actuator needs to move down
        if (delta_x > 0) {

            vc.setSinglePressure(1, static_pressure);

            double piston_p = 1.5*k_p*delta_x + k_d*delta_x_dot + k_i*integrator;
            if (piston_p > 800) piston_p = 800;
            if (piston_p < 0) piston_p = 0;
            vc.setSinglePressure(0, piston_p);

            //log data
            log_file << x_ref << "," << distance << "," << delta_x << "," << "pushing" << "," << piston_p << "," << vc.sensor_pressures[0] << "," << static_pressure << "," << p9*pow(distance,8) + p1*pow(distance,7) + p2*pow(distance,6) + p3*pow(distance,5) + p4*pow(distance,4) + p5*pow(distance,3) + p6*pow(distance,2) + p7*distance + p8 << ","  << pa9*pow(distance,8) + pa1*pow(distance,7) + pa2*pow(distance,6) + pa3*pow(distance,5) + pa4*pow(distance,4) + pa5*pow(distance,3) + pa6*pow(distance,2) + pa7*distance + pa8 <<  "\n";
        }


        //activate McKs when its supposed to move up
        else {
            
            vc.setSinglePressure(0,  k_d*delta_x_dot); //reset piston pressure, otherwise it will keep on pushing
            
            if (static_pressure < 0) {
                vc.setSinglePressure(1, -k_p*delta_x - k_d*delta_x_dot + k_i*integrator);
            } else {
                vc.setSinglePressure(1, static_pressure -k_p*delta_x - k_d*delta_x_dot + k_i*integrator);
            }
            
        
            //log data
            log_file << x_ref << "," << distance << "," << delta_x << "," << "pulling" << "," <<  static_pressure -k_p*delta_x + k_d*delta_x_dot + k_i*integrator << "," << vc.sensor_pressures[1] << "," << static_pressure << ","  << p9*pow(distance,8) + p1*pow(distance,7) + p2*pow(distance,6) + p3*pow(distance,5) + p4*pow(distance,4) + p5*pow(distance,3) + p6*pow(distance,2) + p7*distance + p8 << ","  << pa9*pow(distance,8) + pa1*pow(distance,7) + pa2*pow(distance,6) + pa3*pow(distance,5) + pa4*pow(distance,4) + pa5*pow(distance,3) + pa6*pow(distance,2) + pa7*distance + pa8 <<  "\n";
        }

        //integrator += delta_x;
        r.sleep();
        
    }
    log_file.close();
    return 1;
}
    
    


