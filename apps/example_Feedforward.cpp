#include "3d-soft-trunk/ControllerPCC.h"

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main()
{
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();

    ControllerPCC cpcc(st_params, CurvatureCalculator::SensorType::qualisys);
    cpcc.set_log_filename("feedforward_log");
    double dt = 0.01;
    
    cpcc.toggle_log();
    //for stuff
    double pMax = 350.0;
    double pMin = 100.0;
    double fMax = 10.0;
    double fMin = 1;
    int period = 10;   // time period
    int totNumExp = 80; //total number of experiments
    srl::Rate r{1. / dt};   
    while (cpcc.experiment < totNumExp)
    {
        
        cpcc.amp1 = fRand(pMin, pMax);
        cpcc.amp2 = fRand(pMin, pMax);
        cpcc.T1 = fRand(fMin, fMax);
        cpcc.T2 = fRand(fMin, fMax);
        double time = 0;
        while (time < period)
        {
             
            for (int i = 0; i < 3; i++)
            {
                cpcc.p(i) = cpcc.amp1 * pow(sin(time * 2 * PI / cpcc.T1 + i * 2 * PI / 3), 2) + cpcc.amp1;
            }
            for (int i = 0; i < 3; i++)
            {
                cpcc.p(3 + i) = cpcc.amp2 * pow(sin(time * 2 * PI / cpcc.T2 + i * 2 * PI / 3), 2) + cpcc.amp2;
            }
            time += dt;
            cpcc.cc->get_curvature(cpcc.state);
            cpcc.actuate(cpcc.p);
            r.sleep();
            //std::cout << time << std::endl;
            //std::cout << cpcc.p << std::endl;
            //std::cout << cpcc.T1 << " : " << cpcc.T2 << std::endl;
        }
        cpcc.experiment += 1;
        std::cout << "experiment #" << cpcc.experiment << std::endl;
       std::cout << cpcc.T1 << " : " << cpcc.T2 << std::endl;
    }
    cpcc.toggle_log();
}