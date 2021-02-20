#include <3d-soft-trunk/SoftTrunkModel.h>
#include <time.h>
#include <stdlib.h> // for srand

int main(){
    srand(time(0)); // seed random number generator

    SoftTrunkModel stm = SoftTrunkModel();
    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);

    p[0] = 300 * 100;
    p[3] = 300 * 100;
    fmt::print("{}",q);
    
    VectorXd ddq;
    double dt = 0.001;
    srl::Rate r{1./dt};
    for (int i = 0; i < 10000; i++)
    {
        stm.updateState(q, dq);
        ddq = stm.B.inverse() * (stm.A * p - stm.C * dq - (-stm.g) - stm.K * q  -stm.D * dq);
        //fmt::print("B: {}\n", stm.B);
        //fmt::print("Binv:{}\n", stm.B.inverse());
        //fmt::print("ddq: {}\n", ddq);
        dq += dt * ddq;
        q += dt * dq;
        r.sleep();
        }
}