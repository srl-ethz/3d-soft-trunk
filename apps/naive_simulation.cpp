#include <3d-soft-trunk/SoftTrunkModel.h>
#include <time.h>
#include <stdlib.h> // for srand

int main(){
    srand(time(0)); // seed random number generator

    SoftTrunkModel stm = SoftTrunkModel();
    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);

    // initialize pose- set same random curvature to all sections in the same segment
    for (int i = 0; i < st_params::num_segments; i++)
    {
        Vector2d rand = 0.7 / st_params::sections_per_segment * Vector2d::Random();
        for (int j = 0; j < st_params::sections_per_segment; j++)
            q.segment(2*i*st_params::sections_per_segment + 2*j, 2) = rand;
    }
    fmt::print("{}",q);
    
    VectorXd ddq;
    double dt = 0.001;
    srl::Rate r{1./dt};
    for (int i = 0; i < 10000; i++)
    {
        stm.updateState(q, dq);
        ddq = stm.B.inverse() * (stm.A * p - stm.C * dq - (-stm.g) - stm.K * q  -stm.D * dq);
        dq += dt * ddq;
        q += dt * dq;
        r.sleep();
    }
}