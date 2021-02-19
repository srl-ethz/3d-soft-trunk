#include <3d-soft-trunk/SoftTrunkModel.h>

int main(){
    SoftTrunkModel stm = SoftTrunkModel();
    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);
    q = 1.  / st_params::sections_per_segment * q.setRandom();
    
    VectorXd ddq;

    fmt::print("{}",q);

    double dt = 0.001;
    srl::Rate r{1./dt};
    for (int i = 0; i < 1000; i++)
    {
        stm.updateState(q, dq);
        ddq = stm.B.inverse() * (stm.A * p - stm.C * dq + stm.g - stm.K * q - stm.D * q);
        dq += dt * ddq;
        q += dt * dq;
        r.sleep();
    }
}