#include <3d-soft-trunk/ControllerPCC.h>
#include <chrono>
/*
    simple forward simulation of the trunk
*/

int main(){

    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.sensors = {SensorType::simulator};
    st_params.finalize();
    ControllerPCC cpcc = ControllerPCC(st_params);

    srl::State state = st_params.getBlankState();
    VectorXd p = VectorXd::Zero(st_params.p_size);  // each chamber + gripper
    double time = 4.0;
    const double dt = 0.01;

    for (int i = 0; i < st_params.num_segments; i++) {
        // set to have about the same curvature as a whole regardless of scale
        double rand = -2.093 / st_params.sections_per_segment / st_params.num_segments;
        for (int j = 0; j < st_params.sections_per_segment; j++){
            state.q(2*i*st_params.sections_per_segment + 2*j + 1) = -rand;
            state.q(2*i*st_params.sections_per_segment + 2*j ) = rand * 0.022 / 0.19;
        }

    }
    std::cout << "q: " << state.q.transpose() << std::endl;
    cpcc.state_ = state;
    cpcc.state_prev_ = state;
    std::cout << "x: " << cpcc.state_.tip_transforms[st_params.num_segments+st_params.prismatic].translation().transpose() << std::endl;
    const double hz = 1./dt;
    srl::Rate r(hz);
    cpcc.dt_ = dt;

    auto start = std::chrono::steady_clock::now();

    for (double t=0; t < time; t+=dt){
        bool ret = cpcc.simulate(p);
        r.sleep();
    }


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulated " << time << "s of motion in " << elapsed.count() <<"s realtime using timestep " << dt << "\n";
}