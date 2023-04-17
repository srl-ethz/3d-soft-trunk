#include <3d-soft-trunk/ControllerPCC.h>
#include <chrono>
/*
    simple forward simulation of the trunk
*/

void keys_to_pressure_thread(VectorXd& p){
    /**
     * keep listening to the keyboard, when user presses 1, the first chamber is pressurized, and so on.
     * when user presses 0, all chambers are depressurized.
     */
    while(true){
        char c;
        while(true){
            c = getchar();
            // convert to int if it's a number
            if (c >= '0' && c <= '9'){
                c -= '0';
                if (c == 0){
                    p.setZero();
                } else if (c <= p.size()){
                    p(c-1) += 100;
                }
                std::cout << "p: " << p.transpose() << std::endl;
            }
        }
    }
}

int main(){

    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.sensors = {SensorType::simulator};
    st_params.finalize();
    ControllerPCC cpcc = ControllerPCC(st_params);

    srl::State state = st_params.getBlankState();
    VectorXd p = VectorXd::Zero(st_params.p_size);  // each chamber + gripper
    double time = 30.0;
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

    std::thread key_thread(keys_to_pressure_thread, std::ref(p));
    std::cout << "Press 1-9 and [ENTER] to pressurize the corresponding chamber, 0 [ENTER] to depressurize all chambers." << std::endl;

    for (double t=0; t < time; t+=dt){
        bool ret = cpcc.simulate(p);
        r.sleep();
    }


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulated " << time << "s of motion in " << elapsed.count() <<"s realtime using timestep " << dt << "\n";
}