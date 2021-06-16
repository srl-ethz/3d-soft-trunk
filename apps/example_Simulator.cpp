#include <3d-soft-trunk/ControllerPCC.h>
#include <chrono>

int main(){

    SoftTrunkParameters st_params;
    st_params.finalize();
    ControllerPCC cpcc = ControllerPCC(st_params, CurvatureCalculator::SensorType::simulator, true);
    srl::State state = st_params.getBlankState();
    VectorXd p = VectorXd::Zero(3*st_params.num_segments);
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
    cpcc.set_state(state);
    const double hz = 1./dt;
    cpcc.set_frequency(hz);

    auto start = std::chrono::steady_clock::now();

    for (double t=0; t < time; t+=dt){
        cpcc.simulate(p);
        cpcc.get_state(state);
    }


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulated " << time << "s of motion in " << elapsed.count() <<"s realtime using timestep " << dt << "\n";
}
