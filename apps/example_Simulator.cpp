#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/Simulator.h>

//todo: write a quick file which iterates to determine after which dt the sim starts crashing

int main(){


    SoftTrunkModel stm = SoftTrunkModel();
    srl::State state;
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);
    double dt = 0.0001;

    for (int i = 0; i < st_params::num_segments; i++) {
        // set to have about the same curvature as a whole regardless of scale
        double rand = -2.093 / st_params::sections_per_segment / st_params::num_segments;
        for (int j = 0; j < st_params::sections_per_segment; j++){
            state.q(2*i*st_params::sections_per_segment + 2*j + 1) = -rand;
            state.q(2*i*st_params::sections_per_segment + 2*j ) = rand * 0.022 / 0.19;
        }
            
    }

    Simulator sim = Simulator(stm, Simulator::SimType::beeman, dt, 10);
    sim.start_log("example_filename");
    for (double t=0; t < 0.5; t+=dt){
        if (!sim.simulate(p, state)){
            std::cout << "Sim crashed! Returning...\n";
            break;
        }
    }
    sim.end_log();
    

}
