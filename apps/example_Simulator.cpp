#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/Simulator.h>

int main(){


    SoftTrunkModel stm = SoftTrunkModel();
    Pose pose;
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);

    for (int i = 0; i < st_params::num_segments; i++) {
        // set to have about the same curvature as a whole regardless of scale
        double rand = -2.093 / st_params::sections_per_segment / st_params::num_segments;
        for (int j = 0; j < st_params::sections_per_segment; j++){
            pose.q(2*i*st_params::sections_per_segment + 2*j + 1) = -rand;
            pose.q(2*i*st_params::sections_per_segment + 2*j ) = rand * 0.022 / 0.19;
        }
            
    }

    Simulator sim = Simulator(stm, pose, Simulator::SimType::beeman);
    sim.start_log("example_filename");
    sim.simulate(p, 0.0005, 10);
    sim.end_log();
    

}
