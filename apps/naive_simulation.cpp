#include <3d-soft-trunk/SoftTrunkModel.h>
#include <time.h>
#include <stdlib.h> // for srand
#include <fstream>

/**
 * @brief do forward simulation on the dynamic model using the Euler-Richardson algorithm for integration of ddq.
 * Log the tip positions to log_sim.csv
 * https://www.compadre.org/PICUP/resources/Numerical-Integration/
 * http://hep.fcfm.buap.mx/cursos/2013/FCI/ejs_sip_ch03.pdf
 */
int main(){
    srand(time(NULL) * 2); // seed random number generator
    SoftTrunkModel stm = SoftTrunkModel();
    srl::State state;
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);

    // initialize state- set same random curvature to all sections in the same segment
    for (int i = 0; i < st_params::num_segments; i++)
    {
        // set to have about the same curvature as a whole regardless of scale
        double rand = -2.093 / st_params::sections_per_segment / st_params::num_segments;
        for (int j = 0; j < st_params::sections_per_segment; j++){
            state.q(2*i*st_params::sections_per_segment + 2*j + 1) = -rand;
            state.q(2*i*st_params::sections_per_segment + 2*j ) = rand * 0.022 / 0.19;
        }
            
    }

    // set initial pressure
//    p[0] = 300 * 100;
//    p[2] = 200 * 100;
//    p[3] = 300 * 100;

    fmt::print("initial state: {}\n", state.q.transpose());
    fmt::print("initial pressure: {}\n", p.transpose());

    std::fstream log_file;
    std::string filename = fmt::format("{}/log_sim.csv", SOFTTRUNK_PROJECT_DIR);
    fmt::print("outputting log to {}...\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "timestamp";
    for (int i=0; i<st_params::num_segments; i++)
        log_file << fmt::format(", x_{}, y_{}, z_{}", i, i, i);
    log_file << "\n";
    
    srl::State state_mid;

    double dt = 0.00007;
    srl::Rate r{1./dt};
    for (double t = 0; t < 3; t+=dt)
    {
        stm.updateState(state);

        log_file << t;
        for (int i=0; i < st_params::num_segments; i++){
            VectorXd x_tip = stm.get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        log_file << "\n";

        state.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * state.q  -stm.D * state.dq);

        state_mid.dq = state.dq + state.ddq * dt / 2;
        state_mid.q = state.q + state.dq * dt / 2;
        stm.updateState(state_mid);
        state_mid.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * state_mid.q  -stm.D * state_mid.dq);
        
        state.dq = state.dq + state_mid.dq * dt;
        state.q = state.q + state_mid.dq * dt;

        r.sleep();
        }
    log_file.close();
}
