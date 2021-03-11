#include <3d-soft-trunk/SoftTrunkModel.h>
#include <csv.h>
#include <fstream>

int sinusoid(double phase) { return 300 + 300 * sin(phase); }

/**
 * @brief do forward simulation on the dynamic model using the Euler-Richardson algorithm for integration of ddq.
 * linearly interpolate the given csv (log_pressure.csv), and simulate the model
 */
int main(){
    SoftTrunkModel stm = SoftTrunkModel();
    srl::State state;
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);

    // read and save the csv data
    io::CSVReader<7> in(fmt::format("{}/log_pressure_rotate.csv", SOFTTRUNK_PROJECT_DIR));
    in.read_header(io::ignore_extra_column, "time(sec)", "p_meas[0]", "p_meas[1]", "p_meas[2]", "p_meas[3]", "p_meas[4]", "p_meas[5]");

    // read the CSV, and save the datapoints to a vector
    std::vector<double> log_t;
    std::array<std::vector<double>, 3*st_params::num_segments> log_p;
    {
        double t, p0, p1, p2, p3, p4, p5;
        while (in.read_row(t, p0, p1, p2, p3, p4, p5)){
            log_t.push_back(t);
            log_p[0].push_back(p0);
            log_p[1].push_back(p1);
            log_p[2].push_back(p2);
            log_p[3].push_back(p3);
            log_p[4].push_back(p4);
            log_p[5].push_back(p5);
        }
    }

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
    double t, p0, p1;
    int log_index = 0; // index of log currently being referred to for pressure data
    for (double t = log_t[0]; t < log_t[log_t.size()-1]; t+=dt)
    {
        if (log_t[log_index] < t)
            log_index ++; // move to next point in log
        p(0) = log_p[0][log_index];
        p(1) = log_p[1][log_index];
        p(2) = log_p[2][log_index];
        p(3) = log_p[3][log_index];
        p(4) = log_p[4][log_index];
        p(5) = log_p[5][log_index];
        p *= 100.; // mbar to Pa

        /// run the simulation
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
        
        state.dq = state.dq + state_mid.ddq * dt;
        state.q = state.q + state_mid.dq * dt;

        r.sleep();
        }
    log_file.close();
}
