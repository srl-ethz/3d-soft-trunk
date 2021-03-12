#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/Simulator.h>
#include <csv.h>
#include <fstream>

/**
 * @file simulate_logged_pressure.cpp
 * @brief do forward simulation based on pressure log, using the Simulator class. (Currently hardcoded for 2 segment & 6 chambers)
 * 
 * Usage: 
 * ```bash
 * ./bin/simulate_logged_pressure your_log_pressure.csv
 * ```
 * linearly interpolate the given csv, and simulate the model
 */
int main(int argc, char *argv[]){
    double step = 0.01;
    SoftTrunkModel stm = SoftTrunkModel();
    Simulator sim = Simulator(stm, step, 1);
    srl::State state;
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);

    // read and save the csv data
    std::string p_filename;
    if (argc == 1)
        p_filename = fmt::format("{}/log_pressure.csv", SOFTTRUNK_PROJECT_DIR);
    else
        p_filename = argv[1];
    fmt::print("reading pressure log from {}\n", p_filename);
    io::CSVReader<7> in(p_filename);
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

    // srl::Rate r{1./dt};
    int log_index = 0; // index of log currently being referred to for pressure data
    sim.start_log("log_sim");
    for (double t = log_t[0]; t < log_t[log_t.size()-1]; t+=step)
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

        // run the simulation
        sim.simulate(p, state);
//        r.sleep();
    }
    sim.end_log();
}
