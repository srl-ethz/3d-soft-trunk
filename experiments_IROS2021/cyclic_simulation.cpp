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
    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
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
    
    VectorXd ddq;
    VectorXd q_mid;
    VectorXd dq_mid;
    VectorXd ddq_mid;

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
        stm.updateState(q, dq);

        log_file << t;
        for (int i=0; i < st_params::num_segments; i++){
            VectorXd x_tip = stm.get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        log_file << "\n";

        ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * q  -stm.D * dq);

        dq_mid = dq + ddq * dt / 2;
        q_mid = q + dq * dt / 2;
        stm.updateState(q_mid, dq_mid);
        ddq_mid = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * q_mid  -stm.D * dq_mid);
        
        dq = dq + ddq_mid * dt;
        q = q + dq_mid * dt;

        r.sleep();
        }
    log_file.close();
}