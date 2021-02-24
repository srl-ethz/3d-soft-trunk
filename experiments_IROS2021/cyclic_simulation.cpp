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
    io::CSVReader<3> in(fmt::format("{}/log_pressure_rotate.csv", SOFTTRUNK_PROJECT_DIR));
    in.read_header(io::ignore_extra_column, "time(sec)", "p_des[0]", "p_des[1]");

    // read the CSV, and save the datapoints

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

    double dt = 0.0002;
    srl::Rate r{1./dt};
    double t, p0, p1;
    for (double t = 0; t < 3; t+=dt)
    {
        // set the pressure to that of the CSV at the current time
        // p(0) = 000*100;//sinusoid(phase);
        // p(1) = sinusoid(phase + 2*PI/3);
        // p(2) = sinusoid(phase + 4*PI/3);
        // p(3) = sinusoid(phase);
        // p(4) = sinusoid(phase + 2*PI/3);
        // p(5) = sinusoid(phase + 4*PI/3);


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