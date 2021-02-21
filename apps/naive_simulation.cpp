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
    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd p = VectorXd::Zero(3*st_params::num_segments);

    // initialize pose- set same random curvature to all sections in the same segment
    for (int i = 0; i < st_params::num_segments; i++)
    {
        // set to have about the same curvature as a whole regardless of scale
        Vector2d rand = 1.3 / st_params::sections_per_segment / st_params::num_segments * Vector2d::Random();
        for (int j = 0; j < st_params::sections_per_segment; j++)
            q.segment(2*i*st_params::sections_per_segment + 2*j, 2) = rand;
    }

    // set initial pressure
    p[0] = 300 * 100;
    p[2] = 200 * 100;
    p[3] = 300 * 100;

    fmt::print("initial pose: {}\n", q.transpose());
    fmt::print("initial pressure: {}\n", p.transpose());

    std::fstream log_file;
    std::string filename = fmt::format("{}/log_sim.csv", SOFTTRUNK_PROJECT_DIR);
    fmt::print("outputting log to {}...\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "timestamp, x, y, z\n";
    
    VectorXd ddq;
    VectorXd q_mid;
    VectorXd dq_mid;
    VectorXd ddq_mid;

    double dt = 0.002;
    srl::Rate r{1./dt};
    for (double t = 0; t < 10; t+=dt)
    {
        stm.updateState(q, dq);
        log_file << fmt::format("{}, {}, {}, {}\n", t, stm.H_tip.translation()(0), stm.H_tip.translation()(1), stm.H_tip.translation()(2));
        ddq = stm.B.inverse() * (stm.A * p - stm.C * dq - (-stm.g) - stm.K * q  -stm.D * dq);

        dq_mid = dq + ddq * dt / 2;
        q_mid = q + dq * dt / 2;
        stm.updateState(q_mid, dq_mid);
        ddq_mid = stm.B.inverse() * (stm.A * p - stm.C * dq_mid - (-stm.g) - stm.K * q_mid  -stm.D * dq_mid);
        
        dq = dq + ddq_mid * dt;
        q = q + dq_mid * dt;

        r.sleep();
        }
    log_file.close();
}