#include <mobilerack-interface/QualisysClient.h>
#include <3d-soft-trunk/SoftTrunk_common.h>
#include <fstream>

/**
 * @brief gets frames from QTM and records the tip of each segment (rel. to base) and saves log with timestamps 
 */
int main(){
    QualisysClient qc{"192.168.0.101", st_params::num_segments + 1};
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames;
    unsigned long long int timestamp;

    std::string filename = fmt::format("{}/log_qtm_frames.csv", SOFTTRUNK_PROJECT_DIR);
    std::fstream log_file;
    fmt::print("outputting to {}...\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "timestamp";
    for (int i = 0; i < st_params::num_segments; i++)
    {
        log_file << fmt::format(", x_{}, y_{}, z_{}", i, i, i);
    }
    log_file << "\n";

    VectorXd x;
    srl::Rate r{50};
    while (true) {
        qc.getData(frames, timestamp);
        log_file << (double)timestamp/1.e6; // qtm timestamp is in microseconds
        for (int i = 0; i < st_params::num_segments; i++)
        {
            x = (frames[0].inverse() * frames[i+1]).translation();
            log_file << fmt::format(", {}, {}, {}", x(0), x(1), x(2));
        }
        log_file << "\n";
        r.sleep();
    }
}