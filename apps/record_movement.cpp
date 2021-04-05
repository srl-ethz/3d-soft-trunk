#include <3d-soft-trunk/ControllerPCC.h>
#include <mobilerack-interface/SerialInterface.h>
#include <3d-soft-trunk/Simulator.h>

int main(){
    SoftTrunkModel stm{};
    SerialInterface si{"/dev/ttyACM0", 38400};
    std::string filename;
    std::fstream log_file;
    srl::State state;
    std::vector<float> bendLab_data;
    double time = 0;
    VectorXd bendLab_offset = VectorXd::Zero(4); // use the first N measurements as offset
    int N = 10;
    fmt::print("taking {} measurements to calibrate baseline. Keep arm straight and don't move it...\n", N);
    for (int i = 0; i < N; i++)
    {
        si.getData(bendLab_data);
        for (int j = 0; j < 4; j++)
            bendLab_offset(j) += bendLab_data[j];
        srl::sleep(0.1);
    }
    bendLab_offset /= N;
    fmt::print("sensor offset is {}\n Enter filename to log to, logging starts after entering\n", bendLab_offset.transpose());
    std::cin >> filename;
    filename = filename + ".csv";

    log_file.open(filename, std::fstream::out);
    log_file << fmt::format("{},{}\n", st_params::num_segments, st_params::sections_per_segment);
    for (int j = 0; j < 4; j++)
        log_file << fmt::format("{}, ", bendLab_offset(j));
    log_file << "\n";

    srl::Rate r{30};
    while (true)
    {
        si.getData(bendLab_data);
        log_file << time;
        for (int i = 0; i < st_params::num_segments*st_params::sections_per_segment; i++)
        {
            // copy data from bendlab to Soft Trunk pose
            // divide curvauture equally across PCC sections
            int segment_id = i / st_params::sections_per_segment; // switch order
            state.q(2*i) = (bendLab_data[2*segment_id] - bendLab_offset[2*segment_id]) * PI / 180. / st_params::sections_per_segment;
            state.q(2*i+1) = (bendLab_data[2*segment_id+1] - bendLab_offset[2*segment_id+1]) * PI / 180. / st_params::sections_per_segment;
            log_file << fmt::format(",{},{}", state.q(2*i), state.q(2*i+1));
        }
        stm.updateState(state);
        log_file << "\n";
        time += 1./30;
        r.sleep();
    }
    
}
