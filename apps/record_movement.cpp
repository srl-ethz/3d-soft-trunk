#include <3d-soft-trunk/ControllerPCC.h>
#include <mobilerack-interface/SerialInterface.h>
#include <3d-soft-trunk/Simulator.h>

int main(){
    SoftTrunkModel stm{};
    
    std::string filename;
    std::fstream log_file;
    srl::State state;
    double time = 0;

    CurvatureCalculator cc(CurvatureCalculator::SensorType::qualisys);

    fmt::print("\n Enter filename to log to, logging starts after entering\n");
    std::cin >> filename;
    filename = filename + ".csv";

    log_file.open(filename, std::fstream::out);     //create header
    log_file << "time";
    for (int i = 0; i < st_params::q_size; i++){
        log_file << fmt::format(",q_{}", i);
    }

    srl::Rate r{30};
    while (true)
    {
        cc.get_curvature(state);                    //log curvatures+time
        log_file << time;
        for (int i = 0; i < st_params::num_segments*st_params::sections_per_segment; i++)
        {
            log_file << fmt::format(",{},{}", state.q(2*i), state.q(2*i+1));
        }
        stm.updateState(state);
        log_file << "\n";
        time += 1./30;
        r.sleep();
    }
    
}
