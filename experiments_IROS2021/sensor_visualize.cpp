#include <3d-soft-trunk/SoftTrunkModel.h>
#include <mobilerack-interface/SerialInterface.h>

/**
 * @brief just visualize the sensors assuming that for a single segment, it is constant curvature. 
 */
int main(){
    SoftTrunkParameters st_params;
    st_params.finalize();
    SoftTrunkModel stm{st_params};
    SerialInterface si{"/dev/ttyACM0", 115200};

    srl::State state = st_params.empty_state();
    std::vector<float> bendLab_data;
    VectorXd bendLab_offset = VectorXd::Zero(6); // use the first N measurements as offset
    int N = 10;
    fmt::print("taking {} measurements to calibrate baseline. Keep arm straight and don't move it...\n", N);
    srl::sleep(0.5); // wait for sensor value to settle down initially
    for (int i = 0; i < N; i++)
    {
        si.getData(bendLab_data);
        for (int j = 0; j < 6; j++)
            bendLab_offset(j) += bendLab_data[j];
        srl::sleep(0.1);
    }
    bendLab_offset /= N;
    fmt::print("sensor offset is {}\n", bendLab_offset.transpose());
    
    srl::Rate r{30};
    while (true)
    {
        si.getData(bendLab_data);
        for (int i = 0; i < st_params.num_segments*st_params.sections_per_segment; i++)
        {
            // copy data from bendlab to Soft Trunk pose
            // divide curvauture equally across PCC sections
            int segment_id = st_params.num_segments - 1 - i / st_params.sections_per_segment; // switch order
            state.q(2*i) = (bendLab_data[2*segment_id] - bendLab_offset[2*segment_id]) * PI / 180. / st_params.sections_per_segment;
            state.q(2*i+1) = (bendLab_data[2*segment_id+1] - bendLab_offset[2*segment_id+1]) * PI / 180. / st_params.sections_per_segment;
        }
        stm.updateState(state);
        r.sleep();
    }
    
}
