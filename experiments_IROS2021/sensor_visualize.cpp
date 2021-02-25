#include <3d-soft-trunk/SoftTrunkModel.h>
#include <mobilerack-interface/SerialInterface.h>

/**
 * @brief just visualize the sensors assuming that for a single segment, it is constant curvature. 
 */
int main(){
    SoftTrunkModel stm{};
    SerialInterface si{"/dev/ttyACM0", 38400};

    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(q.size());
    std::vector<float> bendLab_data;
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
    fmt::print("sensor offset is {}\n", bendLab_offset.transpose());
    
    srl::Rate r{30};
    while (true)
    {
        si.getData(bendLab_data);
        for (int i = 0; i < st_params::num_segments*st_params::sections_per_segment; i++)
        {
            // copy data from bendlab to Soft Trunk pose
            // divide curvauture equally across PCC sections
            int segment_id = i / st_params::sections_per_segment; // switch order
            q(2*i) = (bendLab_data[2*segment_id] - bendLab_offset[2*segment_id]) * PI / 180. / st_params::sections_per_segment;
            q(2*i+1) = (bendLab_data[2*segment_id+1] - bendLab_offset[2*segment_id+1]) * PI / 180. / st_params::sections_per_segment;
        }
        stm.updateState(q, dq);
        r.sleep();
    }
    
}
