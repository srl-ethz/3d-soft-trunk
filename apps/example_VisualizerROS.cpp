#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/VisualizerROS.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "example_VisualizerROS");

    SoftTrunkParameters st_params{};
    st_params.finalize();
    SoftTrunkModel stm{st_params};
    VisualizerROS vis{stm};
    srl::State state = st_params.getBlankState();
    Vector3d rgb;
    rgb << 1,0,0;
    Vector3d xyz;
    xyz << 0,0,0.1;

    double dt = 0.1;
    srl::Rate r{1./dt};
    for (double t = 0; t < 50; t += 0.1)
    {
        for (int i = 0; i < state.q.size()/2; i++)
        {
            state.q(2*i) = 2 * sin(t) / state.q.size();
            state.q(2*i+1) = 2 * cos(t) / state.q.size();;
        }
        stm.updateState(state);
        vis.publishState();
        vis.publishArrow(1, xyz, rgb, false);
        r.sleep();
    }
}