#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/VisualizerROS.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "example_VisualizerROS");

    SoftTrunkModel stm{};
    VisualizerROS vis{stm};
    srl::State state;
    Vector3d rgb;
    rgb << 1,0,0;
    Vector3d xyz;
    xyz << 0,0,0.1;

    double dt = 0.1;
    srl::Rate r{1./dt};
    for (double t = 0; t < 5; t += 0.1)
    {
        state.q(0) = sin(t);
        state.q(1) = cos(t);
        stm.updateState(state);
        vis.publishState();
        vis.publishArrow(1, xyz, rgb, false);
        r.sleep();
    }
}