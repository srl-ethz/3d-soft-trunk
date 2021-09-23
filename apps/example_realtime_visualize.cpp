//
// Created by yasu on 21/12/2020.
//

#include "3d-soft-trunk/Sensors/CurvatureCalculator.h"
#include "3d-soft-trunk/Models/AugmentedRigidArm.h"

int main() {
    // obtain data from motion tracking system and update the rigid model visualization in real time.
    // run drake-visualizer as well to see visualization
    // also calculates PCC FK tip position (relative to base link) and logs it, together with ground truth tip position

    SoftTrunkParameters st_params;
    st_params.finalize();
    std::string filename = fmt::format("{}/log_tip_diff.csv", SOFTTRUNK_PROJECT_DIR);
    std::fstream log_file;
    fmt::print("logging tip positions to {}...\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "t[microsec], x_fk, y_fk, z_fk, x_gt, y_gt, z_gt, distance\n";

    CurvatureCalculator cc{st_params, CurvatureCalculator::SensorType::qualisys};
    AugmentedRigidArm ara{st_params};
    srl::State state = st_params.getBlankState();
    Eigen::Transform< double, 3, Eigen::Affine > H_tip_groundtruth; // pose of tip, relative to base (ground truth data from qualisys)

    unsigned long long int timestamp; // microsecond timestamp from Qualisys data
    srl::Rate r{30};
    while (true) {
        cc.get_curvature(state);
        timestamp = cc.get_timestamp();
        ara.update(state);

        // log tip position data
        log_file << fmt::format("{}", timestamp);
        log_file << fmt::format(", {}, {}, {}", ara.get_H_tip().translation()(0), ara.get_H_tip().translation()(1), ara.get_H_tip().translation()(2));
        H_tip_groundtruth = cc.get_frame(0).inverse() * cc.get_frame(st_params.num_segments);
        log_file << fmt::format(", {}, {}, {}", H_tip_groundtruth.translation()(0), H_tip_groundtruth.translation()(1), H_tip_groundtruth.translation()(2));
        log_file << fmt::format(", {}\n", (ara.get_H_tip().translation() - H_tip_groundtruth.translation()).norm());

        fmt::print("----------\n{}\n", state.q.transpose());
        r.sleep();
    }
    return 1;
}
