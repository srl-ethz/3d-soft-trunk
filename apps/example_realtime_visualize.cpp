//
// Created by yasu on 21/12/2020.
//

#include "3d-soft-trunk/CurvatureCalculator.h"
#include "3d-soft-trunk/AugmentedRigidArm.h"

int main() {
    // obtain data from motion tracking system and update the rigid model visualization in real time.
    // run drake-visualizer as well to see visualization
    // also calculates PCC FK tip position (relative to base link) and logs it, together with ground truth tip position

    std::string filename = fmt::format("{}/log_tip_diff.csv", SOFTTRUNK_PROJECT_DIR);
    std::fstream log_file;
    fmt::print("logging tip positions to {}...\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "t, x_fk, y_fk, z_fk, x_gt, y_gt, z_gt, distance\n";

    CurvatureCalculator cc{CurvatureCalculator::SensorType::qualisys, "192.168.0.0"};
    AugmentedRigidArm ara{};
    VectorXd q, dq, ddq;
    Eigen::Transform< double, 3, Eigen::Affine > H_tip_groundtruth;

    double dt = 1./30;
    Rate r{1./dt};
    for (double t=0; true; t+=dt) {
        cc.get_curvature(q, dq, ddq);
        ara.update(q, dq);

        // log tip position data
        log_file << fmt::format("{}", t);
        log_file << fmt::format(", {}, {}, {}", ara.H_tip.translation()(0), ara.H_tip.translation()(1), ara.H_tip.translation()(2));
        H_tip_groundtruth = cc.get_frame(0).inverse() * cc.get_frame(st_params::num_segments);
        log_file << fmt::format(", {}, {}, {}", H_tip_groundtruth.translation()(0), H_tip_groundtruth.translation()(1), H_tip_groundtruth.translation()(2));
        log_file << fmt::format(", {}\n", (ara.H_tip.translation() - H_tip_groundtruth.translation()).norm());

        fmt::print("----------\n{}\n", q.transpose());
        r.sleep();
    }
    return 1;
}