#include "CurvatureCalculator.h"


CurvatureCalculator::CurvatureCalculator(int sensorType)
        : sensorType(sensorType) {
    // initialize size of arrays that record transforms
    for (int i = 0; i <= NUM_ELEMENTS; i++) {
        abs_transforms.push_back(Eigen::Transform<double, 3, Eigen::Affine>().Identity());
    }
    for (int j = 0; j < NUM_ELEMENTS; ++j) {
        rel_transforms.push_back(Eigen::Transform<double, 3, Eigen::Affine>().Identity());
    }
}

void CurvatureCalculator::setupOptiTrack(std::string localAddress,
                                         std::string serverAddress) {
    std::cout << "Setting up connection with OptiTrack...\n";
    if (sensorType != USE_OPTITRACK) {
        std::cout << "error: CurvatureCalculator not set up to use OptiTrack"
                  << '\n';
        return;
    }
    optiTrackClient = new OptiTrackClient(localAddress, serverAddress);
    std::cout << "done.\n";
}

void CurvatureCalculator::start() {
    std::cout << "starting CurvatureCalculator's calculator thread... \n";

    run = true;
    calculatorThread = std::thread(&CurvatureCalculator::calculatorThreadFunction, this);

    // get the current q, to subtract from measurements to get rid of the offset
    std::cout << "Waiting for 2 seconds to wait for the arm to stop swinging, and measure the initial q... \n";
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    initial_q = q;
    std::cout << "initial q measured. This value is considered the offset and will be subtracted from future measurements.\n";
}

void CurvatureCalculator::calculatorThreadFunction() {
    Vector2Nd prev_q = Vector2Nd::Zero();
    Vector2Nd prev_dq = Vector2Nd::Zero();
    double interval = 0.005; // loop interval
    while (run) {
        // this loop continuously monitors the current state.
        calculateCurvature();
        // todo: is there a smarter algorithm to calculate time derivative, that can smooth out noises?
//        presmooth_dq = (q - prev_q) / interval;
        dq = (q - prev_q) / interval;;// (1 - 0.2) * presmooth_dq + 0.2 * dq;
//        presmooth_ddq = (dq - prev_dq) / interval;
        ddq = (dq - prev_dq) / interval;;//(1 - 0.2) * presmooth_ddq + 0.2 * ddq;
        prev_q = Vector2Nd(q);
        prev_dq = Vector2Nd(dq);
        std::this_thread::sleep_for(std::chrono::milliseconds((int) (interval * 1000)));
    }
}

void CurvatureCalculator::setupIntegratedSensor() {
    // for future.
}

double sign(double val) {
    if (val == 0) return 0.0;
    else if (val > 0) return 1.0;
    else return -1.0;
}

void CurvatureCalculator::calculateCurvature() {
    // first, update the internal data for transforms of each frame
    if (sensorType == USE_OPTITRACK) {
        std::vector<RigidBody> rigidBodies = optiTrackClient->getData();
        for (int i = 0; i < rigidBodies.size(); i++) {
            int id = rigidBodies[i].id();
            if (0 <= id && id < NUM_ELEMENTS + 1) {
                Point3f position = rigidBodies[i].location();

                Quaternion4f quaternion = rigidBodies[i].orientation();
                Eigen::Quaterniond quaternion_eigen;

                quaternion_eigen.x() = quaternion.qx;
                quaternion_eigen.y() = quaternion.qy;
                quaternion_eigen.z() = quaternion.qz;
                quaternion_eigen.w() = quaternion.qw;
                quaternion_eigen.normalize();
                abs_transforms[id] = quaternion_eigen * Eigen::Translation3d(position.x, position.y, position.z);
            }
        }
    } else if (sensorType == USE_INTEGRATEDSENSOR) {
        // to be written?
    }

    // next, calculate the parameters
    for (int i = 0; i < NUM_ELEMENTS; i++) {
        rel_transforms[i] = abs_transforms[i + 1] * abs_transforms[i].inverse();

        Eigen::Transform<double, 3, Eigen::Affine>::MatrixType matrix = rel_transforms[i].matrix();
        phi = atan(matrix(1, 3) / matrix(0, 3)); // -PI/2 ~ PI/2
        theta = sign(matrix(0, 3)) * fabs(asin(sqrt(pow(matrix(0, 2), 2) + pow(matrix(1, 2), 2))));

        q(2 * i) = -TRUNK_RADIUS * cos(phi) * theta; // deltaLa (the difference in the length of La compared to neutral state)
        q(2 * i + 1) = -TRUNK_RADIUS * cos(PI / 2 - phi) * theta; // deltaLb
    }
    q -= initial_q;
}

void CurvatureCalculator::stop() {
    std::cout << "stopping CurvatureCalculator's calculator thread...\n";
    run = false;
    calculatorThread.join();
    if (sensorType == USE_OPTITRACK) {
        optiTrackClient->stop();
        delete optiTrackClient;
    }
}
