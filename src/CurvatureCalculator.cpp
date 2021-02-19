#include "3d-soft-trunk/CurvatureCalculator.h"


CurvatureCalculator::CurvatureCalculator(CurvatureCalculator::SensorType sensor_type, std::string address): sensor_type(sensor_type) {
    // initialize size of arrays that record transforms
    abs_transforms.resize(st_params::num_segments + 1);

    q = VectorXd::Zero(st_params::num_segments * 2);
    dq = VectorXd::Zero(st_params::num_segments * 2);
    ddq = VectorXd::Zero(st_params::num_segments * 2);

    if (sensor_type == CurvatureCalculator::SensorType::qualisys){
        fmt::print("Using Qualisys to measure curvature...\n");
        setupQualisys(address);
    }
    else if (sensor_type == CurvatureCalculator::SensorType::bend_labs){
        fmt::print("Using Bend Labs sensor to measure curvature...\n");
        setupIntegratedSensor(address);
    }

    calculatorThread = std::thread(&CurvatureCalculator::calculator_loop, this);

    // get the current q, to subtract from measurements to get rid of the offset
//    fmt::print("Waiting for 2 seconds to wait for the arm to stop swinging, and measure the initial q... \n");
//    sleep(2);
//    initial_q = q;
//    fmt::print("initial q measured:{}. This value is considered the offset and will be subtracted from future measurements.\n", initial_q);
}

void CurvatureCalculator::setupQualisys(std::string qtm_address) {
    optiTrackClient = std::make_unique<QualisysClient>(qtm_address.c_str(), st_params::num_segments + 1);
}

void CurvatureCalculator::setupIntegratedSensor(std::string portname) {
    serialInterface = std::make_unique<SerialInterface>(portname, 38400);
}


void CurvatureCalculator::calculator_loop() {
    std::fstream log_file;
    if (log) {
        std::string filename = "log_curvature.csv";
        fmt::print("logging to {}\n", filename);
        log_file.open(filename, std::fstream::out);
        log_file << "timestamp";
        for (int i = 0; i < st_params::num_segments; ++i) {
            log_file << fmt::format(", La_{}, Lb_{}", i, i);
        }
        log_file << "\n";
    }

    VectorXd prev_q = VectorXd::Zero(q.size());
    VectorXd prev_dq = VectorXd::Zero(dq.size());
    double interval = 0.01;
    srl::Rate rate{1. / interval};
    run = true;
    unsigned long long int last_timestamp;
    while (run) {
        rate.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        // first get data from the sensors
        if (sensor_type == CurvatureCalculator::SensorType::qualisys){
            // first, update the internal data for transforms of each frame
            optiTrackClient->getData(abs_transforms, timestamp);
            // ignore if current timestep is same as previous
            if (last_timestamp == timestamp)
                continue;
            last_timestamp = timestamp;
        }
        else if (sensor_type == CurvatureCalculator::SensorType::bend_labs){
            timestamp += 1; /** @todo somehow get timestamp for bend lab too */
            serialInterface->getData(bendLab_data);
        }

        calculateCurvature();
        /** todo: is there a smarter algorithm to calculate time derivative, that can smooth out noises? */
//        presmooth_dq = (q - prev_q) / interval;
        dq = (q - prev_q) / interval;;// (1 - 0.2) * presmooth_dq + 0.2 * dq;
//        presmooth_ddq = (dq - prev_dq) / interval;
        ddq = (dq - prev_dq) / interval;;//(1 - 0.2) * presmooth_ddq e+ 0.2 * ddq;
        prev_q = q;
        prev_dq = dq;
        if (log) {
            log_file << timestamp;
            for (int i = 0; i < 2 * st_params::num_segments; ++i) {
                log_file << fmt::format(", {}", q(i));
            }
            log_file << "\n";
        }
    }
    if (log)
        log_file.close();
}

double sign(double val) {
    if (val == 0) return 0.0;
    else if (val > 0) return 1.0;
    else return -1.0;
}

void CurvatureCalculator::get_curvature(VectorXd &q, VectorXd &dq, VectorXd &ddq) {
    std::lock_guard<std::mutex> lock(mtx);
    q = this->q;
    dq = this->dq;
    ddq = this->ddq;
}

Eigen::Transform<double, 3, Eigen::Affine> CurvatureCalculator::get_frame(int id){
    std::lock_guard<std::mutex> lock(mtx);
    assert(0 <= id && id < abs_transforms.size());
    assert(sensor_type == SensorType::qualisys);
    return abs_transforms[id];
}

double a2theta(double a, double L){
    /**
     * @brief calculate curvature theta from measurment. Uses the solution of the 3rd-order approximation of actual function.
     * see document of CurvatureCalculator for other details.
     * solved y = x/2 - x^3/24 for x in Wolfram Alpha, then of the 3 solutions, used the one that corresponded to the desired area.
     * since its solution is a complex function, std::complex math is used.
     */
    static const std::complex<double> i(0,1);
    // declare static variables for a tiny bit of efficiency
    static std::complex<double> y;
    static std::complex<double> tmp;
    static std::complex<double> z;
    y = a / L; // Qualisys is in mm
    tmp = pow(pow(9. * y*y - 4., 0.5) - 3.*y, 1./3.);
    z = i * cbrt(2.) *(sqrt(3.) + i) / tmp - (1. + i*sqrt(3.)) * tmp / cbrt(2.);
    return z.real();
}

void CurvatureCalculator::calculateCurvature() {
    if (sensor_type == CurvatureCalculator::SensorType::bend_labs)
    {
        for (int i = 0; i < 1; i++){
            /** @todo change 1 to st_params::num_segments */
            q(2*i+0) = bendLab_data[2*i+1] * PI / 180.;
            q(2*i+1) = bendLab_data[2*i+0] * PI / 180.;
        }
        return;
    }
    MatrixXd matrix;
    double phi, theta;
    // next, calculate the parameters
    for (int i = 0; i < st_params::num_segments; i++) {
        matrix = (abs_transforms[i].inverse() * abs_transforms[i + 1]).matrix();
        // see documentation in header file for how this is calculated
        phi = atan2(matrix(1, 3), matrix(0, 3));
        theta = a2theta(sqrt(pow(matrix(0,3), 2) + pow(matrix(1,3), 2)), L);
        phiTheta2longitudinal(phi, theta, q(2*i), q(2*i+1));
    }
    // q -= initial_q; // implement better way to get rid of initial error...
}

CurvatureCalculator::~CurvatureCalculator() {
    run = false;
    calculatorThread.join();
}
