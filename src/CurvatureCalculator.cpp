#include "3d-soft-trunk/CurvatureCalculator.h"


CurvatureCalculator::CurvatureCalculator(const SoftTrunkParameters& st_params, CurvatureCalculator::SensorType sensor_type, std::string address, int extra_frames): st_params(st_params), sensor_type(sensor_type), extra_frames(extra_frames) {
    assert(st_params.is_finalized());
    // initialize size of arrays that record transforms
    abs_transforms.resize(st_params.num_segments + 1 + extra_frames);

    if (sensor_type == CurvatureCalculator::SensorType::qualisys){
        fmt::print("Using Qualisys to measure curvature...\n");
        setupQualisys();
    }
    else if (sensor_type == CurvatureCalculator::SensorType::bend_labs){
        fmt::print("Using Bend Labs sensor to measure curvature...\n");
        setupIntegratedSensor(address);
    }

    calculatorThread = std::thread(&CurvatureCalculator::calculator_loop, this);

    state.setSize(st_params.q_size);
}

void CurvatureCalculator::setupQualisys() {
    std::vector<int> emptyCameraList = {};
    optiTrackClient = std::make_unique<QualisysClient>(st_params.num_segments + 1 + extra_frames, emptyCameraList, "6D", true);
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
        // log both parametrizations
        for (int i = 0; i < st_params.num_segments; ++i)
            log_file << fmt::format(", La_{}, Lb_{}", i, i);
        for (int i = 0; i < st_params.num_segments; i++)
            log_file << fmt::format(", phi_{}, theta_{}", i, i);
        log_file << "\n";
    }

    srl::State state_prev = st_params.getBlankState();
    double interval = 0.001;
    srl::Rate rate{1. / interval};
    run = true;
    unsigned long long int last_timestamp;
    double interval_measured; // actual measured interval between timesteps
    while (run) {
        rate.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        // first get data from the sensors
        if (sensor_type == CurvatureCalculator::SensorType::qualisys){
            // first, update the internal data for transforms of each frame
            optiTrackClient->getData(abs_transforms, timestamp);

            // if ANY of the frames has not been received, disregard the entire data.
            bool all_frames_received = true;
            for (int i=0; i<st_params.num_segments+1; i++){
                if (std::isnan(abs_transforms[i](0,0)))
                    all_frames_received = false;
            }
            if (!all_frames_received)
                continue;

            // ignore if current timestep is same as previous
            if (last_timestamp == timestamp)
                continue;
            interval_measured = (timestamp - last_timestamp) / 1.0e6;
            last_timestamp = timestamp;
        }
        else if (sensor_type == CurvatureCalculator::SensorType::bend_labs){
            timestamp += 1; /** @todo somehow get timestamp for bend lab too */
            serialInterface->getData(bendLab_data);
            interval_measured = interval;
        }

        calculateCurvature();
        state.dq = (state.q - state_prev.q) / interval_measured;
        state.ddq = (state.dq - state_prev.dq) / interval_measured;
        state_prev.q = state.q;
        state_prev.dq = state.dq;
        if (log) {
            log_file << timestamp;
            for (int i = 0; i < 2 * st_params.num_segments; ++i)
                log_file << fmt::format(", {}", state.q(i));
            double phi, theta;
            for (int i = 0; i < st_params.num_segments; i++){
                longitudinal2phiTheta(state.q(2*i), state.q(2*i+1), phi, theta);
                log_file << fmt::format(", {}, {}", phi, theta);
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

void CurvatureCalculator::get_curvature(srl::State &state) {
    //std::lock_guard<std::mutex> lock(mtx);
    state = this->state;
}

void CurvatureCalculator::get_tip_posision(VectorXd &position) {
    //std::lock_guard<std::mutex> lock(mtx);
    MatrixXd matrix;
    matrix = (this->abs_transforms[0].inverse() * this->abs_transforms[2]).matrix();
    position = matrix.block<3,1>(0,3);
}

unsigned long long int CurvatureCalculator::get_timestamp(){
    std::lock_guard<std::mutex> lock(mtx);
    return timestamp;
}

Eigen::Transform<double, 3, Eigen::Affine> CurvatureCalculator::get_frame(int id){
    //std::lock_guard<std::mutex> lock(mtx);
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
            /** @todo change 1 to st_params.num_segments */
            state.q(2*i+0) = bendLab_data[2*i+1] * PI / 180.;
            state.q(2*i+1) = bendLab_data[2*i+0] * PI / 180.;
        }
        return;
    }
    MatrixXd matrix;
    double phi, theta;
    // next, calculate the parameters
    for (int i = 0; i < st_params.num_segments; i++) {
        matrix = (abs_transforms[i].inverse() * abs_transforms[i + 1]).matrix();
        // see documentation in header file for how this is calculated
        if (calcMethod == CalcMethod::orientation)
        {
            phi = atan2(matrix(1, 2), matrix(0, 2));
            theta = acos(matrix(2,2));
        }
        else if (calcMethod == CalcMethod::position)
        {
            phi = atan2(matrix(1, 3), matrix(0, 3));
            theta = a2theta(sqrt(pow(matrix(0,3), 2) + pow(matrix(1,3), 2)), L);
        }
        phiTheta2longitudinal(phi, theta, state.q(2*i), state.q(2*i+1));
         state.q(2*i) = phi;
         state.q(2*i+1) = theta;
    }
}



CurvatureCalculator::~CurvatureCalculator() {
    run = false;
    calculatorThread.join();
}


