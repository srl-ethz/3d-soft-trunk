#include "3d-soft-trunk/Sensors/MotionCapture.h"

MotionCapture::MotionCapture(const SoftTrunkParameters& st_params) : st_params_(st_params){
    assert(st_params.is_finalized());

    //initialize transformation vector to also contain objects    
    abs_transforms_.resize(st_params_.num_segments + 1 + st_params_.objects + st_params_.prismatic);

    //initialize client
    std::vector<int> emptyCameraList = {};
    optiTrackClient = std::make_unique<QualisysClient>(st_params.num_segments + 1 + st_params_.prismatic + st_params_.objects, emptyCameraList, "6D", true);

    state_ = st_params_.getBlankState();

    calculatorThread = std::thread(&MotionCapture::calculator_loop, this);
    fmt::print("Motion Capture initialized with {} extra objects.\n", st_params_.objects);
}

MotionCapture::~MotionCapture(){
    run_ = false;
    calculatorThread.join();
}

Eigen::Transform<double, 3, Eigen::Affine> MotionCapture::get_frame(int id){
    assert(0 <= id && id < abs_transforms_.size());
    return abs_transforms_[id];
}


void MotionCapture::calculator_loop(){
    std::fstream log_file;
    
    std::string filename = "qualisys_log.csv";
    fmt::print("logging to {}\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "timestamp";
    for (int i = 0; i < st_params_.q_size; ++i)
        log_file << fmt::format(", q_{}", i);
    log_file << "\n";


    srl::State state_prev_ = st_params_.getBlankState();

    double frequency = 500.;
    if (st_params_.sensor_refresh_rate < 500.){     //qualisys max refresh rate is 500hz
        frequency = st_params_.sensor_refresh_rate;                           
    }
    srl::Rate rate{frequency};

    run_ = true;
    double interval_measured; // actual measured interval between timesteps

    while(run_){
        rate.sleep();
        optiTrackClient->getData(abs_transforms_, timestamp_);

        // if ANY of the frames has not been received, disregard the entire data.
        bool all_frames_received = true;
        for (int i=0; i<st_params_.num_segments+1; i++){
            if (std::isnan(abs_transforms_[i](0,0)))
                all_frames_received = false;
        }
        if (!all_frames_received)
            continue;

        // ignore if current timestep is same as previous
        if (last_timestamp_ == timestamp_)
            continue;
        interval_measured = (timestamp_ - last_timestamp_) / 1.0e6;
        last_timestamp_ = timestamp_;

        double phi, theta;
        MatrixXd matrix;


        mtx.lock(); //ensure that while the state is being changed nothing is getting pulled

        Matrix3d rot;
        rot << 0, 0, -1, 0, 1, 0, 1, 0, 0; //this matrix rotates the base frame into the desired orientation;
        Matrix3d rot_trans;
        rot_trans << -1, 0, 0, 0, 1, 0, 0, 0, -1;
        for (int i = st_params_.num_segments + st_params_.prismatic; i >= 0; i--){
            //make translation relative to base frame and align orientation correctly
            abs_transforms_[i].translation() = rot*rot_trans*(abs_transforms_[i].translation() - abs_transforms_[0].translation());
            //bring rotation into desired orientation
            abs_transforms_[i].matrix().block(0,0,3,3) = abs_transforms_[i].matrix().block(0,0,3,3)*rot;
        }

        for (int i = 0; i < st_params_.num_segments; i++) {
            matrix = (abs_transforms_[i + st_params_.prismatic].inverse() * abs_transforms_[i + 1 + st_params_.prismatic]).matrix();
            // calculates phi, theta based on orientation w.r.t z-axis
            phi = atan2(matrix(1, 2), matrix(0, 2));
            theta = acos(matrix(2,2));

            //convert to desired coordinate type
            switch (st_params_.coord_type) {
                case CoordType::phitheta:
                    state_.q(2*i+st_params_.prismatic) = phi;
                    state_.q(2*i+1+st_params_.prismatic) = theta;
                    break;
                case CoordType::thetax:
                    state_.q(2*i+st_params_.prismatic) = -cos(phi) * theta;
                    state_.q(2*i+1+st_params_.prismatic) = -sin(phi) * theta;
                    break;
            }
            if (st_params_.prismatic){
                state_.q(0) = (abs_transforms_[0].translation()-abs_transforms_[1].translation()).norm();
            }
        }     

        

        //derivatives are evaluated numerically
        state_.dq = (state_.q - state_prev_.q) / interval_measured;
        state_.ddq = (state_.dq - state_prev_.dq) / interval_measured;
        state_.timestamp = timestamp_;

        // all transforms belonging to the arm go into the tip transform vector
        for (int i = 0; i < st_params_.num_segments + 1 + st_params_.prismatic; i++){
            state_.tip_transforms[i] = abs_transforms_[i];
        }

        //objects to object vector
        for (int i = 0; i < st_params_.objects; i++){
            state_.objects[i] = abs_transforms_[st_params_.num_segments + 1 + st_params_.prismatic + i];
        }

        mtx.unlock(); //unlock the mutex after modifying state

        state_prev_ = state_;


        log_file << timestamp_;
        for (int i = 0; i < 2 * st_params_.num_segments; ++i)
            log_file << fmt::format(", {}", state_.q(i));
        log_file << "\n";
    }
    log_file.close();
}
