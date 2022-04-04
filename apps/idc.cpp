#include "3d-soft-trunk/Controllers/IDCon.h"

Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

void gain(IDCon& IDCon){ //change parameters online with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                IDCon.kp_*=1.1;
                break;
            case 'a':
                IDCon.kp_*=0.9;
                break;
            case 'e':
                IDCon.kd_*=1.1;
                break;
            case 'd':
                IDCon.kd_*=0.9;
                break;
            case 'm':
                IDCon.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", IDCon.kp_, IDCon.kd_);
    }
}

void printer(IDCon& IDCon){ //print some stuff every once in a while
    srl::Rate r{0.3};
    while(true){
        fmt::print("------------------------------------\n");
        fmt::print("q: {}\n",IDCon.state_.q.transpose());
        fmt::print("x tip: {}\n", IDCon.x_.transpose());
        fmt::print("x target: {}\n", IDCon.x_ref_.transpose());
        fmt::print("x error normalized: {}\n", (x_ref-IDCon.x_).norm());
        fmt::print("gravity: {}\n",IDCon.gravity_compensate(IDCon.state_).transpose());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("stiffness_vertical_test2.yaml");
    st_params.finalize();
    IDCon IDCon(st_params);
    VectorXd p;
    srl::State state = st_params.getBlankState();

    Vector3d x_ref_center;
    
    //x_ref << 0.1,0.00,-0.235;


    
    double t = 0;
    double dt = 1./60;
    Vector3d traj1;
    Vector3d traj2;
    Vector3d d_traj;
    Vector3d dd_traj;

    double period = 8;
    double coef = 2 * 3.1415 / period;
    IDCon.gripperAttached_ = true;
    IDCon.loadAttached_ = 0;
    getchar();

    const drake::math::RigidTransformd X_W_B {IDCon.state_.tip_transforms[0].matrix()}, X_W_T{IDCon.state_.tip_transforms[2].matrix()};
    const auto X_B_T {X_W_B.inverse() * X_W_T};
    x_ref << 0.0, 0.0, 0.05;
    x_ref << X_W_T*x_ref;

    getchar();
    std::thread print_thread(printer, std::ref(IDCon));
    std::thread gain_thread(gain, std::ref(IDCon));
    IDCon.set_ref(x_ref, dx_ref, ddx_ref);
    srl::sleep(0.1);

    getchar();
    IDCon.toggle_log();
    t=0;
    while (t<2.5*period){ 
        double radius = 0.05;
        //vertical configuration circle
        /*
        x_ref << radius*cos(coef*t), radius*sin(coef*t),-0.235;
        dx_ref << -radius*coef*sin(coef*t), radius*coef*cos(coef*t),0;
        ddx_ref << -radius*coef*coef*cos(coef*t), -radius*coef*coef*sin(coef*t),0;
        */

        // horizontal configuration circle
        /*
        x_ref << 0, -radius * sin(coef * t), radius * cos(coef * t);
        dx_ref << 0, radius * coef * cos(coef * t), -radius * coef * sin(coef * t);
        ddx_ref << 0, -radius * coef * coef * sin(coef * t), -radius * coef * coef * cos(coef * t);
        */

       //horizontal configuration triangle
       double loc = std::fmod(t,period);
       if (loc < period/3) {
            traj1 << 0, radius*sin(0), radius*cos(0);
            traj2 << 0, radius*sin(2*3.1415/3), radius*cos(2*3.1415/3);
       } else if (loc < 2*period/3){
           traj1 << 0, radius*sin(2*3.1415/3), radius*cos(2*3.1415/3);
           traj2 << 0, radius*sin(2*2*3.1415/3), radius*cos(2*2*3.1415/3);
       } else if (loc < period) {
           traj1 << 0, radius*sin(2*2*3.1415/3), radius*cos(2*2*3.1415/3);
           traj2 << 0, radius*sin(0), radius*cos(0);
       }

        double pos = std::fmod(t,period/3);
        x_ref = traj1 + pos*(traj2 - traj1)/(period/3);
        dx_ref = (traj2 - traj1)/(period/3);
        ddx_ref << 0, 0, 0;

        //translate into horizontal
        x_ref = X_W_T * x_ref;
        dx_ref = X_W_T.rotation() * dx_ref;
        ddx_ref = X_W_T.rotation() * ddx_ref;

        IDCon.set_ref(x_ref,dx_ref, ddx_ref);
        
        t+=dt;
        srl::sleep(dt);
    }
    IDCon.toggle_log();
    return 1;
}
