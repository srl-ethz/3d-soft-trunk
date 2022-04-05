#include "3d-soft-trunk/Controllers/QuasiStatic.h"

Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

void gain(QuasiStatic& QuasiStatic){ //change parameters online with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                QuasiStatic.kp_*=1.1;
                break;
            case 'a':
                QuasiStatic.kp_*=0.9;
                break;
            case 'e':
                QuasiStatic.kd_*=1.1;
                break;
            case 'd':
                QuasiStatic.kd_*=0.9;
                break;
            case 'm':
                QuasiStatic.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", QuasiStatic.kp_, QuasiStatic.kd_);
    }
}

void printer(QuasiStatic& QuasiStatic){ //print some stuff every once in a while
    srl::Rate r{0.3};
    while(true){
        fmt::print("------------------------------------\n");
        fmt::print("q: {}\n",QuasiStatic.state_.q.transpose());
        fmt::print("x tip: {}\n", QuasiStatic.x_.transpose());
        fmt::print("x target: {}\n", QuasiStatic.x_ref_.transpose());
        fmt::print("x error normalized: {}\n", (x_ref-QuasiStatic.x_).norm());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("stateoftheart.yaml");
    st_params.finalize();
    QuasiStatic QuasiStatic(st_params);
    VectorXd p;
    srl::State state = st_params.getBlankState();

    Vector3d x_ref_center;
    
    //x_ref << 0.1,0.00,-0.235;


    
    double t = 0;
    double dt = 0.05;
    Vector3d traj1;
    Vector3d traj2;

    double period = 37;
    double coef = 2 * 3.1415 / period;
    QuasiStatic.gripperAttached_ = true;
    QuasiStatic.loadAttached_ = 0;

    getchar();
    std::thread print_thread(printer, std::ref(QuasiStatic));
    std::thread gain_thread(gain, std::ref(QuasiStatic));

    srl::sleep(0.1);

    const drake::math::RigidTransformd X_W_B {QuasiStatic.state_.tip_transforms[0].matrix()}, X_W_T{QuasiStatic.state_.tip_transforms[2].matrix()};
    const auto X_B_T {X_W_B.inverse() * X_W_T};

    x_ref << 0, 0, 0.05;
    x_ref = X_W_T*x_ref;

    QuasiStatic.set_ref(x_ref, dx_ref, ddx_ref);

    getchar();
    QuasiStatic.toggle_log();
    t=0;
    while (t<1.3*period){ //circle
        double radius = 0.05;
        //vertical configuration circle
        /*
        circle << radius*cos(coef*t), radius*sin(coef*t),-0.235;
        d_circle << -radius*coef*sin(coef*t), radius*coef*cos(coef*t),0;
        dd_circle << -radius*coef*coef*cos(coef*t), -radius*coef*coef*sin(coef*t),0;
        */

        // horizontal configuration circle
        /*
        circle << 0, -radius * sin(coef * t), radius * cos(coef * t);
        d_circle << 0, radius * coef * cos(coef * t), -radius * coef * sin(coef * t);
        dd_circle << 0, -radius * coef * coef * sin(coef * t), -radius * coef * coef * cos(coef * t);
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
        QuasiStatic.set_ref(x_ref);
        
        t+=dt;
        srl::sleep(dt);
        
    }
    QuasiStatic.toggle_log();
    return 1;
}
