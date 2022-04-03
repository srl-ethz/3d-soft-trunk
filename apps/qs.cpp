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
    st_params.load_yaml("stiffness_vertical_test.yaml");
    st_params.finalize();
    QuasiStatic QuasiStatic(st_params);
    VectorXd p;
    srl::State state = st_params.getBlankState();

    Vector3d x_ref_center;
    
    //x_ref << 0.1,0.00,-0.235;
    x_ref << 0.265, 0.0, 0.0;


    
    double t = 0;
    double dt = 0.05;
    Vector3d circle;
    Vector3d d_circle;
    Vector3d dd_circle;

    double coef = 2 * 3.1415 / 46;
    QuasiStatic.gripperAttached_ = true;
    QuasiStatic.loadAttached_ = 0;

    getchar();
    std::thread print_thread(printer, std::ref(QuasiStatic));
    std::thread gain_thread(gain, std::ref(QuasiStatic));
    QuasiStatic.set_ref(x_ref, dx_ref, ddx_ref);
    srl::sleep(0.1);

    const drake::math::RigidTransformd X_W_B {QuasiStatic.state_.tip_transforms[0].matrix()}, X_W_T{QuasiStatic.state_.tip_transforms[2].matrix()};
    const auto X_B_T {X_W_B.inverse() * X_W_T};

    fmt::print("X_W_B: \n{}\n", X_W_B.GetAsMatrix4());
    fmt::print("X_W_T: \n{}\n", X_W_T.GetAsMatrix4());
    fmt::print("X_B_T: \n{}\n", X_B_T.GetAsMatrix4());

    getchar();
    QuasiStatic.toggle_log();
    t=0;
    while (t<20){ //circle
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

        x_ref = X_W_T * circle;
        dx_ref = X_W_T.rotation() * d_circle;
        ddx_ref = X_W_T.rotation() * dd_circle;
        QuasiStatic.set_ref(x_ref);
        */
        t+=dt;
        srl::sleep(dt);
        
    }
    QuasiStatic.toggle_log();
    return 1;
}
