#include "3d-soft-trunk/IDCon.h"

bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref = Vector3d::Zero();
Vector3d ddx_ref = Vector3d::Zero();
Vector3d circle = Vector3d::Zero();
Vector3d d_circle = Vector3d::Zero();
Vector3d dd_circle = Vector3d::Zero();

void gain(IDCon &id){ //change gain with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                id.set_kp(id.get_kp()*1.1);
                break;
            case 'a':
                id.set_kp(id.get_kp()*0.9);
                break;
            case 'e':
                id.set_kd(id.get_kd()*1.1);
                break;
            case 'd':
                id.set_kd(id.get_kd()*0.9);
                break;
            case 'g':
                x_ref = id.get_objects()[0];
                id.set_ref(x_ref, dx_ref, ddx_ref);
                break;
            case 't':
                id.toggleGripper();
                freedom = true;
                break;
            case 'v':
                x_ref(1) *= -1;
                id.set_ref(x_ref,dx_ref, ddx_ref);
                break;
            case 'r':
                id.toggle_log();
                srl::sleep(5);
                id.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", id.get_kp(), id.get_kd());
    }
}

void printer(IDCon &id){
    srl::Rate r{1};
    while(true){
        Vector3d x;
        id.get_x(x);
        fmt::print("------------------------------------\n");
        //fmt::print("A_pseudo.inverse(): \n {} \n",id.stm->A_pseudo.inverse());
        fmt::print("x: {} \n",x.transpose());
        fmt::print("x error: {}\n", (x_ref-x).transpose());
        fmt::print("x error normalized: {}\n", (x_ref-x).norm());
        VectorXd p;
        id.get_pressure(p);
        fmt::print("pressure: {}\n", p.transpose());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    IDCon id(st_params, CurvatureCalculator::SensorType::qualisys, 1);
    std::thread print_thread(printer, std::ref(id));
    std::thread gain_thread(gain, std::ref(id));
    double t = 0;
    double dt = 0.01;
    x_ref << 0.12,0,-0.445;
    double amplitude = 0.2;
    double coef = 2 * 3.1415/16;
    bool freedom = false;
    id.toggle_log();
    id.set_ref(x_ref,dx_ref,ddx_ref);

    srl::sleep(3);
    //getchar();
    

    while (t<20){
        
        double r = 0.12;

        //Circle:
/*
        circle << r*cos(coef*t), r*sin(coef*t), -0.465;
        d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;
        x_ref = circle;
        dx_ref = d_circle;
        ddx_ref = dd_circle;
  */      
        
        //Slanted Circle:

        x_ref << r*cos(coef*t), r*sin(coef*t), -0.445 + 0.02*sin(coef*t);

        dx_ref << -r*coef*sin(coef*t), r*coef*cos(coef*t), 0.02*coef*cos(coef*t);

        ddx_ref << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t), -0.02*coef*coef*sin(coef*t);

/*
        //Wavy Circle with n waves:

        x_ref << r*cos(coef*t), r*sin(coef*t), -0.4 + 0.02*sin(coef*n*t);

        dx_ref << -r*coef*sin(coef*t), r*coef*cos(coef*t), 0.02*coef*cos(coef*n*t);

        ddx_ref << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t), -0.02*coef*coef*sin(coef*n*t);


       // Helix:

        x_ref << r*cos(coef*t), r*sin(coef*t), -0.42 + 0.02*coef*t;

        dx_ref << -r*coef*sin(coef*t), r*coef*cos(coef*t), 0.02*coef;

        ddx_ref << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t), 0;


        //Line along z:

        x_ref << 0.12, 0, -0.42 + 0.1*t;

        dx_ref << 0, 0, 0.1;

        ddx_ref << 0, 0, 0;
*/

        id.set_ref(x_ref,dx_ref,ddx_ref);

        t+=dt;
        srl::sleep(dt);
    }
    id.toggle_log();

    srl::sleep(2);
    
    
}