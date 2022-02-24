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
    double dt = 1./50;
    x_ref << 0.,0.,-0.385;
    double coef = 2 * 3.1415/16;
    bool freedom = false;
    id.set_kp(100);
    id.set_kd(5.5);
    id.set_ref(x_ref,dx_ref,ddx_ref);
    id.toggle_log();

    getchar();

    /*
    srl::Rate r1{1./dt};
    for (double i = 0; i < 2.5; i +=dt){
        x_ref << 0., i*0.12/2.5, -0.385;
        dx_ref << 0, 0.12/2.5, 0;
        id.set_ref(x_ref, dx_ref);
        r1.sleep();
    }
        
    getchar();
    srl::Rate r3{1./dt};
    for (double i = 0; i < 2.5; i +=dt){
        x_ref << 0., 0.12, -0.385 - 0.03*i/2.5;
        dx_ref << 0, 0, -0.03/2.5;
        id.set_ref(x_ref, dx_ref);
        r3.sleep();
    }
    x_ref << 0., 0.12, -0.415;
    id.set_ref(x_ref);

    getchar();
    id.toggleGripper();
    srl::sleep(0.5);
    x_ref << 0., 0.12, -0.385;
    id.set_ref(x_ref);

    srl::Rate r4{1./dt};
    for (double i = 0; i < 2.5; i +=dt){
        x_ref << 0., 0.12, -0.415 + 0.03*i/2.5;
        dx_ref << 0, 0, 0.03/2.5;
        id.set_ref(x_ref, dx_ref);
        r4.sleep();
    }
    x_ref << 0., 0.12, -0.385;
    id.set_ref(x_ref);

    getchar();
    srl::Rate r2{1./dt};
    for (double i = 0; i < 5; i +=dt){
        x_ref << 0., 0.12 - 0.12*i/2.5, -0.385;
        dx_ref << 0, -0.12/2.5, 0;
        id.set_ref(x_ref, dx_ref);
        r2.sleep();
    }
    dx_ref << 0,0,0;
    id.set_ref(x_ref,dx_ref);

    getchar();
    id.toggleGripper();
    getchar();

    */

    /*
    srl::Rate rate{1./dt};
    while (t<16){
        
        double r = 0.12;
        Vector3d x;
        id.get_x(x);
        id.set_ref(x);
       


        //Slanted Circle:
    /*
        x_ref << r*cos(coef*t), r*sin(coef*t), -0.38 + 0.02*sin(coef*t);

        dx_ref << -r*coef*sin(coef*t), r*coef*cos(coef*t), 0.02*coef*cos(coef*t);

        ddx_ref << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t), -0.02*coef*coef*sin(coef*t);
    */

        
        //Helix:
/*
        x_ref << r*cos(coef*t), r*sin(coef*t), -0.40 + 0.04*t/16;

        dx_ref << -r*coef*sin(coef*t), r*coef*cos(coef*t), 0.04/16;

        ddx_ref << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t), 0;
*/

        //Line along x:
/*
        x_ref << 0, 0.1 - (0.2*t)/16, -0.4;

        dx_ref << 0, -(0.2/16), 0;

        ddx_ref << 0, 0, 0;
*/
       // id.set_ref(x_ref,dx_ref,ddx_ref);
       /*
        rate.sleep();
        t+=dt;
    }*/
    id.toggle_log();

    
}