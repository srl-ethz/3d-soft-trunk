
#include "3d-soft-trunk/OSC.h"
#include<iostream>

bool freedom = false;
Vector3d x_ref;
Vector3d x;
Vector3d dx_ref;
Vector3d ddx_ref;

//for crossing
double crosstime;
Vector3d newx;
Vector3d diff;

void gain(OSC& osc){ //change gain with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while(true) {
        c = getchar();
        switch (c) {
            case 'q':
                osc.set_kp(osc.get_kp()*1.1);
                break;
            case 'a':
                osc.set_kp(osc.get_kp()*0.9);
                break;
            case 'e':
                osc.set_kd(osc.get_kd()*1.1);
                break;
            case 'd':
                osc.set_kd(osc.get_kd()*0.9);
                break;
            case 'i':
                osc.potfields[0].set_cutoff(osc.potfields[0].get_cutoff()*1.1);
                break;
            case 'k':
                osc.potfields[0].set_cutoff(osc.potfields[0].get_cutoff()*0.9);
                break;
            case 'o':
                osc.potfields[0].set_strength(osc.potfields[0].get_strength()*1.1);
                break;
            case 'l':
                osc.potfields[0].set_strength(osc.potfields[0].get_strength()*0.9);
                break;
            case 'g':
                x_ref = osc.get_objects()[0];
                osc.set_ref(x_ref, dx_ref, ddx_ref);
                break;
            case 't':
                osc.toggleGripper();
                freedom = true;
                break;
            case 'v':
                //mirror across 0,0 in time "crosstime"
                crosstime = 1.5;
                ddx_ref = Vector3d::Zero();
                newx = x_ref;
                newx(1) *= -1;
                newx(0) *= -1;
                osc.loadAttached = 0.2;
                //osc.toggle_log();
                /*diff = newx - x_ref;
                
                for(double tl = 0; tl < crosstime; tl += 0.1){
                    osc.set_ref(x_ref + (tl/crosstime) * diff, dx_ref, ddx_ref);
                    srl::sleep(0.1);
                }*/
                x_ref(0) = 0.17;
                x_ref(1) = 0;
                x_ref(2) = -0.27;
                osc.set_ref(x_ref,dx_ref,ddx_ref);
                //srl::sleep(1);
                x_ref = newx;
                osc.set_ref(x_ref,dx_ref,ddx_ref);
                //srl::sleep(8);
                //osc.toggle_log();
                break;

            case 'b':
                osc.loadAttached = .2;
                break;
            case 'f': 
                osc.freeze = !osc.freeze;
                fmt::print("Freeze status: {}\n", osc.freeze);
                break;
            case 'm':
                osc.toggle_log();
                break;
        }
        fmt::print("kp = {}, kd = {}\n", osc.get_kp(), osc.get_kd());
        fmt::print("cutoff = {}, strength = {}\n", osc.potfields[0].get_cutoff(), osc.potfields[0].get_strength());
    }
}

void printer(OSC& osc){
    srl::Rate r{0.3};
    while(true){
        Vector3d x;
        osc.get_x(x);
        fmt::print("------------------------------------\n");
        fmt::print("extra object1: {}\n", osc.get_objects()[0].transpose());
        fmt::print("extra object1: {}\n", osc.get_objects()[1].transpose());
        fmt::print("x tip: {}\n", x.transpose());
        fmt::print("x error: {}\n", (x_ref-x).transpose());
        fmt::print("x error normalized: {}\n", (x_ref-x).norm());
        r.sleep();
    }
}

int main(){
    SoftTrunkParameters st_params;
    st_params.finalize();
    OSC osc(st_params, CurvatureCalculator::SensorType::qualisys, 2);
    VectorXd p;
    srl::State state = st_params.getBlankState();

    Vector3d x_ref_center;
    
    x_ref_center << 0.15*cos(0*0.01745329),0.15*sin(0*0.01745329),-0.215;
    //x_ref = x_ref_center;
    x_ref << 0.2*0.11,-0.11,-0.22;
    std::thread print_thread(printer, std::ref(osc));

    
    double t = 0;
    double dt = 0.01;
    Vector3d circle;
    Vector3d d_circle;
    Vector3d dd_circle;

    double coef = 2 * 3.1415 / 8;
    //osc.gripperAttached = true;
    osc.loadAttached = 0;
    //getchar();
    //osc.toggleGripper();

    getchar();
    osc.set_ref(x_ref, dx_ref, ddx_ref);
    getchar();
    // arguments to pass by reference must be explicitly designated as so
    // https://en.cppreference.com/w/cpp/thread/thread/thread
    std::thread gain_thread(gain, std::ref(osc));
    
    osc.set_kd(5.5);
    osc.set_kp(100);
    osc.toggle_log();

    /*while (t<16){
        double r = 0.13;
        circle << r*cos(coef*t), r*sin(coef*t),-0.215;
        d_circle << -r*coef*sin(coef*t), r*coef*cos(coef*t),0;
        dd_circle << -r*coef*coef*cos(coef*t), -r*coef*coef*sin(coef*t),0;
        x_ref = circle;
        dx_ref = d_circle;
        ddx_ref = dd_circle;
        //x_ref = osc.get_objects()[0];
        osc.set_ref(x_ref,dx_ref, ddx_ref);
        /*osc.get_x(x);
        if ((x_ref - x).norm() < 0.07){
            freedom = true;
            osc.toggleGripper();
        }
        
        t+=dt;
        srl::sleep(dt);
    }*/

    double leng = 0.11;
    double period = 6;
    double velo = leng/period;

    /*while (t<period){
        x_ref << 1.0*leng-velo*t,leng,-0.22;
        dx_ref << -velo,0,0;
        ddx_ref << 0,0,0;
        osc.set_ref(x_ref,dx_ref,ddx_ref);

        t+=dt;
        srl::sleep(dt);
    }
    t= 0;
    while (t<period){
        x_ref << 0*leng,leng-velo*t,-0.22;
        dx_ref << 0,-velo,0;
        ddx_ref << 0,0,0;
        osc.set_ref(x_ref,dx_ref,ddx_ref);

        t+=dt;
        srl::sleep(dt);
    }
    t= 0;
    while (t<period){
        x_ref << 0*leng+velo*t,0,-0.22;
        dx_ref << velo,0,0;
        ddx_ref << 0,0,0;
        osc.set_ref(x_ref,dx_ref,ddx_ref);

        t+=dt;
        srl::sleep(dt);
    }
    t= 0;
    while (t<period){
        x_ref << 1.0*leng,-velo*t,-0.22;
        dx_ref << 0,-velo,0;
        ddx_ref << 0,0,0;
        osc.set_ref(x_ref,dx_ref,ddx_ref);

        t+=dt;
        srl::sleep(dt);
    }
    t= 0;
    while (t<period){
        x_ref << 0*leng-velo*t,-leng,-0.22;
        dx_ref << -velo,0,0;
        ddx_ref << 0,0,0;
        osc.set_ref(x_ref,dx_ref,ddx_ref);

        t+=dt;
        srl::sleep(dt);
    }*/
    
    /*
    while (t<2*period){
        x_ref << 0.2*leng,leng-velo*t,-0.22;
        dx_ref << 0,-velo,0;
        ddx_ref << -0,0,0;

        osc.set_ref(x_ref,dx_ref,ddx_ref);
        t+=dt;
        srl::sleep(dt);
    }
    t = 0;
    while (t<period){
        x_ref << 0.2*leng+velo*t,-leng,-0.22;
        dx_ref << velo,0,0;
        ddx_ref << -0,0,0;

        osc.set_ref(x_ref,dx_ref,ddx_ref);
        t+=dt;
        srl::sleep(dt);
    }*/
    while (t<2*period){
        x_ref << 0.1*leng,-leng+velo*t,-0.24;
        dx_ref << 0,velo,0;
        ddx_ref << -0,0,0;

        osc.set_ref(x_ref,dx_ref,ddx_ref);
        t+=dt;
        srl::sleep(dt);
    }
    t=0;
    while (t<period){
        x_ref << 0.1*leng+velo*t,leng,-0.24;
        dx_ref << velo,0,0;
        ddx_ref << -0,0,0;

        osc.set_ref(x_ref,dx_ref,ddx_ref);
        t+=dt;
        srl::sleep(dt);
    }
    t=0;    
    while (t<period){
        x_ref << 1.1*leng,leng-velo*t,-0.24;
        dx_ref << 0,-velo,0;
        ddx_ref << -0,0,0;

        osc.set_ref(x_ref,dx_ref,ddx_ref);
        t+=dt;
        srl::sleep(dt);
    }
    t=0;
    while (t<period){
        x_ref << 1.1*leng-velo*t,0,-0.24;
        dx_ref << -velo,0,0;
        ddx_ref << -0,0,0;

        osc.set_ref(x_ref,dx_ref,ddx_ref);
        t+=dt;
        srl::sleep(dt);
    }  
    t=0;
    while (t<1.414*period){
        x_ref << 0.1*leng+velo*t,-velo*t,-0.24;
        dx_ref << velo,-velo,0;
        ddx_ref << -0,0,0;

        osc.set_ref(x_ref,dx_ref,ddx_ref);
        t+=dt;
        srl::sleep(dt);
    }  
    t=0;     

    osc.toggle_log();
    return 1;
    
}
