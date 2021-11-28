#include "3d-soft-trunk/Adaptive.h"
#include <iostream>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
double sigma = 0.0;
double dsigma = 0.0;
double ddsigma = 0.0;
double T = 30;        // final time changes depending on the timing law
double num_round = 3; // number of circular period

bool freedom = false;
bool pause = true;
Vector3d x_ref;
Vector3d dx_ref = Vector3d::Zero();
Vector3d ddx_ref = Vector3d::Zero();
Vector3d circle = Vector3d::Zero();
Vector3d d_circle = Vector3d::Zero();
Vector3d dd_circle = Vector3d::Zero();

void gain(Adaptive &ad)
{ //change gain with keyboard to avoid recompiling, q/a change kp, w/s change kd, i/k change potfield size and o/l change potfield strength
    char c;
    while (true)
    {
        c = getchar();
        switch (c)
        {
        case 'q':
            ad.increase_kp();
            break;
        case 'a':
            ad.decrease_kp();
            break;
        case 'w':
            ad.increase_kd();
            break;
        case 's':
            ad.decrease_kd();
            break;
        case 'e':
            ad.increase_gamma();
            break;
        case 'd':
            ad.decrease_gamma();
            break;
        case 'r':
            ad.increase_delta();
            break;
        case 'f':
            ad.decrease_delta();
            break;
        case 't':
            ad.increase_rate1();
            break;
        case 'g':
            ad.decrease_rate1();
            break;
        case 'y':
            ad.increase_rate2();
            break;
        case 'h':
            ad.decrease_rate2();
            break;
        case 'u':
            ad.increase_eps();
            break;
        case 'j':
            ad.decrease_eps();
            break;
        case 'i':
            ad.increase_stiffness(0);
            break;
        case 'k':
            ad.decrease_stiffness(0);
            break;
        case 'o':
            ad.increase_stiffness(1);
            break;
        case 'l':
            ad.decrease_stiffness(1);
            break;
        case 'p':
            ad.increase_damping();
            break;
        case ';':
            ad.decrease_damping();
            break;
        case '[':
            ad.increase_alpha();
            break;
        case ']':
            ad.decrease_alpha();
            break;
        case '1':
            ad.change_ref1();
            break;
        case '2':
            ad.change_ref2();
            break;
        case '3':
            ad.change_ref3();
            break;
        case '4':
            ad.change_ref4();
            break;
        case 'z':
            ad.show_x();
            break;
        case 'v':
            ad.toggleGripper();
            break;
        case 'c':
            pause = false;
            ad.toggle_log();
            ad.start_AD();
            // ad.start_ID();
        }
    }
}

void Task_8(double t, double T, double r, double offset)
{
    double dpy_tmp;
    double dpy_tmp_tmp;
    double dpz_tmp;
    double px_tmp_tmp;
    double px_tmp_tmp_tmp;

    //  offset = 0.03;
    px_tmp_tmp_tmp = 6.2831853071795862 / T * t;
    px_tmp_tmp = std::sin(px_tmp_tmp_tmp);

    //  syms x0 y0 z0 s a t T offset
    //  simplify(expand(diff(dpz,t)))
    dpy_tmp_tmp = 12.566370614359172 * t / T;
    dpy_tmp = std::cos(dpy_tmp_tmp);
    dpz_tmp = std::sin(dpy_tmp_tmp);
    x_ref[0] = r * px_tmp_tmp;
    x_ref[1] = 2.0 * r * px_tmp_tmp * std::cos(px_tmp_tmp_tmp);
    x_ref[2] = offset * (px_tmp_tmp * px_tmp_tmp) + -0.22;
    dpy_tmp_tmp = 6.2831853071795862 * t / T;
    dx_ref[0] = 2.0 * r * 3.1415926535897931 * std::cos(dpy_tmp_tmp) / T;
    dx_ref[1] = 4.0 * r * 3.1415926535897931 * dpy_tmp / T;
    dx_ref[2] = 2.0 * offset * 3.1415926535897931 * dpz_tmp / T;
    px_tmp_tmp_tmp = T * T;
    ddx_ref[0] = -(4.0 * r * 9.869604401089358 * std::sin(dpy_tmp_tmp)) /
                 px_tmp_tmp_tmp;
    ddx_ref[1] = -(16.0 * r * 9.869604401089358 * dpz_tmp) / px_tmp_tmp_tmp;
    ddx_ref[2] = 8.0 * offset * 9.869604401089358 * dpy_tmp / px_tmp_tmp_tmp;
}

void Task_88(double t, double T, double r, double offset)
{
    double dpx_tmp;
    double dpx_tmp_tmp;
    double dpz_tmp;
    double px_tmp_tmp;
    double px_tmp_tmp_tmp;

    //  offset = 0.03;
    px_tmp_tmp_tmp = 6.2831853071795862 / T * t;
    px_tmp_tmp = std::sin(px_tmp_tmp_tmp);

    //  syms x0 y0 z0 s r t T offset
    //  simplify(expand(diff(px,t)))
    dpx_tmp_tmp = 12.566370614359172 * t / T;
    dpx_tmp = std::cos(dpx_tmp_tmp);
    dpz_tmp = std::sin(dpx_tmp_tmp);
    x_ref[0] = 2.0 * r * px_tmp_tmp * std::cos(px_tmp_tmp_tmp);
    x_ref[1] = r * px_tmp_tmp;
    x_ref[2] = offset * (px_tmp_tmp * px_tmp_tmp) + -0.22;
    dx_ref[0] = 4.0 * r * 3.1415926535897931 * dpx_tmp / T;
    dpx_tmp_tmp = 6.2831853071795862 * t / T;
    dx_ref[1] = 2.0 * r * 3.1415926535897931 * std::cos(dpx_tmp_tmp) / T;
    dx_ref[2] = 2.0 * offset * 3.1415926535897931 * dpz_tmp / T;
    px_tmp_tmp_tmp = T * T;
    ddx_ref[0] = -(16.0 * r * 9.869604401089358 * dpz_tmp) / px_tmp_tmp_tmp;
    ddx_ref[1] = -(4.0 * r * 9.869604401089358 * std::sin(dpx_tmp_tmp)) /
                 px_tmp_tmp_tmp;
    ddx_ref[2] = 8.0 * offset * 9.869604401089358 * dpx_tmp / px_tmp_tmp_tmp;
}

void Task_Rose(double t, double T, double a)
{
    // a proportional to radious of leafs
    double f = 6.2831853071795862 / T;
    double dpx_tmp_tmp = 6.2831853071795862 * t / T;
    double b_dpx_tmp_tmp = std::sin(dpx_tmp_tmp);
    double dpy_tmp_tmp = std::cos(dpx_tmp_tmp);
    double p_tmp = t * f;
    dpx_tmp_tmp = a * std::cos(2.0 * t * f);
    x_ref[0] = dpx_tmp_tmp * std::cos(p_tmp);
    x_ref[1] = dpx_tmp_tmp * std::sin(p_tmp);
    x_ref[2] = -0.22;
    dpx_tmp_tmp = 2.0 * a * 3.1415926535897931;
    p_tmp = pow(b_dpx_tmp_tmp, 3.0);
    dx_ref[0] = -(dpx_tmp_tmp * (5.0 * b_dpx_tmp_tmp - 6.0 * p_tmp)) / T;
    double dp_tmp = pow(dpy_tmp_tmp, 3.0);
    dx_ref[1] = -(dpx_tmp_tmp * (5.0 * dpy_tmp_tmp - 6.0 * dp_tmp)) / T;
    dx_ref[2] = 0.0;
    dpx_tmp_tmp = 4.0 * a * 9.869604401089358;
    f = T * T;
    ddx_ref[0] = dpx_tmp_tmp * (13.0 * dpy_tmp_tmp - 18.0 * dp_tmp) / f;
    ddx_ref[1] = -(dpx_tmp_tmp * (13.0 * b_dpx_tmp_tmp - 18.0 * p_tmp)) / f;
    ddx_ref[2] = 0.0;
}

void Task_Circle(double t, double T, double r)
{
    double coef = 2 * 3.1415 / T;
    x_ref << r * cos(coef * t) , r * sin(coef * t), -0.22;
    dx_ref << -r * coef * sin(coef * t), r * coef * cos(coef * t), 0;
    ddx_ref << -r * coef * coef * cos(coef * t), -r * coef * coef * sin(coef * t), 0;
}

// Timing Law
void s_trapezoidal_speed(double t, double *sigma, double *dsigma, double *ddsigma, double *T)
{

    double l, dsigma_max, ddsigma_max, Ts, Tf;
    // circle
    double n = 2; //rounds of circle
    double r = 0.01;    //radius of the circle
    l = 2 * PI * n * r; //l > v_max ^ 2 / a_max
    // linear
    //Eigen::Vector3d p_i;
    //Eigen::Vector3d p_f;
    //p_i << 0, 0, 0;
    //p_f << 1, 1, 1;
    //Eigen::Vector3d d = p_f - p_i;
    //l = d.norm();

    dsigma_max = 0.07;  // maximum velocity
    ddsigma_max = 0.01; // maximum acc

    Ts = dsigma_max / ddsigma_max;
    Tf = (l * ddsigma_max + (dsigma_max * dsigma_max)) / (ddsigma_max * dsigma_max); // the total time
    *T = Tf;

    if (t <= Ts)
    {

        *sigma = (ddsigma_max * t * t) / 2;
        *dsigma = ddsigma_max * t;
        *ddsigma = ddsigma_max;
    }

    else if (t > Ts && t <= Tf - Ts)
    {

        *sigma = (dsigma_max * t) - ((dsigma_max * dsigma_max) / (2 * ddsigma_max));
        *dsigma = dsigma_max;
        *ddsigma = 0;
    }

    else if (t > Tf - Ts && t <= Tf)
    {

        *sigma = (-0.5 * ddsigma_max * (t - Tf) * (t - Tf)) + (dsigma_max * Tf) - (dsigma_max * dsigma_max / ddsigma_max);
        *dsigma = ddsigma_max * (Tf - t);
        *ddsigma = -ddsigma_max;
    }

    else if (t > Tf)
    {
        t = Tf;
        *sigma = (-0.5 * ddsigma_max * (t - Tf) * (t - Tf)) + (dsigma_max * Tf) - (dsigma_max * dsigma_max / ddsigma_max);
        *dsigma = 0;  //;ddsigma_max*(Tf-t);
        *ddsigma = 0; //-ddsigma_max;
    }
}

void Task_Circle_r2r(double sigma, double dsigma, double ddsigma)
{
    // circle parameters
    double cx = 0.0;
    double cy = 0.0;
    double cz = -0.246;
    double r = 0.15;
    double h = 0;
    double phi = 0;

    x_ref[0] = cx + r * cos(sigma / r + phi);
    x_ref[1] = cy + r * sin(sigma / r + phi);
    x_ref[2] = cz + h * cos(sigma / r + phi);

    dx_ref[0] = -sin(sigma / r + phi) * dsigma;
    dx_ref[1] = cos(sigma / r + phi) * dsigma;
    dx_ref[2] = -(h * sin(phi + sigma / r)) / r * dsigma;

    ddx_ref[0] = (-cos(phi + sigma / r) / r) * dsigma * dsigma + (-sin(sigma / r + phi)) * ddsigma;
    ddx_ref[1] = (-sin(phi + sigma / r) / r) * dsigma * dsigma + (cos(sigma / r + phi)) * ddsigma;
    ddx_ref[2] = (-(h * cos(phi + sigma / r)) / (r * r)) * dsigma * dsigma + (-(h * sin(phi + sigma / r)) / r) * ddsigma;
}

void Task_Linear_r2r(double sigma, double dsigma, double ddsigma)
{
    Eigen::Vector3d p_i; //initial point
    Eigen::Vector3d p_f; //final point
    p_i << 0, 0, 0;
    p_f << 1, 1, 1;
    Eigen::Vector3d d = p_f - p_i;
    double L = d.norm(); //distance

    x_ref[0] = p_i(1) + sigma*(p_f(1)-p_i(1))/L;
    x_ref[1] = p_i(2) + sigma*(p_f(2)-p_i(2))/L;
    x_ref[2] = p_i(3) + sigma*(p_f(3)-p_i(3))/L;

    dx_ref[0] = (p_f(1)-p_i(1))/L * dsigma;
    dx_ref[1] = (p_f(2)-p_i(2))/L * dsigma;
    dx_ref[2] = (p_f(3)-p_i(3))/L * dsigma;

    ddx_ref[0] = (p_f(1)-p_i(1))/L * ddsigma;
    ddx_ref[1] = (p_f(2)-p_i(2))/L * ddsigma;
    ddx_ref[2] = (p_f(3)-p_i(3))/L * ddsigma;
}

int main()
{
    bool gripping = false;
    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Adaptive ad(st_params, CurvatureCalculator::SensorType::qualisys, 0);

    double t = 0.0;
    double dt = 1. / 10;

    //Task_8(t, 12.0, 0.12,0.03);
    //Task_Rose(t, 20.0, 0.1);
    //Task_Circle(t, 12, 0.12);
    //x_ref << 0, -0.12, -0.22;
    //s_trapezoidal_speed(t, &sigma, &dsigma, &ddsigma, &T);
    //Task_Circle_r2r(sigma, dsigma, ddsigma);
    //Task_Linear_r2r(sigma, dsigma, ddsigma);
    ad.set_ref(x_ref, dx_ref, ddx_ref);
    Vector3d x_dum = ad.x_qualisys;
    fmt::print("a = {}\n", ad.a);
    getchar();
    //ad.toggle_log();i
    ad.start_AD();
    //ad.toggleGripper();
    std::thread gain_thread(gain, std::ref(ad));
    srl::sleep(0.1); //wait to get to the initial position
    //start adaptation now:
    T = ad.T;
    fmt::print("T = {}\n", T);
    //std::cout << ad.Ka(9);
    //std::cout << ad.Ka(10);
    ad.toggle_fastlog(T);
    while (t<=T)
    {
    //ad.Ka(7)= 0;
    //ad.Ka(8) = 0;
    //ad.Ka(9) = 1;
    //ad.Ka(10) = 1;
    //ad.rate1 = 0.001;
    //ad.rate2 = 0;//0.00001;
        //std::cout << x_ref << "\n";
        //Task_8(t, 12.0, 0.12,0.03); // 8 shape traj. with radious 0.1m and period 20s
        //Task_Rose(t, 16.0, 0.1); // Rose shape traj. with radious 0.1m and period 20s
        //Task_Circle(t, 12, 0.12); // circular traj. with radius 0.15m and period 8s
        //fmt::print("a = {}\n", ad.a);
        //s_trapezoidal_speed(t, &sigma, &dsigma, &ddsigma, &T);
        //Task_Circle_r2r(sigma, dsigma, ddsigma);
        //ad.set_ref(x_ref, dx_ref, ddx_ref);
        /**
        x_dum = ad.x_qualisys;
        if ((x_dum - ad.get_objects()[0]).norm() < 0.07 && (x_dum - ad.get_objects()[0]).norm() > 0.001 &&  !gripping){
            ad.toggleGripper();
            gripping = true;
        }
        */
        t += dt;
        srl::sleep(dt);
    }
    ad.toggle_fastlog(0);

    srl::sleep(2);
}