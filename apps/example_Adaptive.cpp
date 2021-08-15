#include "3d-soft-trunk/Adaptive.h"

bool freedom = false;
Vector3d x_ref;
Vector3d x;
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
        case 'w':
            ad.increase_kd();
            break;
        case 's':
            ad.decrease_kd();
            break;
        case 'q':
            ad.increase_kp();
            break;
        case 'a':
            ad.decrease_kp();
            break;
        }
    }
}

void Task_8(double t, double T, double a)
{
// a proportional to radious of leafs
  double px_tmp_tmp = 6.2831853071795862 / T * t;
  double px_tmp = std::sin(px_tmp_tmp);
  x_ref[0] = 2.0 * a * px_tmp * std::cos(px_tmp_tmp);
  x_ref[1] = a * px_tmp;
  x_ref[2] = -0.2;
  px_tmp_tmp = 12.566370614359172 * t / T;
  dx_ref[0] = 4.0 * a * 3.1415926535897931 * std::cos(px_tmp_tmp) / T;
  px_tmp = 6.2831853071795862 * t / T;
  dx_ref[1] = 2.0 * a * 3.1415926535897931 * std::cos(px_tmp) / T;
  dx_ref[2] = 0.0;
  double ddp_tmp = T * T;
  ddx_ref[0] = -(16.0 * a * 9.869604401089358 * std::sin(px_tmp_tmp)) / ddp_tmp;
  ddx_ref[1] = -(4.0 * a * 9.869604401089358 * std::sin(px_tmp)) / ddp_tmp;
  ddx_ref[2] = 0.0;
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
    x_ref[2] = -0.2;
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
    x_ref << r * cos(coef * t), r * sin(coef * t), -0.25;
    dx_ref << -r * coef * sin(coef * t), r * coef * cos(coef * t), 0;
    ddx_ref << -r * coef * coef * cos(coef * t), -r * coef * coef * sin(coef * t), 0;
}

int main()
{

    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();
    Adaptive ad(st_params, CurvatureCalculator::SensorType::qualisys, 1);

    double t = 0.0;
    double dt = 0.1;
    //Task_8(t, 20.0, 0.1);
    //Task_Rose(t, 20.0, 0.1);
    Task_Circle(t, 8, 0.15);
    std::thread gain_thread(gain, std::ref(ad));
    ad.set_ref(x_ref, dx_ref, ddx_ref);
    ad.toggle_log();
    while (true)
    {
        //Task_8(t, 20.0, 0.1); // 8 shape traj. with radious 0.1m and period 20s
        //Task_Rose(t, 20.0, 0.1); // Rose shape traj. with radious 0.1m and period 20s
        Task_Circle(t, 8, 0.15); // circular traj. with radius 0.15m and period 8s
        ad.set_ref(x_ref, dx_ref, ddx_ref);

        t += dt;
        srl::sleep(dt);
    }
    ad.toggle_log();

    srl::sleep(2);
}