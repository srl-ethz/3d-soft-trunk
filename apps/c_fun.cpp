//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: c_fun.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 27-Jul-2021 19:01:13
//

// Include Files
#include "c_fun.h"
//#include "rt_nonfinite.h"
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  /*if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {*/
      y = std::pow(u0, u1);
    //}
  //}

  return y;
}

//
// C_FUN
//     Cdq = c_fun(q,dq,L,m);
// Arguments    : const double in1[4]
//                const double in2[4]
//                const double in3[2]
//                const double in4[2]
//                double c[4]
// Return Type  : void
//
void c_fun(const double in1[4], const double in2[4], const double in3[2], const
           double in4[2], double c[4])
{
  double ab_c_tmp;
  double ab_c_tmp_tmp;
  double ac_c_tmp;
  double ad_c_tmp;
  double ae_c_tmp;
  double af_c_tmp;
  double ag_c_tmp;
  double b_c_tmp;
  double b_c_tmp_tmp;
  double b_c_tmp_tmp_tmp;
  double bb_c_tmp;
  double bb_c_tmp_tmp;
  double bc_c_tmp;
  double bd_c_tmp;
  double be_c_tmp;
  double bf_c_tmp;
  double bg_c_tmp;
  double c_c_tmp;
  double c_c_tmp_tmp;
  double c_tmp;
  double c_tmp_tmp;
  double c_tmp_tmp_tmp;
  double cb_c_tmp;
  double cb_c_tmp_tmp;
  double cc_c_tmp;
  double cd_c_tmp;
  double ce_c_tmp;
  double cf_c_tmp;
  double cg_c_tmp;
  double d;
  double d1;
  double d10;
  double d11;
  double d12;
  double d13;
  double d14;
  double d15;
  double d16;
  double d17;
  double d18;
  double d19;
  double d2;
  double d20;
  double d21;
  double d22;
  double d23;
  double d24;
  double d25;
  double d26;
  double d27;
  double d28;
  double d29;
  double d3;
  double d30;
  double d31;
  double d32;
  double d33;
  double d34;
  double d35;
  double d36;
  double d37;
  double d38;
  double d39;
  double d4;
  double d40;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  double d_c_tmp;
  double d_c_tmp_tmp;
  double db_c_tmp;
  double db_c_tmp_tmp;
  double dc_c_tmp;
  double dd_c_tmp;
  double de_c_tmp;
  double df_c_tmp;
  double dg_c_tmp;
  double e_c_tmp;
  double e_c_tmp_tmp;
  double eb_c_tmp;
  double ec_c_tmp;
  double ed_c_tmp;
  double ee_c_tmp;
  double ef_c_tmp;
  double eg_c_tmp;
  double f_c_tmp;
  double f_c_tmp_tmp;
  double fb_c_tmp;
  double fc_c_tmp;
  double fd_c_tmp;
  double fe_c_tmp;
  double ff_c_tmp;
  double fg_c_tmp;
  double g_c_tmp;
  double g_c_tmp_tmp;
  double gb_c_tmp;
  double gc_c_tmp;
  double gd_c_tmp;
  double ge_c_tmp;
  double gf_c_tmp;
  double gg_c_tmp;
  double h_c_tmp;
  double h_c_tmp_tmp;
  double hb_c_tmp;
  double hc_c_tmp;
  double hd_c_tmp;
  double he_c_tmp;
  double hf_c_tmp;
  double hg_c_tmp;
  double i_c_tmp;
  double i_c_tmp_tmp;
  double ib_c_tmp;
  double ic_c_tmp;
  double id_c_tmp;
  double ie_c_tmp;
  double if_c_tmp;
  double ig_c_tmp;
  double j_c_tmp;
  double j_c_tmp_tmp;
  double jb_c_tmp;
  double jc_c_tmp;
  double jd_c_tmp;
  double je_c_tmp;
  double jf_c_tmp;
  double jg_c_tmp;
  double k_c_tmp;
  double k_c_tmp_tmp;
  double kb_c_tmp;
  double kc_c_tmp;
  double kd_c_tmp;
  double ke_c_tmp;
  double kf_c_tmp;
  double kg_c_tmp;
  double l_c_tmp;
  double l_c_tmp_tmp;
  double lb_c_tmp;
  double lc_c_tmp;
  double ld_c_tmp;
  double le_c_tmp;
  double lf_c_tmp;
  double lg_c_tmp;
  double m_c_tmp;
  double m_c_tmp_tmp;
  double mb_c_tmp;
  double mc_c_tmp;
  double md_c_tmp;
  double me_c_tmp;
  double mf_c_tmp;
  double mg_c_tmp;
  double n_c_tmp;
  double n_c_tmp_tmp;
  double nb_c_tmp;
  double nc_c_tmp;
  double nd_c_tmp;
  double ne_c_tmp;
  double nf_c_tmp;
  double ng_c_tmp;
  double o_c_tmp;
  double o_c_tmp_tmp;
  double ob_c_tmp;
  double oc_c_tmp;
  double od_c_tmp;
  double oe_c_tmp;
  double of_c_tmp;
  double og_c_tmp;
  double p_c_tmp;
  double p_c_tmp_tmp;
  double pb_c_tmp;
  double pc_c_tmp;
  double pd_c_tmp;
  double pe_c_tmp;
  double pf_c_tmp;
  double pg_c_tmp;
  double q_c_tmp;
  double q_c_tmp_tmp;
  double qb_c_tmp;
  double qc_c_tmp;
  double qd_c_tmp;
  double qe_c_tmp;
  double qf_c_tmp;
  double r_c_tmp;
  double r_c_tmp_tmp;
  double rb_c_tmp;
  double rc_c_tmp;
  double rd_c_tmp;
  double re_c_tmp;
  double rf_c_tmp;
  double s_c_tmp;
  double s_c_tmp_tmp;
  double sb_c_tmp;
  double sc_c_tmp;
  double sd_c_tmp;
  double se_c_tmp;
  double sf_c_tmp;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t17;
  double t19;
  double t2;
  double t20;
  double t21;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t31;
  double t32;
  double t33;
  double t34;
  double t35;
  double t36;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  double t_c_tmp;
  double t_c_tmp_tmp;
  double tb_c_tmp;
  double tc_c_tmp;
  double td_c_tmp;
  double te_c_tmp;
  double tf_c_tmp;
  double u_c_tmp;
  double u_c_tmp_tmp;
  double ub_c_tmp;
  double uc_c_tmp;
  double ud_c_tmp;
  double ue_c_tmp;
  double uf_c_tmp;
  double v_c_tmp;
  double v_c_tmp_tmp;
  double vb_c_tmp;
  double vc_c_tmp;
  double vd_c_tmp;
  double ve_c_tmp;
  double vf_c_tmp;
  double w_c_tmp;
  double w_c_tmp_tmp;
  double wb_c_tmp;
  double wc_c_tmp;
  double wd_c_tmp;
  double we_c_tmp;
  double wf_c_tmp;
  double x_c_tmp;
  double x_c_tmp_tmp;
  double xb_c_tmp;
  double xc_c_tmp;
  double xd_c_tmp;
  double xe_c_tmp;
  double xf_c_tmp;
  double y_c_tmp;
  double y_c_tmp_tmp;
  double yb_c_tmp;
  double yc_c_tmp;
  double yd_c_tmp;
  double ye_c_tmp;
  double yf_c_tmp;

  //     This function was generated by the Symbolic Math Toolbox version 8.7.
  //     26-Jul-2021 21:32:52
  t2 = std::cos(in1[0]);
  t3 = std::cos(in1[1]);
  t4 = std::cos(in1[2]);
  t5 = std::cos(in1[3]);
  t6 = std::sin(in1[0]);
  t7 = std::sin(in1[1]);
  t8 = std::sin(in1[2]);
  t9 = std::sin(in1[3]);
  t10 = in3[0] * in3[0];
  t11 = in3[1] * in3[1];
  t12 = in2[0] * in2[0];
  t13 = in2[1] * in2[1];
  t14 = in2[2] * in2[2];
  t15 = in2[3] * in2[3];
  t17 = in1[1] * 2.0;
  t19 = in1[3] * 2.0;
  t20 = in1[1] * in1[1];
  t21 = rt_powd_snf(in1[1], 3.0);
  t23 = in1[3] * in1[3];
  t24 = rt_powd_snf(in1[1], 5.0);
  t25 = rt_powd_snf(in1[3], 3.0);
  t22 = t20 * t20;
  t26 = t23 * t23;
  t27 = t2 * t2;
  t28 = t3 * t3;
  t29 = t4 * t4;
  t30 = t5 * t5;
  t31 = std::sin(in1[0] * 2.0);
  t32 = std::sin(t17);
  t33 = std::sin(in1[2] * 2.0);
  t34 = std::sin(t19);
  t35 = 1.0 / t21;
  t36 = 1.0 / t25;
  d = in2[0] * in2[3];
  d1 = d * in4[1];
  c_tmp = d1 * t11;
  d2 = c_tmp * t21;
  d3 = in2[0] * in2[1];
  d4 = d3 * in4[1];
  d5 = d4 * t10 * t25;
  b_c_tmp = in2[2] * in2[3];
  d6 = b_c_tmp * in4[1];
  d7 = d6 * t11 * t21;
  d8 = in4[1] * in1[3];
  c_c_tmp = d8 * t11;
  d9 = d3 * in4[0];
  d10 = d4 * in1[1];
  d11 = d1 * in1[3];
  d_c_tmp = d11 * t9 * t11;
  d12 = d_c_tmp * t21;
  d13 = d9 * in1[1];
  d14 = in2[0] * in2[2];
  d15 = d14 * in4[1];
  d16 = d15 * in1[3];
  d17 = d6 * in1[3];
  d18 = d1 * t3;
  d19 = in4[1] * t4;
  d20 = in3[0] * in3[1];
  e_c_tmp = d20 * in2[0];
  d21 = e_c_tmp * in2[1] * in4[1];
  d22 = d20 * in4[1];
  d23 = d22 * t2;
  d24 = d23 * t3;
  d25 = d22 * t3;
  d26 = d25 * t4;
  d27 = in2[1] * in2[3];
  d28 = d27 * in4[1];
  d29 = d28 * t4;
  d30 = -in4[1] * in1[3];
  d31 = d8 * t2;
  d32 = d31 * t6;
  d33 = in4[1] * t2;
  d34 = d28 * in1[3];
  d35 = d34 * t2;
  d36 = d11 * t3;
  d37 = d4 * in1[3];
  d38 = d1 * t2;
  d39 = d28 * t2;
  d40 = d8 * t4;
  f_c_tmp = d23 * t8;
  c_tmp_tmp = d22 * t4;
  g_c_tmp = c_tmp_tmp * t6;
  h_c_tmp = d9 * t10 * t19 * t23;
  i_c_tmp = c_tmp * t17 * t20;
  j_c_tmp = d2 * t27;
  k_c_tmp = d2 * t28;
  l_c_tmp = d37 * t7 * t11 * t21;
  m_c_tmp = i_c_tmp * t29;
  n_c_tmp = j_c_tmp * t28;
  j_c_tmp *= t29;
  o_c_tmp = k_c_tmp * t29;
  b_c_tmp_tmp = d31 * t5;
  p_c_tmp = b_c_tmp_tmp * t6 * t11;
  q_c_tmp = d22 * in1[3];
  r_c_tmp = d22 * in1[1];
  c_c_tmp_tmp = r_c_tmp * t2;
  s_c_tmp = c_c_tmp_tmp * t8;
  t_c_tmp = d37 * t3;
  u_c_tmp = d16 * t4;
  c_tmp_tmp_tmp = in2[1] * in2[2] * in4[1];
  d_c_tmp_tmp = c_tmp_tmp_tmp * in1[3];
  v_c_tmp = d_c_tmp_tmp * t5 * t7 * t11 * t21;
  e_c_tmp_tmp = d_c_tmp_tmp * t7 * t11;
  w_c_tmp = e_c_tmp_tmp * t17 * t20;
  x_c_tmp = d39 * t6 * t7 * t11;
  y_c_tmp = in2[1] * in2[3] * in4[1] * t4 * t7 * t8 * t11 * t21;
  f_c_tmp_tmp = d8 * t3;
  ab_c_tmp = f_c_tmp_tmp * t4;
  g_c_tmp_tmp = d40 * t5;
  bb_c_tmp = g_c_tmp_tmp * t8 * t11;
  h_c_tmp_tmp = d31 * t3;
  i_c_tmp_tmp = h_c_tmp_tmp * t6 * t11 * t13;
  cb_c_tmp = i_c_tmp_tmp * t21;
  j_c_tmp_tmp = p_c_tmp * t12;
  db_c_tmp = j_c_tmp_tmp * t21;
  p_c_tmp = p_c_tmp * t13 * t21;
  eb_c_tmp = ab_c_tmp * t8 * t11 * t13 * t21;
  fb_c_tmp = bb_c_tmp * t12;
  gb_c_tmp = fb_c_tmp * t21;
  bb_c_tmp *= t13;
  k_c_tmp_tmp = d40 * t8 * t11;
  hb_c_tmp = k_c_tmp_tmp * t12;
  ib_c_tmp = d21 * in1[1];
  l_c_tmp_tmp = e_c_tmp * in2[3] * in4[1];
  jb_c_tmp = l_c_tmp_tmp * in1[3];
  b_c_tmp_tmp_tmp = d20 * in2[2] * in2[3] * in4[1];
  kb_c_tmp = b_c_tmp_tmp_tmp * in1[3];
  lb_c_tmp = e_c_tmp * in2[2] * in4[1];
  m_c_tmp_tmp = ib_c_tmp * t2;
  mb_c_tmp = m_c_tmp_tmp * t4;
  n_c_tmp_tmp = jb_c_tmp * t2;
  nb_c_tmp = n_c_tmp_tmp * t4;
  ob_c_tmp = kb_c_tmp * t2;
  pb_c_tmp = ob_c_tmp * t4;
  qb_c_tmp = ib_c_tmp * t3;
  rb_c_tmp = jb_c_tmp * t3;
  sb_c_tmp = ib_c_tmp * t6 * t8 * t23;
  tb_c_tmp = jb_c_tmp * t6 * t8 * t20;
  ub_c_tmp = lb_c_tmp * t2;
  vb_c_tmp = lb_c_tmp * t4;
  wb_c_tmp = r_c_tmp * t4;
  xb_c_tmp = q_c_tmp * t2;
  yb_c_tmp = wb_c_tmp * t5 * t6;
  ac_c_tmp = c_c_tmp_tmp * t5;
  bc_c_tmp = s_c_tmp * t12 * t17 * t23;
  o_c_tmp_tmp = d24 * t5 * t8;
  p_c_tmp_tmp = in3[0] * in3[1] * in4[1] * t2 * t5 * t8;
  cc_c_tmp = d16 * t2;
  dc_c_tmp = d37 * t5;
  q_c_tmp_tmp = d37 * t2;
  ec_c_tmp = q_c_tmp_tmp * t4;
  fc_c_tmp = d11 * t2;
  gc_c_tmp = d34 * t4;
  hc_c_tmp = d17 * t3;
  r_c_tmp_tmp = t_c_tmp * t5;
  ic_c_tmp = r_c_tmp_tmp * t7 * t11 * t21;
  s_c_tmp_tmp = cc_c_tmp * t5;
  jc_c_tmp = s_c_tmp_tmp * t6 * t11 * t21;
  kc_c_tmp = ec_c_tmp * t9 * t11;
  t_c_tmp_tmp = u_c_tmp * t5;
  lc_c_tmp = t_c_tmp_tmp * t8 * t11 * t21;
  u_c_tmp_tmp = fc_c_tmp * t4;
  mc_c_tmp = u_c_tmp_tmp * t7 * t11;
  nc_c_tmp = d37 * t6 * t8 * t9 * t11;
  oc_c_tmp = d11 * t6 * t7 * t8 * t11;
  pc_c_tmp = u_c_tmp * t8 * t11;
  v_c_tmp_tmp = d11 * t5;
  qc_c_tmp = v_c_tmp_tmp * t9 * t11;
  rc_c_tmp = qc_c_tmp * t17 * t20;
  sc_c_tmp = cc_c_tmp * t6 * t11;
  w_c_tmp_tmp = in2[0] * in2[1] * in4[1] * in1[3] * t3 * t7 * t11 * t21;
  tc_c_tmp = w_c_tmp_tmp * t27;
  uc_c_tmp = sc_c_tmp * t21;
  vc_c_tmp = pc_c_tmp * t21;
  wc_c_tmp = vc_c_tmp * t27;
  xc_c_tmp = v_c_tmp * t27;
  vc_c_tmp *= t28;
  qc_c_tmp *= t21;
  yc_c_tmp = qc_c_tmp * t27;
  qc_c_tmp *= t28;
  ad_c_tmp = w_c_tmp * t27;
  bd_c_tmp = d_c_tmp * t17 * t20;
  w_c_tmp *= t29;
  cd_c_tmp = e_c_tmp_tmp * t21 * t27 * t29;
  e_c_tmp_tmp = d38 * t4;
  dd_c_tmp = e_c_tmp_tmp * t5;
  ed_c_tmp = d28 * t3 * t4;
  x_c_tmp_tmp = d6 * t2;
  fd_c_tmp = x_c_tmp_tmp * t4;
  gd_c_tmp = e_c_tmp_tmp * t6 * t8 * t11 * t21;
  hd_c_tmp = d29 * t5;
  id_c_tmp = x_c_tmp * t17 * t20;
  x_c_tmp = x_c_tmp * t21 * t29;
  jd_c_tmp = y_c_tmp * t27;
  kd_c_tmp = b_c_tmp_tmp * t7 * t8 * t9 * t11;
  ld_c_tmp = g_c_tmp_tmp * t6 * t7 * t9 * t11;
  b_c_tmp_tmp = h_c_tmp_tmp * t5;
  md_c_tmp = b_c_tmp_tmp * t6 * t11 * t13;
  e_c_tmp_tmp = ab_c_tmp * t5;
  nd_c_tmp = e_c_tmp_tmp * t8 * t11 * t13 * t21;
  od_c_tmp = gb_c_tmp * t27;
  pd_c_tmp = eb_c_tmp * t27;
  qd_c_tmp = hb_c_tmp * t17 * t20 * t27;
  hb_c_tmp *= t21;
  g_c_tmp_tmp = r_c_tmp * t3;
  rd_c_tmp = g_c_tmp_tmp * t4;
  sd_c_tmp = c_c_tmp_tmp * t3;
  td_c_tmp = sd_c_tmp * t5 * t8;
  ud_c_tmp = yb_c_tmp * t12 * t17 * t23;
  y_c_tmp_tmp = fc_c_tmp * t3;
  vd_c_tmp = y_c_tmp_tmp * t4;
  wd_c_tmp = q_c_tmp_tmp * t3 * t4;
  xd_c_tmp = gc_c_tmp * t5;
  yd_c_tmp = d35 * t3;
  ec_c_tmp = ec_c_tmp * t5 * t9 * t11 * t21;
  ae_c_tmp = vd_c_tmp * t7 * t11 * t21;
  be_c_tmp = dc_c_tmp * t6 * t8 * t9 * t11 * t21;
  ce_c_tmp = d36 * t6 * t7 * t8 * t11 * t21;
  de_c_tmp = d35 * t6 * t7 * t9 * t11 * t21;
  ee_c_tmp = gc_c_tmp * t7 * t8 * t9 * t11;
  fe_c_tmp = d34 * t3 * t4;
  ab_c_tmp_tmp = d17 * t2;
  ge_c_tmp = ab_c_tmp_tmp * t4 * t7 * t11;
  he_c_tmp = ic_c_tmp * t27;
  ie_c_tmp = jc_c_tmp * t28;
  je_c_tmp = lc_c_tmp * t27;
  sc_c_tmp = sc_c_tmp * t17 * t20 * t28;
  ke_c_tmp = tc_c_tmp * t29;
  le_c_tmp = uc_c_tmp * t28 * t29;
  me_c_tmp = wc_c_tmp * t28;
  ne_c_tmp = yc_c_tmp * t28;
  bb_c_tmp_tmp = d38 * t3;
  oe_c_tmp = bb_c_tmp_tmp * t4;
  pe_c_tmp = dd_c_tmp * t6 * t8 * t11 * t21;
  qe_c_tmp = gd_c_tmp * t28;
  re_c_tmp = i_c_tmp_tmp * t17 * t20 * t29;
  se_c_tmp = in4[1] * in1[3] * t2 * t6 * t11 * t12 * t17 * t20 * t28 * t29;
  i_c_tmp_tmp = ob_c_tmp * t3;
  te_c_tmp = i_c_tmp_tmp * t4;
  ue_c_tmp = mb_c_tmp * t5 * t23;
  ve_c_tmp = nb_c_tmp * t5 * t20;
  we_c_tmp = ib_c_tmp * t5 * t6 * t8 * t23;
  xe_c_tmp = jb_c_tmp * t5 * t6 * t8 * t20;
  ye_c_tmp = d21 * t2;
  af_c_tmp = ye_c_tmp * t4;
  bf_c_tmp = ub_c_tmp * t3;
  cb_c_tmp_tmp = lb_c_tmp * t3;
  cf_c_tmp = cb_c_tmp_tmp * t4;
  df_c_tmp = l_c_tmp_tmp * t2;
  ef_c_tmp = b_c_tmp_tmp_tmp * t2;
  ff_c_tmp = l_c_tmp_tmp * t3;
  gf_c_tmp = ub_c_tmp * t5 * t8 * t20 * t23;
  hf_c_tmp = vb_c_tmp * t5 * t6 * t20 * t23;
  if_c_tmp = df_c_tmp * t4 * t9 * t20 * t23;
  jf_c_tmp = ye_c_tmp * t3 * t4;
  kf_c_tmp = d21 * t3;
  db_c_tmp_tmp = d_c_tmp_tmp * t2;
  lf_c_tmp = db_c_tmp_tmp * t4;
  mf_c_tmp = u_c_tmp_tmp * t5;
  nf_c_tmp = cc_c_tmp * t3;
  of_c_tmp = u_c_tmp_tmp * t6 * t8 * t9 * t11 * t21;
  pf_c_tmp = lf_c_tmp * t6 * t7 * t8 * t11 * t21;
  qf_c_tmp = in2[1] * in2[3] * in4[1] * in1[3] * t2 * t5 * t6 * t7 * t9 * t11;
  rf_c_tmp = xd_c_tmp * t7 * t8 * t9 * t11 * t21;
  sf_c_tmp = wd_c_tmp * t5;
  u_c_tmp_tmp = d16 * t3;
  tf_c_tmp = u_c_tmp_tmp * t4;
  uf_c_tmp = wd_c_tmp * t6 * t7 * t8 * t11 * t21;
  vf_c_tmp = mf_c_tmp * t6 * t8 * t9 * t11 * t21;
  wf_c_tmp = n_c_tmp_tmp * t3;
  xf_c_tmp = kb_c_tmp * t3;
  yf_c_tmp = xb_c_tmp * t3;
  ag_c_tmp = q_c_tmp * t3;
  bg_c_tmp = df_c_tmp * t3;
  cg_c_tmp = ef_c_tmp * t3;
  dg_c_tmp = b_c_tmp_tmp_tmp * t3;
  eg_c_tmp = m_c_tmp_tmp * t3;
  fg_c_tmp = c_c_tmp * t12 * t21;
  gg_c_tmp = c_c_tmp * t13 * t21;
  hg_c_tmp = d16 * t11 * t21;
  ig_c_tmp = d * in4[1] * t5 * t11 * t21;
  jg_c_tmp = d24 * t8;
  kg_c_tmp = b_c_tmp * in4[1] * t3 * t11 * t17 * t20;
  lg_c_tmp = ig_c_tmp * t27;
  mg_c_tmp = ig_c_tmp * t28;
  ng_c_tmp = d32 * t11 * t12 * t21;
  og_c_tmp = d8 * t4 * t8 * t11 * t12 * t21;
  t19 = d31 * t7 * t8 * t9 * t11;
  d_c_tmp = in4[1] * in1[3] * t4 * t8 * t11 * t13;
  e_c_tmp = d26 * t5 * t6 * t15 * t20;
  c_c_tmp = d12 * t27;
  c_tmp = in2[2] * in2[3] * in4[1] * t5;
  b_c_tmp = d17 * t6 * t7 * t8 * t11;
  pg_c_tmp = d18 * t5;
  c[0] = t35 * t36 * ((((((((((((((((((((((((((((((((((((((((((((d2 * 4.0 + d5 *
    8.0) - d7 * 2.0) + fg_c_tmp * t31 / 2.0) + gg_c_tmp * t31 / 2.0) - fg_c_tmp *
    t33 / 2.0) - gg_c_tmp * t33 / 2.0) - d9 * t3 * t10 * t25 * 4.0) - d18 * t11 *
    t21 * 4.0) - d4 * t3 * t10 * t25 * 16.0) - ig_c_tmp * 4.0) + c_tmp * t11 *
    t21 * 4.0) + h_c_tmp) + d5 * t28 * 8.0) - d7 * t30 * 2.0) - f_c_tmp * t15 *
    t20 * 4.0) + g_c_tmp * t15 * t20 * 4.0) + f_c_tmp * t13 * t23 * 8.0) -
    g_c_tmp * t13 * t23 * 8.0) - d13 * t7 * t10 * t25 * 2.0) - l_c_tmp * 2.0) +
    ((((((((((((((((((((d10 * t7 * t10 * t25 * -8.0 - d12 * 2.0) + d13 * t10 *
    t25 * t32) - hg_c_tmp * t31) + d10 * t10 * t25 * t32 * 4.0) + hg_c_tmp * t33)
    - d17 * t11 * t21 * t34) + pg_c_tmp * t11 * t21 * 8.0) - d6 * t3 * t5 * t11 *
    t21 * 4.0) + kg_c_tmp) - lg_c_tmp * 4.0) - d * in4[1] * t3 * t11 * t21 * t30
    * 4.0) - mg_c_tmp * 4.0) - in2[0] * in2[3] * in4[1] * t5 * t11 * t21 * t29 *
    4.0) + i_c_tmp * t27) + h_c_tmp * t28) + m_c_tmp) - n_c_tmp * 2.0) - j_c_tmp
    * 4.0) - o_c_tmp * 2.0) + k_c_tmp * t30 * 4.0)) + (((((((((((((((((d19 * t6 *
    t7 * t11 * t15 * t21 * -2.0 - d21 * t9 * t20 * t23 * 4.0) + jg_c_tmp * t15 *
    t20 * 4.0) - d26 * t6 * t15 * t20 * 4.0) - jg_c_tmp * t13 * t23 * 8.0) + d25
    * t4 * t6 * t13 * t23 * 8.0) + d23 * t5 * t8 * t15 * t20 * 4.0) - c_tmp_tmp *
    t5 * t6 * t15 * t20 * 4.0) - d22 * t2 * t5 * t8 * t13 * t23 * 8.0) + d20 *
    in4[1] * t4 * t5 * t6 * t13 * t23 * 8.0) - d20 * in4[1] * t4 * t6 * t12 *
    t20 * t23 * 2.0) - in3[0] * in3[1] * in4[1] * t2 * t8 * t13 * t20 * t23 *
    2.0) - g_c_tmp * t14 * t20 * t23 * 2.0) + dc_c_tmp * t7 * t11 * t21 * 4.0) +
    d36 * t9 * t11 * t21 * 4.0) - hc_c_tmp * t9 * t11 * t21 * 2.0) + d17 * t9 *
    t11 * t17 * t20) - l_c_tmp * t30 * 2.0)) + ((((((((((((((((((c_c_tmp * -2.0
    - d12 * t28 * 2.0) - d12 * t29 * 2.0) - d29 * t6 * t9 * t11 * t21 * 2.0) -
    d29 * t7 * t8 * t11 * t21 * 2.0) + kg_c_tmp * t30) + lg_c_tmp * t28 * 4.0) +
    lg_c_tmp * t29 * 8.0) + mg_c_tmp * t29 * 4.0) + in2[0] * in2[3] * in4[1] *
    t11 * t17 * t20 * t27 * t30) + m_c_tmp * t30) + in2[0] * in2[3] * in4[1] *
    t11 * t21 * t27 * t28 * t29 * 4.0) - n_c_tmp * t30 * 2.0) - j_c_tmp * t30 *
    4.0) - o_c_tmp * t30 * 2.0) - cb_c_tmp) - db_c_tmp * 2.0) - p_c_tmp * 2.0) +
    eb_c_tmp)) + (((((((((((((((((d30 * t2 * t6 * t11 * t12 * t21 * t28 -
    ng_c_tmp * t29 * 2.0) + ng_c_tmp * t30) - d32 * t11 * t13 * t21 * t29 * 2.0)
    + d31 * t6 * t11 * t13 * t21 * t30) + og_c_tmp * t28) - og_c_tmp * t30) -
    d_c_tmp * t21 * t30) - d33 * t5 * t7 * t8 * t11 * t15 * t21 * 2.0) + d33 *
    t7 * t8 * t11 * t15 * t17 * t20) - d20 * in4[1] * in1[1] * t2 * t7 * t8 *
    t13 * t23 * 4.0) + in3[0] * in3[1] * in4[1] * in1[1] * t4 * t6 * t7 * t13 *
    t23 * 4.0) + xb_c_tmp * t8 * t9 * t15 * t20 * 4.0) - q_c_tmp * t4 * t6 * t9 *
    t15 * t20 * 4.0) + bc_c_tmp) + wb_c_tmp * t6 * t13 * t17 * t23) + s_c_tmp *
    t14 * t17 * t23) - o_c_tmp_tmp * t15 * t20 * 4.0)) + (((((((((((((((e_c_tmp *
    4.0 + o_c_tmp_tmp * t13 * t23 * 8.0) - d25 * t4 * t5 * t6 * t13 * t23 * 8.0)
    - d23 * t3 * t8 * t12 * t20 * t23 * 4.0) + d22 * t3 * t4 * t6 * t12 * t20 *
    t23 * 4.0) - d22 * t3 * t4 * t6 * t13 * t20 * t23 * 2.0) - d20 * in4[1] * t2
    * t3 * t8 * t14 * t20 * t23 * 2.0) - p_c_tmp_tmp * t12 * t20 * t23 * 2.0) -
    in3[0] * in3[1] * in4[1] * t4 * t5 * t6 * t13 * t20 * t23 * 2.0) -
    p_c_tmp_tmp * t14 * t20 * t23 * 2.0) - in3[0] * in3[1] * in4[1] * t2 * t5 *
    t8 * t15 * t20 * t23 * 2.0) - g_c_tmp * t12 * t20 * t23 * t28 * 2.0) +
    jc_c_tmp * 4.0) - ic_c_tmp * 4.0) - ge_c_tmp * t21 * 2.0) - lc_c_tmp * 4.0))
    + ((((((((((((((((d35 * t5 * t8 * t11 * t21 * -2.0 - d36 * t5 * t9 * t11 *
    t21 * 4.0) - b_c_tmp * t21 * 2.0) - t_c_tmp * t7 * t11 * t21 * t27 * 2.0) +
    d15 * in1[3] * t2 * t6 * t11 * t21 * t29 * 4.0) - d4 * in1[3] * t3 * t7 *
    t11 * t21 * t29 * 2.0) - d14 * in4[1] * in1[3] * t2 * t6 * t11 * t21 * t30 *
    2.0) + w_c_tmp_tmp * t30 * 4.0) - in2[0] * in2[2] * in4[1] * in1[3] * t4 *
    t8 * t11 * t21 * t27 * 4.0) - vc_c_tmp * 2.0) - xc_c_tmp * 4.0) - v_c_tmp *
    t29 * 4.0) - gc_c_tmp * t6 * t11 * t21 * t30 * 2.0) + qc_c_tmp * 4.0) +
    ad_c_tmp) + w_c_tmp) - cd_c_tmp * 4.0)) + ((((((((((((((((d11 * t9 * t11 *
    t21 * t27 * t29 * 4.0 - d38 * t4 * t6 * t8 * t11 * t21 * 4.0) - d39 * t5 *
    t6 * t7 * t11 * t21 * 4.0) - d1 * t2 * t4 * t7 * t9 * t11 * t21 * 4.0) - d39
    * t3 * t8 * t9 * t11 * t21 * 2.0) - d27 * in4[1] * t2 * t5 * t8 * t9 * t11 *
    t21 * 2.0) + d27 * in4[1] * t4 * t5 * t7 * t8 * t11 * t21 * 4.0) - in2[0] *
    in2[3] * in4[1] * t6 * t7 * t8 * t9 * t11 * t21 * 4.0) + id_c_tmp) + d39 *
    t8 * t9 * t11 * t17 * t20) - x_c_tmp * 4.0) + jd_c_tmp * 4.0) - y_c_tmp *
    t30 * 2.0) - in2[0] * in2[3] * in4[1] * t5 * t11 * t21 * t27 * t28 * t29 *
    8.0) + n_c_tmp * t29 * t30 * 4.0) - nd_c_tmp * 2.0) + t19 * t12 * t21)) +
    (((((((((((((((((d30 * t4 * t6 * t7 * t9 * t11 * t12 * t21 - t19 * t13 * t21)
    + d40 * t6 * t7 * t9 * t11 * t13 * t21) + d8 * t2 * t7 * t8 * t9 * t11 * t14
    * t21) - d8 * t4 * t6 * t7 * t9 * t11 * t14 * t21) - in4[1] * in1[3] * t2 *
    t7 * t8 * t9 * t11 * t15 * t21) + in4[1] * in1[3] * t4 * t6 * t7 * t9 * t11 *
    t15 * t21) + fb_c_tmp * t17 * t20) + bb_c_tmp * t17 * t20) - cb_c_tmp * t30)
    + db_c_tmp * t29 * 4.0) + p_c_tmp * t29 * 4.0) - pd_c_tmp * 2.0) - od_c_tmp *
    4.0) - gb_c_tmp * t28 * 2.0) - bb_c_tmp * t21 * t27 * 4.0) + eb_c_tmp * t30)
     + qd_c_tmp)) + ((((((((((((((((d_c_tmp * t17 * t20 * t27 - d8 * t2 * t6 *
    t11 * t12 * t21 * t28 * t30) - d31 * t6 * t11 * t12 * t21 * t29 * t30 * 2.0)
    - og_c_tmp * t27 * t28 * 2.0) - d8 * t2 * t6 * t11 * t13 * t21 * t29 * t30 *
    2.0) + hb_c_tmp * t28 * t30) + d19 * t5 * t6 * t7 * t11 * t15 * t17 * t20) -
    mb_c_tmp * t23 * 4.0) - nb_c_tmp * t20 * 4.0) + pb_c_tmp * t20 * 4.0) -
    sb_c_tmp * 4.0) - tb_c_tmp * 4.0) + ib_c_tmp * t7 * t9 * t23 * 4.0) +
    jb_c_tmp * t7 * t9 * t20 * 4.0) + kb_c_tmp * t6 * t8 * t20 * 4.0) - ub_c_tmp
                      * t8 * t20 * t23 * 4.0) + vb_c_tmp * t6 * t20 * t23 * 4.0))
    + ((((((((((((((d20 * in2[0] * in2[1] * in4[1] * t3 * t9 * t20 * t23 * -4.0
    - d20 * in2[0] * in2[3] * in4[1] * t5 * t7 * t20 * t23 * 4.0) + in3[0] *
    in3[1] * in2[0] * in2[1] * in4[1] * t9 * t20 * t23 * t28 * 8.0) + d20 * in2
    [0] * in2[1] * in4[1] * in1[1] * t2 * t3 * t4 * t23 * 8.0) + in3[0] * in3[1]
    * in2[0] * in2[3] * in4[1] * in1[3] * t2 * t3 * t4 * t20 * 8.0) + ue_c_tmp *
    4.0) + ve_c_tmp * 4.0) - te_c_tmp * t20 * 4.0) - pb_c_tmp * t5 * t20 * 4.0)
    + qb_c_tmp * t6 * t8 * t23 * 8.0) + rb_c_tmp * t6 * t8 * t20 * 8.0) -
          qb_c_tmp * t7 * t9 * t23 * 4.0) + we_c_tmp * 4.0) - rb_c_tmp * t7 * t9
        * t20 * 4.0) + xe_c_tmp * 4.0)) + ((((((((((((((d20 * in2[2] * in2[3] *
    in4[1] * in1[3] * t3 * t6 * t8 * t20 * -4.0 - d20 * in2[2] * in2[3] * in4[1]
    * in1[3] * t5 * t6 * t8 * t20 * 4.0) - d20 * in2[0] * in2[1] * in4[1] * in1
    [1] * t2 * t4 * t23 * t28 * 4.0) - in3[0] * in3[1] * in2[0] * in2[3] * in4[1]
    * in1[3] * t2 * t4 * t20 * t28 * 4.0) - sb_c_tmp * t28 * 4.0) - tb_c_tmp *
    t28 * 4.0) + af_c_tmp * t7 * t20 * t23 * 8.0) + bf_c_tmp * t8 * t20 * t23 *
    8.0) - cf_c_tmp * t6 * t20 * t23 * 8.0) + gf_c_tmp * 4.0) - hf_c_tmp * 4.0)
    + if_c_tmp * 4.0) + ff_c_tmp * t5 * t7 * t20 * t23 * 4.0) - ef_c_tmp * t4 *
    t9 * t20 * t23 * 4.0) + d21 * t6 * t7 * t8 * t20 * t23 * 8.0)) +
    (((((((((((((((d20 * in2[0] * in2[3] * in4[1] * t6 * t8 * t9 * t20 * t23 *
    4.0 - b_c_tmp_tmp_tmp * t6 * t8 * t9 * t20 * t23 * 4.0) - d20 * in2[0] *
    in2[2] * in4[1] * t2 * t8 * t20 * t23 * t28 * 4.0) + in3[0] * in3[1] * in2[0]
    * in2[2] * in4[1] * t4 * t6 * t20 * t23 * t28 * 4.0) + ac_c_tmp * t7 * t8 *
    t13 * t23 * 4.0) - yb_c_tmp * t7 * t13 * t23 * 4.0) - yf_c_tmp * t8 * t9 *
    t15 * t20 * 4.0) + ag_c_tmp * t4 * t6 * t9 * t15 * t20 * 4.0) + sd_c_tmp *
    t8 * t13 * t17 * t23) + rd_c_tmp * t6 * t14 * t17 * t23) + ud_c_tmp) +
         ac_c_tmp * t8 * t13 * t17 * t23) + yb_c_tmp * t14 * t17 * t23) +
       yb_c_tmp * t15 * t17 * t23) + bc_c_tmp * t28) + o_c_tmp_tmp * t12 * t20 *
     t23 * 4.0)) + (((((((((((((((d20 * in4[1] * t3 * t4 * t5 * t6 * t12 * t20 *
    t23 * -4.0 - d20 * in4[1] * t2 * t3 * t5 * t8 * t13 * t20 * t23 * 2.0) -
    in3[0] * in3[1] * in4[1] * t3 * t4 * t5 * t6 * t14 * t20 * t23 * 2.0) -
    e_c_tmp * t23 * 2.0) - p_c_tmp_tmp * t12 * t20 * t23 * t28 * 2.0) - ae_c_tmp
    * 2.0) - ec_c_tmp * 2.0) - fe_c_tmp * t5 * t6 * t11 * t21 * 2.0) - ce_c_tmp *
    2.0) - cc_c_tmp * t7 * t8 * t9 * t11 * t21 * 2.0) - de_c_tmp * 2.0) -
                        be_c_tmp * 2.0) + kc_c_tmp * t17 * t20) + mc_c_tmp * t17
                      * t20) + xd_c_tmp * t6 * t11 * t17 * t20) + hc_c_tmp * t5 *
                    t9 * t11 * t17 * t20)) + (((((((((((((((nc_c_tmp * t17 * t20
    + oc_c_tmp * t17 * t20) + he_c_tmp * 4.0) - ie_c_tmp * 4.0) - kc_c_tmp * t21
    * t28 * 4.0) - jc_c_tmp * t29 * 8.0) + ic_c_tmp * t29 * 4.0) + je_c_tmp *
    8.0) - mc_c_tmp * t21 * t30 * 4.0) + lc_c_tmp * t28 * 4.0) - yd_c_tmp * t8 *
    t11 * t21 * t30 * 2.0) - nc_c_tmp * t21 * t28 * 4.0) + sc_c_tmp) - oc_c_tmp *
    t21 * t30 * 4.0) + pc_c_tmp * t17 * t20 * t30) + rc_c_tmp * t27)) +
    (((((((((((((((d35 * t8 * t11 * t17 * t20 * t30 + rc_c_tmp * t29) + ke_c_tmp
    * 4.0) - le_c_tmp * 4.0) - tc_c_tmp * t30 * 2.0) + uc_c_tmp * t29 * t30 *
    4.0) + me_c_tmp * 4.0) - in2[0] * in2[1] * in4[1] * in1[3] * t3 * t7 * t11 *
             t21 * t29 * t30 * 2.0) - wc_c_tmp * t30 * 4.0) + xc_c_tmp * t29 *
           8.0) - vc_c_tmp * t30 * 2.0) - ne_c_tmp * 2.0) - yc_c_tmp * t29 * 4.0)
       - qc_c_tmp * t29 * 2.0) + bd_c_tmp * t27 * t28) + ad_c_tmp * t30)) +
    (((((((((((((((bd_c_tmp * t28 * t29 + w_c_tmp * t30) - c_c_tmp * t28 * t29 *
    4.0) - cd_c_tmp * t30 * 4.0) + oe_c_tmp * t7 * t9 * t11 * t21 * 4.0) +
    pe_c_tmp * 8.0) + dd_c_tmp * t7 * t9 * t11 * t21 * 4.0) - ed_c_tmp * t5 * t6
             * t9 * t11 * t21 * 2.0) - fd_c_tmp * t5 * t7 * t9 * t11 * t21 * 2.0)
           + d18 * t6 * t7 * t8 * t9 * t11 * t21 * 4.0) + d1 * t5 * t6 * t7 * t8
          * t9 * t11 * t21 * 4.0) - c_tmp * t6 * t7 * t8 * t9 * t11 * t21 * 2.0)
        + ed_c_tmp * t6 * t9 * t11 * t17 * t20) + fd_c_tmp * t7 * t9 * t11 * t17
       * t20) + hd_c_tmp * t6 * t9 * t11 * t17 * t20) + qe_c_tmp * 4.0)) +
    (((((((((((((((d6 * t6 * t7 * t8 * t9 * t11 * t17 * t20 - gd_c_tmp * t30 *
    4.0) + in2[1] * in2[3] * in4[1] * t2 * t5 * t6 * t7 * t11 * t21 * t29 * 8.0)
    - hd_c_tmp * t7 * t8 * t11 * t21 * t27 * 8.0) + id_c_tmp * t30) - x_c_tmp *
               t30 * 4.0) + jd_c_tmp * t30 * 4.0) - h_c_tmp_tmp * t7 * t8 * t9 *
             t11 * t12 * t21) + ab_c_tmp * t6 * t7 * t9 * t11 * t12 * t21) -
           kd_c_tmp * t12 * t21) + ld_c_tmp * t12 * t21) + kd_c_tmp * t13 * t21)
        - ld_c_tmp * t13 * t21) + md_c_tmp * t17 * t20) - kd_c_tmp * t14 * t21)
     + ld_c_tmp * t14 * t21)) + (((((((((((((((md_c_tmp * t21 * t29 * -4.0 +
    nd_c_tmp * t27 * 4.0) + re_c_tmp) + j_c_tmp_tmp * t17 * t20 * t28) -
    db_c_tmp * t28 * t29 * 4.0) + od_c_tmp * t28 * 4.0) - pd_c_tmp * t30 * 2.0)
    + se_c_tmp) + qd_c_tmp * t30) + k_c_tmp_tmp * t13 * t17 * t20 * t27 * t30) -
    hb_c_tmp * t27 * t28 * t30 * 2.0) + rd_c_tmp * t5 * t6 * t13 * t17 * t23) +
    td_c_tmp * t14 * t17 * t23) + td_c_tmp * t15 * t17 * t23) + ud_c_tmp * t28)
    - sf_c_tmp * t9 * t11 * t21 * 2.0)) + ((((((((((((((vd_c_tmp * t5 * t7 * t11
    * t21 * -2.0 - pf_c_tmp * 4.0) - tf_c_tmp * t6 * t7 * t9 * t11 * t21 * 2.0)
    - r_c_tmp_tmp * t6 * t8 * t9 * t11 * t21 * 2.0) + of_c_tmp * 4.0) - in2[0] *
    in2[3] * in4[1] * in1[3] * t3 * t5 * t6 * t7 * t8 * t11 * t21 * 2.0) -
    t_c_tmp_tmp * t6 * t7 * t9 * t11 * t21 * 2.0) + wd_c_tmp * t9 * t11 * t17 *
    t20) + mf_c_tmp * t7 * t11 * t17 * t20) - rf_c_tmp * 2.0) + yd_c_tmp * t5 *
    t8 * t11 * t17 * t20) + t_c_tmp * t6 * t8 * t9 * t11 * t17 * t20) + u_c_tmp *
    t6 * t7 * t9 * t11 * t17 * t20) + v_c_tmp_tmp * t6 * t7 * t8 * t11 * t17 *
    t20) + ee_c_tmp * t17 * t20)) + ((((((((((((((ec_c_tmp * t28 * 4.0 +
    ae_c_tmp * t30 * 4.0) + be_c_tmp * t28 * 4.0) + ce_c_tmp * t30 * 4.0) +
    de_c_tmp * t29 * 4.0) - ee_c_tmp * t21 * t27 * 4.0) + fe_c_tmp * t6 * t11 *
    t17 * t20 * t30) + ge_c_tmp * t17 * t20 * t30) - he_c_tmp * t29 * 8.0) +
    ie_c_tmp * t29 * 8.0) + b_c_tmp * t17 * t20 * t30) - je_c_tmp * t28 * 8.0) +
    sc_c_tmp * t30) + ke_c_tmp * t30 * 4.0) - le_c_tmp * t30 * 4.0)) +
    (((((((((((((me_c_tmp * t30 * 4.0 + ne_c_tmp * t29 * 4.0) - oe_c_tmp * t5 *
                t7 * t9 * t11 * t21 * 4.0) - pg_c_tmp * t6 * t7 * t8 * t9 * t11 *
               t21 * 4.0) + in2[1] * in2[3] * in4[1] * t2 * t3 * t5 * t8 * t9 *
              t11 * t17 * t20) - pe_c_tmp * t28 * 8.0) + qe_c_tmp * t30 * 4.0) +
           b_c_tmp_tmp * t7 * t8 * t9 * t11 * t12 * t21) - e_c_tmp_tmp * t6 * t7
          * t9 * t11 * t12 * t21) + re_c_tmp * t30) + se_c_tmp * t30) - eg_c_tmp
       * t4 * t5 * t23 * 8.0) - wf_c_tmp * t4 * t5 * t20 * 8.0) + te_c_tmp * t5 *
     t20 * 4.0)) + (((((((((((((qb_c_tmp * t5 * t6 * t8 * t23 * -8.0 - rb_c_tmp *
    t5 * t6 * t8 * t20 * 8.0) + xf_c_tmp * t5 * t6 * t8 * t20 * 4.0) + ue_c_tmp *
    t28 * 4.0) + ve_c_tmp * t28 * 4.0) + we_c_tmp * t28 * 4.0) + xe_c_tmp * t28 *
    4.0) - jf_c_tmp * t7 * t20 * t23 * 8.0) - af_c_tmp * t5 * t7 * t20 * t23 *
    8.0) - bf_c_tmp * t5 * t8 * t20 * t23 * 8.0) + cf_c_tmp * t5 * t6 * t20 *
                       t23 * 8.0) - bg_c_tmp * t4 * t9 * t20 * t23 * 8.0) +
                     cg_c_tmp * t4 * t9 * t20 * t23 * 4.0) - kf_c_tmp * t6 * t7 *
                    t8 * t20 * t23 * 8.0)) + (((((((((((((d21 * t5 * t6 * t7 *
    t8 * t20 * t23 * -8.0 - ff_c_tmp * t6 * t8 * t9 * t20 * t23 * 8.0) +
    dg_c_tmp * t6 * t8 * t9 * t20 * t23 * 4.0) + gf_c_tmp * t28 * 4.0) -
    hf_c_tmp * t28 * 4.0) + if_c_tmp * t28 * 4.0) + l_c_tmp_tmp * t6 * t8 * t9 *
    t20 * t23 * t28 * 4.0) + jf_c_tmp * t5 * t7 * t20 * t23 * 8.0) + kf_c_tmp *
    t5 * t6 * t7 * t8 * t20 * t23 * 8.0) + uf_c_tmp * 4.0) + lf_c_tmp * t5 * t6 *
    t7 * t8 * t11 * t21 * 8.0) - nf_c_tmp * t5 * t7 * t8 * t9 * t11 * t21 * 2.0)
    - vf_c_tmp * 4.0) + nf_c_tmp * t7 * t8 * t9 * t11 * t17 * t20)) +
                      (((((((((s_c_tmp_tmp * t7 * t8 * t9 * t11 * t17 * t20 +
    qf_c_tmp * t17 * t20) - of_c_tmp * t28 * 4.0) - pf_c_tmp * t30 * 4.0) -
    qf_c_tmp * t21 * t29 * 4.0) + rf_c_tmp * t27 * 4.0) - sf_c_tmp * t6 * t7 *
    t8 * t11 * t21 * 8.0) + tf_c_tmp * t5 * t6 * t7 * t9 * t11 * t17 * t20) +
                        uf_c_tmp * t30 * 4.0) + vf_c_tmp * t28 * 4.0)) * -0.25;
  d = in4[0] * t10;
  c_tmp = d * t13;
  d2 = in4[1] * t10;
  d5 = d2 * t13;
  b_c_tmp = in4[0] * t3 * t10;
  d7 = in4[1] * t3;
  c_c_tmp = d20 * in2[1];
  c_tmp_tmp = d * t12;
  f_c_tmp = c_tmp_tmp * t20 * t25;
  b_c_tmp_tmp = d2 * t12;
  g_c_tmp = b_c_tmp_tmp * t20 * t25;
  e_c_tmp_tmp = d22 * t9;
  h_c_tmp = e_c_tmp_tmp * t15 * t22;
  i_c_tmp = d37 * t11 * t24;
  j_c_tmp = d_c_tmp_tmp * t11 * t24;
  j_c_tmp_tmp = d28 * t5 * t11 * t24;
  k_c_tmp_tmp = d28 * t11;
  k_c_tmp = k_c_tmp_tmp * t17 * t22;
  o_c_tmp_tmp = d8 * t7 * t11 * t12;
  l_c_tmp = o_c_tmp_tmp * t24;
  m_c_tmp = c_c_tmp * in2[3] * in4[1];
  p_c_tmp_tmp = d23 * t4;
  n_c_tmp = d25 * t9;
  r_c_tmp_tmp = d22 * t6 * t8;
  o_c_tmp = j_c_tmp_tmp * t27;
  p_c_tmp = k_c_tmp * t27;
  k_c_tmp *= t29;
  s_c_tmp = k_c_tmp_tmp * t24 * t27 * t29;
  k_c_tmp_tmp = d31 * t4;
  t_c_tmp = k_c_tmp_tmp * t9 * t11;
  u_c_tmp = d8 * t6 * t8 * t9 * t11;
  s_c_tmp_tmp = f_c_tmp_tmp * t7 * t11 * t12;
  v_c_tmp = s_c_tmp_tmp * t24;
  w_c_tmp = o_c_tmp_tmp * t17 * t22;
  x_c_tmp = d33 * t4;
  y_c_tmp = d24 * t4;
  ab_c_tmp = p_c_tmp_tmp * t5;
  bb_c_tmp = p_c_tmp_tmp * t7;
  o_c_tmp_tmp = d22 * t7 * t9;
  cb_c_tmp = p_c_tmp_tmp * t12 * t21 * t23;
  db_c_tmp = r_c_tmp_tmp * t12 * t21 * t23;
  t_c_tmp_tmp = d16 * t5;
  eb_c_tmp = t_c_tmp_tmp * t7 * t11 * t24;
  fb_c_tmp = db_c_tmp_tmp * t6 * t11 * t24;
  gb_c_tmp = d_c_tmp_tmp * t4;
  hb_c_tmp = d37 * t4;
  mb_c_tmp = fc_c_tmp * t8 * t11;
  nb_c_tmp = d17 * t4 * t6 * t11;
  d_c_tmp_tmp = d16 * t7 * t11;
  pb_c_tmp = d_c_tmp_tmp * t17 * t22;
  d_c_tmp_tmp *= t24;
  sb_c_tmp = d_c_tmp_tmp * t27;
  tb_c_tmp = d34 * t9 * t11;
  v_c_tmp_tmp = d1 * t4;
  vb_c_tmp = v_c_tmp_tmp * t5;
  wb_c_tmp = d38 * t6 * t7 * t11 * t24;
  yb_c_tmp = v_c_tmp_tmp * t7 * t8 * t11;
  ac_c_tmp = d8 * t5;
  bc_c_tmp = v_c_tmp * t27;
  dc_c_tmp = ac_c_tmp * t7 * t11 * t12 * t24;
  ec_c_tmp = w_c_tmp * t27;
  w_c_tmp *= t29;
  gc_c_tmp = l_c_tmp * t27 * t29;
  hc_c_tmp = m_c_tmp * in1[3];
  ic_c_tmp = c_c_tmp * in2[2] * in4[1];
  jc_c_tmp = hc_c_tmp * t2 * t4;
  jb_c_tmp *= t4;
  kb_c_tmp *= t4;
  kc_c_tmp = jb_c_tmp * t6;
  lc_c_tmp = hc_c_tmp * t6;
  ub_c_tmp *= t4;
  mc_c_tmp = d21 * t4;
  nc_c_tmp = mc_c_tmp * t6;
  oc_c_tmp = ic_c_tmp * t2;
  ic_c_tmp *= t4;
  pc_c_tmp = ic_c_tmp * t6;
  qc_c_tmp = lb_c_tmp * t6;
  rc_c_tmp = y_c_tmp * t5;
  sc_c_tmp = rc_c_tmp * t15 * t21;
  tc_c_tmp = ab_c_tmp * t7;
  t19 = d25 * t5;
  w_c_tmp_tmp = t19 * t6;
  uc_c_tmp = w_c_tmp_tmp * t8;
  vc_c_tmp = uc_c_tmp * t15 * t21;
  wc_c_tmp = d25 * t6;
  e_c_tmp = d22 * t5;
  xc_c_tmp = e_c_tmp * t6;
  yc_c_tmp = xc_c_tmp * t7 * t8;
  ad_c_tmp = wc_c_tmp * t8;
  xc_c_tmp *= t8;
  bd_c_tmp = xc_c_tmp * t15;
  cd_c_tmp = d11 * t4;
  cc_c_tmp *= t4;
  dd_c_tmp = q_c_tmp_tmp * t5 * t6 * t11 * t24;
  ed_c_tmp = hb_c_tmp * t5 * t8 * t11 * t24;
  fd_c_tmp = db_c_tmp_tmp * t5 * t6 * t11 * t24;
  gd_c_tmp = d36 * t4 * t6 * t11;
  hd_c_tmp = gb_c_tmp * t5 * t8 * t11 * t24;
  id_c_tmp = cd_c_tmp * t6 * t11;
  gb_c_tmp = gb_c_tmp * t8 * t11;
  jd_c_tmp = ab_c_tmp_tmp * t8 * t11;
  ab_c_tmp_tmp = d34 * t5 * t9 * t11;
  kd_c_tmp = ab_c_tmp_tmp * t17 * t22;
  ld_c_tmp = q_c_tmp_tmp * t6 * t11;
  hb_c_tmp = hb_c_tmp * t8 * t11 * t24;
  md_c_tmp = eb_c_tmp * t27;
  nd_c_tmp = fb_c_tmp * t29;
  od_c_tmp = gb_c_tmp * t24 * t27;
  pd_c_tmp = sb_c_tmp * t29;
  qd_c_tmp = d39 * t4;
  rd_c_tmp = d38 * t5;
  td_c_tmp = d6 * t4;
  ud_c_tmp = rd_c_tmp * t6 * t7 * t11 * t24;
  vd_c_tmp = vb_c_tmp * t7 * t8 * t11 * t24;
  wd_c_tmp = qd_c_tmp * t6 * t8 * t11 * t24;
  xd_c_tmp = yb_c_tmp * t17 * t22;
  yd_c_tmp = wb_c_tmp * t29;
  yb_c_tmp = yb_c_tmp * t24 * t27;
  ae_c_tmp = f_c_tmp_tmp * t5;
  be_c_tmp = h_c_tmp_tmp * t4;
  ce_c_tmp = ae_c_tmp * t7 * t11 * t12;
  de_c_tmp = ce_c_tmp * t17 * t22;
  ce_c_tmp *= t24;
  ee_c_tmp = s_c_tmp_tmp * t17 * t22;
  fe_c_tmp = ab_c_tmp * t12;
  ge_c_tmp = fc_c_tmp * t6 * t7 * t9 * t11;
  he_c_tmp = cd_c_tmp * t7 * t8 * t9 * t11 * t24;
  ie_c_tmp = y_c_tmp_tmp * t8 * t11;
  je_c_tmp = d18 * t4;
  ke_c_tmp = k_c_tmp_tmp * t5;
  le_c_tmp = k_c_tmp_tmp * t6 * t7 * t8 * t11 * t12 * t24;
  me_c_tmp = ke_c_tmp * t9 * t11;
  ac_c_tmp = ac_c_tmp * t6 * t8 * t9 * t11;
  ne_c_tmp = ee_c_tmp * t27 * t29;
  rb_c_tmp *= t4;
  oe_c_tmp = jc_c_tmp * t5;
  pe_c_tmp = xf_c_tmp * t4;
  qe_c_tmp = n_c_tmp_tmp * t5;
  jb_c_tmp = jb_c_tmp * t5 * t6;
  re_c_tmp = hc_c_tmp * t5 * t6;
  se_c_tmp = bf_c_tmp * t4;
  te_c_tmp = ub_c_tmp * t5;
  ue_c_tmp = ye_c_tmp * t5;
  mc_c_tmp = mc_c_tmp * t5 * t6;
  ve_c_tmp = oc_c_tmp * t5;
  ic_c_tmp = ic_c_tmp * t5 * t6;
  we_c_tmp = m_c_tmp * t2 * t4;
  lb_c_tmp = lb_c_tmp * t5 * t6;
  xe_c_tmp = l_c_tmp_tmp * t4 * t6;
  af_c_tmp = m_c_tmp * t6;
  bf_c_tmp = cc_c_tmp * t5;
  cf_c_tmp = d35 * t4;
  gf_c_tmp = nf_c_tmp * t4;
  hf_c_tmp = cc_c_tmp * t6 * t7 * t8 * t11 * t24;
  fc_c_tmp = fc_c_tmp * t5 * t6 * t7 * t9 * t11 * t24;
  cd_c_tmp = cd_c_tmp * t5 * t7 * t8 * t9 * t11;
  if_c_tmp = be_c_tmp * t5;
  jf_c_tmp = be_c_tmp * t6 * t7 * t8 * t11 * t12 * t17 * t22;
  kf_c_tmp = in4[1] * t5;
  lf_c_tmp = p_c_tmp_tmp * t13;
  mf_c_tmp = r_c_tmp_tmp * t13;
  nf_c_tmp = o_c_tmp_tmp * t13;
  of_c_tmp = y_c_tmp * t13;
  pf_c_tmp = ab_c_tmp * t13;
  qf_c_tmp = ad_c_tmp * t13;
  rf_c_tmp = o_c_tmp_tmp * t12;
  sf_c_tmp = rc_c_tmp * t13;
  tf_c_tmp = uc_c_tmp * t13;
  uf_c_tmp = y_c_tmp * t12;
  vf_c_tmp = ad_c_tmp * t12;
  xf_c_tmp = x_c_tmp * t5;
  fg_c_tmp = kf_c_tmp * t6;
  gg_c_tmp = ye_c_tmp * t7 * t8;
  hg_c_tmp = nc_c_tmp * t7;
  ig_c_tmp = xc_c_tmp * t12;
  jg_c_tmp = in4[1] * t6;
  kg_c_tmp = ue_c_tmp * t7 * t8;
  lg_c_tmp = mc_c_tmp * t7;
  mg_c_tmp = d27 * in4[1] * in1[3] * t9 * t11 * t24;
  ng_c_tmp = d20 * in4[1] * t6 * t7 * t8;
  og_c_tmp = o_c_tmp_tmp * t15;
  c[1] = t36 * ((((((((((((((((((((((((((((((((((((((((((c_tmp * t25 * -4.0 - d5
    * t25 * 16.0) - d28 * t11 * t24 * 4.0) + b_c_tmp * t13 * t25 * 4.0) + d7 *
    t10 * t13 * t25 * 16.0) + f_c_tmp) - c_tmp * t20 * t25) + g_c_tmp * 4.0) -
    d5 * t20 * t25 * 4.0) + in4[0] * in1[1] * t7 * t10 * t13 * t25 * 4.0) + in4
    [1] * in1[1] * t7 * t10 * t13 * t25 * 16.0) - l_c_tmp * 2.0) - x_c_tmp * t11
    * t15 * t24 * 2.0) - jg_c_tmp * t8 * t11 * t15 * t24 * 2.0) - b_c_tmp * t12 *
    t20 * t25 * 2.0) - in4[0] * t3 * t10 * t13 * t20 * t25) - in4[1] * t3 * t10 *
    t12 * t20 * t25 * 8.0) - in4[1] * t3 * t10 * t13 * t20 * t25 * 4.0) - in4[0]
    * t7 * t10 * t12 * t21 * t25) - in4[1] * t7 * t10 * t12 * t21 * t25 * 4.0) +
    f_c_tmp * t28) + g_c_tmp * t28 * 4.0) + c_tmp_tmp * t21 * t25 * t32 / 2.0) +
    h_c_tmp * 4.0) + ((((((((((((((((((((j_c_tmp_tmp * 4.0 - q_c_tmp * t5 * t15 *
    t22 * 4.0) + p_c_tmp_tmp * t15 * t21 * 4.0) + r_c_tmp_tmp * t15 * t21 * 4.0)
    - og_c_tmp * t21 * 4.0) - d20 * in4[1] * t9 * t13 * t20 * t23 * 4.0) - d20 *
    in4[1] * t9 * t12 * t22 * t23 * 2.0) - h_c_tmp * t23 * 2.0) + i_c_tmp * t31)
    - i_c_tmp * t33) - j_c_tmp * t31) + j_c_tmp * t33) - o_c_tmp * 4.0) -
    j_c_tmp_tmp * t29 * 4.0) + p_c_tmp) + k_c_tmp) - s_c_tmp * 4.0) + dc_c_tmp *
    4.0) - l_c_tmp * t30 * 2.0) + b_c_tmp_tmp * t17 * t20 * t25 * t32) -
    hc_c_tmp * t9 * t21 * 4.0)) + (((((((((((((((((m_c_tmp * t5 * t21 * t23 *
    4.0 + d22 * in1[3] * t5 * t7 * t15 * t21 * 4.0) - d22 * t2 * t3 * t4 * t15 *
    t21 * 4.0) - d20 * in4[1] * t2 * t4 * t5 * t15 * t21 * 4.0) - d20 * in4[1] *
    t3 * t6 * t8 * t15 * t21 * 4.0) - bd_c_tmp * t21 * 4.0) - cb_c_tmp * 6.0) -
    p_c_tmp_tmp * t14 * t21 * t23 * 2.0) + n_c_tmp * t13 * t20 * t23 * 4.0) -
    n_c_tmp * t12 * t22 * t23 * 2.0) - db_c_tmp * 6.0) - r_c_tmp_tmp * t14 * t21
    * t23 * 2.0) + e_c_tmp_tmp * t12 * t22 * t23 * t28 * 4.0) - id_c_tmp * t24 *
    2.0) - eb_c_tmp * 4.0) - jd_c_tmp * t24 * 2.0) + pb_c_tmp) + tb_c_tmp * t17 *
    t22)) + (((((((((((((((((d15 * in1[3] * t7 * t11 * t24 * t27 * -2.0 - d15 *
    in1[3] * t7 * t11 * t24 * t29 * 2.0) - mg_c_tmp * t27 * 2.0) - mg_c_tmp *
    t29 * 2.0) - wb_c_tmp * 2.0) - d38 * t8 * t9 * t11 * t24 * 2.0) - td_c_tmp *
    t6 * t9 * t11 * t24 * 2.0) + o_c_tmp * t29 * 8.0) + p_c_tmp * t30) + k_c_tmp
    * t30) - s_c_tmp * t30 * 4.0) - ce_c_tmp * 2.0) - t_c_tmp * t14 * t24) +
    t_c_tmp * t15 * t24) - u_c_tmp * t14 * t24) + u_c_tmp * t15 * t24) -
              bc_c_tmp) - v_c_tmp * t29)) + (((((((((((((((((d8 * t5 * t7 * t11 *
    t12 * t24 * t27 * -4.0 - d8 * t5 * t7 * t11 * t12 * t24 * t29 * 4.0) +
    ec_c_tmp) + w_c_tmp) - gc_c_tmp * 4.0) + xf_c_tmp * t11 * t15 * t17 * t22) +
    fg_c_tmp * t8 * t11 * t15 * t17 * t22) - xb_c_tmp * t4 * t9 * t15 * t21 *
    4.0) - q_c_tmp * t6 * t8 * t9 * t15 * t21 * 4.0) + sc_c_tmp * 4.0) +
    vc_c_tmp * 4.0) + uf_c_tmp * t21 * t23 * 8.0) + fe_c_tmp * t21 * t23 * 6.0)
    - pf_c_tmp * t21 * t23 * 2.0) - bb_c_tmp * t13 * t20 * t23 * 4.0) + bb_c_tmp
    * t12 * t22 * t23 * 6.0) + vf_c_tmp * t21 * t23 * 8.0) - d25 * t7 * t9 * t12
    * t21 * t23 * 2.0)) + (((((((((((((((((d22 * t5 * t6 * t8 * t12 * t21 * t23 *
    6.0 - d22 * t5 * t6 * t8 * t13 * t21 * t23 * 2.0) - ng_c_tmp * t13 * t20 *
    t23 * 4.0) + ng_c_tmp * t12 * t22 * t23 * 6.0) + lf_c_tmp * t17 * t20 * t23)
    + mf_c_tmp * t17 * t20 * t23) + rf_c_tmp * t17 * t20 * t23) + nf_c_tmp * t17
    * t20 * t23) - cb_c_tmp * t28 * 2.0) + og_c_tmp * t17 * t20 * t23) -
    db_c_tmp * t28 * 2.0) - dd_c_tmp * 4.0) - ie_c_tmp * t24 * 2.0) + fd_c_tmp *
    4.0) + ed_c_tmp * 4.0) - hd_c_tmp * 4.0) + mb_c_tmp * t17 * t22) + nb_c_tmp *
    t17 * t22)) + ((((((((((((((((d3 * in4[1] * in1[3] * t2 * t6 * t11 * t24 *
    t29 * -4.0 + d3 * in4[1] * in1[3] * t4 * t8 * t11 * t24 * t27 * 4.0) +
    nd_c_tmp * 4.0) + md_c_tmp * 4.0) - fb_c_tmp * t30 * 2.0) - od_c_tmp * 4.0)
    - hb_c_tmp * t30 * 2.0) + eb_c_tmp * t29 * 4.0) - mb_c_tmp * t24 * t30 * 2.0)
    - nb_c_tmp * t24 * t30 * 2.0) + pb_c_tmp * t30) + pd_c_tmp * 4.0) - sb_c_tmp
                       * t30 * 2.0) - d_c_tmp_tmp * t29 * t30 * 2.0) + tb_c_tmp *
                     t24 * t27 * t29 * 4.0) + ud_c_tmp * 4.0) - wd_c_tmp * 4.0))
    + ((((((((((((((((je_c_tmp * t6 * t9 * t11 * t24 * -2.0 - vb_c_tmp * t6 * t9
    * t11 * t24 * 2.0) - vd_c_tmp * 4.0) - x_c_tmp_tmp * t5 * t8 * t9 * t11 *
    t24 * 2.0) + v_c_tmp_tmp * t6 * t9 * t11 * t17 * t22) + xd_c_tmp) +
    x_c_tmp_tmp * t8 * t9 * t11 * t17 * t22) + yd_c_tmp * 4.0) - wb_c_tmp * t30 *
    2.0) - yb_c_tmp * 4.0) + me_c_tmp * t14 * t24) + ac_c_tmp * t14 * t24) -
           t_c_tmp * t12 * t24 * t28 * 2.0) - u_c_tmp * t12 * t24 * t28 * 2.0) +
         ee_c_tmp * t30) - bc_c_tmp * t30) + dc_c_tmp * t27 * t29 * 8.0)) +
    ((((((((((((((((d30 * t3 * t7 * t11 * t12 * t24 * t29 * t30 + ec_c_tmp * t30)
    + w_c_tmp * t30) - gc_c_tmp * t30 * 4.0) + jc_c_tmp * t22 * 4.0) -
    n_c_tmp_tmp * t8 * t21 * 8.0) + kc_c_tmp * t21 * 8.0) + ob_c_tmp * t8 * t21 *
    4.0) - kb_c_tmp * t6 * t21 * 4.0) + hc_c_tmp * t3 * t9 * t21 * 4.0) +
           lc_c_tmp * t8 * t22 * 4.0) + ub_c_tmp * t21 * t23 * 8.0) - ye_c_tmp *
         t8 * t22 * t23 * 4.0) + nc_c_tmp * t22 * t23 * 4.0) - m_c_tmp * t3 * t5
       * t21 * t23 * 4.0) + oc_c_tmp * t8 * t22 * t23 * 4.0) - pc_c_tmp * t22 *
     t23 * 4.0)) + ((((((((((((((qc_c_tmp * t8 * t21 * t23 * 8.0 + wf_c_tmp * t8
    * t21 * 8.0) - rb_c_tmp * t6 * t21 * 8.0) - oe_c_tmp * t22 * 4.0) - jc_c_tmp
    * t7 * t21 * 4.0) + qe_c_tmp * t8 * t21 * 8.0) - jb_c_tmp * t21 * 8.0) -
    i_c_tmp_tmp * t8 * t21 * 4.0) + pe_c_tmp * t6 * t21 * 4.0) - ob_c_tmp * t5 *
    t8 * t21 * 4.0) + kb_c_tmp * t5 * t6 * t21 * 4.0) + n_c_tmp_tmp * t7 * t8 *
                       t22 * 4.0) - kc_c_tmp * t7 * t22 * 4.0) - re_c_tmp * t8 *
                     t22 * 4.0) - lc_c_tmp * t7 * t8 * t21 * 4.0)) +
    ((((((((((((((se_c_tmp * t21 * t23 * -8.0 - te_c_tmp * t21 * t23 * 8.0) -
    ub_c_tmp * t7 * t22 * t23 * 4.0) + ue_c_tmp * t8 * t22 * t23 * 4.0) -
    mc_c_tmp * t22 * t23 * 4.0) + gg_c_tmp * t21 * t23 * 4.0) - hg_c_tmp * t21 *
             t23 * 4.0) - cb_c_tmp_tmp * t6 * t8 * t21 * t23 * 8.0) - ve_c_tmp *
           t8 * t22 * t23 * 4.0) + ic_c_tmp * t22 * t23 * 4.0) - oc_c_tmp * t7 *
         t8 * t21 * t23 * 4.0) + pc_c_tmp * t7 * t21 * t23 * 4.0) - we_c_tmp *
       t9 * t22 * t23 * 4.0) - lb_c_tmp * t8 * t21 * t23 * 8.0) + df_c_tmp * t8 *
     t9 * t21 * t23 * 8.0)) + ((((((((((((((xe_c_tmp * t9 * t21 * t23 * -8.0 -
    qc_c_tmp * t7 * t8 * t22 * t23 * 4.0) - ef_c_tmp * t8 * t9 * t21 * t23 * 4.0)
    + b_c_tmp_tmp_tmp * t4 * t6 * t9 * t21 * t23 * 4.0) - af_c_tmp * t8 * t9 *
    t22 * t23 * 4.0) + yf_c_tmp * t4 * t9 * t15 * t21 * 4.0) + ag_c_tmp * t6 *
    t8 * t9 * t15 * t21 * 4.0) - rc_c_tmp * t12 * t21 * t23 * 8.0) - sf_c_tmp *
    t21 * t23 * 2.0) - rc_c_tmp * t14 * t21 * t23 * 2.0) - sc_c_tmp * t23 * 2.0)
    - y_c_tmp * t7 * t12 * t22 * t23 * 4.0) + tc_c_tmp * t13 * t20 * t23 * 4.0)
    - tc_c_tmp * t12 * t22 * t23 * 6.0) - uc_c_tmp * t12 * t21 * t23 * 8.0)) +
                       (((((((((((((((tf_c_tmp * t21 * t23 * -2.0 - uc_c_tmp *
    t14 * t21 * t23 * 2.0) - vc_c_tmp * t23 * 2.0) - wc_c_tmp * t7 * t8 * t12 *
    t22 * t23 * 4.0) + of_c_tmp * t17 * t20 * t23) + yc_c_tmp * t13 * t20 * t23 *
    4.0) + y_c_tmp * t14 * t17 * t20 * t23) - yc_c_tmp * t12 * t22 * t23 * 6.0)
    + ab_c_tmp * t14 * t17 * t20 * t23) + ab_c_tmp * t15 * t17 * t20 * t23) +
    qf_c_tmp * t17 * t20 * t23) + ad_c_tmp * t14 * t17 * t20 * t23) + xc_c_tmp *
    t14 * t17 * t20 * t23) + bd_c_tmp * t17 * t20 * t23) - gf_c_tmp * t9 * t11 *
    t24 * 2.0) - bf_c_tmp * t9 * t11 * t24 * 2.0)) + (((((((((((((((u_c_tmp_tmp *
    t6 * t8 * t9 * t11 * t24 * -2.0 - t_c_tmp_tmp * t6 * t8 * t9 * t11 * t24 *
    2.0) + gd_c_tmp * t17 * t22) - he_c_tmp * 2.0) + cc_c_tmp * t9 * t11 * t17 *
    t22) + d16 * t6 * t8 * t9 * t11 * t17 * t22) + dd_c_tmp * t29 * 8.0) -
    ed_c_tmp * t27 * 8.0) - fd_c_tmp * t29 * 8.0) - gd_c_tmp * t24 * t30 * 2.0)
    + hd_c_tmp * t27 * 8.0) + ld_c_tmp * t17 * t22 * t30) + id_c_tmp * t17 * t22
    * t30) + gb_c_tmp * t17 * t22 * t30) + kd_c_tmp * t27) + jd_c_tmp * t17 *
    t22 * t30)) + (((((((((((((((kd_c_tmp * t29 - ld_c_tmp * t24 * t29 * t30 *
    4.0) + hb_c_tmp * t27 * t30 * 4.0) - md_c_tmp * t29 * 8.0) + nd_c_tmp * t30 *
    4.0) - od_c_tmp * t30 * 4.0) - ab_c_tmp_tmp * t24 * t27 * t29 * 4.0) +
    pd_c_tmp * t30 * 4.0) + qd_c_tmp * t5 * t6 * t8 * t11 * t24 * 8.0) -
    bb_c_tmp_tmp * t5 * t8 * t9 * t11 * t24 * 2.0) + bb_c_tmp_tmp * t8 * t9 *
                        t11 * t17 * t22) + rd_c_tmp * t8 * t9 * t11 * t17 * t22)
                      + td_c_tmp * t5 * t6 * t9 * t11 * t17 * t22) - ud_c_tmp *
                     t29 * 8.0) + vd_c_tmp * t27 * 8.0) - wd_c_tmp * t30 * 4.0))
                    + (((((((((((((((xd_c_tmp * t30 + yd_c_tmp * t30 * 4.0) -
    yb_c_tmp * t30 * 4.0) - if_c_tmp * t9 * t11 * t12 * t24 * 2.0) - le_c_tmp *
    4.0) - ae_c_tmp * t6 * t8 * t9 * t11 * t12 * t24 * 2.0) + be_c_tmp * t9 *
    t11 * t12 * t17 * t22) + f_c_tmp_tmp * t6 * t8 * t9 * t11 * t12 * t17 * t22)
    + de_c_tmp * t27) + de_c_tmp * t29) - ce_c_tmp * t27 * t29 * 4.0) + ne_c_tmp)
    + rc_c_tmp * t7 * t12 * t22 * t23 * 4.0) + w_c_tmp_tmp * t7 * t8 * t12 * t22
    * t23 * 4.0) + fe_c_tmp * t17 * t20 * t23 * t28) + ig_c_tmp * t17 * t20 *
                       t23 * t28)) + ((((((((((((((hf_c_tmp * 4.0 - fc_c_tmp *
    2.0) + cf_c_tmp * t6 * t8 * t9 * t11 * t24 * 4.0) + ge_c_tmp * t17 * t22) -
    ge_c_tmp * t24 * t29 * 4.0) + he_c_tmp * t27 * 4.0) + ie_c_tmp * t17 * t22 *
    t30) + je_c_tmp * t5 * t6 * t9 * t11 * t17 * t22) + ke_c_tmp * t6 * t7 * t8 *
    t11 * t12 * t24 * 8.0) - le_c_tmp * t30 * 4.0) + me_c_tmp * t12 * t17 * t22 *
    t28) + ac_c_tmp * t12 * t17 * t22 * t28) + ne_c_tmp * t30) - wf_c_tmp * t5 *
    t8 * t21 * 8.0) + rb_c_tmp * t5 * t6 * t21 * 8.0)) + (((((((((((((oe_c_tmp *
    t7 * t21 * 4.0 + i_c_tmp_tmp * t5 * t8 * t21 * 4.0) - pe_c_tmp * t5 * t6 *
    t21 * 4.0) - qe_c_tmp * t7 * t8 * t22 * 4.0) + jb_c_tmp * t7 * t22 * 4.0) +
    re_c_tmp * t7 * t8 * t21 * 4.0) + se_c_tmp * t5 * t21 * t23 * 8.0) +
    te_c_tmp * t7 * t22 * t23 * 4.0) - kg_c_tmp * t21 * t23 * 4.0) + lg_c_tmp *
    t21 * t23 * 4.0) + cb_c_tmp_tmp * t5 * t6 * t8 * t21 * t23 * 8.0) - bg_c_tmp
    * t8 * t9 * t21 * t23 * 8.0) + ff_c_tmp * t4 * t6 * t9 * t21 * t23 * 8.0) +
    ve_c_tmp * t7 * t8 * t21 * t23 * 4.0)) + (((((((((((((ic_c_tmp * t7 * t21 *
    t23 * -4.0 + we_c_tmp * t7 * t9 * t21 * t23 * 4.0) + cg_c_tmp * t8 * t9 *
    t21 * t23 * 4.0) - dg_c_tmp * t4 * t6 * t9 * t21 * t23 * 4.0) + lb_c_tmp *
    t7 * t8 * t22 * t23 * 4.0) - df_c_tmp * t7 * t8 * t9 * t22 * t23 * 4.0) +
    xe_c_tmp * t7 * t9 * t22 * t23 * 4.0) + af_c_tmp * t7 * t8 * t9 * t21 * t23 *
    4.0) - bf_c_tmp * t6 * t7 * t8 * t11 * t24 * 8.0) - cf_c_tmp * t5 * t6 * t8 *
    t9 * t11 * t24 * 4.0) + gf_c_tmp * t5 * t9 * t11 * t17 * t22) + u_c_tmp_tmp *
    t5 * t6 * t8 * t9 * t11 * t17 * t22) + cd_c_tmp * t17 * t22) + hf_c_tmp *
    t30 * 4.0)) + ((((fc_c_tmp * t29 * 4.0 - cd_c_tmp * t24 * t27 * 4.0) -
                     if_c_tmp * t6 * t7 * t8 * t11 * t12 * t24 * 4.0) + jf_c_tmp)
                   + jf_c_tmp * t30)) / (t24 * 4.0);
  c_tmp_tmp = in3[1] * in2[0];
  f_c_tmp = c_tmp_tmp * in2[3];
  g_c_tmp = in3[1] * in1[3];
  h_c_tmp = g_c_tmp * t12 * t21;
  i_c_tmp = g_c_tmp * t13 * t21;
  j_c_tmp = in3[1] * in2[2] * in2[3];
  k_c_tmp = in3[0] * t2;
  b_c_tmp_tmp = k_c_tmp * t8;
  l_c_tmp = b_c_tmp_tmp * t13;
  d_c_tmp_tmp = in3[0] * in2[0] * in2[1];
  m_c_tmp = d_c_tmp_tmp * in1[1];
  n_c_tmp = f_c_tmp * t3;
  e_c_tmp_tmp = g_c_tmp * t2;
  o_c_tmp = e_c_tmp_tmp * t5 * t6;
  p_c_tmp = e_c_tmp_tmp * t6;
  q_c_tmp = g_c_tmp * t4;
  f_c_tmp_tmp = q_c_tmp * t8;
  h_c_tmp_tmp = f_c_tmp_tmp * t12;
  s_c_tmp = h_c_tmp_tmp * t21;
  t_c_tmp = in3[0] * t3 * t4 * t6;
  u_c_tmp = m_c_tmp * t2;
  v_c_tmp = in3[1] * in2[1] * in2[3];
  w_c_tmp = c_tmp_tmp * in2[1] * in1[3];
  ab_c_tmp = f_c_tmp * in1[3];
  bb_c_tmp = w_c_tmp * t5 * t7;
  cb_c_tmp = bb_c_tmp * t21;
  db_c_tmp = w_c_tmp * t7;
  eb_c_tmp = db_c_tmp * t17 * t20;
  db_c_tmp *= t21;
  fb_c_tmp = in3[0] * in1[1];
  c_tmp_tmp = o_c_tmp * t12;
  gb_c_tmp = c_tmp_tmp * t21;
  q_c_tmp = q_c_tmp * t5 * t8;
  hb_c_tmp = q_c_tmp * t13 * t21;
  jb_c_tmp = p_c_tmp * t12;
  kb_c_tmp = fb_c_tmp * t4 * t6;
  lb_c_tmp = jb_c_tmp * t21 * t28;
  mb_c_tmp = v_c_tmp * in1[3];
  nb_c_tmp = cb_c_tmp * t27;
  o_c_tmp *= t13;
  q_c_tmp *= t12;
  ob_c_tmp = q_c_tmp * t21;
  pb_c_tmp = w_c_tmp * t2;
  rb_c_tmp = pb_c_tmp * t4;
  c[2] = in3[1] * in4[1] * t35 * t36 * (t5 - 1.0) *
    (((((((((((((((((((((((((j_c_tmp * t21 * -2.0 - n_c_tmp * t21 * 2.0) -
    f_c_tmp * t5 * t21 * 2.0) + f_c_tmp * t17 * t20) - h_c_tmp * t31 / 2.0) +
    i_c_tmp * t31 / 2.0) + h_c_tmp * t33 / 2.0) - i_c_tmp * t33 / 2.0) + l_c_tmp
                      * t23 * 4.0) - in3[0] * t4 * t6 * t13 * t23 * 4.0) -
                    db_c_tmp * 2.0) - ab_c_tmp * t9 * t21 * 2.0) + j_c_tmp * t5 *
                  t17 * t20) - k_c_tmp * t3 * t8 * t13 * t23 * 4.0) + t_c_tmp *
                t13 * t23 * 4.0) - l_c_tmp * t20 * t23 * 2.0) - u_c_tmp * t4 *
              t23 * 4.0) - m_c_tmp * t6 * t8 * t23 * 4.0) + j_c_tmp * in1[3] *
            t9 * t17 * t20) - v_c_tmp * t4 * t6 * t9 * t21 * 2.0) + n_c_tmp * t5
          * t17 * t20) + gb_c_tmp) - o_c_tmp * t21) + (((((((((((((((((((-in3[1]
    * in1[3] * t4 * t5 * t8 * t12 * t21 + hb_c_tmp) + kb_c_tmp * t13 * t17 * t23)
    + lb_c_tmp) - p_c_tmp * t13 * t21 * t29 * 2.0) - s_c_tmp * t27 * 2.0) -
    s_c_tmp * t28) - t_c_tmp * t12 * t20 * t23 * 2.0) - b_c_tmp_tmp * t12 * t20 *
    t23 * t28 * 2.0) + u_c_tmp * t3 * t4 * t23 * 4.0) + m_c_tmp * t3 * t6 * t8 *
    t23 * 4.0) - mb_c_tmp * t2 * t5 * t8 * t21 * 2.0) + bb_c_tmp * t17 * t20) +
              ab_c_tmp * t3 * t9 * t17 * t20) - nb_c_tmp * 2.0) - cb_c_tmp * t29
            * 2.0) + eb_c_tmp * t27) + eb_c_tmp * t29) - db_c_tmp * t27 * t29 *
         4.0) - f_c_tmp * t2 * t4 * t7 * t9 * t21 * 2.0)) +
      ((((((((((((((((((f_c_tmp * t6 * t7 * t8 * t9 * t21 * -2.0 + d_c_tmp_tmp *
    t2 * t4 * t7 * t20 * t23 * 4.0) + v_c_tmp * t2 * t8 * t9 * t17 * t20) +
                      d_c_tmp_tmp * t6 * t7 * t8 * t20 * t23 * 4.0) + fb_c_tmp *
                     t2 * t3 * t8 * t12 * t17 * t23) - gb_c_tmp * t28) -
                   gb_c_tmp * t29 * 2.0) + ob_c_tmp * t28) - hb_c_tmp * t27 *
                 2.0) + jb_c_tmp * t17 * t20 * t29) + kb_c_tmp * t12 * t17 * t23
               * t28) + f_c_tmp_tmp * t13 * t17 * t20 * t27) - lb_c_tmp * t29 *
             2.0) + mb_c_tmp * t4 * t5 * t6 * t17 * t20) + nb_c_tmp * t29 * 4.0)
          + e_c_tmp_tmp * t3 * t7 * t8 * t9 * t12 * t21) - g_c_tmp * t3 * t4 *
         t6 * t7 * t9 * t12 * t21) + o_c_tmp * t17 * t20 * t29) + q_c_tmp * t17 *
       t20 * t27)) + ((((((((ob_c_tmp * t27 * t28 * -2.0 + h_c_tmp_tmp * t17 *
             t20 * t27 * t28) - rb_c_tmp * t6 * t7 * t8 * t21 * 4.0) + pb_c_tmp *
           t3 * t4 * t9 * t17 * t20) + ab_c_tmp * t2 * t4 * t5 * t7 * t17 * t20)
         + w_c_tmp * t3 * t6 * t8 * t9 * t17 * t20) + ab_c_tmp * t5 * t6 * t7 *
                        t8 * t17 * t20) + c_tmp_tmp * t17 * t20 * t28 * t29) +
                      rb_c_tmp * t5 * t6 * t7 * t8 * t21 * 4.0)) * -0.25;
  f_c_tmp = in4[1] * t11;
  g_c_tmp = f_c_tmp * t15 * t21;
  c_tmp_tmp = kf_c_tmp * t11;
  h_c_tmp = c_tmp_tmp * t15 * t21;
  i_c_tmp = in4[1] * t9 * t11;
  b_c_tmp_tmp = f_c_tmp * t12;
  j_c_tmp = b_c_tmp_tmp * t21 * t23;
  k_c_tmp = f_c_tmp * t13 * t21 * t23;
  d_c_tmp_tmp = f_c_tmp * t14 * t21;
  f_c_tmp = d_c_tmp_tmp * t23;
  l_c_tmp = d15 * t11;
  m_c_tmp = c_tmp_tmp * t13;
  n_c_tmp = m_c_tmp * t21 * t23;
  e_c_tmp_tmp = i_c_tmp * t12;
  o_c_tmp = e_c_tmp_tmp * t21 * t25;
  f_c_tmp_tmp = i_c_tmp * t13;
  p_c_tmp = f_c_tmp_tmp * t21 * t25;
  q_c_tmp = j_c_tmp * t27;
  s_c_tmp = k_c_tmp * t27;
  t_c_tmp = j_c_tmp * t28;
  u_c_tmp = j_c_tmp * t29;
  v_c_tmp = k_c_tmp * t29;
  w_c_tmp = d15 * t3;
  ab_c_tmp = l_c_tmp * t17 * t20 * t23;
  h_c_tmp_tmp = kf_c_tmp * t9 * t11;
  i_c_tmp_tmp = h_c_tmp_tmp * t12;
  bb_c_tmp = i_c_tmp_tmp * t21 * t25;
  cb_c_tmp = h_c_tmp_tmp * t13 * t21 * t25;
  h_c_tmp_tmp = c_tmp_tmp * t12;
  db_c_tmp = h_c_tmp_tmp * t17 * t20 * t23;
  eb_c_tmp = d7 * t11 * t12 * t17 * t20 * t23;
  h_c_tmp_tmp = h_c_tmp_tmp * t21 * t23;
  fb_c_tmp = h_c_tmp_tmp * t27;
  gb_c_tmp = n_c_tmp * t27;
  hb_c_tmp = o_c_tmp * t27;
  jb_c_tmp = o_c_tmp * t28;
  kb_c_tmp = q_c_tmp * t28;
  lb_c_tmp = t_c_tmp * t29;
  mb_c_tmp = s_c_tmp * t29;
  nb_c_tmp = r_c_tmp * t5;
  j_c_tmp_tmp = p_c_tmp_tmp * t9;
  ob_c_tmp = j_c_tmp_tmp * t13;
  pb_c_tmp = xc_c_tmp * t13;
  k_c_tmp_tmp = r_c_tmp_tmp * t9;
  rb_c_tmp = k_c_tmp_tmp * t13;
  sb_c_tmp = d4 * t2;
  tb_c_tmp = w_c_tmp * t5;
  ub_c_tmp = sb_c_tmp * t8;
  l_c_tmp_tmp = c_tmp_tmp_tmp * t4;
  vb_c_tmp = l_c_tmp_tmp * t6;
  wb_c_tmp = d7 * t5;
  xb_c_tmp = bb_c_tmp * t27;
  yb_c_tmp = bb_c_tmp * t28;
  ac_c_tmp = cb_c_tmp * t27;
  bc_c_tmp = fb_c_tmp * t28;
  cc_c_tmp = b_c_tmp_tmp * t17 * t20 * t23 * t27 * t29;
  dc_c_tmp = kb_c_tmp * t29;
  ib_c_tmp *= t4;
  ec_c_tmp = m_c_tmp_tmp * t8;
  fc_c_tmp = ib_c_tmp * t6;
  gc_c_tmp = c_c_tmp_tmp * t4;
  hc_c_tmp = r_c_tmp * t6 * t8;
  b_c_tmp_tmp_tmp = d4 * t4;
  b_c_tmp_tmp = b_c_tmp_tmp_tmp * t5;
  ic_c_tmp = b_c_tmp_tmp * t6;
  c_c_tmp_tmp = c_tmp_tmp_tmp * t2;
  jc_c_tmp = c_c_tmp_tmp * t5 * t8;
  n_c_tmp_tmp = in2[0] * in2[1] * in4[1] * t3 * t4;
  kc_c_tmp = n_c_tmp_tmp * t6;
  lc_c_tmp = sb_c_tmp * t5;
  mc_c_tmp = b_c_tmp_tmp_tmp * t7 * t8;
  o_c_tmp_tmp = sb_c_tmp * t6 * t7;
  nc_c_tmp = o_c_tmp_tmp * t11 * t21 * t23;
  oc_c_tmp = kc_c_tmp * t11;
  pc_c_tmp = mc_c_tmp * t11;
  p_c_tmp_tmp = d15 * t2 * t4;
  qc_c_tmp = p_c_tmp_tmp * t7;
  rc_c_tmp = d15 * t6 * t7 * t8;
  sc_c_tmp = b_c_tmp_tmp_tmp * t6;
  tc_c_tmp = c_c_tmp_tmp * t8;
  uc_c_tmp = d33 * t3 * t4;
  vc_c_tmp = uc_c_tmp * t7;
  c_c_tmp_tmp = xf_c_tmp * t6 * t8;
  wc_c_tmp = c_c_tmp_tmp * t11;
  xc_c_tmp = xf_c_tmp * t7;
  yc_c_tmp = x_c_tmp * t6 * t8;
  bd_c_tmp = d7 * t6 * t7 * t8;
  cd_c_tmp = fg_c_tmp * t7 * t8;
  dd_c_tmp = yc_c_tmp * t11;
  ed_c_tmp = vc_c_tmp * t11 * t12 * t21 * t25;
  fd_c_tmp = dd_c_tmp * t12;
  dd_c_tmp = dd_c_tmp * t13 * t21 * t23;
  gd_c_tmp = bd_c_tmp * t11 * t12 * t21 * t25;
  x_c_tmp *= t7;
  hd_c_tmp = jg_c_tmp * t7 * t8;
  id_c_tmp = xb_c_tmp * t28;
  jd_c_tmp = sd_c_tmp * t4;
  sb_c_tmp *= t3;
  kd_c_tmp = lc_c_tmp * t6 * t7;
  ld_c_tmp = sb_c_tmp * t8;
  lc_c_tmp *= t8;
  md_c_tmp = l_c_tmp_tmp * t5 * t6;
  nd_c_tmp = kd_c_tmp * t11 * t21 * t23;
  b_c_tmp_tmp = b_c_tmp_tmp * t7 * t8;
  od_c_tmp = b_c_tmp_tmp * t11 * t21 * t23;
  pd_c_tmp = o_c_tmp_tmp * t9 * t11;
  mc_c_tmp = mc_c_tmp * t9 * t11 * t21 * t25;
  qd_c_tmp = ld_c_tmp * t11;
  rd_c_tmp = pc_c_tmp * t17 * t20 * t23;
  sd_c_tmp = nc_c_tmp * t29;
  pc_c_tmp = pc_c_tmp * t21 * t23 * t27;
  yc_c_tmp = yc_c_tmp * t9 * t11;
  td_c_tmp = wc_c_tmp * t12 * t21 * t23;
  ud_c_tmp = fd_c_tmp * t17 * t20 * t23;
  fd_c_tmp = fd_c_tmp * t21 * t23 * t28;
  qb_c_tmp *= t4;
  vd_c_tmp = eg_c_tmp * t8;
  wd_c_tmp = qb_c_tmp * t6;
  xd_c_tmp = p_c_tmp_tmp * t5 * t7;
  yd_c_tmp = d15 * t5 * t6 * t7 * t8;
  kd_c_tmp = kd_c_tmp * t9 * t11 * t21 * t25;
  ae_c_tmp = b_c_tmp_tmp * t9 * t11;
  uc_c_tmp = uc_c_tmp * t5 * t7;
  be_c_tmp = c_c_tmp_tmp * t9 * t11;
  ce_c_tmp = wb_c_tmp * t6 * t7 * t8;
  de_c_tmp = be_c_tmp * t12;
  ee_c_tmp = yc_c_tmp * t12;
  c[3] = 1.0 / rt_powd_snf(in1[3], 5.0) * t35 *
    ((((((((((((((((((((((((((((((((((g_c_tmp * 4.0 - h_c_tmp * 4.0) - j_c_tmp *
    2.0) - k_c_tmp * 2.0) - f_c_tmp) + g_c_tmp * t23) - d8 * t9 * t11 * t15 *
    t21 * 4.0) + h_c_tmp * t23) + o_c_tmp) + p_c_tmp) + i_c_tmp * t14 * t21 *
    t25) - q_c_tmp) + s_c_tmp) - u_c_tmp) + v_c_tmp) - f_c_tmp * t30) -
                       d_c_tmp_tmp * t25 * t34 / 2.0) + nb_c_tmp * t13 * t26 *
                      4.0) - r_c_tmp * t9 * t13 * t25 * 4.0) - lf_c_tmp * t25 *
                    4.0) - e_c_tmp * t7 * t13 * t26 * 4.0) - mf_c_tmp * t25 *
                  4.0) + nf_c_tmp * t25 * 4.0) + (((((((((((((((((((((d14 * in4
    [1] * t3 * t11 * t21 * t23 * -2.0 - d14 * in4[1] * t5 * t11 * t21 * t23 *
    4.0) - in2[0] * in2[2] * in4[1] * t9 * t11 * t21 * t25 * 2.0) + ab_c_tmp) +
    l_c_tmp * t21 * t25 * t34) - wb_c_tmp * t11 * t12 * t21 * t23 * 4.0) - d7 *
    t9 * t11 * t12 * t21 * t25 * 2.0) + eb_c_tmp) + db_c_tmp) + m_c_tmp * t17 *
    t20 * t23) + c_tmp_tmp * t14 * t17 * t20 * t23) - gb_c_tmp * 2.0) - n_c_tmp *
    t29 * 2.0) + hb_c_tmp) + jb_c_tmp) - p_c_tmp * t27) + o_c_tmp * t29) -
    p_c_tmp * t29) + kb_c_tmp) - q_c_tmp * t30) + lb_c_tmp) - mb_c_tmp * 2.0)) +
               (((((((((((((((((((t_c_tmp * t30 * -2.0 + s_c_tmp * t30) -
    u_c_tmp * t30) + v_c_tmp * t30) + of_c_tmp * t25 * 4.0) + pf_c_tmp * t25 *
    4.0) + ob_c_tmp * t26 * 4.0) + qf_c_tmp * t25 * 4.0) + pb_c_tmp * t25 * 4.0)
    + rb_c_tmp * t26 * 4.0) - rf_c_tmp * t20 * t25 * 2.0) + tb_c_tmp * t11 * t21
    * t23 * 4.0) - w_c_tmp * t11 * t21 * t23 * t30 * 2.0) + ab_c_tmp * t30) -
                     xb_c_tmp) - yb_c_tmp * 2.0) + ac_c_tmp) - bb_c_tmp * t29) +
                 cb_c_tmp * t29) + db_c_tmp * t27)) + ((((((((((((((((((eb_c_tmp
    * t30 + db_c_tmp * t28) + db_c_tmp * t29) - bc_c_tmp * 2.0) - fb_c_tmp * t29
    * 4.0) - h_c_tmp_tmp * t28 * t29 * 2.0) + gb_c_tmp * t29 * 4.0) - hb_c_tmp *
    t28) - hb_c_tmp * t29 * 2.0) - jb_c_tmp * t29) + cc_c_tmp) - dc_c_tmp * 2.0)
    + kb_c_tmp * t30) + lb_c_tmp * t30) - mb_c_tmp * t30 * 2.0) + gc_c_tmp * t13
    * t17 * t25) + nb_c_tmp * t7 * t12 * t17 * t26) + hc_c_tmp * t13 * t17 * t25)
    - sf_c_tmp * t25 * 4.0)) + (((((((((((((((y_c_tmp * t9 * t13 * t26 * -4.0 -
    tf_c_tmp * t25 * 4.0) - ad_c_tmp * t9 * t13 * t26 * 4.0) - uf_c_tmp * t20 *
    t25 * 2.0) - pf_c_tmp * t20 * t25 * 2.0) - t19 * t7 * t12 * t20 * t26 * 2.0)
    - ob_c_tmp * t20 * t26 * 2.0) - vf_c_tmp * t20 * t25 * 2.0) - pb_c_tmp * t20
    * t25 * 2.0) - rb_c_tmp * t20 * t26 * 2.0) - qd_c_tmp * t21 * t25 * 2.0) -
    nc_c_tmp * 2.0) - ic_c_tmp * t11 * t21 * t25 * 2.0) - ub_c_tmp * t9 * t11 *
    t21 * t23 * 2.0) - jc_c_tmp * t11 * t21 * t25 * 2.0) - tb_c_tmp * t9 * t11 *
              t21 * t25 * 2.0)) + (((((((((((((((((vb_c_tmp * t9 * t11 * t21 *
    t23 * -2.0 + w_c_tmp * t9 * t11 * t17 * t20 * t25) - ub_c_tmp * t11 * t21 *
    t25 * t30 * 2.0) - vb_c_tmp * t11 * t21 * t25 * t30 * 2.0) + ed_c_tmp) -
    xc_c_tmp * t11 * t12 * t21 * t25 * 2.0) - dd_c_tmp * 2.0) + gd_c_tmp) -
    cd_c_tmp * t11 * t12 * t21 * t25 * 2.0) + wb_c_tmp * t9 * t11 * t12 * t17 *
    t20 * t25) + id_c_tmp) + yb_c_tmp * t29) - ac_c_tmp * t29 * 2.0) +
    f_c_tmp_tmp * t17 * t20 * t25 * t27 * t29) + bc_c_tmp * t29 * 4.0) +
    cc_c_tmp * t30) - dc_c_tmp * t30 * 2.0) - ec_c_tmp * t25 * 4.0)) +
           (((((((((((((((fc_c_tmp * t25 * 4.0 + vd_c_tmp * t25 * 4.0) -
    wd_c_tmp * t25 * 4.0) + m_c_tmp_tmp * t5 * t8 * t25 * 4.0) - ib_c_tmp * t5 *
                       t6 * t25 * 4.0) + ec_c_tmp * t9 * t26 * 4.0) - fc_c_tmp *
                     t9 * t26 * 4.0) + gg_c_tmp * t20 * t25 * 4.0) - hg_c_tmp *
                   t20 * t25 * 4.0) + g_c_tmp_tmp * t7 * t9 * t12 * t17 * t25) +
                 gc_c_tmp * t12 * t17 * t25 * t28) + hc_c_tmp * t12 * t17 * t25 *
                t28) - fe_c_tmp * t20 * t25 * t28 * 2.0) - j_c_tmp_tmp * t12 *
              t20 * t26 * t28 * 2.0) - ig_c_tmp * t20 * t25 * t28 * 2.0) -
            k_c_tmp_tmp * t12 * t20 * t26 * t28 * 2.0)) +
          (((((((((((((((nd_c_tmp * 4.0 - kc_c_tmp * t9 * t11 * t21 * t23 * 2.0)
    - qc_c_tmp * t9 * t11 * t21 * t23 * 2.0) - ic_c_tmp * t9 * t11 * t21 * t23 *
                       2.0) - od_c_tmp * 4.0) - jc_c_tmp * t9 * t11 * t21 * t23 *
                     2.0) - mc_c_tmp * 2.0) + oc_c_tmp * t17 * t20 * t25) -
                  rc_c_tmp * t9 * t11 * t21 * t23 * 2.0) + lc_c_tmp * t11 * t17 *
                 t20 * t25) + sc_c_tmp * t9 * t11 * t17 * t20 * t23) + rd_c_tmp)
              + md_c_tmp * t11 * t17 * t20 * t25) + tc_c_tmp * t9 * t11 * t17 *
             t20 * t23) + sd_c_tmp * 4.0) - nc_c_tmp * t30 * 2.0)) +
         (((((((((((((((oc_c_tmp * t21 * t25 * t30 * -2.0 - pc_c_tmp * 4.0) -
                       qc_c_tmp * t11 * t21 * t25 * t30 * 2.0) - rc_c_tmp * t11 *
                      t21 * t25 * t30 * 2.0) + sc_c_tmp * t11 * t17 * t20 * t25 *
                     t30) + tc_c_tmp * t11 * t17 * t20 * t25 * t30) + uc_c_tmp *
                   t11 * t12 * t21 * t25) - vc_c_tmp * t9 * t11 * t12 * t21 *
                  t23 * 2.0) - td_c_tmp * 4.0) + wc_c_tmp * t13 * t21 * t23 *
                4.0) - xc_c_tmp * t9 * t11 * t12 * t21 * t23 * 2.0) - ee_c_tmp *
              t21 * t25 * 2.0) + ce_c_tmp * t11 * t12 * t21 * t25) - bd_c_tmp *
            t9 * t11 * t12 * t21 * t23 * 2.0) - cd_c_tmp * t9 * t11 * t12 * t21 *
           t23 * 2.0) + ud_c_tmp)) + (((((((((((((((x_c_tmp * t9 * t11 * t12 *
    t17 * t20 * t23 + hd_c_tmp * t9 * t11 * t12 * t17 * t20 * t23) - ed_c_tmp *
    t30 * 2.0) - fd_c_tmp * 2.0) - dd_c_tmp * t30 * 2.0) - gd_c_tmp * t30 * 2.0)
    + x_c_tmp * t11 * t12 * t17 * t20 * t25 * t30) + hd_c_tmp * t11 * t12 * t17 *
    t20 * t25 * t30) + i_c_tmp_tmp * t17 * t20 * t25 * t27 * t29) - id_c_tmp *
    t29 * 2.0) + e_c_tmp_tmp * t17 * t20 * t25 * t27 * t28 * t29) + jd_c_tmp *
             t5 * t12 * t17 * t25) + jd_c_tmp * t9 * t12 * t17 * t26) +
           g_c_tmp_tmp * t5 * t6 * t8 * t12 * t17 * t25) + g_c_tmp_tmp * t6 * t8
          * t9 * t12 * t17 * t26) - sb_c_tmp * t5 * t8 * t9 * t11 * t21 * t23 *
         2.0)) + ((((((((((((((kd_c_tmp * -2.0 + xd_c_tmp * t11 * t17 * t20 *
    t25) + ld_c_tmp * t9 * t11 * t17 * t20 * t23) + lc_c_tmp * t9 * t11 * t17 *
    t20 * t23) + pd_c_tmp * t17 * t20 * t25) + md_c_tmp * t9 * t11 * t17 * t20 *
    t23) + yd_c_tmp * t11 * t17 * t20 * t25) - nd_c_tmp * t29 * 8.0) + od_c_tmp *
                        t27 * 8.0) - pd_c_tmp * t21 * t25 * t29 * 4.0) +
                      mc_c_tmp * t27 * 4.0) + qd_c_tmp * t17 * t20 * t25 * t30)
                    + rd_c_tmp * t30) + sd_c_tmp * t30 * 4.0) - pc_c_tmp * t30 *
                  4.0)) + ((((((((((((((be_c_tmp * t13 * t21 * t25 * -2.0 +
    yc_c_tmp * t13 * t17 * t20 * t25) + td_c_tmp * t28 * 4.0) + ud_c_tmp * t30)
    - fd_c_tmp * t30 * 2.0) - eg_c_tmp * t5 * t8 * t25 * 4.0) + qb_c_tmp * t5 *
    t6 * t25 * 4.0) - vd_c_tmp * t9 * t26 * 4.0) + wd_c_tmp * t9 * t26 * 4.0) -
            kg_c_tmp * t20 * t25 * 4.0) + lg_c_tmp * t20 * t25 * 4.0) - gg_c_tmp
          * t9 * t20 * t26 * 4.0) + hg_c_tmp * t9 * t20 * t26 * 4.0) +
        n_c_tmp_tmp * t5 * t6 * t9 * t11 * t17 * t20 * t23) + xd_c_tmp * t9 *
       t11 * t17 * t20 * t23)) + ((((((((ae_c_tmp * t17 * t20 * t25 + yd_c_tmp *
             t9 * t11 * t17 * t20 * t23) + kd_c_tmp * t29 * 4.0) - ae_c_tmp *
           t21 * t25 * t27 * 4.0) + uc_c_tmp * t9 * t11 * t12 * t17 * t20 * t23)
         + de_c_tmp * t17 * t20 * t25) + ce_c_tmp * t9 * t11 * t12 * t17 * t20 *
        t23) - de_c_tmp * t21 * t25 * t28 * 2.0) + ee_c_tmp * t17 * t20 * t25 *
      t28)) * -0.25;
}
int main(){
  double q[4] = {0.3, 0.4, 0.3, 0.5};
  double dq[4] = {0.5, 0.3, 0.2, 0.1};
  double L[2] = {0.125, 0.125};
  double m[2] = {0.16, 0.082};
  double c[4];
  c_fun(q,dq,L,m,c);
  Eigen::VectorXd Cdq = Eigen::VectorXd::Zero(4);
  for (int i = 0; i < 4; i++){
      Cdq(i) = c[i];
    }
  std::cout << Cdq;
}
//
// File trailer for c_fun.cpp
//
// [EOF]
//
