#include "3d-soft-trunk/MPC_ts.h"

// This controller it's made to work in task-space directly


MatrixXd Rotx_1(double theta){

    MatrixXd rot = MatrixXd::Zero(3,3); 
    rot(0,0) = 1;
    rot(1,1) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(1,2) = -sin(theta);
    rot(2,1) = sin(theta); 

    //std::cout << "X rotation : " << rot << std::endl; 
     

    return rot; 
}

MatrixXd Roty_1(double theta){

    MatrixXd rot = MatrixXd::Zero(3,3); 
    rot(1,1) = 1;
    rot(0,0) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(0,2) = sin(theta);
    rot(2,0) = -sin(theta); 

    //std::cout << "Y rotation : " << rot << std::endl; 


    return rot; 
}

MatrixXd axis_angle_1(double thetax, double thetay){

    MatrixXd rot = MatrixXd::Zero(3,3); 
    double theta = sqrt(pow(thetax,2)+pow(thetay,2));
    if (theta == 0){
        return rot; 
    }
    // if (thetax<0){   // just check this, since cos(phi) should always be positive
    //     //theta = -theta; 
    
    //     std::cout << "Negative rotation" << std::endl;  
    // }

    double ux = - thetay / theta; 
    double uy = - thetax / theta; 
    double t2 = theta/2; 

    rot(0,0) = cos(t2) + pow(ux,2)*(1-cos(t2)); 
    rot(0,1) = ux*uy*(1-cos(t2)); 
    rot(0,2) = uy*sin(t2); 
    rot(1,0) = rot(0,1); 
    rot(1,1) = cos(t2) + pow(uy,2)*(1-cos(t2)); 
    rot(1,2) = -ux*sin(t2);
    rot(2,0) = - rot(0,2); 
    rot(2,1) = - rot(1,2);
    rot(2,2) = cos(t2);

    //std::cout << "Axis-angle matrix :" << rot << std::endl; 

    return rot; 

}

MatrixXd ee_position_1(VectorXd thetax, VectorXd thetay, VectorXd length1, VectorXd length2){    // computes position of end effector for each segment

    MatrixXd tot_rot = axis_angle_1(thetax(0), thetay(0));

    MatrixXd len1 = MatrixXd::Zero(3,2);
    MatrixXd len2 = MatrixXd::Zero(3,2);

    VectorXd theta(2); 
    theta << sqrt(pow(thetax(0),2)+pow(thetay(0),2)), sqrt(pow(thetax(1),2)+pow(thetay(1),2)); 


    for (int i = 0; i<2 ; i++){


        if (theta(i) != 0){
        len1(2,i) = 2*length1(i)*sin(theta(i)/2)/theta(i);
        }
        else {
            len1(2,i) = length1(i);
        }

        len2(2,i) = length2(i); 

    }

    MatrixXd inter(3,1);
    
    inter = tot_rot * len1.col(0); 

    //std::cout << "Inter step : " << inter << std::endl; 

    tot_rot *= axis_angle_1(thetax(0), thetay(0));

    inter += tot_rot*len2.col(0); 

    tot_rot *= axis_angle_1(thetax(1), thetay(1)); 

    inter += tot_rot*len1.col(1);

    //tot_rot *= axis_angle(thetax(1), thetay(1)); 

    //inter += tot_rot*len2.col(1); 

     
    return inter;  

    //return tot_rot* len; 

}

MatrixXd ee_position_2(VectorXd thetax, VectorXd thetay, VectorXd length1, VectorXd length2){

    MatrixXd len1 = MatrixXd::Zero(3,2);
    MatrixXd len2 = MatrixXd::Zero(3,2);

    VectorXd theta(2); 
    theta << sqrt(pow(thetax(0),2)+pow(thetay(0),2)), sqrt(pow(thetax(1),2)+pow(thetay(1),2)); 

    // std::cout << "~~~~~~~~~~~~~~~~~~~" << std::endl;
    // std::cout << "theta " << theta(0) << " " << sin(theta(0)/2)/theta(0) << " sin(theta)/theta" << std::endl; 
    // std::cout << "~~~~~~~~~~~~~~~~~~~" << std::endl;
    

    for (int i = 0; i<2 ; i++){


        // if (theta(i) != 0){
        // len1(2,i) = 2*length1(i)*sin(theta(i)/2)/theta(i);
        // }
        // else {
        //     len1(2,i) = length1(i);
        // }

        len1(2,i) = 0.98*length1(i); 
        len2(2,i) = length2(i); 

    }

    MatrixXd totRot = Roty_1(-thetax(0)/2)*Rotx_1(-thetay(0)/2);
    MatrixXd inter(3,1);

    inter = totRot*len1.col(0); 
    totRot *= Roty_1(-thetax(0)/2)*Rotx_1(-thetay(0)/2); 
    inter += totRot*len2.col(0); 
    totRot *= Roty_1(-thetax(1)/2)*Rotx_1(-thetay(1)/2);  
    inter += totRot*len1.col(1);

    return inter; 
    
}






MPC_ts::MPC_ts(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(st_params, sensor_type){
    filename = "MPC_logger";

    // Horizon = 5;
    dt = 1./15;

    // Take model
    ctrl = define_problem();   // define matrices as parameters, so we can update them without re-initalizing the problem
    solved = false; 

    //OptiAdvanced ctrl_deb = ctrl.debug();

    control_thread = std::thread(&MPC_ts::control_loop, this);
}

void MPC_ts::control_loop(){
    srl::Rate r{1./dt};

    while(true){

        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //auto start = std::chrono::steady_clock::now();

        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
            
        stm->updateState(state);
            
        if (!is_initial_ref_received){
            //only control after receiving a reference position
            continue;
        } 

        if (sensor_type != CurvatureCalculator::SensorType::simulator){
            x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        }
        else {
            x = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-1).translation();
        }


        // Debug for kinematics
        
        // std::cout << "=========================================" << std::endl; 
        // std::cout << "End-effector position real : " << x.transpose() << std::endl;  

        // MatrixXd ee_t = MatrixXd::Zero(3,1); 
        // MatrixXd ee_s = MatrixXd::Zero(3,1);

        // VectorXd thetax(2), thetay(2), length1(2), length2(2); 

        // for (int kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

        //     thetax(kk) = state.q(2*kk,0); 
        //     thetay(kk) = state.q(2*kk+1,0);
        //     length1(kk) = -st_params.lengths[2*kk]; 
        //     length2(kk) = -st_params.lengths[2*kk+1]; 

        // }

        // ee_t = ee_position_1(thetax, thetay, length1, length2); 
        // ee_s = ee_position_2(thetax, thetay, length1, length2);

    
        // std::cout << "End-effector position mine (long): " << ee_t.transpose() << std::endl;
        // std::cout << "End-effector position mine (short): " << ee_s.transpose() << std::endl;
        // std::cout << "=========================================" << std::endl;
        


        // sp_A = MatrixXd::Zero(2*st_params.q_size, 2*st_params.q_size);
        // sp_B = MatrixXd::Zero(2*st_params.q_size, 2*st_params.num_segments);  
        // sp_w = MatrixXd::Zero(2*st_params.q_size, 1);

        //MatrixXd sp_A(2*st_params.q_size, 2*st_params.q_size);
        //MatrixXd sp_B(2*st_params.q_size, 2*st_params.num_segments);   // SS matrices

        // need a coversion DM to MatrixXd and viceversa
        
        get_state_space(stm->B, stm->c, stm->g, stm->K, stm->D, stm->A_pseudo, sp_A, sp_B, sp_w, dt);

        //https://github.com/casadi/casadi/issues/2563
        //https://groups.google.com/g/casadi-users/c/npPcKItdLN8
        // DM <--> Eigen

        // sp_A_temp = DM::nan(2*st_params.q_size, 2*st_params.q_size);
        // sp_B_temp = DM::nan(2*st_params.q_size, 2*st_params.num_segments);
        // sp_w_temp = DM::nan(2*st_params.q_size, 1);

        //x_r_temp = DM::nan(3,1);  // conversion placeholders
        // tr_r_temp = DM::nan(3,Horizon+1); 

        // q_0_temp = DM::nan(st_params.q_size, 1);
        // q_dot_0_temp = DM::nan(st_params.q_size, 1);
        // DM q_0_large = DM::nan(st_params.q_size, Horizon+1);    // needed for warm-start
        // DM q_dot_0_large = DM::nan(st_params.q_size, Horizon+1); 
        // DM u_large = DM::nan(2*st_params.num_segments, Horizon); 
        

        std::copy(sp_A.data(), sp_A.data() + sp_A.size(), sp_A_temp.ptr());
        std::copy(sp_B.data(), sp_B.data() + sp_B.size(), sp_B_temp.ptr());
        std::copy(sp_w.data(), sp_w.data() + sp_w.size(), sp_w_temp.ptr());
        //std::copy(x_ref.data(), x_ref.data() + x_ref.size(), x_r_temp.ptr());
        std::copy(traj_ref.data(), traj_ref.data() + traj_ref.size(), tr_r_temp.ptr()); 
        std::copy(state.q.data(), state.q.data() + state.q.size(), q_0_temp.ptr());
        std::copy(state.dq.data(), state.dq.data() + state.dq.size(), q_dot_0_temp.ptr());

        for (int i = 0; i < Horizon+1; i++){
            q_0_large(Slice(),i) = q_0_temp; 
            q_dot_0_large(Slice(),i) = q_dot_0_temp; 
        }


        // solve problem

        if (!solved){
            u_temp = DM::zeros(2*st_params.num_segments,1); 
        }

        for (int i = 0; i < Horizon; i++){
            u_large(Slice(),i) = u_temp; 
        }

        ctrl.set_value(A, sp_A_temp);   // need to define them already as global variables
        ctrl.set_value(B, sp_B_temp);
        ctrl.set_value(w, sp_w_temp); 
        //ctrl.set_value(x_r, x_r_temp);
        ctrl.set_value(tr_r, tr_r_temp);
        ctrl.set_value(q_0, q_0_temp); 
        ctrl.set_value(q_dot_0, q_dot_0_temp); 
        ctrl.set_value(u_prev, u_temp); 

        ctrl.set_initial(q, q_0_large);
        ctrl.set_initial(q_dot, q_dot_0_large);

        OptiSol sol = ctrl.solve(); 
         
        
        if (!solved){
            solved = TRUE; 
        }

        //ctrl.set_initial(sol.value_variables());   // to be investigated, as it is doesn't work

        
        //std::cout << "complete u : " <<sol.value(u) << std::endl; 
        //std::cout << "-----------------" << std::endl;
        //std::cout << stm->A_pseudo << std::endl;
        //std::cout << stm-> A << std::endl;

        //auto u_temp = static_cast<std::vector<double>>(sol.value(u));
        u_temp = sol.value(u)(Slice(),0); 

        //std::cout << mtimes(sol.value(q_dot)(Slice(),Horizon).T(), sol.value(q_dot)(Slice(),Horizon)) << std::endl; 

        //std::cout << "solution :" << u_temp << std::endl;
        
        // p_temp = MatrixXd::Zero(2*st_params.num_segments,1); 

        p_temp = Eigen::VectorXd::Map(DM::densify(u_temp).nonzeros().data(),2*st_params.num_segments,1 ); 

        ///// DEBUG
        /*
        std::cout << "DEBUG" << std::endl; 
        std::cout << stm->A_pseudo.size() << std::endl;   // size 16? 
        std::cout << p_temp.size() << std::endl;   // size 2
        */
        /////

        // tau_ref = stm->A_pseudo* p_temp; 

        // // apply input                     // need to check how to do this, seems really small
        // pxy = stm->A_pseudo.inverse()*tau_ref/10000;
        // p = stm->pseudo2real(pxy);  // possibly add gravity compensation

        p = stm->pseudo2real(p_temp ); //+ gravity_compensate(state)/2

        std::cout << "pressure input : " << p.transpose() << std::endl; 

        if (sensor_type != CurvatureCalculator::SensorType::simulator) {actuate(p);} // actuate(p)
        else {
            assert(simulate(p));
        }

        // auto end = std::chrono::steady_clock::now();
        // std::chrono::duration<double> elapsed = end - start;

        // counter += 1; 
        // total_time += elapsed.count(); 
        // if (counter < 5){
        //     total_time -= elapsed.count(); 
        // }
        // if (elapsed.count() > slow_execution(4)){
        //     for (int i = 0; i<5; i++){
        //         if (elapsed.count() > slow_execution(i)){
        //             for(int k = 4; k>i; k--){
        //                 slow_execution(k) = slow_execution(k-1); 
        //             }
        //             slow_execution(i) = elapsed.count(); 
        //             break; 
        //         }
        //     }
        // }

        // if (counter%1 == 0){
        //     std::cout << "==================" << std::endl; 
        //     std::cout << "Average time : " << total_time / counter << std::endl; 
        //     std::cout << "Slowest execution :" << slow_execution.transpose() << std::endl; 
        //     std::cout << "==================" << std::endl; 
        // }
        
    }
    
}

Opti MPC_ts::define_problem(){
    
    Opti prob = casadi::Opti();

    q = prob.variable(st_params.q_size, Horizon+1);
    q_dot = prob.variable(st_params.q_size, Horizon+1);
    u = prob.variable(2*st_params.num_segments, Horizon);  //pressure

    q_0 = prob.parameter(st_params.q_size,1);
    q_dot_0 = prob.parameter(st_params.q_size,1); 

    // define as parameters the model matrices and set point

    A = prob.parameter(2*st_params.q_size, 2*st_params.q_size);
    B = prob.parameter(2*st_params.q_size, 2*st_params.num_segments);
    w = prob.parameter(2*st_params.q_size, 1); 

    //x_r = prob.parameter(3,1); 
    tr_r = prob.parameter(3, Horizon); 

    u_prev = prob.parameter(2*st_params.num_segments,1); 



    MX J = 0;
    MX A1 = MX::zeros(st_params.q_size, st_params.q_size);
    MX b1 = MX::ones(st_params.q_size,1);
    MX A2 = MX::zeros(st_params.q_size, st_params.q_size);
    MX b2 = MX::ones(st_params.q_size,1);
    MX T1 = MX::zeros(3,1);
    MX T2 = MX::zeros(st_params.q_size,1);
    //MX T3 = MX::ones(st_params.q_size,1)*0.5; 

    MX p_min = MX::ones(2*st_params.num_segments,1)*-500;
    MX p_max = MX::ones(2*st_params.num_segments,1)*500;

    MX Du = MX::ones(2*st_params.num_segments,1)*20;   // 8 for simulation, 20 real 

    MX end_effector = MX::zeros(3,1); 

    T1 = fabs(tr_r(Slice(), Horizon-1));
    //T1 = tr_r(Slice(), Horizon); 
    T2 = MX::zeros(st_params.q_size,1);  //terminal condition with delta formulation

    MX Q = MX::eye(3)*5e2;                              // circular : 5e2     square : 5e2 
    MX Q2 = MX::eye(st_params.q_size)*1e-4;             // 20s      : 1e-4    10s    : 1e-4
    MX R = MX::eye(2*st_params.num_segments)*1e-10;     // 15 Hz    : 1e-10   15Hz   : 2e-10               

    MX thetax = MX::zeros(2,1);
    MX thetay = MX::zeros(2,1);
    MX length1 = MX::zeros(2,1);
    MX length2 = MX::zeros(2,1);

    std::cout << "Variables initialized" << std::endl; 

    int k, kk;

    // use delta-formulation with state reference  <<<<--------------<<<<<<<<<<<<----------<<<<<<<<<<<<----------
    //end_effector = MX::zeros(3,1);

    J += mtimes((u(Slice(),0)-u_prev).T(), mtimes(R, (u(Slice(),0)-u_prev))); 

    for (k = 0; k < Horizon; k++) 
    {
        // J += mtimes((q(Slice(),k)-q_r).T(), mtimes(Q, (q(Slice(),k)-q_r)));   // probaly Slice(0,st_params.q_size,1) has same effect
        J += mtimes((q_dot(Slice(),k)).T(), mtimes(Q2, (q_dot(Slice(),k))));    // keep limit on speeds
        J += mtimes((u(Slice(),k)).T(), mtimes(R, (u(Slice(),k)))); 
        if (k>0){
            J += mtimes((u(Slice(),k)-u(Slice(),k-1)).T(), mtimes(R, (u(Slice(),k)-u(Slice(),k-1)))); 
        

            for (kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

                thetax(kk) = q(2*kk,k); 
                thetay(kk) = q(2*kk+1,k);
                length1(kk) = -st_params.lengths[2*kk]; 
                length2(kk) = -st_params.lengths[2*kk+1];
                
                // end_effector += ee_position(q(2*kk,k), q(2*kk+1,k), -st_params.lengths[2*kk] - st_params.lengths[2*kk+1]); 
            }

            end_effector = ee_position(thetax, thetay, length1, length2); 

            J += mtimes((end_effector-tr_r(Slice(),k-1)).T(), mtimes(Q, (end_effector-tr_r(Slice(),k-1)))); 
        }

        //J += mtimes(u(Slice(),k).T(), mtimes(R, u(Slice(),k)));
    }


    //end_effector = MX::zeros(3,1);    // needed also later for terminal constraint

    for (kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

        thetax(kk) = q(2*kk,Horizon); 
        thetay(kk) = q(2*kk+1,Horizon);
        length1(kk) = -st_params.lengths[2*kk]; 
        length2(kk) = -st_params.lengths[2*kk+1];
        
        // end_effector += ee_position(q(2*kk,Horizon), q(2*kk+1,Horizon), -st_params.lengths[2*kk] - st_params.lengths[2*kk+1]); 
    }

    end_effector = ee_position(thetax, thetay, length1, length2);

    
    J += mtimes((end_effector-tr_r(Slice(),Horizon-1)).T(), mtimes(Q, (end_effector-tr_r(Slice(), Horizon-1))));

    //J += mtimes(q(Slice(),Horizon).T(), mtimes(Q, q(Slice(),Horizon))); 
    //J += mtimes(q_dot(Slice(),Horizon).T(), mtimes(Q, q_dot(Slice(),Horizon)));  // terminal cost
    
    prob.minimize(J);

    std::cout << "Cost function initialized" << std::endl;

    ///////////////// DEBUG STUFF 
    /*
    std::cout << "DEBUG" << std::endl; 
    std::cout << A.size() << std::endl;
    std::cout << A(Slice(0, st_params.q_size), Slice(0, st_params.q_size)).size() << std::endl;
    std::cout << A(Slice(0, st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)).size()<< std::endl;
    std::cout << A(Slice(st_params.q_size,2*st_params.q_size), Slice(0, st_params.q_size)).size() << std::endl;
    std::cout << A(Slice(st_params.q_size,2*st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)).size() << std::endl; 
    std::cout << B(Slice(0, st_params.q_size), Slice()).size() << std::endl;
    std::cout << B(Slice(st_params.q_size, 2*st_params.q_size), Slice()).size() << std::endl; 
    */
    //////////////////

    // Slice function (0,2) select 0-1 only

    prob.subject_to(q(Slice(),0) == q_0);
    prob.subject_to(q_dot(Slice(),0) == q_dot_0);

    for (k = 0; k < Horizon; k++)
    {
        prob.subject_to(q(Slice(),k+1) == mtimes(A(Slice(0, st_params.q_size), Slice(0, st_params.q_size)), q(Slice(),k)) + mtimes(A(Slice(0, st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)), q_dot(Slice(),k)) + mtimes(B(Slice(0, st_params.q_size), Slice()), u(Slice(),k)) + w(Slice(0,st_params.q_size))); 
        prob.subject_to(q_dot(Slice(),k+1) == mtimes(A(Slice(st_params.q_size, 2*st_params.q_size), Slice(0, st_params.q_size)), q(Slice(),k)) + mtimes(A(Slice(st_params.q_size,2*st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)), q_dot(Slice(),k)) + mtimes(B(Slice(st_params.q_size, 2*st_params.q_size), Slice()), u(Slice(),k)) + w(Slice(st_params.q_size, 2*st_params.q_size)));
        //prob.subject_to(mtimes(A1, q(Slice(),k)) <= b1);
        //prob.subject_to(mtimes(A2, q_dot(Slice(),k)) <= b2);
        // some stuff on pressure
    }

    prob.subject_to(-Du <= (u(Slice(),0)-u_prev) <= Du); 
    for (k = 0; k < Horizon/4; k++){
        prob.subject_to(-Du <= (u(Slice(),k+1)-u(Slice(),k)) <= Du); 
        prob.subject_to(p_min < u(Slice(), k) < p_max);
    }

    // terminal constraint


    //prob.subject_to( fabs(end_effector) >= 0.5*T1); 
    prob.subject_to(q_dot(Slice(),Horizon) == T2); 
    //prob.subject_to( mtimes(q_dot(Slice(),Horizon).T(), q_dot(Slice(),Horizon)) < 1e-23); 
    

    //prob.subject_to(end_effector == T1);
    // prob.subject_to(q_dot(Slice(),Horizon) == T2);

    std::cout << "Constraints initialized" << std::endl;


    Dict opts_dict=Dict();   // to stop printing out solver data
    //opts_dict["ipopt.tol"] = 1e-4;
    opts_dict["ipopt.acceptable_tol"] = 1e4;
    opts_dict["ipopt.print_level"] = 0; 
    opts_dict["ipopt.sb"] = "yes";
    opts_dict["print_time"] = 0;
    // opts_dict["jit"] = true;
    // opts_dict["compiler"] = "shell"; 

    prob.solver("ipopt", opts_dict);

    std::cout<< "MPC problem initialized correctly."<< std::endl; 

    return prob; 
}

void MPC_ts::get_state_space(MatrixXd B, MatrixXd c, MatrixXd g, MatrixXd K, MatrixXd D, MatrixXd A, MatrixXd &sp_A, MatrixXd &sp_B, MatrixXd &sp_w, double Ts){
    // to get state space matrices for state evolution, given dynamics equation
    // already discretized
    // missing the constant coriolis + stuff, check paper

    // MatrixXd sp_A_c(2*st_params.q_size, 2*st_params.q_size);
    // MatrixXd sp_B_c(2*st_params.q_size, 2*st_params.num_segments); 
    // MatrixXd sp_w_c(2*st_params.q_size, 1);

    sp_A_c << MatrixXd::Zero(st_params.q_size,st_params.q_size), MatrixXd::Identity(st_params.q_size, st_params.q_size), - B.inverse() * K, -B.inverse() * D;
    sp_B_c << MatrixXd::Zero(st_params.q_size, 2*st_params.num_segments), B.inverse()*A;
    sp_w_c << MatrixXd::Zero(st_params.q_size,1), - B.inverse()*(c+g); 
    
    // sp_A = MatrixXd::Identity(2*st_params.q_size, 2*st_params.q_size) + Ts*sp_A; 
    // sp_B = Ts*sp_B; 
    // sp_w = Ts*sp_w; 

    sp_A = matrix_exponential(sp_A_c*Ts, 2*st_params.q_size); 
    sp_B = sp_A_c.inverse() * (sp_A - MatrixXd::Identity(2*st_params.q_size, 2*st_params.q_size)) * sp_B_c; 
    sp_w = Ts*sp_w; 

}


MatrixXd MPC_ts::matrix_exponential(MatrixXd A, int size){

    Ad = MatrixXd::Identity(size,size) + A + (A*A)/2 + (A*A*A)/6 + (A*A*A*A)/24 + (A*A*A*A*A)/120;   //up to 5th order
    return Ad; 
}

MX MPC_ts::Rotx(MX theta){

    rot = MX::zeros(3,3); 
    rot(0,0) = 1;
    rot(1,1) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(1,2) = -sin(theta);
    rot(2,1) = sin(theta); 

    // for (int i = 0; i<3; i++){
    //     std::cout << "rotation x diagonal: " << rot(i,i) << std::endl;
    // }

     

    return rot; 
}

MX MPC_ts::Roty(MX theta){

    rot = MX::zeros(3,3); 
    rot(1,1) = 1;
    rot(0,0) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(0,2) = sin(theta);
    rot(2,0) = -sin(theta); 

    //std::cout << "rotation y : " << rot << std::endl; 

    return rot; 
}

MX MPC_ts::axis_angle(MX thetax, MX thetay){

    MX rot = MX::zeros(3,3); 
    MX theta = sqrt(pow(thetax,2)+pow(thetay,2));
    if (theta.is_zero()){
        return rot; 
    }

    MX ux = - thetay / theta; 
    MX uy = - thetax / theta; 
    MX t2 = theta/2; 

    rot(0,0) = cos(t2) + pow(ux,2)*(1-cos(t2)); 
    rot(0,1) = ux*uy*(1-cos(t2)); 
    rot(0,2) = uy*sin(t2); 
    rot(1,0) = rot(0,1); 
    rot(1,1) = cos(t2) + pow(uy,2)*(1-cos(t2)); 
    rot(1,2) = -ux*sin(t2);
    rot(2,0) = - rot(0,2); 
    rot(2,1) = - rot(1,2);
    rot(2,2) = cos(t2);

    //std::cout << "Axis-angle matrix :" << rot << std::endl; 

    return rot; 

}

MX MPC_ts::ee_position(MX thetax, MX thetay, MX length1, MX length2){    // computes position of end effector for each segment (simplified version)


    //MX tot_rot = axis_angle(thetax(0), thetay(0));

    len1 = MX::zeros(3,2);
    len2 = MX::zeros(3,2);

    // MX theta(2,1); 
    // theta(0) = sqrt(pow(thetax(0),2)+pow(thetay(0),2));
    // theta(1) = sqrt(pow(thetax(1),2)+pow(thetay(1),2)); 


    // for (int i = 0; i<2 ; i++){


    //     // if (!theta(i).is_zero()){
    //     // len1(2,i) = 2*length1(i)*sin(theta(i)/2)/theta(i);
    //     // }
    //     // else {
    //     //     len1(2,i) = length1(i);
    //     // }
    //     // len1(2,i) = 2*length1(i)*sin(theta(i)/2)/theta(i);
    //     len1(2,i) = 0.98*length1(i);
    //     len2(2,i) = length2(i); 

    // }

    len1(2,Slice()) = 0.98*length1;
    len2(2,Slice()) = length2; 

    // MX inter(3,1);
    
    // inter = mtimes(tot_rot, len1(Slice(), 0)); 
    // tot_rot = mtimes(tot_rot, axis_angle(thetax(0), thetay(0)));
    // inter += mtimes(tot_rot, len2(Slice(),0));
    // tot_rot = mtimes(tot_rot, axis_angle(thetax(1), thetay(1))); 
    // inter += mtimes(tot_rot, len1(Slice(),1));

    totRot = mtimes(Roty(-thetax(0)/2),Rotx(-thetay(0)/2));
    

    inter = mtimes(totRot, len1(Slice(),0)); 
    totRot = mtimes(totRot, mtimes(Roty(-thetax(0)/2), Rotx(-thetay(0)/2))); 
    inter += mtimes(totRot, len2(Slice(),0)); 
    totRot = mtimes(totRot, mtimes(Roty(-thetax(1)/2),Rotx(-thetay(1)/2)));  
    inter += mtimes(totRot, len1(Slice(),1));

     
    return inter;  

}

void MPC_ts::set_ref(const MatrixXd refx){
    std::lock_guard<std::mutex> lock(mtx);
    // x_r = refx;
    // if (x_ref.colwise().norm() > 0.27) {
    //     x_r = 0.27*x_ref.colwise().normalized();
    // }

    //std::cout << "reference : " << x_r << std::endl;

    length = refx.cols();
    int i; 
    if (length == 0){
        ; 
    }
    else if (length <= Horizon){
        for (i = 0; i < length; i++){
            this->traj_ref.col(i) = refx.col(i); 
        }
        for (i = length; i< Horizon; i++){
            this->traj_ref.col(i) = refx.col(length-1); 
        }
    }
    else{
        for (i = 0; i< Horizon; i++){
            this->traj_ref.col(i) = refx.col(i);
        }
    }


    this->x_ref = traj_ref.col(0); 

    if (!is_initial_ref_received)
        is_initial_ref_received = true;

    //std::cout << "Trajectory : " << traj_ref << std::endl; 
}