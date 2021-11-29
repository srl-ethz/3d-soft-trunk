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

MatrixXd axis_angle(double thetax, double thetay){

    MatrixXd rot = MatrixXd::Zero(3,3); 
    double theta = sqrt(pow(thetax,2)+pow(thetay,2));
    if (theta == 0){
        return rot; 
    }

    double ux = -thetay / theta; 
    double uy = thetax / theta; 
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

    return rot; 

}

MatrixXd ee_position_1(double thetax, double thetay, double length1, double length2){    // computes position of end effector for each segment

    MatrixXd rotx = Rotx_1(thetay); 
    MatrixXd roty = Roty_1(thetax); 

    //MatrixXd tot_rot = -rotx * roty; 

    MatrixXd tot_rot = axis_angle(thetax, thetay);

    MatrixXd len = MatrixXd::Zero(3,1);
    double theta = sqrt(pow(thetax,2)+pow(thetay,2)); 
    if (theta != 0){
        len(2,0) = 2*length1*sin(theta/2)/theta;
    }
    else {
        len(2,0) = length1;
    }

    len(2,0) += length2; 
     

    //std::cout << "Inter step : " << tot_rot*len << std::endl; 

    return tot_rot* len; 

}






MPC_ts::MPC_ts(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(st_params, sensor_type){
    filename = "MPC_logger";

    Horizon = 20;
    dt = 1./10;

    // Take model
    ctrl = define_problem();   // define matrices as parameters, so we can update them without re-initalizing the problem
    solved = false; 

    control_thread = std::thread(&MPC_ts::control_loop, this);
}

void MPC_ts::control_loop(){
    srl::Rate r{1./dt};

    while(true){

        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
            
        stm->updateState(state);
            
        if (!is_initial_ref_received){
            //only control after receiving a reference position
            continue;
        } 

        Vector3d ee_x = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-1).translation();

        std::cout << "End-effector position real : " << ee_x.transpose() << std::endl;  

        MatrixXd ee_t = MatrixXd::Zero(3,1); 

        for (int kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

            ee_t += ee_position_1(state.q(2*kk,0), state.q(2*kk+1,0), -st_params.lengths[2*kk], - st_params.lengths[2*kk+1]); 

            std::cout << "theta_x : " << state.q(2*kk,0) << std::endl; 
            std::cout << "theta_y : " << state.q(2*kk+1,0) << std::endl; 

            //std::cout << "temp = " << ee_t << std::endl; 
        }

    
        std::cout << "End-effector position mine : " << ee_t.transpose() << std::endl;



        sp_A = MatrixXd::Zero(2*st_params.q_size, 2*st_params.q_size);
        sp_B = MatrixXd::Zero(2*st_params.q_size, 2*st_params.num_segments);  
        sp_w = MatrixXd::Zero(2*st_params.q_size, 1);

        //MatrixXd sp_A(2*st_params.q_size, 2*st_params.q_size);
        //MatrixXd sp_B(2*st_params.q_size, 2*st_params.num_segments);   // SS matrices

        // need a coversion DM to MatrixXd and viceversa
        
        get_state_space(stm->B, stm->c, stm->g, stm->K, stm->D, stm->A_pseudo, sp_A, sp_B, sp_w, dt);

        //https://github.com/casadi/casadi/issues/2563
        //https://groups.google.com/g/casadi-users/c/npPcKItdLN8
        // DM <--> Eigen

        sp_A_temp = DM::nan(2*st_params.q_size, 2*st_params.q_size);
        sp_B_temp = DM::nan(2*st_params.q_size, 2*st_params.num_segments);
        sp_w_temp = DM::nan(2*st_params.q_size, 1);

        x_r_temp = DM::nan(3,1);  // conversion placeholders

        q_0_temp = DM::nan(st_params.q_size, 1);
        q_dot_0_temp = DM::nan(st_params.q_size, 1);

        // OLD IMPLEMENTATION
        //DM sp_B_temp(Sparsity::dense(2*st_params.q_size, 2*st_params.num_segments));
        //DM q_r_temp(Sparsity::dense(st_params.q_size, 1));
        //DM q_dot_r_temp(Sparsity::dense(st_params.q_size, 1));  // conversion placeholders

        std::copy(sp_A.data(), sp_A.data() + sp_A.size(), sp_A_temp.ptr());
        std::copy(sp_B.data(), sp_B.data() + sp_B.size(), sp_B_temp.ptr());
        std::copy(sp_w.data(), sp_w.data() + sp_w.size(), sp_w_temp.ptr());
        std::copy(x_ref.data(), x_ref.data() + x_ref.size(), x_r_temp.ptr()); 
        std::copy(state.q.data(), state.q.data() + state.q.size(), q_0_temp.ptr());
        std::copy(state.dq.data(), state.dq.data() + state.dq.size(), q_dot_0_temp.ptr());


        // solve problem

        if (!solved){
            u_temp = DM::zeros(2*st_params.num_segments,1); 
        }

        ctrl.set_value(A, sp_A_temp);   // need to define them already as global variables
        ctrl.set_value(B, sp_B_temp);
        ctrl.set_value(w, sp_w_temp); 
        ctrl.set_value(x_r, x_r_temp);
        ctrl.set_value(q_0, q_0_temp); 
        ctrl.set_value(q_dot_0, q_dot_0_temp); 
        ctrl.set_value(u_prev, u_temp); 
        OptiSol sol = ctrl.solve(); 
        
        if (!solved){
            solved = TRUE; 
        }

        
        //std::cout << "complete u : " <<sol.value(u) << std::endl; 
        //std::cout << "-----------------" << std::endl;
        //std::cout << stm->A_pseudo << std::endl;
        //std::cout << stm-> A << std::endl;

        //auto u_temp = static_cast<std::vector<double>>(sol.value(u));
        u_temp = sol.value(u)(Slice(),0); 

        std::cout << "solution :" << u_temp << std::endl;
        
        p_temp = MatrixXd::Zero(2*st_params.num_segments,1); 
        //std::copy(u_temp.data(), u_temp.data() + u_temp.size(), p_temp);  // <<<<<<<<<<<< error, missing DM to Eigen part

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

        p = stm->pseudo2real(p_temp); 

        std::cout << "pressure input : " << p.transpose() << std::endl; 

        if (sensor_type != CurvatureCalculator::SensorType::simulator) {actuate(p);}
        else {
            assert(simulate(p));
        }
        
    }
    
}

Opti MPC_ts::define_problem(){
    
    Opti prob = casadi::Opti();

    MX q = prob.variable(st_params.q_size, Horizon+1);
    MX q_dot = prob.variable(st_params.q_size, Horizon+1);
    u = prob.variable(2*st_params.num_segments, Horizon);  //pressure

    q_0 = prob.parameter(st_params.q_size,1);
    q_dot_0 = prob.parameter(st_params.q_size,1); 

    // define as parameters the model matrices and set point

    A = prob.parameter(2*st_params.q_size, 2*st_params.q_size);
    B = prob.parameter(2*st_params.q_size, 2*st_params.num_segments);
    w = prob.parameter(2*st_params.q_size, 1); 

    x_r = prob.parameter(3,1); 

    u_prev = prob.parameter(2*st_params.num_segments,1); 



    MX J = 0;
    MX A1 = MX::zeros(st_params.q_size, st_params.q_size);
    MX b1 = MX::ones(st_params.q_size,1);
    MX A2 = MX::zeros(st_params.q_size, st_params.q_size);
    MX b2 = MX::ones(st_params.q_size,1);
    MX T1 = MX::zeros(3,1);
    MX T2 = MX::zeros(st_params.q_size,1);

    MX Du = MX::ones(2*st_params.num_segments,1)*500; 

    MX end_effector = MX::zeros(3,1); 

    //std::cout << "Delta u = " << Du << std::endl; 

    // for (int i = 0; i < 3; i++){
    //     T1(i) == fabs(x_r(i)); 
    // }
    T1 = fabs(x_r); 
    //T2 = q_dot_r;  //terminal condition with delta formulation

    MX Q = MX::eye(3); 
    MX Q2 = MX::eye(st_params.q_size); 
    MX R = MX::eye(2*st_params.num_segments);

    std::cout << "Variables initialized" << std::endl; 

    // use delta-formulation with state reference  <<<<--------------<<<<<<<<<<<<----------<<<<<<<<<<<<----------

    for (int k = 0; k < Horizon; k++) 
    {
        // J += mtimes((q(Slice(),k)-q_r).T(), mtimes(Q, (q(Slice(),k)-q_r)));   // probaly Slice(0,st_params.q_size,1) has same effect
        J += mtimes((q_dot(Slice(),k)).T(), mtimes(Q2, (q_dot(Slice(),k))));    // keep lomit on speeds
        end_effector = MX::zeros(3,1); 

        for (int kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

            end_effector += ee_position(q(2*kk,k), q(2*kk+1,k), -st_params.lengths[2*kk] - st_params.lengths[2*kk+1]); 
        }

        J += mtimes((end_effector-x_r).T(), mtimes(Q, (end_effector-x_r))); 

        //J += mtimes(u(Slice(),k).T(), mtimes(R, u(Slice(),k)));
    }


    end_effector = MX::zeros(3,1);    // needed also later for terminal constraint

    for (int kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

        end_effector += ee_position(q(2*kk,Horizon), q(2*kk+1,Horizon), -st_params.lengths[2*kk] - st_params.lengths[2*kk+1]); 
    }

    
    J += mtimes((end_effector-x_r).T(), mtimes(Q, (end_effector-x_r)));

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

    for (int k = 0; k < Horizon; k++)
    {
        prob.subject_to(q(Slice(),k+1) == mtimes(A(Slice(0, st_params.q_size), Slice(0, st_params.q_size)), q(Slice(),k)) + mtimes(A(Slice(0, st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)), q_dot(Slice(),k)) + mtimes(B(Slice(0, st_params.q_size), Slice()), u(Slice(),k)) + w(Slice(0,st_params.q_size))); 
        prob.subject_to(q_dot(Slice(),k+1) == mtimes(A(Slice(st_params.q_size, 2*st_params.q_size), Slice(0, st_params.q_size)), q(Slice(),k)) + mtimes(A(Slice(st_params.q_size,2*st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)), q_dot(Slice(),k)) + mtimes(B(Slice(st_params.q_size, 2*st_params.q_size), Slice()), u(Slice(),k)) + w(Slice(st_params.q_size, 2*st_params.q_size)));
        //prob.subject_to(mtimes(A1, q(Slice(),k)) <= b1);
        //prob.subject_to(mtimes(A2, q_dot(Slice(),k)) <= b2);
        // some stuff on pressure
    }

    prob.subject_to(-Du <= (u(Slice(),0)-u_prev) <= Du); 
    for (int k = 1; k < Horizon/4; k++){
        prob.subject_to(-Du <= (u(Slice(),k)-u(Slice(),k-1)) <= Du); 
    }

    // terminal constraint


    //prob.subject_to( fabs(end_effector - x_r) <= 1*T1); 

    //prob.subject_to(end_effector == T1);
    // prob.subject_to(q_dot(Slice(),Horizon) == T2);

    std::cout << "Constraints initialized" << std::endl;

    
    prob.solver("ipopt");

    std::cout<< "MPC problem initialized correctly."<< std::endl; 

    return prob; 
}

void MPC_ts::get_state_space(MatrixXd B, MatrixXd c, MatrixXd g, MatrixXd K, MatrixXd D, MatrixXd A, MatrixXd &sp_A, MatrixXd &sp_B, MatrixXd &sp_w, double Ts){
    // to get state space matrices for state evolution, given dynamics equation
    // already discretized
    // missing the constant coriolis + stuff, check paper

    MatrixXd sp_A_c(2*st_params.q_size, 2*st_params.q_size);
    MatrixXd sp_B_c(2*st_params.q_size, 2*st_params.num_segments); 
    MatrixXd sp_w_c(2*st_params.q_size, 1);

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

    MatrixXd Ad = MatrixXd::Identity(size,size) + A + (A*A)/2 + (A*A*A)/6 + (A*A*A*A)/24 + (A*A*A*A*A)/120;   //up to 5th order
    return Ad; 
}

MX MPC_ts::Rotx(MX theta){

    MX rot = MX::zeros(3,3); 
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

    MX rot = MX::zeros(3,3); 
    rot(1,1) = 1;
    rot(0,0) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(0,2) = sin(theta);
    rot(2,0) = -sin(theta); 

    //std::cout << "rotation y : " << rot << std::endl; 

    return rot; 
}

MX MPC_ts::ee_position(MX thetax, MX thetay, double length){    // computes position of end effector for each segment

    MX rotx = Rotx(thetay); 
    MX roty = Roty(thetax); 

    MX tot_rot = mtimes(-rotx, roty); 
    MX len = MX::zeros(3,1);
    len(2,0) = length; 

    return mtimes(tot_rot, len); 

}