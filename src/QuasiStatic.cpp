#include "3d-soft-trunk/QuasiStatic.h"

Opti QuasiStatic::define_problem(){
    
    Opti prob = casadi::Opti();

    A = prob.parameter(4,6); // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<< put all dimensions
    tau = prob.parameter(4,1); 
    u = prob.variable(6,1); 
    MX p_min = MX::zeros(6,1);
    MX p_max = MX::ones(6,1)*500; 

    std::cout << "Variables initialized" << std::endl; 


    MX J = 0; 
    //J += mtimes((tau - mtimes(A, u)).T(),(tau - mtimes(A,u))); 
    MX temp = tau - mtimes(A,u);
    J += mtimes(temp.T(), temp);
    J += mtimes(u.T(),u)*1e-3; 

    prob.minimize(J);

    std::cout << "Cost function initialized" << std::endl;


    prob.subject_to(p_min < u < p_max);    

    std::cout << "Constraints initialized" << std::endl;

    
    prob.solver("ipopt");

    std::cout<< "Pressure problem initialized correctly."<< std::endl; 

    return prob; 

}


QuasiStatic::optimal_solution QuasiStatic::pressure_finder(VectorXd torque, MatrixXd A_real){

    // need to define optimal probelm already in init

    DM A_temp = DM::nan(4,6);
    DM tau_temp = DM::nan(4,1);
    DM p_temp = DM::nan(6,1);

    std::copy(A_real.data(), A_real.data() + A_real.size(), A_temp.ptr());
    std::copy(torque.data(), torque.data() + torque.size(), tau_temp.ptr());

    ctrl.set_value(A, A_temp); 
    ctrl.set_value(tau, tau_temp); 

    OptiSol sol = ctrl.solve(); 

    if(!solved){
        solved = true; 
    }

    p_temp = sol.value(u)(Slice(),0);  

    VectorXd pressure(6,1); 

    pressure = Eigen::VectorXd::Map(DM::densify(p_temp).nonzeros().data(),6,1 );

    QuasiStatic::optimal_solution solution = {pressure, sol};

    return solution; 

}

QuasiStatic::optimal_solution QuasiStatic::pressure_finder_warm(VectorXd torque, MatrixXd A_real, DM old_sol){

    // need to define optimal probelm already in init

    DM A_temp = DM::nan(4,6);
    DM tau_temp = DM::nan(4,1);
    DM p_temp = DM::nan(6,1);
    //DM p_old = DM::nan(6,1); 

    std::copy(A_real.data(), A_real.data() + A_real.size(), A_temp.ptr());
    std::copy(torque.data(), torque.data() + torque.size(), tau_temp.ptr());
    //std::copy(old_sol.data(), old_sol.data() + old_sol.size(), p_old.ptr());

    ctrl.set_value(A, A_temp); 
    ctrl.set_value(tau, tau_temp); 
    ctrl.set_initial(u, old_sol); 

    OptiSol sol = ctrl.solve(); 

    if(!solved){
        solved = true; 
    }

    p_temp = sol.value(u)(Slice(),0);  

    VectorXd pressure(6,1); 

    pressure = Eigen::VectorXd::Map(DM::densify(p_temp).nonzeros().data(),6,1 );

    QuasiStatic::optimal_solution solution = {pressure, sol};

    return solution; 

}



QuasiStatic::QuasiStatic(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type, int objects) : ControllerPCC::ControllerPCC(st_params, sensor_type, objects){
    filename = "QS_logger";


    //set the gains
    kp = 10;
    kd = 5.5;


    //qs
    dt = 1./10;

    ctrl = define_problem();
    solved = false; 

    control_thread = std::thread(&QuasiStatic::control_loop, this);
    chamberMatrix << 1, -0.5, -0.5, 0, sqrt(3) / 2, -sqrt(3) / 2;
 
    mapping_matrix << chamberMatrix, MatrixXd::Zero(2,3), MatrixXd::Zero(2,3), chamberMatrix;  
}

void QuasiStatic::control_loop(){
    srl::Rate r{1./dt};
    while(true){
        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        //update the internal visualization
        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
        
        stm->updateState(state);
        
        if (!is_initial_ref_received) //only control after receiving a reference position
            continue;

        J = stm->J[st_params.num_segments-1]; //tip jacobian
        //J_mid << stm->J[st_params.num_segments-2], stm->J[st_params.num_segments-1] ; //middle seg jacobian

        //this x is from qualisys directly
        if (sensor_type != CurvatureCalculator::SensorType::simulator){
            x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        }
   //     x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        //this x is from forward kinematics, use when using bendlabs sensors


        else {
            x = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-1).translation();
        }

        dx = J*state.dq;
        
        ddx_des = ddx_ref + kp*(x_ref - x).normalized()*0.05;            //desired acceleration from PD controller
    //normed to always assume a distance of 5cm


        for (int i = 0; i < singularity(J); i++){               //reduce jacobian order if the arm is in a singularity
            J.block(0,(st_params.num_segments-1-i)*2,3,2) += 0.02*(i+1)*MatrixXd::Identity(3,2);
        }


        tau_ref = J.transpose()*ddx_des;
        VectorXd pxy = stm->A_pseudo.inverse()*tau_ref/10000;
        p = stm->pseudo2real(pxy+ p_prev);

        DM pressure_old(6,1); 
        
        if (!solved){
            pressure_old = DM::zeros(6,1); 
        }

        QuasiStatic::optimal_solution optimization_result = pressure_finder_warm(pxy + p_prev ,mapping_matrix, pressure_old);
        
        
        VectorXd p_new = optimization_result.pressure; 
        OptiSol old_solution = optimization_result.solution; 
        pressure_old = old_solution.value(u)(Slice(),0); 

        MatrixXd temp = mapping_matrix.completeOrthogonalDecomposition().pseudoInverse(); 

        // std::cout << temp << std::endl; 
        // std::cout << "pressure :" << pxy.transpose() << std::endl; 

        VectorXd p_ls = temp * (pxy+p_prev); 


        std::cout << "P_old : " << p.transpose() << std::endl;
        std::cout << "P_new : " << p_new.transpose() << std::endl; 
        std::cout << "P_LS : " << p_ls.transpose() << std::endl; 


        double error = (pxy+p_prev - mapping_matrix*p_new).transpose() * (pxy+p_prev - mapping_matrix*p_new);

        std::cout << "Reconstruction error : " << error << std::endl; 

        p_prev += pxy;

        // double error2 = (pxy - mapping_matrix*p_new).norm(); 

        // std::cout << "Reconstruction error : " << error2 << std::endl; 

        // double error3 = (pxy - mapping_matrix*p_new).squaredNorm(); 

        // std::cout << "Reconstruction error : " << error3 << std::endl; 


        if (sensor_type != CurvatureCalculator::SensorType::simulator) {actuate(p);}
        else {
            assert(simulate(p));
        }

        // std::cout << "pressure input : " << p.transpose() << std::endl; 
        // std::cout << "curvature : " << state.q.transpose() << std::endl; 
    }
}

int QuasiStatic::singularity(const MatrixXd &J) {
    int order = 0;
    std::vector<Eigen::Vector3d> plane_normals(st_params.num_segments);            //normals to planes create by jacobian
    for (int i = 0; i < st_params.num_segments; i++) {
        Vector3d j1 = J.col(2*i).normalized();   //Eigen hates fun so we have to do this
        Vector3d j2 = J.col(2*i+1).normalized();
        plane_normals[i] = j1.cross(j2);
    }

    for (int i = 0; i < st_params.num_segments - 1; i++) {                         //check for singularities
        for (int j = 0; j < st_params.num_segments - 1 - i; j++){
            if (abs(plane_normals[i].dot(plane_normals[i+j+1])) > 0.995) order+=1;  //if the planes are more or less the same, we are near a singularity
        }
    }
    return order;
}


double QuasiStatic::get_kd(){
    return this->kd;
}
double QuasiStatic::get_kp(){
    return this->kp;
}
void QuasiStatic::set_kd(double kd){
    this->kd = kd;
}
void QuasiStatic::set_kp(double kp){
    this->kp = kp;
}