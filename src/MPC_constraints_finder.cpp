#include "3d-soft-trunk/MPC_constraints_finder.h"

// this file implements a simplified version of Algorithm 1 from
// Dynamic Control of Soft Robots with Internal Constraints in the Presence of Obstacles
// Cosimo Della Santina  , Antonio Bicchi  , Daniela Rus 


MPC_constraints_finder::MPC_constraints_finder(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(st_params, sensor_type){
    
    N_obs = 10; 
    N_tar = 100;   // targets are defined as points along the trajectory (task-space)

    // MatrixXd obstacles = MatrixXd::Zero(3, N_obs); 
    // MatrixXd targets = MatrixXd::Zero(3, N_tar); 

    VectorXd q_l = VectorXd::Zero(st_params.q_size); // maximum  q_l < q < q_u
    VectorXd q_u = VectorXd::Zero(st_params.q_size); 
    VectorXd q_l_temp = VectorXd::Zero(st_params.q_size); // temprary during optimization
    VectorXd q_u_temp = VectorXd::Zero(st_params.q_size); 
    VectorXd q_l_opt = VectorXd::Zero(st_params.q_size); // optimal
    VectorXd q_u_opt = VectorXd::Zero(st_params.q_size); 


    N_trials = 1000;
    N_check = 1000;

    std::default_random_engine generator;

 //////////////////////////////////////////////////////

    q_l_temp = q_l; // before check for those
    q_u_temp = q_u; 

    if (check_inclusion(q_l_temp, q_u_temp)){
        q_l_opt = q_l_temp; 
        q_u_opt = q_u_temp;
    }
    else{
        q_l_temp = VectorXd::Zero(st_params.q_size);
        q_u_temp = VectorXd::Zero(st_params.q_size);

        for (int i =0; i < N_trials; i++){
            
            std::uniform_real_distribution<double> distribution0(q_l(0), q_u(0));
            std::uniform_real_distribution<double> distribution1(q_l(1), q_u(1));
            std::uniform_real_distribution<double> distribution2(q_l(2), q_u(2));
            std::uniform_real_distribution<double> distribution3(q_l(3), q_u(3));

            q_l_temp << distribution0(generator), distribution1(generator), distribution2(generator), distribution3(generator);

            std::uniform_real_distribution<double> distribution4(q_l_temp(0), q_u(0));
            std::uniform_real_distribution<double> distribution5(q_l_temp(1), q_u(1));
            std::uniform_real_distribution<double> distribution6(q_l_temp(2), q_u(2));
            std::uniform_real_distribution<double> distribution7(q_l_temp(3), q_u(3));

            q_u_temp << distribution4(generator), distribution5(generator), distribution6(generator), distribution7(generator);

            if ( (q_u_opt - q_l_opt).norm() < (q_u_temp - q_l_temp).norm() ){
                if (check_inclusion(q_l_temp, q_u_temp)){
                    q_l_opt = q_l_temp; 
                    q_u_opt = q_u_temp;
                } 
            }
        }
    }

    std::cout << "q_lower_bound : " << q_l_opt.transpose() << std::endl; 
    std::cout << "q_upper_bound : " << q_u_opt.transpose() << std::endl; 

}


bool MPC_constraints_finder::check_inclusion(VectorXd q_low, VectorXd q_up){
    bool included = true;
    bool target_reached[N_tar] = {false}; 
    int target_missed = 0; 
    int i, ii; 

    VectorXd thetax(2), thetay(2), length1(2), length2(2); // for FK
    MatrixXd ee_mid = MatrixXd::Zero(3,1); 
    MatrixXd ee_end = MatrixXd::Zero(3,1);

    VectorXd q_test = VectorXd::Zero(st_params.q_size);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution0(q_low(0), q_up(0));
    std::uniform_real_distribution<double> distribution1(q_low(1), q_up(1));
    std::uniform_real_distribution<double> distribution2(q_low(2), q_up(2));
    std::uniform_real_distribution<double> distribution3(q_low(3), q_up(3));

    for (i = 0; i<N_check; i++){
        q_test << distribution0(generator), distribution1(generator), distribution2(generator), distribution3(generator); 

        for (ii = 0; ii < st_params.q_size/2; ii++){

            thetax(ii) = q_test(2*ii,0); 
            thetay(ii) = q_test(2*ii+1,0);
            length1(ii) = -st_params.lengths[2*ii]; 
            length2(ii) = -st_params.lengths[2*ii+1]; 

        }

        ee_mid = ee_position_1(thetax, thetay, length1, length2);   // change to account for middle segment
        ee_end = ee_position_2(thetax, thetay, length1, length2);

        for (ii = 0; ii < N_obs; ii++){
            if ( (ee_mid - obstacles.col(ii)).norm() < 0.01 ){  // 1cm
                included = false; 
                break; 
            }

            if ( (ee_end - obstacles.col(ii)).norm() < 0.01 ){  // 1cm
                included = false; 
                break; 
            }
        }

        if (included = false){
            break;
        }


        for (ii = 0; ii < N_tar; ii++){
            if ( (ee_end - targets.col(ii)).norm() < 0.02 ){  
                target_reached[ii] = true;  
            }
        }
        
    }

    target_missed = std::count(target_reached, target_reached+N_tar, 0); 

    if (target_missed > N_tar/3){
        included = false; 
    }

    return included;
} 


MatrixXd MPC_constraints_finder::Rotx_1(double theta){

    MatrixXd rot = MatrixXd::Zero(3,3); 
    rot(0,0) = 1;
    rot(1,1) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(1,2) = -sin(theta);
    rot(2,1) = sin(theta); 

    //std::cout << "X rotation : " << rot << std::endl; 
     

    return rot; 
}

MatrixXd MPC_constraints_finder::Roty_1(double theta){

    MatrixXd rot = MatrixXd::Zero(3,3); 
    rot(1,1) = 1;
    rot(0,0) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(0,2) = sin(theta);
    rot(2,0) = -sin(theta); 

    //std::cout << "Y rotation : " << rot << std::endl; 


    return rot; 
}


MatrixXd MPC_constraints_finder::ee_position_1(VectorXd thetax, VectorXd thetay, VectorXd length1, VectorXd length2){    // computes position of end effector for each segment

    MatrixXd len1 = MatrixXd::Zero(3,2);
    MatrixXd len2 = MatrixXd::Zero(3,2);

    VectorXd theta(2); 
    theta << sqrt(pow(thetax(0),2)+pow(thetay(0),2)), sqrt(pow(thetax(1),2)+pow(thetay(1),2)); 

    // std::cout << "~~~~~~~~~~~~~~~~~~~" << std::endl;
    // std::cout << "theta " << theta(0) << " " << sin(theta(0)/2)/theta(0) << " sin(theta)/theta" << std::endl; 
    // std::cout << "~~~~~~~~~~~~~~~~~~~" << std::endl;
    

    for (int i = 0; i<2 ; i++){


        if (theta(i) != 0){
        len1(2,i) = 2*length1(i)*sin(theta(i)/2)/theta(i);
        }
        else {
            len1(2,i) = length1(i);
        }

        // len1(2,i) = 0.98*length1(i); 
        // len2(2,i) = length2(i); 

    }

    MatrixXd totRot = Roty_1(-thetax(0)/2)*Rotx_1(-thetay(0)/2);
    MatrixXd inter(3,1);

    inter = totRot*len1.col(0); 
    totRot *= Roty_1(-thetax(0)/2)*Rotx_1(-thetay(0)/2); 
    inter += totRot*len2.col(0); 

    return inter; 


}

MatrixXd MPC_constraints_finder::ee_position_2(VectorXd thetax, VectorXd thetay, VectorXd length1, VectorXd length2){

    MatrixXd len1 = MatrixXd::Zero(3,2);
    MatrixXd len2 = MatrixXd::Zero(3,2);

    VectorXd theta(2); 
    theta << sqrt(pow(thetax(0),2)+pow(thetay(0),2)), sqrt(pow(thetax(1),2)+pow(thetay(1),2)); 

    // std::cout << "~~~~~~~~~~~~~~~~~~~" << std::endl;
    // std::cout << "theta " << theta(0) << " " << sin(theta(0)/2)/theta(0) << " sin(theta)/theta" << std::endl; 
    // std::cout << "~~~~~~~~~~~~~~~~~~~" << std::endl;
    

    for (int i = 0; i<2 ; i++){


        if (theta(i) != 0){
        len1(2,i) = 2*length1(i)*sin(theta(i)/2)/theta(i);
        }
        else {
            len1(2,i) = length1(i);
        }

        // len1(2,i) = 0.98*length1(i); 
        // len2(2,i) = length2(i); 

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


