#include "3d-soft-trunk/MPC_constraints_finder.h"

// this file implements a simplified version of Algorithm 1 from
// Dynamic Control of Soft Robots with Internal Constraints in the Presence of Obstacles
// Cosimo Della Santina  , Antonio Bicchi  , Daniela Rus 


// this algorithm can be used only for circluar obstacles or for linear allowed trajectories. 
// we need wither PHI almost constant or THETA upper-bounded. Otherwise formulation with thetax and thetay is simply wrong.
// But the random trials can be reused in a different way, by finding thetax and thetay references to by looked for. 


MPC_constraints_finder::MPC_constraints_finder(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(st_params, sensor_type){
    
    N_obs = 10; 
    N_tar = 100;   // targets are defined as points along the trajectory (task-space)


    obstacles = MatrixXd::Zero(3, N_obs); 
    targets = MatrixXd::Zero(3, N_tar); 

    // need to define in THETA and PHI  :  q = THETA_1, PHI_1, THETA_2, PHI_2

    VectorXd q_l = VectorXd::Zero(st_params.q_size); // maximum  q_l < q < q_u
    VectorXd q_u = VectorXd::Zero(st_params.q_size); 
    VectorXd q_l_temp = VectorXd::Zero(st_params.q_size); // temporary during optimization
    VectorXd q_u_temp = VectorXd::Zero(st_params.q_size); 
    VectorXd q_l_opt = VectorXd::Zero(st_params.q_size); // optimal
    VectorXd q_u_opt = VectorXd::Zero(st_params.q_size); 

    // phi in [0, 2pi], theta in [0, pi/2], so thetax and thetay in [-pi/2, pi/2]

    // init option 1 (THETA, PHI)

    // for (int i = 0; i < st_params.q_size/2; i++){
    //     q_l(2*i) = 0; 
    //     q_l(2*i + 1) = 0; 
    // }

    // for (int i = 0; i < st_params.q_size/2; i++){
    //     q_u(2*i) = PI/2; 
    //     q_u(2*i + 1) = 2*PI; 
    // }
    
    // init option 2 (thetax, thetay)

    q_l = VectorXd::Ones(st_params.q_size) * -PI/2;
    q_u = VectorXd::Ones(st_params.q_size) * PI/2; 

    // fill obstacles and targets

    double coef = 2 * 3.1415 / N_tar;
    double r = 0.02;

    for (int i = 0; i< N_tar; i++){
        targets(0,i) = r*cos(coef*i);
        targets(1,i) = r*sin(coef*i);
        targets(2,i) = -0.26;
    }

    obstacles.col(0) << 0, 0, -0.1;
    obstacles.col(1) << 0., 0., -0.15; 
    obstacles.col(2) << 0, 0, -0.20; 
    obstacles.col(3) << 0, 0, -0.25; 


    N_trials = 1e5;
    N_check = 1e4;

    std::random_device generator;

 //////////////////////////////////////////////////////

    q_l_temp = q_l; // before check for those
    q_u_temp = q_u; 

    if (check_inclusion(q_l_temp, q_u_temp)){
        q_l_opt = q_l_temp; 
        q_u_opt = q_u_temp;

        std::cout << "Configuration limits work" << std::endl;   // works for no obstacles
    }
    else{
        q_l_temp = VectorXd::Zero(st_params.q_size);
        q_u_temp = VectorXd::Zero(st_params.q_size);

        std::cout << "Start looking for limits" << std::endl;

        for (int i =0; i < N_trials; i++){

            std::cout << "Iteration " << i << std::endl; 
            
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

            std::cout << "Proposal = " << q_l_temp.transpose() << " --- " << q_u_temp.transpose() << std::endl;

            if ( (q_u_opt - q_l_opt).norm() < (q_u_temp - q_l_temp).norm() ){

                std::cout << "Proposal larger" << std::endl;
                if (check_inclusion(q_l_temp, q_u_temp)){
                    q_l_opt = q_l_temp; 
                    q_u_opt = q_u_temp;
                    std::cout << "Proposal accepted at " << i << std::endl;
                } 
            }
        }
    }

    // option 1
    // std::cout << "                THETA1    PHI1    THETA2    PHI2" << std::endl; 
    // std::cout << "Lower bound : " << q_l_opt.transpose() << std::endl; 
    // std::cout << "Upper bound : " << q_u_opt.transpose() << std::endl; 


    // // now map back to thetax and thetay

    // VectorXd q_lower_bound = VectorXd::Zero(st_params.q_size); // maximum  q_l < q < q_u
    // VectorXd q_upper_bound = VectorXd::Zero(st_params.q_size);

    // for (int i = 0; i < st_params.q_size/2; i++){
    //     q_lower_bound(2*i) = q_l_opt(2*i)*cos(q_l_opt(2*i+1)); 
    //     q_lower_bound(2*i+1) = q_l_opt(2*i)*sin(q_l_opt(2*i+1));

    //     q_upper_bound(2*i) = q_u_opt(2*i)*cos(q_u_opt(2*i+1));     // this doesn't make any sense
    //     q_upper_bound(2*i+1) = q_u_opt(2*i)*sin(q_u_opt(2*i+1));
    // }


    // option 2

    std::cout << "q_lower_bound : " << q_l_opt.transpose() << std::endl; 
    std::cout << "q_upper_bound : " << q_u_opt.transpose() << std::endl; 

    // Vector2d THETA_low; 
    // THETA_low << sqrt(pow(q_l_opt(0),2) + pow(q_l_opt(1),2))*180/PI, sqrt(pow(q_l_opt(2),2) + pow(q_l_opt(3),2))*180/PI; 
    // Vector2d THETA_up; 
    // THETA_up << sqrt(pow(q_u_opt(0),2) + pow(q_u_opt(1),2))*180/PI, sqrt(pow(q_u_opt(2),2) + pow(q_u_opt(3),2))*180/PI; 

    // Vector2d PHI_low;
    // Vector2d PHI_up;
    // PHI_low << atan2(q_l_opt(1), q_l_opt(0))*180/PI, atan2(q_l_opt(3), q_l_opt(2))*180/PI; 
    // PHI_up << atan2(q_u_opt(1), q_u_opt(0))*180/PI, atan2(q_u_opt(3), q_u_opt(2))*180/PI; 

    // std::cout << "THETA lower bound : " << THETA_low.transpose() << std::endl; 
    // std::cout << "THETA upper bound : " << THETA_up.transpose() << std::endl; 
    // std::cout << "PHI lower bound : " << PHI_low.transpose() << std::endl; 
    // std::cout << "PHI upper bound : " << PHI_up.transpose() << std::endl; 

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

    std::random_device generator;
    std::uniform_real_distribution<double> distribution0(q_low(0), q_up(0));
    std::uniform_real_distribution<double> distribution1(q_low(1), q_up(1));
    std::uniform_real_distribution<double> distribution2(q_low(2), q_up(2));
    std::uniform_real_distribution<double> distribution3(q_low(3), q_up(3));

    for (i = 0; i<N_check; i++){

        //std::cout << "Check " << i << std::endl;

        q_test << distribution0(generator), distribution1(generator), distribution2(generator), distribution3(generator); 

        // option 1
        // for (ii = 0; ii < st_params.q_size/2; ii++){

        //     thetax(ii) = q_test(2*ii)*cos(q_test(2*ii+1)); 
        //     thetay(ii) = q_test(2*ii)*sin(q_test(2*ii+1));
        //     length1(ii) = -st_params.lengths[2*ii]; 
        //     length2(ii) = -st_params.lengths[2*ii+1]; 

        // }

        // option 2
        for (ii = 0; ii < st_params.q_size/2; ii++){

            thetax(ii) = q_test(2*ii); 
            thetay(ii) = q_test(2*ii+1);
            length1(ii) = -st_params.lengths[2*ii]; 
            length2(ii) = -st_params.lengths[2*ii+1]; 

        }

        ee_mid = ee_position_1(thetax, thetay, length1, length2);   // change to account for middle segment
        ee_end = ee_position_2(thetax, thetay, length1, length2);

        for (ii = 0; ii < N_obs; ii++){
            if ( (ee_mid - obstacles.col(ii)).norm() < 0.02 ){  // 1cm
                included = false; 
                std::cout << ee_mid.transpose() << " hits obstacle " << obstacles.col(ii).transpose() << std::endl;
                //int flag = getchar();   // to pause the program
                break; 
            }

            if ( (ee_end - obstacles.col(ii)).norm() < 0.02 ){  // 1cm
                included = false; 
                std::cout << ee_end.transpose() << " hits obstacle " << obstacles.col(ii).transpose() << std::endl;
                //int flag = getchar(); 
                break; 
            }
        }

        if (included == false){
            break;
        }


        for (ii = 0; ii < N_tar; ii++){
            if ( (ee_end - targets.col(ii)).norm() < 0.05 ){  
                target_reached[ii] = true;  
                //std::cout << ee_end.transpose() << " reaches " << targets.col(ii).transpose() << std::endl;
            }
        }
        
    }

    target_missed = std::count(target_reached, target_reached+N_tar, 0); 

    std::cout << " Proposal missed " << target_missed << " targets" << std::endl;

    if (target_missed > N_tar/3){
        included = false; 
    }

    std::cout << "Proposal accepted : " << included << std::endl; 

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