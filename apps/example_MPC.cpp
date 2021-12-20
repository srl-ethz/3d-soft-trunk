#include <3d-soft-trunk/ControllerPCC.h>
#include "3d-soft-trunk/MPC.h"
#include "3d-soft-trunk/MPC_ts.h"
#include <chrono>

int main(){

    bool task_space_control = true;   // set to true to operate in task space   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    bool logger = true; 

    std::cout << "Task space control enabled : " << task_space_control << std::endl; 

    SoftTrunkParameters st_params;
    st_params.finalize();
    ControllerPCC cpcc = ControllerPCC(st_params, CurvatureCalculator::SensorType::simulator);

    // Choose mpc1 for joint-space control, mpc2 for end-effector space control   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //MPC mpc1(st_params, CurvatureCalculator::SensorType::simulator);
    MPC_ts mpc2(st_params, CurvatureCalculator::SensorType::simulator);

    // if (!task_space_control){
    //     MPC mpc(st_params, CurvatureCalculator::SensorType::simulator);
    // }
    // else{
    //     MPC_ts mpc(st_params, CurvatureCalculator::SensorType::simulator);
    // }
    
    srl::State state = st_params.getBlankState();
    srl::State state_ref = st_params.getBlankState(); 
    VectorXd p = VectorXd::Zero(3*st_params.num_segments);
    double time = 30.0;
    const double dt = 0.01;

    for (int i = 0; i < st_params.num_segments; i++) {
        // set to have about the same curvature as a whole regardless of scale
        double rand = -2.093 / st_params.sections_per_segment / st_params.num_segments / 10;
        for (int j = 0; j < st_params.sections_per_segment; j++){
            state.q(2*i*st_params.sections_per_segment + 2*j + 1) = -rand;
            state.q(2*i*st_params.sections_per_segment + 2*j ) = rand * 0.022 / 0.19;
        }

    }

    std::unique_ptr<SoftTrunkModel> stm;
    std::unique_ptr<CurvatureCalculator> cc;


    //Vector3d q_ref, q_act; 
    //Vector3d dq_ref;

    Vector3d x_act; 
    MatrixXd x_ref; 
    Vector3d trajectory; 
    //MatrixXd trajectory = MatrixXd::Zero(3, 1);

    double coef = 2 * 3.1415 / time;
    double r = 0.1;
    double tol = 0.1;
    double t = 0;
    bool flag = 0; // to have 1 disturbance, and one only
    double disturbance = 2.093 / st_params.sections_per_segment / st_params.num_segments / 10;

    int counter = 0;
    int new_counter;

    int avg = 20;

    VectorXd error_avg = VectorXd::Zero(avg); 


    //mpc1.set_state(state);               // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< change here to switch between modes
    mpc2.set_state(state);
    const double hz = 1./dt;
    //mpc1.set_frequency(hz);
    mpc2.set_frequency(hz);

    if (logger){
        mpc2.toggle_log();
    }

    auto start = std::chrono::steady_clock::now();

    // comment and uncomment each loop <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
/*
    if (!task_space_control){

        while (t < time/2){
        
            state_ref.q << -0.440825, 0.373653, -0.0481017, -0.105963;
            mpc1.set_ref(state_ref);

            mpc1.get_state(state); 

            if (counter >= avg){
                counter = 0;
            }

            if (counter < avg){
                error_avg(counter) = (state_ref.q - state.q).norm(); 
                counter ++; 
            }

            std::cout << "Reference : "<< state_ref.q.transpose()<< "\nCurrent state : " << state.q.transpose() << std::endl;  
            // std::cout << "Error vector : " << error_avg.transpose() << std::endl;
            
            std::cout << "Error : " << error_avg.mean() << std::endl; 

            std::cout<< "time : " << t <<std::endl; 

            // if ( (state.q - state_ref.q).norm() < tol){
            //     std::cout << "Convergerged in " << t << std::endl;
            // }

            t += dt; 
        }

        counter = 0;
        error_avg = VectorXd::Zero(avg);

        while (t < time){

            state_ref.q << 0.0633816, 0.290002, 0.655952, -0.0951374;
            mpc1.set_ref(state_ref);

            mpc1.get_state(state); 

            if (counter >= avg){
                counter = 0;
            }

            if (counter < avg){
                error_avg(counter) = (state_ref.q - state.q).norm(); 
                counter ++; 
            }

            std::cout << "Reference : "<< state_ref.q.transpose()<< "\nCurrent state : " << state.q.transpose() << std::endl;  
            // std::cout << "Error vector : " << error_avg.transpose() << std::endl;

            std::cout << "Error : " << error_avg.mean() << std::endl;
            
            std::cout<< "time : " << t <<std::endl; 
            // if ( (state.q - state_ref.q).norm() < tol){
            //     std::cout << "Convergerged in " << t << std::endl;
            // }

            t += dt; 
        }
    }
*/
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

   
    counter = 0;
    error_avg = VectorXd::Zero(avg);

    if (task_space_control){

        while (t < time/3){

            // trajectory << 0.099968, -0.00253146, -0.215; 
            trajectory(0,0) = 0.09; 
            trajectory(1,0) = -0.01; 
            trajectory(2,0) = -0.215; 
    

            x_ref = trajectory; 

            mpc2.set_ref(x_ref); 
            mpc2.get_x(x_act);

            //std::cout << "End_effector other :" << x_act.transpose() << std::endl; 

            if (counter >= avg){
                counter = 0;
            }

            if (counter < avg){
                error_avg(counter) = (trajectory.col(0) - x_act).norm(); 
                counter ++; 
            }

            //std::cout << "Error : " << error_avg.mean() << std::endl;

            srl::sleep(0.05); 
            t += dt; 

            std::cout << t << std::endl; 

        }

        counter = 0;
        error_avg = VectorXd::Zero(avg);

        while (t < 2*time/3)
        {
            // std::cout << "NEXT PHASE" << std::endl; 
            // break;
            // trajectory << 0.0125192, -0.0992133, -0.215; 
            trajectory(0,0) = 0.01; 
            trajectory(1,0) = -0.09; 
            trajectory(2,0) = -0.215; 

            x_ref = trajectory;

            mpc2.set_ref(x_ref); 
            mpc2.get_x(x_act);

            if (counter >= avg){
                counter = 0;
            }

            if (counter < avg){
                error_avg(counter) = (trajectory.col(0) - x_act).norm(); 
                counter ++; 
            }

            // std::cout << "Error : " << error_avg.mean() << std::endl;

            srl::sleep(0.05); 
            t += dt; 

        }

        counter = 0;
        error_avg = VectorXd::Zero(avg);

        while (t < time)
        {
            // break;
            // trajectory << -0.0962055, -0.0272855, -0.215;  
            trajectory(0,0) = -0.09; 
            trajectory(1,0) = 0.01; 
            trajectory(2,0) = -0.215; 

            x_ref = trajectory;

            mpc2.set_ref(x_ref); 
            mpc2.get_x(x_act);

            if (counter >= avg){
                counter = 0;
            }

            if (counter < avg){
                error_avg(counter) = (trajectory.col(0) - x_act).norm(); 
                counter ++; 
            }

            // std::cout << "Error : " << error_avg.mean() << std::endl;

            srl::sleep(0.05); 
            t += dt; 

        }
        

    }

    if (logger){
        mpc2.toggle_log();
    }

    std::cout << "Simulation end" << std::endl; 

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulated " << time << "s of motion in " << elapsed.count() <<"s realtime using timestep " << dt << "\n";

}