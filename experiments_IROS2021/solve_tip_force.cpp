#include <3d-soft-trunk/SoftTrunkModel.h>
#include <mobilerack-interface/SerialInterface.h>

// https://drake.mit.edu/doxygen_cxx/group__solvers.html
// https://github.com/RobotLocomotion/drake/blob/master/solvers/test/quadratic_program_examples.cc
#include <drake/solvers/equality_constrained_qp_solver.h>
#include <drake/solvers/mathematical_program.h>

int main(){
    SoftTrunkModel stm{};
    SerialInterface si{"/dev/ttyACM0", 38400};

    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(q.size());
    VectorXd p = VectorXd::Zero(6);
    VectorXd s = VectorXd::Zero(4);

    std::vector<float> bendLab_data;
    VectorXd bendLab_offset = VectorXd::Zero(4); // use the first N measurements as offset
    int N = 100;
    fmt::print("taking {} measurements to calibrate baseline. Keep arm straight and don't move it...\n", N);
    for (int i = 0; i < N; i++)
    {
        si.getData(bendLab_data);
        for (int j = 0; j < 4; j++)
            bendLab_offset(j) += bendLab_data[j];
        srl::sleep(0.02);
    }
    bendLab_offset /= N;
    fmt::print("sensor offset is {}\n", bendLab_offset.transpose()*PI/180.);

    // set up matrices to be used in optimization
    MatrixXd S(4, 12); /** sensor mapping */
    S << 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1;
    MatrixXd Mq = MatrixXd::Zero(12, 15);
    Mq.block(0,0,12,12) = MatrixXd::Identity(12,12);
    MatrixXd Mf = MatrixXd::Zero(3, 15);
    Mf.block(0,12,3,3) = Matrix3d::Identity();
    drake::solvers::MathematicalProgram prog;
    drake::solvers::EqualityConstrainedQPSolver solver;
    drake::solvers::MathematicalProgramResult result;
    fmt::print("Mq:\n{}\nMf:\n{}\n", Mq, Mf);

    srl::Rate r{2};
    while (true)
    {
        si.getData(bendLab_data);
        for (int i = 0; i < 4; i++){
            s(i) = (bendLab_data[i] - bendLab_offset[i]) * PI / 180;
        }
           
        
        for (int i = 0; i < st_params::num_segments*st_params::sections_per_segment; i++)
        {
            // copy data from bendlab to Soft Trunk pose
            // divide curvauture equally across PCC sections
            int segment_id = i / st_params::sections_per_segment;
            q(2*i) = s(2*segment_id) / st_params::sections_per_segment;
            q(2*i+1) = s(2*segment_id+1) / st_params::sections_per_segment;
        }
        stm.updateState(q, dq);

        VectorXd e = stm.g - stm.A*p;

        double ratio = 0.01;
        MatrixXd Q = Mq.transpose()*stm.K.transpose()*stm.K*Mq + Mf.transpose()*stm.J*stm.J.transpose()*Mf - 2*Mq.transpose()*stm.K.transpose()*stm.J.transpose()*Mf + ratio*Mf.transpose()*Mf; 
        Q *= 2;
        VectorXd b = 2*e.transpose()*stm.K*Mq - 2*e.transpose()*stm.J.transpose()*Mf;

        drake::solvers::VectorDecisionVariable<15> x = prog.NewContinuousVariables<15>();
        prog.AddQuadraticCost(Q, b, x);
        prog.AddLinearEqualityConstraint(S, s, x.segment<12>(0)); // constraint is to make it make sense with the sensor measurements
        result = solver.Solve(prog);
        VectorXd x_result = result.GetSolution(x);
        fmt::print("result: {}\nsensor: {}\tforce: {}\n", result.is_success(), s.transpose(), x_result.segment(12, 3).transpose());

        VectorXd f_est = stm.J * (stm.J.transpose()*stm.J).inverse() * (/*stm.g +*/ stm.K*q - stm.A*p);
        fmt::print("simpler solution: f={}\n", f_est);

        stm.updateState(x_result.segment(0,12), dq);

        r.sleep();
    }
    

    return 1;
}