#include <3d-soft-trunk/ControllerPCC.h>
#include <casadi/casadi.hpp>
using namespace casadi;
#include <casadi/core/optistack.hpp>


int main(){

    DM q_0 = {1,2,3,4}; 
    DM q_dot_0 = {1,1,1,1}; 
    DM b1 = {5,5,5,5,5,5,5,5}; 
    DM b2 = {3,3,3,3,3,3,3,3};

    DM Q0 = DM::zeros(2*4,1);
    DM Q_DOT_0 = DM::zeros(2*4,1);
    for (int i = 0; i < 2*4; i++){
        Q0(i,0) = pow(-1,i)*q_0(int(i/2)) + b1(i,0);
        Q_DOT_0(i,0) = ((-1)^(i))*q_dot_0(int(i/2)) + b2(i,0);
    }

    std::cout << Q0.T() << std::endl;
    std::cout << Q_DOT_0.T() << std::endl;

/*
    DM A = DM::nan(2,1);
    DM B = DM::nan(2,1); 

    A = {30, 20}; 
    B = {20, 30}; 

    std::cout << fmax(A,B) << std::endl;
*/
/*
    DM A = DM::zeros(3,1);
    DM B = DM::ones(3,1); 

    //A(Slice(),2) = 0.9*B; 

    DM C = vertcat(A,B); 

    for (int i = 0; i<6; i++){
        for (int j=0; j<1; j++){
            std::cout << C(i,j);
        }
        std::cout << std::endl; 
    }

    std::cout << "Is this? " << C << C.rows() << std::endl;
*/


/*
    MatrixXd test = MatrixXd::Identity(10,10);
    std::cout << test.rightCols(5) << std::endl; 
*/

/*
    std::vector<double> lengths = {-0.125, -0.02, -0.125, -0.02};

    std::cout << std::abs(lengths[0]) << std::endl;  
*/
    
/*    
    const double pi = std::acos(-1.0);

    MatrixXd A(3,3);
    A << 0,    -pi/4, 0,
        pi/4, 0,     0,
        0,    0,     0;

    std::cout << "The matrix A is:\n" << A << "\n\n";

    //std::cout << "check : " << A(0,1) << std::endl; 


    MatrixXd Ad = MatrixXd::Identity(3,3) + A + (A*A)/2 + (A*A*A)/6 + (A*A*A*A)/24 + (A*A*A*A*A)/120; 
    std::cout << "The matrix exponential of A is:\n" << Ad << "\n\n";
*/

/*
    MatrixXd M(6,6);

    M << MatrixXd::Identity(3,3), MatrixXd::Zero(3,3), MatrixXd::Identity(3,3), MatrixXd::Identity(3,3);

    std::cout << M << std::endl;

    DM M1(Sparsity::dense(6,6));

    std::copy(M.data(), M.data() + M.size(), M1.ptr());

    std::cout << M1(Slice(0,5), Slice()) << std::endl; 

*/
/*
    Opti opti = casadi::Opti();

    MX x = opti.variable();
    MX y = opti.variable();

    opti.minimize(  pow(y-pow(x,2),2)  );
    opti.subject_to( pow(x,2)+ pow(y,2) ==1 );
    opti.subject_to(       x+y>=1 );

    Dict opts_dict=Dict();   // to stop printing out solver data
    // opts_dict["jit"] = true;
    // opts_dict["compiler"] = "shell"; 

    opti.solver("ipopt", opts_dict);

    Function test = opti.to_function("test", {}, {x,y}); 


    OptiSol sol = opti.solve();

    //test.generate_out(); 

    std::cout << sol.value(x) << " --- "<< sol.value(y) << std::endl; 


*/


/*
    MX x = MX::sym("x",2); // Two states

    // Expression for ODE right-hand side
    MX z = 1-pow(x(1),2);
    MX rhs = vertcat(z*x(0)-x(1),x(0));

    MXDict ode;         // ODE declaration
    ode["x"]   = x;     // states
    ode["ode"] = rhs;   // right-hand side

    // Construct a Function that integrates over 4s
    Function F = integrator("F","cvodes",ode,{{"tf",4}});

    // Start from x=[0;1]
    DMDict res = F(DMDict{{"x0",std::vector<double>{0,1}}});

    // Sensitivity wrt initial state
    MXDict ress = F(MXDict{{"x0",x}});
    Function S("S",{x},{jacobian(ress["xf"],x)});
    std::cout << S(DM(std::vector<double>{0,1})) << std::endl;
*/

/*
    MX x = MX::sym("x",2);
    MX y = MX::sym("y",2);
    
    // Function f = Function("f", {x,y}, {sin(y)*x});

    MX g0 = sin(x+y);
    MX g1 = cos(x-y);

    Function g = Function("g", {x,y}, {g0,g1}); 
    Function G = rootfinder("G", "newton", g);

    
    std::cout << G << std::endl;

    //G.generate("gen.c");

    //std::cout<< "solution: " << F << std::endl;
*/
/*
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX z = MX::sym("z");
    MX f = pow(x,2)+100*pow(z,2);
    MX g = z+pow(1-x,2)-y;

    MXDict nlp;                 // NLP declaration
    nlp["x"]= vertcat(x,y,z);   // decision vars
    nlp["f"] = f;               // objective
    nlp["g"] = g;               // constraints

    // Create solver instance
    Function F = nlpsol("F","ipopt",nlp);

    // Solve the problem using a guess
    F(DMDict{{"x0",DM({2.5,3.0,0.75})},{"ubg",0},{"lbg",0}});
    return 0;
*/

/*
MX x = MX::sym("x",2); // Two states
MX p = MX::sym("p");   // Free parameter

// Expression for ODE right-hand side
MX z = 1-pow(x(1),2);
MX rhs = vertcat(z*x(0)-x(1)+2*tanh(p),x(0));

// ODE declaration with free parameter
MXDict ode = {{"x",x},{"p",p},{"ode",rhs}};

// Construct a Function that integrates over 1s
Function F = integrator("F","cvodes",ode,{{"tf",1}});

// Control vector
MX u = MX::sym("u",4,1);

x = DM(std::vector<double>{0,1});  // Initial state
for (int k=0;k<4;++k) {
  // Integrate 1s forward in time:
  // call integrator symbolically
  MXDict res = F({{"x0",x},{"p",u(k)}});
  x = res["xf"];
}

// NLP declaration
MXDict nlp = {{"x",u},{"f",dot(u,u)},{"g",x}};

// Solve using IPOPT
Function solver = nlpsol("solver","ipopt",nlp);
//Function solver = nlpsol("solver","sqpmethod",nlp);
DMDict res = solver(DMDict{{"x0",0.2},{"lbg",0},{"ubg",0}});
*/

}