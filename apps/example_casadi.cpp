#include <3d-soft-trunk/ControllerPCC.h>
#include <casadi/casadi.hpp>
using namespace casadi;
#include <casadi/core/optistack.hpp>


int main(){

    MX test  = MX::ones(3,1); 
    test = -test; 


    std::cout << test << std::endl; 
    std::cout << fabs(test) << std::endl; 

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

    opti.solver("ipopt");


    OptiSol sol = opti.solve();

    std::cout << sol.value(x) << " --- "<<sol.value(y) << std::endl; 


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


*/
    //std::cout<< "solution: " << F << std::endl;

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
DMDict res = solver(DMDict{{"x0",0.2},{"lbg",0},{"ubg",0}});
*/
}