#include "osqp_interface.hpp"
#include <iostream>

using namespace Eigen;
using namespace std;

bool OsqpItf::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, const Eigen::VectorXd& c, const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, const Eigen::VectorXd& b, const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& d, const Eigen::VectorXd& f, const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x, const bool ignoreUnknownError, const bool verbose) {
    int nx = Q.rows();  // nx is the number of primal variables (x).
    // OOQPEI_ASSERT_GT(range_error, nx, 0, "Matrix Q has size 0.");
    x.setZero(nx);


    MatrixXd A_dense,H_dense;
    int n_cons = A.cols() + C.cols() + u.size();

    MatrixXd I_dense = MatrixXd::Identity(u.size(),u.size()); 
 
    A_dense = MatrixXd(A);
    H_dense = MatrixXd(Q);
    // Horizontal concatenation of D and H
    MatrixXd cons_dense(A_dense.rows()+ C.rows()+ I_dense.rows(), C.cols());

    VectorXd cc;
    for(int i = 0; i< nx; i++){
        cc(i) = c(i);
    }
    
    VectorXd ll = b + d + l;
    VectorXd uu = b + f + u;

    SparseMatrix<double> hessian_sparse = H_dense.sparseView();
    SparseMatrix<double> constraint_sparse = cons_dense.sparseView();
    // vars->x->writefToStream( cout, "x[%{index}] = %{value}" );

    IOSQP qp_solver;
    qp_solver.setMats(hessian_sparse,cc,constraint_sparse,ll,uu,1.0e-8,1.0e-4);
    qp_solver.solve();

    int ret = qp_solver.getStatus();
    cout<< "osqp solve status is" << ret << endl;
    if(ret != 1){
        return false;
    }
    x = qp_solver.getPrimalSol();

}


void OsqpItf::printProblemFormulation(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, const Eigen::VectorXd& c, const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, const Eigen::VectorXd& b, const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& d, const Eigen::VectorXd& f, const Eigen::VectorXd& l, const Eigen::VectorXd& u) {
    cout << "-------------------------------" << endl;
    cout << "Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and l <= x <= u" << endl;
    cout << "Q (triangular) << " << endl << MatrixXd(Q) << endl;
    cout << "c << " << c.transpose() << endl;
    cout << "A << " << endl << MatrixXd(A) << endl;
    cout << "b << " << b.transpose() << endl;
    cout << "C << " << endl << MatrixXd(C) << endl;
    cout << "d << " << d.transpose() << endl;
    cout << "f << " << f.transpose() << endl;
    cout << "l << " << l.transpose() << endl;
    cout << "u << " << u.transpose() << endl;
}

void OsqpItf::printLimits(const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit, const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit, const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit) {
    cout << "useLowerLimit << " << std::boolalpha << useLowerLimit.cast<bool>().transpose() << endl;
    cout << "lowerLimit << " << lowerLimit.transpose() << endl;
    cout << "useUpperLimit << " << std::boolalpha << useUpperLimit.cast<bool>().transpose() << endl;
    cout << "upperLimit << " << upperLimit.transpose() << endl;
}

void OsqpItf::printSolution(const int status, const Eigen::VectorXd& x) {
    if (status == 0) {
        cout << "-------------------------------" << endl;
        cout << "SOLUTION" << endl;
        cout << "Ok, ended with status " << status << "." << endl;
        cout << "x << " << x.transpose() << endl;
    } else {
        cout << "-------------------------------" << endl;
        cout << "SOLUTION" << endl;
        cout << "Error, ended with status " << status << "." << endl;
    }
}