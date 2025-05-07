#include "osqp_solver.hpp"
#include <iostream>
#include "calculation.hpp"
#include "osqp_interface/osqp_interface.h"
#include <glog/logging.h>

using namespace Eigen;
using namespace std;

bool OsqpItf::solveqp(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, const Eigen::VectorXd& c, const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, const Eigen::VectorXd& b, const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& d, const Eigen::VectorXd& f, const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x, const bool ignoreUnknownError, const bool verbose) {
    int nx = Q.rows();  // nx is the number of primal variables (x).
    // OOQPEI_ASSERT_GT(range_error, nx, 0, "Matrix Q has size 0.");
    x.setZero(nx);

    auto ccopy(c);
    auto Acopy(A);
    auto bcopy(b);
    auto dcopy(d);
    auto fcopy(f);
    auto Ccopy(C);

    auto Qcopy(Q);

    int n_cons = Acopy.rows() + Ccopy.rows();


    cout<< "osqp solve start" << nx << endl;
    MatrixXd A_dense,H_dense,C_dense;

    MatrixXd I_dense = MatrixXd::Identity(u.size(),u.size()); 
    
    A_dense = Eigen::MatrixXd(Acopy);
    H_dense = Eigen::MatrixXd(Qcopy);
    C_dense = Eigen::MatrixXd(Ccopy);
    // Horizontal concatenation of D and H
    //cout<< "osqp A_sparse" << Acopy << endl;
    //cout<< "osqp A_dense" << A_dense << endl;
    Eigen::MatrixXd cons_dense(A_dense.rows() + C_dense.rows(), C_dense.cols());
    // 使用逗号初始化器进行垂直拼接
    cons_dense << A_dense,  
              C_dense;  
    
    
    VectorXd cc(nx);
    std::vector<double> gradient(nx, 0.0);
    for(int i = 0; i< nx; i++){
        cc(i) = ccopy(i);
        gradient[i] = ccopy(i);
    }

 
    
    VectorXd ll(n_cons),uu(n_cons);
    
    ll <<  bcopy , dcopy;
    //bcopy = bcopy.array();
    uu <<  bcopy , fcopy;

    std::vector<double>lowerBound(n_cons, 0.0);
    std::vector<double>upperBound(n_cons, 0.0);

    for(int i = 0 ; i< n_cons; i++){
        lowerBound[i] = ll(i);
        upperBound[i] = uu(i);
    }
   

    cout<< "osqp solve start cons size ini" << n_cons<< endl;
    cout<< "osqp solve start cons size final" << cons_dense.rows()<< endl;
    //cout << "bound d - f" << ll - uu << endl;

    cout<< "osqp solve start b size" << bcopy.size() << endl;
    cout<< "osqp solve start d size" << dcopy.size() << endl;
    cout<< "osqp solve start l size" << cons_dense.size()/nx << endl;

    //SparseMatrix<double> hessian_sparse = H_dense.sparseView();
    SparseMatrix<double> constraint_sparse = cons_dense.sparseView();
    //SparseMatrix<double> constraint_sparse_s = C_dense.sparseView();
    Eigen::SparseMatrix<double> hessian_sparse = Qcopy;
    hessian_sparse.makeCompressed();
    constraint_sparse.makeCompressed();
    // vars->x->writefToStream( cout, "x[%{index}] = %{value}" );

    Eigen::MatrixXd dq_mat = Eigen::MatrixXd(H_dense);
        // 判断Q矩阵是否对称
        if (dq_mat == dq_mat.transpose()) {
            std::cout << "H_dense is symmetric" << std::endl;
        } else {
            std::cout << "H_dense is not symmetric" << std::endl;
        }
    // 判断Q矩阵是否正定
    bool is_psd = isPsd<Eigen::MatrixXd>(dq_mat);
    if (is_psd) {
        std::cout << "H_dense is psd" << std::endl;
    } else {
        std::cout << "H_dense is not psd" << std::endl;
    }

    
    // Create Solver
    osqp::OSQPInterface qp_solver_;
    qp_solver_.updateMaxIter(10000);
    qp_solver_.updateRhoInterval(0);  // 0 means automatic
    qp_solver_.updateEpsRel(1.0e-6);  // def: 1.0e-4
    qp_solver_.updateEpsAbs(1.0e-6);  // def: 1.0e-4
    qp_solver_.updateVerbose(false);

    // solve the QP problem
    const auto result = qp_solver_.optimize(H_dense, cons_dense, gradient, lowerBound, upperBound);

    const std::vector<double> optval = std::get<0>(result);
    const int state_qp = std::get<2>(result);
    const int value_qp = std::get<3>(result);


    std::cout << "we are here" << optval.size() << "  " << nx << std::endl;
    //std::cout << "we are here x old" << x << std::endl;
    for(int i =0; i<nx; i++){
        x(i) = optval[i];
    }
    std::cout << "we are here x new" << x << std::endl;


    if(value_qp != 1){
        std::cout << "QP OSQP STATUS IS" << value_qp << std::endl;
        LOG(INFO) << "trajectory generation osqp solver failed." ;
        return false;
    }
    else{
        std::cout << "QP OSQP STATUS IS" << value_qp << std::endl;
        LOG(INFO) << "trajectory generation osqp solver succeed." ;
        return true;
    }

    
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