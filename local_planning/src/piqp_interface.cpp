#include "piqp_interface.hpp"
#include <glog/logging.h>
#include <chrono>

using namespace Eigen;
using namespace std;

bool PiQpltf::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, const Eigen::VectorXd& c, const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, const Eigen::VectorXd& b, const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& d, const Eigen::VectorXd& f, const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x, const bool ignoreUnknownError, const bool verbose) {
    int nx = Q.rows();  // nx is the number of primal variables (x).
    // OOQPEI_ASSERT_GT(range_error, nx, 0, "Matrix Q has size 0.");
    x.setZero(nx);

    // Make copies of variables that are changed.
    auto Qcopy(Q);
    auto ccopy(c);
    auto Acopy(A);
    auto bcopy(b);
    auto Ccopy(C);

    // Make sure Q is in lower triangular form (Q is symmetric).
    // Refer to OOQP user guide section 2.2 (p. 11).
    // TODO Check if Q is really symmetric.
    
    // Compress sparse Eigen matrices (refer to Eigen Sparse Matrix user manual).
    Qcopy.makeCompressed();
    Acopy.makeCompressed();
    Ccopy.makeCompressed();

    Eigen::SparseMatrix<double> Q_sparse = Qcopy;
    Eigen::SparseMatrix<double> A_sparse = Acopy;
    Eigen::SparseMatrix<double> C_sparse = Ccopy;

    Eigen::MatrixXd C_dense = Eigen::MatrixXd(Ccopy);
    Eigen::MatrixXd Q_dense = Eigen::MatrixXd(Qcopy);
    Eigen::MatrixXd A_dense = Eigen::MatrixXd(Acopy);
    // Horizontal concatenation of D and H
    //cout<< "osqp A_sparse" << Acopy << endl;
    //cout<< "osqp A_dense" << A_dense << endl;
    Eigen::MatrixXd cons_dense(C_dense.rows() + C_dense.rows(), C_dense.cols());
    // 使用逗号初始化器进行垂直拼接
    cons_dense << C_dense,  
              -C_dense;  
    Eigen::VectorXd h(2*Ccopy.rows());
    h << f, -d;

    Eigen::SparseMatrix<double> G = cons_dense.sparseView();
    G.makeCompressed();

    assert(Ccopy.rows() == d.size());
    assert(Ccopy.rows() == f.size());
    // Determine which limits are active and which are not.
    // Setting up OOQP solver
    // Refer to OOQP user guide section 2.3 (p. 14).

    /*
    piqp::SparseSolver<double> solver;
    solver.settings().verbose = false;
    solver.settings().compute_timings = false;
    solver.settings().eps_abs = 1e-4;
    solver.settings().eps_rel = 1e-5;
    solver.setup(Q_sparse, ccopy, A_sparse, bcopy, G, h, l, u);
    */
    piqp::DenseSolver<double> solver;
    solver.settings().verbose = false;
    solver.settings().compute_timings = true;
    solver.settings().eps_abs = 1e-4;
    solver.settings().eps_rel = 1e-5;
    solver.setup(Q_dense, c, A_dense, b, cons_dense, h, l, u);

    auto start_time = std::chrono::high_resolution_clock::now();
    piqp::Status status = solver.solve();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    LOG(INFO) << "PIQP solver execution time without eigen process: " << duration << " ms";

    std::cout << "status = " << status << std::endl;
    std::cout << "x = " << solver.result().x.transpose() << std::endl;

    x = solver.result().x;

    LOG(INFO) << "trajectory generation PIQP solver statue is" << status;
    if(status != 1){
        std::cout << "QP PIQP STATUS IS" << status << std::endl;
        LOG(INFO) << "trajectory generation PIQP solver failed." ;
        return false;
    }
    else{
        std::cout << "QP PIQP STATUS IS" << status << std::endl;
        LOG(INFO) << "trajectory generation PIQP solver succeed." ;
        return true;
    }

    
    // Initialize new problem formulation.
    
}



void PiQpltf::printProblemFormulation(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, const Eigen::VectorXd& c, const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, const Eigen::VectorXd& b, const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& d, const Eigen::VectorXd& f, const Eigen::VectorXd& l, const Eigen::VectorXd& u) {
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

void PiQpltf::printLimits(const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit, const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit, const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit) {
    cout << "useLowerLimit << " << std::boolalpha << useLowerLimit.cast<bool>().transpose() << endl;
    cout << "lowerLimit << " << lowerLimit.transpose() << endl;
    cout << "useUpperLimit << " << std::boolalpha << useUpperLimit.cast<bool>().transpose() << endl;
    cout << "upperLimit << " << upperLimit.transpose() << endl;
}

void PiQpltf::printSolution(const int status, const Eigen::VectorXd& x) {
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