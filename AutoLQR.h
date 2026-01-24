#ifndef AUTO_LQR_H
#define AUTO_LQR_H

#include <Arduino.h>

/**
 * @file AutoLQR.h
 * @brief Dynamic allocation LQR controller
 * 
 * This class uses heap allocation for flexibility in system sizing.
 * For memory-constrained systems or to avoid heap fragmentation,
 * consider using AutoLQRStatic<N,M> from AutoLQRStatic.h instead.
 * 
 * Features:
 *   - Supports arbitrary state size (stateSize)
 *   - Supports arbitrary control size (controlSize)
 *   - Matrix inversion uses closed-form for 1x1, 2x2, 3x3 (fast)
 *   - Matrix inversion uses Gauss-Jordan for 4x4 and larger
 *   - Controllability checking via rank computation
 * 
 * @see AutoLQRStatic for static allocation variant (limited to controlSize <= 3)
 */

class AutoLQR {
public:
    /**
     * @brief Construct a new AutoLQR controller
     * @param stateSize Number of state variables
     * @param controlSize Number of control inputs
     */
    AutoLQR(int stateSize, int controlSize);

    /**
     * @brief Destroy the AutoLQR controller and free memory
     */
    ~AutoLQR();

    /**
     * @brief Set the system state matrix A
     * @param A Pointer to state matrix (stateSize x stateSize)
     * @return true if successful, false otherwise
     */
    bool setStateMatrix(const float* A);

    /**
     * @brief Set the input matrix B
     * @param B Pointer to input matrix (stateSize x controlSize)
     * @return true if successful, false otherwise
     */
    bool setInputMatrix(const float* B);

    /**
     * @brief Set the cost matrices Q and R
     * @param Q Pointer to state cost matrix (stateSize x stateSize)
     * @param R Pointer to control cost matrix (controlSize x controlSize)
     * @return true if successful, false otherwise
     */
    bool setCostMatrices(const float* Q, const float* R);

    /**
     * @brief Compute optimal feedback gains using current solver parameters
     * @return true if successful, false if computation fails
     */
    bool computeGains();

    /**
     * @brief Compute optimal feedback gains with specified solver parameters
     * @param maxIterations Maximum number of DARE solver iterations
     * @param tolerance Convergence tolerance for DARE solver
     * @return true if successful, false if computation fails
     */
    bool computeGains(int maxIterations, float tolerance);

    /**
     * @brief Set solver parameters for DARE computation
     * @param maxIterations Maximum number of iterations (default: 100)
     * @param tolerance Convergence tolerance (default: 1e-6)
     */
    void setSolverParameters(int maxIterations, float tolerance);

    /**
     * @brief Get current maximum iterations setting
     * @return Maximum iterations value
     */
    int getMaxIterations() const;

    /**
     * @brief Get current tolerance setting
     * @return Tolerance value
     */
    float getTolerance() const;

    /**
     * @brief Update the controller with current state
     * @param currentState Pointer to current state vector (stateSize)
     */
    void updateState(const float* currentState);

    /**
     * @brief Calculate control inputs based on current state
     * @param controlOutput Pointer to control output vector (controlSize)
     */
    void calculateControl(float* controlOutput);

    /**
     * @brief Set pre-computed gain values
     * @param K Pointer to gain matrix (controlSize x stateSize)
     */
    void setGains(const float* K);

    /**
     * @brief Check if the system is controllable using full controllability matrix
     * @return true if controllable, false otherwise
     */
    bool isSystemControllable();

    /**
     * @brief Build and export the controllability matrix C = [B, AB, A²B, ..., A^(n-1)B]
     * @param C Pointer to output array (stateSize x stateSize*controlSize)
     * @return true if successful, false otherwise
     */
    bool getControllabilityMatrix(float* C);

    /**
     * @brief Get the rank of the controllability matrix
     * @return Rank of controllability matrix, or -1 if computation fails
     */
    int getControllabilityRank();

    /**
     * @brief Get the state size
     * @return Number of state variables
     */
    int getStateSize() const;

    /**
     * @brief Get the control size
     * @return Number of control inputs
     */
    int getControlSize() const;

    /**
     * @brief Get the solution of the Riccati equation
     * @return Pointer to the P matrix (stateSize x stateSize)
     */
    const float* getRicattiSolution() const;

    /**
     * @brief Estimate feedforward gain for steady-state tracking
     * @param ffGain Pointer to feedforward gain vector (controlSize)
     * @param desiredState Pointer to desired state vector (stateSize)
     */
    void estimateFeedforwardGain(float* ffGain, const float* desiredState);

    /**
     * @brief Estimate time to convergence
     * @param convergenceThreshold Threshold for considering system converged (default: 0.05)
     * @return Estimated time in seconds, or -1 if estimation fails
     */
    float estimateConvergenceTime(float convergenceThreshold = 0.05f);

    /**
     * @brief Export computed gains to external array
     * @param exportedK Pointer to destination array (controlSize x stateSize)
     * @return true if successful, false otherwise
     */
    bool exportGains(float* exportedK);

    /**
     * @brief Calculate expected cost from current state
     * @return Expected cost value, or -1 if calculation fails
     */
    float calculateExpectedCost();

private:
    int stateSize; ///< Number of state variables
    int controlSize; ///< Number of control inputs

    float* A; ///< State matrix
    float* B; ///< Input matrix
    float* Q; ///< State cost matrix
    float* R; ///< Control cost matrix
    float* K; ///< Control gain matrix
    float* state; ///< Current state
    float* P; ///< Riccati equation solution

    // Solver parameters
    int solverMaxIterations; ///< Maximum DARE solver iterations
    float solverTolerance; ///< DARE convergence tolerance

    // Helper functions

    /**
     * @brief Multiply two matrices
     * @param m1 First matrix
     * @param m2 Second matrix
     * @param result Result matrix
     * @param rows1 Rows in first matrix
     * @param cols1 Columns in first matrix / Rows in second matrix
     * @param cols2 Columns in second matrix
     */
    void matrixMultiply(const float* m1, const float* m2, float* result, int rows1, int cols1, int cols2);

    /**
     * @brief Add two matrices
     * @param m1 First matrix
     * @param m2 Second matrix
     * @param result Result matrix
     * @param rows Rows in matrices
     * @param cols Columns in matrices
     */
    void matrixAdd(const float* m1, const float* m2, float* result, int rows, int cols);

    /**
     * @brief Subtract second matrix from first
     * @param m1 First matrix
     * @param m2 Second matrix
     * @param result Result matrix
     * @param rows Rows in matrices
     * @param cols Columns in matrices
     */
    void matrixSubtract(const float* m1, const float* m2, float* result, int rows, int cols);

    /**
     * @brief Transpose a matrix
     * @param matrix Input matrix
     * @param result Transposed matrix
     * @param rows Rows in input matrix
     * @param cols Columns in input matrix
     */
    void transposeMatrix(const float* matrix, float* result, int rows, int cols);

    /**
     * @brief Invert a small matrix (1x1, 2x2, 3x3)
     * @param matrix Input matrix
     * @param result Inverted matrix
     * @param n Matrix size
     * @return true if successful, false if matrix is singular
     */
    bool invertMatrix(const float* matrix, float* result, int n);

    /**
     * @brief Compute the optimal gain matrix by solving DARE
     * @return true if successful, false otherwise
     */
    bool computeGainMatrix();

    /**
     * @brief Build the controllability matrix internally
     * @param C Pointer to output array (stateSize x stateSize*controlSize)
     */
    void buildControllabilityMatrix(float* C);

    /**
     * @brief Compute rank of a matrix using Gaussian elimination
     * @param matrix Input matrix
     * @param rows Number of rows
     * @param cols Number of columns
     * @return Rank of the matrix
     */
    int computeMatrixRank(float* matrix, int rows, int cols);

    /**
     * @brief Compute determinant of a square matrix (up to 4x4)
     * @param matrix Input matrix
     * @param n Matrix size
     * @return Determinant value
     */
    float computeDeterminant(const float* matrix, int n);
};

#endif
