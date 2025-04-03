#ifndef AUTO_LQR_H
#define AUTO_LQR_H

#include <Arduino.h>

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
     * @brief Compute optimal feedback gains
     * @return true if successful, false if computation fails
     */
    bool computeGains();

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
     * @brief Check if the system is controllable
     * @return true if controllable, false otherwise
     */
    bool isSystemControllable();

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
};

#endif
