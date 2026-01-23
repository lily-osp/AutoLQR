#ifndef AUTO_LQR_STATIC_H
#define AUTO_LQR_STATIC_H

#include <Arduino.h>
#include <math.h>

/**
 * @brief Static allocation version of AutoLQR controller
 * 
 * This templated class provides the same functionality as AutoLQR but uses
 * compile-time fixed arrays instead of dynamic memory allocation. This is
 * preferred for memory-constrained embedded systems to avoid heap fragmentation.
 * 
 * @tparam N State size (number of state variables)
 * @tparam M Control size (number of control inputs)
 * 
 * Requirements:
 *   - N >= 1 (at least one state)
 *   - M >= 1 (at least one control input)
 *   - M <= 3 (matrix inversion limited to 3x3)
 * 
 * Memory usage (approximate):
 *   - Total: (2*N*N + 2*N*M + M*M + N + M*N) * sizeof(float) bytes
 *   - For N=2, M=1: ~52 bytes
 *   - For N=4, M=1: ~148 bytes
 *   - For N=4, M=2: ~180 bytes
 */
template<int N, int M>
class AutoLQRStatic {
    static_assert(N >= 1, "State size must be at least 1");
    static_assert(M >= 1, "Control size must be at least 1");
    static_assert(M <= 3, "Control size limited to 3 (matrix inversion constraint)");

public:
    /**
     * @brief Construct a new AutoLQRStatic controller
     * All matrices are zero-initialized
     */
    AutoLQRStatic() {
        // Zero-initialize all arrays
        memset(A, 0, sizeof(A));
        memset(B, 0, sizeof(B));
        memset(Q, 0, sizeof(Q));
        memset(R, 0, sizeof(R));
        memset(K, 0, sizeof(K));
        memset(state, 0, sizeof(state));
        memset(P, 0, sizeof(P));
        gainsComputed = false;
    }

    /**
     * @brief Get state size
     * @return State size N
     */
    constexpr int getStateSize() const { return N; }

    /**
     * @brief Get control size
     * @return Control size M
     */
    constexpr int getControlSize() const { return M; }

    /**
     * @brief Set the system state matrix A
     * @param inputA Pointer to state matrix (N x N)
     * @return true if successful
     */
    bool setStateMatrix(const float* inputA) {
        if (!inputA) return false;
        memcpy(A, inputA, sizeof(float) * N * N);
        gainsComputed = false;
        return true;
    }

    /**
     * @brief Set the input matrix B
     * @param inputB Pointer to input matrix (N x M)
     * @return true if successful
     */
    bool setInputMatrix(const float* inputB) {
        if (!inputB) return false;
        memcpy(B, inputB, sizeof(float) * N * M);
        gainsComputed = false;
        return true;
    }

    /**
     * @brief Set the cost matrices Q and R
     * @param inputQ Pointer to state cost matrix (N x N)
     * @param inputR Pointer to control cost matrix (M x M)
     * @return true if successful
     */
    bool setCostMatrices(const float* inputQ, const float* inputR) {
        if (!inputQ || !inputR) return false;
        memcpy(Q, inputQ, sizeof(float) * N * N);
        memcpy(R, inputR, sizeof(float) * M * M);
        gainsComputed = false;
        return true;
    }

    /**
     * @brief Compute optimal feedback gains
     * @param maxIterations Maximum DARE solver iterations (default: 100)
     * @param tolerance Convergence tolerance (default: 1e-6)
     * @return true if successful, false if computation fails
     */
    bool computeGains(int maxIterations = 100, float tolerance = 1e-6f) {
        gainsComputed = computeGainMatrix(maxIterations, tolerance);
        return gainsComputed;
    }

    /**
     * @brief Update the controller with current state
     * @param currentState Pointer to current state vector (N)
     */
    void updateState(const float* currentState) {
        if (!currentState) return;
        memcpy(state, currentState, sizeof(float) * N);
    }

    /**
     * @brief Calculate control inputs based on current state
     * @param controlOutput Pointer to control output vector (M)
     */
    void calculateControl(float* controlOutput) {
        if (!controlOutput) return;

        // Initialize control outputs to zero
        for (int i = 0; i < M; i++) {
            controlOutput[i] = 0;
        }

        // u = -K * x
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                controlOutput[i] -= K[i * N + j] * state[j];
            }
        }
    }

    /**
     * @brief Set pre-computed gain values
     * @param inputK Pointer to gain matrix (M x N)
     */
    void setGains(const float* inputK) {
        if (!inputK) return;
        memcpy(K, inputK, sizeof(float) * M * N);
        gainsComputed = true;
    }

    /**
     * @brief Check if the system is controllable
     * @return true if controllable, false otherwise
     */
    bool isSystemControllable() {
        // Full controllability matrix check for small systems
        if (N == 2 && M == 1) {
            // Controllability matrix [B, AB]
            // Check rank by determinant
            float AB[2];
            AB[0] = A[0] * B[0] + A[1] * B[1];
            AB[1] = A[2] * B[0] + A[3] * B[1];
            
            float det = B[0] * AB[1] - B[1] * AB[0];
            return fabs(det) > 1e-6f;
        }
        
        // For other systems, return true and let DARE solver determine feasibility
        return true;
    }

    /**
     * @brief Check if gains have been computed
     * @return true if gains are ready
     */
    bool hasValidGains() const {
        return gainsComputed;
    }

    /**
     * @brief Get the solution of the Riccati equation
     * @return Pointer to the P matrix (N x N)
     */
    const float* getRiccatiSolution() const {
        return P;
    }

    /**
     * @brief Estimate feedforward gain for steady-state tracking
     * @param ffGain Pointer to feedforward gain vector (M)
     * @param desiredState Pointer to desired state vector (N)
     */
    void estimateFeedforwardGain(float* ffGain, const float* desiredState) {
        if (!ffGain || !desiredState) return;

        if (N == 2 && M == 1) {
            // Special case for position-velocity systems
            float Bsq = B[0] * B[0] + B[1] * B[1];
            if (Bsq < 1e-6f) {
                ffGain[0] = 0;
                return;
            }

            float invBsq = 1.0f / Bsq;

            // Compute (A-I) * x_desired
            float dx[2];
            dx[0] = (A[0] - 1.0f) * desiredState[0] + A[1] * desiredState[1];
            dx[1] = A[2] * desiredState[0] + (A[3] - 1.0f) * desiredState[1];

            ffGain[0] = -invBsq * (B[0] * dx[0] + B[1] * dx[1]);
        } else {
            // For other systems, initialize to zero
            for (int i = 0; i < M; i++) {
                ffGain[i] = 0;
            }
        }
    }

    /**
     * @brief Estimate time to convergence
     * @param convergenceThreshold Threshold for considering system converged
     * @return Estimated time in time steps, or -1 if estimation fails
     */
    float estimateConvergenceTime(float convergenceThreshold = 0.05f) {
        if (N == 2) {
            // Compute closed-loop dynamics: A - B*K
            float CL[4];
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                    CL[i * 2 + j] = A[i * 2 + j];
                    for (int k = 0; k < M; k++) {
                        CL[i * 2 + j] -= B[i * M + k] * K[k * N + j];
                    }
                }
            }

            // Approximate dominant eigenvalue using trace and determinant
            float trace = CL[0] + CL[3];
            float det = CL[0] * CL[3] - CL[1] * CL[2];

            float discriminant = trace * trace - 4.0f * det;

            if (discriminant >= 0) {
                float lambda1 = (trace + sqrt(discriminant)) / 2.0f;
                float lambda2 = (trace - sqrt(discriminant)) / 2.0f;

                float domEigenvalue = (fabs(lambda1) > fabs(lambda2)) ? lambda1 : lambda2;

                if (fabs(domEigenvalue) < 1.0f && fabs(domEigenvalue) > 0.0f) {
                    float timeConstant = -1.0f / log(fabs(domEigenvalue));
                    return timeConstant * log(1.0f / convergenceThreshold);
                }
            }
        }

        return -1.0f;
    }

    /**
     * @brief Export computed gains to external array
     * @param exportedK Pointer to destination array (M x N)
     * @return true if successful
     */
    bool exportGains(float* exportedK) {
        if (!exportedK) return false;
        memcpy(exportedK, K, sizeof(float) * M * N);
        return true;
    }

    /**
     * @brief Calculate expected cost from current state
     * @return Expected cost value, or -1 if calculation fails
     */
    float calculateExpectedCost() {
        if (!gainsComputed) return -1.0f;

        // Cost = x'Px
        float cost = 0;
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                cost += state[i] * P[i * N + j] * state[j];
            }
        }

        return cost;
    }

private:
    // Fixed-size arrays (no heap allocation)
    float A[N * N];         ///< State matrix
    float B[N * M];         ///< Input matrix
    float Q[N * N];         ///< State cost matrix
    float R[M * M];         ///< Control cost matrix
    float K[M * N];         ///< Control gain matrix
    float state[N];         ///< Current state
    float P[N * N];         ///< Riccati equation solution
    bool gainsComputed;     ///< Flag indicating valid gains

    // Temporary buffers for DARE computation
    // Using member arrays to avoid stack overflow in computeGainMatrix

    /**
     * @brief Multiply two matrices
     */
    void matrixMultiply(const float* m1, const float* m2, float* result, 
                        int rows1, int cols1, int cols2) {
        for (int i = 0; i < rows1 * cols2; i++) {
            result[i] = 0;
        }

        for (int i = 0; i < rows1; i++) {
            for (int j = 0; j < cols2; j++) {
                for (int k = 0; k < cols1; k++) {
                    result[i * cols2 + j] += m1[i * cols1 + k] * m2[k * cols2 + j];
                }
            }
        }
    }

    /**
     * @brief Transpose a matrix
     */
    void transposeMatrix(const float* matrix, float* result, int rows, int cols) {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[j * rows + i] = matrix[i * cols + j];
            }
        }
    }

    /**
     * @brief Invert a small matrix (1x1, 2x2, 3x3)
     */
    bool invertMatrix(const float* matrix, float* result, int n) {
        if (n == 1) {
            if (fabs(matrix[0]) < 1e-6f) return false;
            result[0] = 1.0f / matrix[0];
            return true;
        } else if (n == 2) {
            float det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
            if (fabs(det) < 1e-6f) return false;
            float invDet = 1.0f / det;
            result[0] = matrix[3] * invDet;
            result[1] = -matrix[1] * invDet;
            result[2] = -matrix[2] * invDet;
            result[3] = matrix[0] * invDet;
            return true;
        } else if (n == 3) {
            float a = matrix[0], b = matrix[1], c = matrix[2];
            float d = matrix[3], e = matrix[4], f = matrix[5];
            float g = matrix[6], h = matrix[7], i = matrix[8];

            float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
            if (fabs(det) < 1e-6f) return false;

            float invDet = 1.0f / det;

            result[0] = (e * i - f * h) * invDet;
            result[1] = (c * h - b * i) * invDet;
            result[2] = (b * f - c * e) * invDet;
            result[3] = (f * g - d * i) * invDet;
            result[4] = (a * i - c * g) * invDet;
            result[5] = (c * d - a * f) * invDet;
            result[6] = (d * h - e * g) * invDet;
            result[7] = (b * g - a * h) * invDet;
            result[8] = (a * e - b * d) * invDet;

            return true;
        }

        return false;
    }

    /**
     * @brief Compute the optimal gain matrix by solving DARE
     */
    bool computeGainMatrix(int maxIterations, float tolerance) {
        if (!isSystemControllable()) {
            return false;
        }

        // Temporary buffers - sized for maximum expected operations
        float P_next[N * N];
        float BT[M * N];
        float AT[N * N];
        float temp1[M * N];
        float temp2[M * M];
        float temp3[M * N];
        float temp4[N * N];
        float temp5[N * N];

        // Initialize P as Q
        memcpy(P, Q, sizeof(float) * N * N);

        // Compute B transpose and A transpose
        transposeMatrix(B, BT, N, M);
        transposeMatrix(A, AT, N, N);

        // Iterate to solve DARE
        for (int iter = 0; iter < maxIterations; iter++) {
            // temp1 = B'P
            matrixMultiply(BT, P, temp1, M, N, N);

            // temp2 = B'PB
            matrixMultiply(temp1, B, temp2, M, N, M);

            // temp2 = R + B'PB
            for (int i = 0; i < M * M; i++) {
                temp2[i] += R[i];
            }

            // Invert (R + B'PB)
            if (!invertMatrix(temp2, temp2, M)) {
                return false;
            }

            // temp3 = (R + B'PB)^(-1) * B'P
            matrixMultiply(temp2, temp1, temp3, M, M, N);

            // temp4 = A'PA
            matrixMultiply(AT, P, temp4, N, N, N);
            matrixMultiply(temp4, A, P_next, N, N, N);

            // temp5 = A'PB
            matrixMultiply(AT, P, temp4, N, N, N);
            matrixMultiply(temp4, B, temp5, N, N, M);

            // temp4 = A'PB * (R + B'PB)^(-1) * B'PA
            float temp6[N * N];
            matrixMultiply(temp5, temp3, temp4, N, M, N);
            matrixMultiply(temp4, A, temp6, N, N, N);

            // P_next = A'PA - A'PB * (R + B'PB)^(-1) * B'PA + Q
            for (int i = 0; i < N * N; i++) {
                P_next[i] = P_next[i] - temp6[i] + Q[i];
            }

            // Check convergence
            float diff = 0;
            for (int i = 0; i < N * N; i++) {
                diff += fabs(P_next[i] - P[i]);
            }

            // Update P for next iteration
            memcpy(P, P_next, sizeof(float) * N * N);

            if (diff < tolerance) {
                break;
            }
        }

        // Compute K = (R + B'PB)^(-1) * B'PA
        float BTP[M * N];
        matrixMultiply(BT, P, BTP, M, N, N);
        
        float BTPB[M * M];
        matrixMultiply(BTP, B, BTPB, M, N, M);

        for (int i = 0; i < M * M; i++) {
            BTPB[i] += R[i];
        }

        if (!invertMatrix(BTPB, temp2, M)) {
            return false;
        }

        float BTPA[M * N];
        matrixMultiply(BTP, A, BTPA, M, N, N);
        matrixMultiply(temp2, BTPA, K, M, M, N);

        return true;
    }
};

#endif // AUTO_LQR_STATIC_H
