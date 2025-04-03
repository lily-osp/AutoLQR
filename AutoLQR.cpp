#include "AutoLQR.h"
#include <math.h>

AutoLQR::AutoLQR(int stateSize, int controlSize)
    : stateSize(stateSize)
    , controlSize(controlSize)
    , A(nullptr)
    , B(nullptr)
    , Q(nullptr)
    , R(nullptr)
    , K(nullptr)
    , state(nullptr)
    , P(nullptr)
{
    if (stateSize > 0 && controlSize > 0) {
        A = new float[stateSize * stateSize]();
        B = new float[stateSize * controlSize]();
        Q = new float[stateSize * stateSize]();
        R = new float[controlSize * controlSize]();
        K = new float[controlSize * stateSize]();
        state = new float[stateSize]();
        P = new float[stateSize * stateSize](); // Store Riccati solution
    }
}

AutoLQR::~AutoLQR()
{
    delete[] A;
    delete[] B;
    delete[] Q;
    delete[] R;
    delete[] K;
    delete[] state;
    delete[] P;
}

bool AutoLQR::setStateMatrix(const float* inputA)
{
    if (!inputA || !A)
        return false;
    memcpy(A, inputA, sizeof(float) * stateSize * stateSize);
    return true;
}

bool AutoLQR::setInputMatrix(const float* inputB)
{
    if (!inputB || !B)
        return false;
    memcpy(B, inputB, sizeof(float) * stateSize * controlSize);
    return true;
}

bool AutoLQR::setCostMatrices(const float* inputQ, const float* inputR)
{
    if (!inputQ || !inputR || !Q || !R)
        return false;
    memcpy(Q, inputQ, sizeof(float) * stateSize * stateSize);
    memcpy(R, inputR, sizeof(float) * controlSize * controlSize);
    return true;
}

void AutoLQR::setGains(const float* inputK)
{
    if (!inputK || !K)
        return;
    memcpy(K, inputK, sizeof(float) * controlSize * stateSize);
}

bool AutoLQR::computeGains()
{
    return computeGainMatrix();
}

void AutoLQR::updateState(const float* currentState)
{
    if (!currentState || !state)
        return;
    memcpy(state, currentState, sizeof(float) * stateSize);
}

void AutoLQR::calculateControl(float* controlOutput)
{
    if (!controlOutput || !K || !state)
        return;

    // Initialize control outputs to zero
    for (int i = 0; i < controlSize; i++) {
        controlOutput[i] = 0;
    }

    // u = -K·x
    for (int i = 0; i < controlSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            controlOutput[i] -= K[i * stateSize + j] * state[j];
        }
    }
}

void AutoLQR::matrixMultiply(const float* m1, const float* m2, float* result, int rows1, int cols1, int cols2)
{
    if (!m1 || !m2 || !result)
        return;

    // Initialize result matrix to zero
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

void AutoLQR::matrixAdd(const float* m1, const float* m2, float* result, int rows, int cols)
{
    if (!m1 || !m2 || !result)
        return;

    for (int i = 0; i < rows * cols; i++) {
        result[i] = m1[i] + m2[i];
    }
}

void AutoLQR::matrixSubtract(const float* m1, const float* m2, float* result, int rows, int cols)
{
    if (!m1 || !m2 || !result)
        return;

    for (int i = 0; i < rows * cols; i++) {
        result[i] = m1[i] - m2[i];
    }
}

void AutoLQR::transposeMatrix(const float* matrix, float* result, int rows, int cols)
{
    if (!matrix || !result)
        return;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result[j * rows + i] = matrix[i * cols + j];
        }
    }
}

// Matrix inversion for small matrices (1x1, 2x2, 3x3)
bool AutoLQR::invertMatrix(const float* matrix, float* result, int n)
{
    if (!matrix || !result)
        return false;

    if (n == 1) {
        if (fabs(matrix[0]) < 1e-6)
            return false;
        result[0] = 1.0f / matrix[0];
        return true;
    } else if (n == 2) {
        float det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
        if (fabs(det) < 1e-6)
            return false;
        float invDet = 1.0f / det;
        result[0] = matrix[3] * invDet;
        result[1] = -matrix[1] * invDet;
        result[2] = -matrix[2] * invDet;
        result[3] = matrix[0] * invDet;
        return true;
    } else if (n == 3) {
        // 3x3 Matrix inversion
        float a = matrix[0], b = matrix[1], c = matrix[2];
        float d = matrix[3], e = matrix[4], f = matrix[5];
        float g = matrix[6], h = matrix[7], i = matrix[8];

        float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        if (fabs(det) < 1e-6)
            return false;

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

    // For larger matrices, use a different method or precompute
    return false;
}

bool AutoLQR::isSystemControllable()
{
    // Basic controllability check for 2x2 systems
    if (stateSize == 2 && controlSize == 1) {
        float det = B[0] * A[1] - B[1] * A[0];
        return fabs(det) > 1e-6;
    }

    // For larger systems, implement a more sophisticated controllability check
    // or return true and let the DARE solver determine feasibility
    return true;
}

const float* AutoLQR::getRicattiSolution() const
{
    return P;
}

bool AutoLQR::computeGainMatrix()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    // Check if system is controllable
    if (!isSystemControllable()) {
        return false;
    }

    // Iterative DARE solver for small systems
    float* P_next = new float[stateSize * stateSize]();
    float* BT = new float[controlSize * stateSize]();
    float* AT = new float[stateSize * stateSize]();
    float* temp1 = new float[controlSize * stateSize]();
    float* temp2 = new float[controlSize * controlSize]();
    float* temp3 = new float[controlSize * stateSize]();
    float* temp4 = new float[stateSize * stateSize]();
    float* temp5 = new float[stateSize * stateSize]();

    // Initialize P as Q
    memcpy(P, Q, sizeof(float) * stateSize * stateSize);

    // Compute B transpose and A transpose
    transposeMatrix(B, BT, stateSize, controlSize);
    transposeMatrix(A, AT, stateSize, stateSize);

    // Iterate to solve DARE: P = A'PA - A'PB(R + B'PB)^(-1)B'PA + Q
    const int maxIterations = 100; // Increased from 50
    const float tolerance = 1e-6; // Tighter tolerance

    for (int iter = 0; iter < maxIterations; iter++) {
        // temp1 = B'P
        matrixMultiply(BT, P, temp1, controlSize, stateSize, stateSize);

        // temp2 = B'PB
        matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);

        // temp2 = R + B'PB
        for (int i = 0; i < controlSize * controlSize; i++) {
            temp2[i] += R[i];
        }

        // Invert (R + B'PB)
        if (!invertMatrix(temp2, temp2, controlSize)) {
            // Clean up memory
            delete[] P_next;
            delete[] BT;
            delete[] AT;
            delete[] temp1;
            delete[] temp2;
            delete[] temp3;
            delete[] temp4;
            delete[] temp5;
            return false;
        }

        // temp3 = (R + B'PB)^(-1) * B'P
        matrixMultiply(temp2, temp1, temp3, controlSize, controlSize, stateSize);

        // temp4 = A'PA
        matrixMultiply(AT, P, temp4, stateSize, stateSize, stateSize);
        matrixMultiply(temp4, A, P_next, stateSize, stateSize, stateSize);

        // temp5 = A'PB
        matrixMultiply(AT, P, temp4, stateSize, stateSize, stateSize);
        matrixMultiply(temp4, B, temp5, stateSize, stateSize, controlSize);

        // temp4 = A'PB * (R + B'PB)^(-1) * B'PA
        matrixMultiply(temp5, temp3, temp4, stateSize, controlSize, stateSize);
        matrixMultiply(temp4, A, temp5, stateSize, stateSize, stateSize);

        // P_next = A'PA - A'PB * (R + B'PB)^(-1) * B'PA + Q
        for (int i = 0; i < stateSize * stateSize; i++) {
            P_next[i] = P_next[i] - temp5[i] + Q[i];
        }

        // Check convergence
        float diff = 0;
        for (int i = 0; i < stateSize * stateSize; i++) {
            diff += fabs(P_next[i] - P[i]);
        }

        // Update P for next iteration
        memcpy(P, P_next, sizeof(float) * stateSize * stateSize);

        if (diff < tolerance) {
            break; // Converged
        }
    }

    // Compute K = (R + B'PB)^(-1) * B'PA
    matrixMultiply(BT, P, temp1, controlSize, stateSize, stateSize);
    matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);

    for (int i = 0; i < controlSize * controlSize; i++) {
        temp2[i] += R[i];
    }

    if (!invertMatrix(temp2, temp2, controlSize)) {
        // Clean up memory
        delete[] P_next;
        delete[] BT;
        delete[] AT;
        delete[] temp1;
        delete[] temp2;
        delete[] temp3;
        delete[] temp4;
        delete[] temp5;
        return false;
    }

    matrixMultiply(temp1, A, temp3, controlSize, stateSize, stateSize);
    matrixMultiply(temp2, temp3, K, controlSize, controlSize, stateSize);

    // Clean up memory
    delete[] P_next;
    delete[] BT;
    delete[] AT;
    delete[] temp1;
    delete[] temp2;
    delete[] temp3;
    delete[] temp4;
    delete[] temp5;

    return true;
}

void AutoLQR::estimateFeedforwardGain(float* ffGain, const float* desiredState)
{
    if (!ffGain || !desiredState || !A || !B || !K)
        return;

    // For steady-state tracking: u_ff = -(inv(B'B) * B' * (A-I)) * x_desired
    // This is simplified for common cases

    if (stateSize == 2 && controlSize == 1) {
        // Special case for position-velocity systems
        float Bsq = B[0] * B[0] + B[1] * B[1];
        if (Bsq < 1e-6)
            return;

        float invBsq = 1.0f / Bsq;

        // Compute (A-I) * x_desired
        float dx[2];
        dx[0] = (A[0] - 1.0f) * desiredState[0] + A[1] * desiredState[1];
        dx[1] = A[2] * desiredState[0] + (A[3] - 1.0f) * desiredState[1];

        // Compute feedforward gain
        ffGain[0] = -invBsq * (B[0] * dx[0] + B[1] * dx[1]);
    } else {
        // For other systems, initialize to zero
        for (int i = 0; i < controlSize; i++) {
            ffGain[i] = 0;
        }
    }
}

float AutoLQR::estimateConvergenceTime(float convergenceThreshold)
{
    // Estimate convergence time based on eigenvalues
    // This is a simplified estimate for 2x2 systems

    if (stateSize == 2) {
        // Compute closed-loop dynamics: A - B*K
        float CL[4];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                CL[i * 2 + j] = A[i * 2 + j];
                for (int k = 0; k < controlSize; k++) {
                    CL[i * 2 + j] -= B[i * controlSize + k] * K[k * stateSize + j];
                }
            }
        }

        // Approximate dominant eigenvalue using trace and determinant
        float trace = CL[0] + CL[3];
        float det = CL[0] * CL[3] - CL[1] * CL[2];

        // Characteristic equation: λ² - trace·λ + det = 0
        float discriminant = trace * trace - 4 * det;

        if (discriminant >= 0) {
            // Real eigenvalues
            float lambda1 = (trace + sqrt(discriminant)) / 2;
            float lambda2 = (trace - sqrt(discriminant)) / 2;

            // Dominant eigenvalue (larger magnitude)
            float domEigenvalue = (fabs(lambda1) > fabs(lambda2)) ? lambda1 : lambda2;

            if (fabs(domEigenvalue) < 1.0f && fabs(domEigenvalue) > 0.0f) {
                // Estimate time constant
                float timeConstant = -1.0f / log(fabs(domEigenvalue));

                // Time to reach convergenceThreshold
                return timeConstant * log(1.0f / convergenceThreshold);
            }
        }
    }

    // Default value if calculation fails
    return -1.0f;
}

bool AutoLQR::exportGains(float* exportedK)
{
    if (!K || !exportedK)
        return false;

    memcpy(exportedK, K, sizeof(float) * controlSize * stateSize);
    return true;
}

float AutoLQR::calculateExpectedCost()
{
    if (!P || !state)
        return -1.0f;

    // Cost = x'Px
    float cost = 0;
    for (int i = 0; i < stateSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            cost += state[i] * P[i * stateSize + j] * state[j];
        }
    }

    return cost;
}
