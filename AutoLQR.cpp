#include "AutoLQR.h"
#include <math.h>

AutoLQR::AutoLQR(int stateSize, int controlSize)
    : stateSize(stateSize), controlSize(controlSize) {
    A = new float[stateSize * stateSize];
    B = new float[stateSize * controlSize];
    Q = new float[stateSize * stateSize];
    R = new float[controlSize * controlSize];
    K = new float[controlSize * stateSize];
    state = new float[stateSize];
}

AutoLQR::~AutoLQR() {
    delete[] A;
    delete[] B;
    delete[] Q;
    delete[] R;
    delete[] K;
    delete[] state;
}

void AutoLQR::setStateMatrix(const float* inputA) {
    memcpy(A, inputA, sizeof(float) * stateSize * stateSize);
}

void AutoLQR::setInputMatrix(const float* inputB) {
    memcpy(B, inputB, sizeof(float) * stateSize * controlSize);
}

void AutoLQR::setCostMatrices(const float* inputQ, const float* inputR) {
    memcpy(Q, inputQ, sizeof(float) * stateSize * stateSize);
    memcpy(R, inputR, sizeof(float) * controlSize * controlSize);
}

void AutoLQR::computeGains() {
    computeGainMatrix();
}

void AutoLQR::updateState(const float* currentState) {
    memcpy(state, currentState, sizeof(float) * stateSize);
}

void AutoLQR::calculateControl(float* controlOutput) {
    for (int i = 0; i < controlSize; i++) {
        controlOutput[i] = 0;
        for (int j = 0; j < stateSize; j++) {
            controlOutput[i] -= K[i * stateSize + j] * state[j];
        }
    }
}

// Helper functions
void AutoLQR::matrixMultiply(const float* m1, const float* m2, float* result, int rows1, int cols1, int cols2) {
    for (int i = 0; i < rows1; i++) {
        for (int j = 0; j < cols2; j++) {
            result[i * cols2 + j] = 0;
            for (int k = 0; k < cols1; k++) {
                result[i * cols2 + j] += m1[i * cols1 + k] * m2[k * cols2 + j];
            }
        }
    }
}

void AutoLQR::transposeMatrix(const float* matrix, float* result, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result[j * rows + i] = matrix[i * cols + j];
        }
    }
}

void AutoLQR::computeGainMatrix() {
    // Simplified computation (not solving Riccati equation directly).
    // Use an iterative method or library for accurate results in practice.
    float* BT = new float[controlSize * stateSize];
    transposeMatrix(B, BT, stateSize, controlSize);

    float* temp1 = new float[controlSize * stateSize];
    float* temp2 = new float[controlSize * controlSize];
    float* temp3 = new float[controlSize * stateSize];

    // K = inv(R + B' * P * B) * (B' * P * A)
    // Approximate: K = inv(R) * B'

    matrixMultiply(BT, Q, temp1, controlSize, stateSize, stateSize);
    matrixMultiply(temp1, B, temp2, controlSize, stateSize, controlSize);

    for (int i = 0; i < controlSize * controlSize; i++) {
        temp2[i] += R[i];
    }

    for (int i = 0; i < controlSize * stateSize; i++) {
        temp3[i] = temp1[i];
    }

    for (int i = 0; i < controlSize * stateSize; i++) {
        K[i] = temp3[i] / temp2[i % controlSize];
    }

    delete[] BT;
    delete[] temp1;
    delete[] temp2;
    delete[] temp3;
}
