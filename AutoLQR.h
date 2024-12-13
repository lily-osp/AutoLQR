#ifndef AUTO_LQR_H
#define AUTO_LQR_H

#include <Arduino.h>

class AutoLQR {
public:
    AutoLQR(int stateSize, int controlSize);
    ~AutoLQR();

    void setStateMatrix(const float* A);
    void setInputMatrix(const float* B);
    void setCostMatrices(const float* Q, const float* R);
    void computeGains();
    void updateState(const float* currentState);
    void calculateControl(float* controlOutput);

private:
    int stateSize;
    int controlSize;

    float* A; // State matrix
    float* B; // Input matrix
    float* Q; // State cost matrix
    float* R; // Control cost matrix
    float* K; // Control gain matrix
    float* state; // Current state

    void matrixMultiply(const float* m1, const float* m2, float* result, int rows1, int cols1, int cols2);
    void transposeMatrix(const float* matrix, float* result, int rows, int cols);
    void computeGainMatrix();
};

#endif
