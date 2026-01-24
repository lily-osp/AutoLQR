/**
 * Native test for Gauss-Jordan matrix inversion
 * 
 * Compile with: g++ -std=c++11 -o test_gauss_jordan test_gauss_jordan.cpp -lm
 * Run with: ./test_gauss_jordan
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Gauss-Jordan matrix inversion (copied from AutoLQR.cpp for testing)
bool invertMatrixGaussJordan(const float* matrix, float* result, int n) {
    if (!matrix || !result)
        return false;

    // Use closed-form solutions for small matrices
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

    // For matrices 4x4 and larger, use Gauss-Jordan elimination
    float* aug = new float[n * 2 * n];
    if (!aug)
        return false;
    
    // Initialize augmented matrix: [A|I]
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            aug[row * 2 * n + col] = matrix[row * n + col];
            aug[row * 2 * n + (col + n)] = (row == col) ? 1.0f : 0.0f;
        }
    }
    
    // Gauss-Jordan elimination with partial pivoting
    for (int col = 0; col < n; col++) {
        // Find pivot
        int maxRow = col;
        float maxVal = fabs(aug[col * 2 * n + col]);
        for (int row = col + 1; row < n; row++) {
            float val = fabs(aug[row * 2 * n + col]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = row;
            }
        }
        
        // Check for singular matrix
        if (maxVal < 1e-6f) {
            delete[] aug;
            return false;
        }
        
        // Swap rows if needed
        if (maxRow != col) {
            for (int k = 0; k < 2 * n; k++) {
                float tmp = aug[col * 2 * n + k];
                aug[col * 2 * n + k] = aug[maxRow * 2 * n + k];
                aug[maxRow * 2 * n + k] = tmp;
            }
        }
        
        // Scale pivot row
        float pivot = aug[col * 2 * n + col];
        for (int k = 0; k < 2 * n; k++) {
            aug[col * 2 * n + k] /= pivot;
        }
        
        // Eliminate column in all other rows
        for (int row = 0; row < n; row++) {
            if (row != col) {
                float factor = aug[row * 2 * n + col];
                for (int k = 0; k < 2 * n; k++) {
                    aug[row * 2 * n + k] -= factor * aug[col * 2 * n + k];
                }
            }
        }
    }
    
    // Extract inverse from right side
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            result[row * n + col] = aug[row * 2 * n + (col + n)];
        }
    }
    
    delete[] aug;
    return true;
}

void printMatrix(const float* M, int n, const char* name) {
    printf("%s =\n", name);
    for (int i = 0; i < n; i++) {
        printf("  [");
        for (int j = 0; j < n; j++) {
            printf("%10.6f", M[i * n + j]);
            if (j < n - 1) printf(", ");
        }
        printf("]\n");
    }
    printf("\n");
}

void matrixMultiply(const float* A, const float* B, float* C, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            C[i * n + j] = 0;
            for (int k = 0; k < n; k++) {
                C[i * n + j] += A[i * n + k] * B[k * n + j];
            }
        }
    }
}

bool checkIdentity(const float* M, int n, float tolerance) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            if (fabs(M[i * n + j] - expected) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

bool testMatrix(const float* A, int n, const char* testName) {
    printf("=== Test: %s (%dx%d) ===\n\n", testName, n, n);
    
    float* inv = new float[n * n];
    float* product = new float[n * n];
    
    printMatrix(A, n, "A");
    
    bool success = invertMatrixGaussJordan(A, inv, n);
    
    if (!success) {
        printf("FAILED: Matrix inversion failed (singular matrix?)\n\n");
        delete[] inv;
        delete[] product;
        return false;
    }
    
    printMatrix(inv, n, "A^(-1)");
    
    // Verify: A * A^(-1) should equal I
    matrixMultiply(A, inv, product, n);
    printMatrix(product, n, "A * A^(-1)");
    
    bool isIdentity = checkIdentity(product, n, 1e-4f);
    
    if (isIdentity) {
        printf("PASSED: A * A^(-1) = I (within tolerance)\n\n");
    } else {
        printf("FAILED: A * A^(-1) != I\n\n");
    }
    
    delete[] inv;
    delete[] product;
    return isIdentity;
}

int main() {
    printf("=========================================\n");
    printf("Gauss-Jordan Matrix Inversion Test\n");
    printf("=========================================\n\n");
    
    int passed = 0;
    int total = 0;
    
    // Test 1: 2x2 matrix (closed-form)
    {
        float A[] = {
            4, 7,
            2, 6
        };
        total++;
        if (testMatrix(A, 2, "2x2 matrix (closed-form)")) passed++;
    }
    
    // Test 2: 3x3 matrix (closed-form)
    {
        float A[] = {
            1, 2, 3,
            0, 1, 4,
            5, 6, 0
        };
        total++;
        if (testMatrix(A, 3, "3x3 matrix (closed-form)")) passed++;
    }
    
    // Test 3: 4x4 matrix (Gauss-Jordan)
    {
        float A[] = {
            4, 3, 2, 1,
            3, 4, 3, 2,
            2, 3, 4, 3,
            1, 2, 3, 4
        };
        total++;
        if (testMatrix(A, 4, "4x4 matrix (Gauss-Jordan)")) passed++;
    }
    
    // Test 4: 5x5 matrix (Gauss-Jordan)
    {
        float A[] = {
            5, 1, 2, 3, 4,
            1, 6, 2, 3, 4,
            2, 2, 7, 3, 4,
            3, 3, 3, 8, 4,
            4, 4, 4, 4, 9
        };
        total++;
        if (testMatrix(A, 5, "5x5 matrix (Gauss-Jordan)")) passed++;
    }
    
    // Test 5: 4x4 diagonal matrix
    {
        float A[] = {
            2, 0, 0, 0,
            0, 3, 0, 0,
            0, 0, 4, 0,
            0, 0, 0, 5
        };
        total++;
        if (testMatrix(A, 4, "4x4 diagonal matrix")) passed++;
    }
    
    // Test 6: 4x4 matrix with larger values (diagonally dominant)
    {
        float A[] = {
            10, 2, 3, 4,
            2, 20, 6, 7,
            3, 6, 30, 8,
            4, 7, 8, 40
        };
        total++;
        if (testMatrix(A, 4, "4x4 diagonally dominant")) passed++;
    }
    
    // Test 7: 6x6 matrix (larger Gauss-Jordan)
    {
        float A[] = {
            6, 1, 1, 1, 1, 1,
            1, 7, 1, 1, 1, 1,
            1, 1, 8, 1, 1, 1,
            1, 1, 1, 9, 1, 1,
            1, 1, 1, 1, 10, 1,
            1, 1, 1, 1, 1, 11
        };
        total++;
        if (testMatrix(A, 6, "6x6 matrix (Gauss-Jordan)")) passed++;
    }
    
    printf("=========================================\n");
    printf("Results: %d / %d tests passed\n", passed, total);
    printf("=========================================\n");
    
    return (passed == total) ? 0 : 1;
}
