/*�ļ���:matprocess*/
/*���ܣ���������*/

/*����˵����ÿ�־�����㺯���������ṹ���ʽ�����أ�����Խṹ���˽ⲻ���ͬѧ���Բ��������*/
#include<iostream>
using namespace std;

//����ṹ��
typedef struct {
    int col;
    int row;
    double* mat;
}Mat;

// ��ӡ����������
void printMatrix(double* matrix, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            printf("%lf ", matrix[i * cols + j]);
        }
        printf("\n");
    }
}
//���غ���
void printMatrix(Mat A) {
    for (int i = 0; i < A.row; i++) {
        for (int j = 0; j < A.col; j++) {
            printf("%lf ", A.mat[i * A.col + j]);
        }
        printf("\n");
    }
}

// ��������
void swapRows(double* matrix, int row1, int row2, int cols) {
    for (int j = 0; j < cols; j++) {
        double temp = matrix[row1 * cols + j];
        matrix[row1 * cols + j] = matrix[row2 * cols + j];
        matrix[row2 * cols + j] = temp;
    }
}

// �������漰������
void inverseMatrix(double* matrix, int size, double* result) {
    // ������λ��
    double* identity = (double*)malloc(size * size * sizeof(double));
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (i == j) {
                identity[i * size + j] = 1.0;
            }
            else {
                identity[i * size + j] = 0.0;
            }
        }
    }

    // ����ԭʼ����
    double* copy = (double*)malloc(size * size * sizeof(double));
    for (int i = 0; i < size * size; i++) {
        copy[i] = matrix[i];
    }

    // ��˹-Լ����Ԫ��
    for (int i = 0; i < size; i++) {
        if (copy[i * size + i] == 0.0) {
            // ����Խ���Ԫ��Ϊ0���򽻻���
            int j = i + 1;
            while (j < size && copy[j * size + i] == 0.0) {
                j++;
            }
            if (j == size) {
                printf("���󲻿��棡\n");
                free(copy);
                free(identity);
                return;
            }
            swapRows(copy, i, j, size);
            swapRows(identity, i, j, size);
        }

        double pivot = copy[i * size + i];
        for (int j = 0; j < size; j++) {
            copy[i * size + j] /= pivot;
            identity[i * size + j] /= pivot;
        }

        for (int j = 0; j < size; j++) {
            if (j != i) {
                double factor = copy[j * size + i];
                for (int k = 0; k < size; k++) {
                    copy[j * size + k] -= factor * copy[i * size + k];
                    identity[j * size + k] -= factor * identity[i * size + k];
                }
            }
        }
    }

    // �ͷŸ��Ƶľ���
    for (int i = 0; i < size * size; i++) result[i] = identity[i];
    free(copy);
    free(identity);
}
// ��������ṹ���β�����
Mat inverseMatrix(Mat A) {
    double* matrix = A.mat;
    Mat result;
    int size = A.col;
    if (A.row != A.col)
        cout << "���󲻿���" << endl;
    // ������λ��
    double* identity = (double*)malloc(size * size * sizeof(double));
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (i == j) {
                identity[i * size + j] = 1.0;
            }
            else {
                identity[i * size + j] = 0.0;
            }
        }
    }

    // ����ԭʼ����
    double* copy = (double*)malloc(size * size * sizeof(double));
    for (int i = 0; i < size * size; i++) {
        copy[i] = matrix[i];
    }

    // ��˹-Լ����Ԫ��
    for (int i = 0; i < size; i++) {
        if (copy[i * size + i] == 0.0) {
            // ����Խ���Ԫ��Ϊ0���򽻻���
            int j = i + 1;
            while (j < size && copy[j * size + i] == 0.0) {
                j++;
            }
            if (j == size) {
                printf("���󲻿��棡\n");
                free(copy);
                free(identity);
                return result;
            }
            swapRows(copy, i, j, size);
            swapRows(identity, i, j, size);
        }

        double pivot = copy[i * size + i];
        for (int j = 0; j < size; j++) {
            copy[i * size + j] /= pivot;
            identity[i * size + j] /= pivot;
        }

        for (int j = 0; j < size; j++) {
            if (j != i) {
                double factor = copy[j * size + i];
                for (int k = 0; k < size; k++) {
                    copy[j * size + k] -= factor * copy[i * size + k];
                    identity[j * size + k] -= factor * identity[i * size + k];
                }
            }
        }
    }

    // �ͷŸ��Ƶľ���
    free(copy);

    result.mat = identity;
    result.row = result.col = size;
    return result;
}

//����˷���������
void matx(double* A, int rowa, int cola, double* B, int rowb, int colb, double* C) {
    if (cola != rowb) { printf("���󲻿ɳ�"); return; }
    int i, j, k, t;
    for (i = 0; i < rowa; i++)
        for (j = 0; j < colb; j++) {
            for (k = i * cola, t = j; k < i * cola + cola; k++, t += colb)
                C[i * colb + j] += A[k] * B[t];
        }
}
//����˷�����
Mat matx(Mat A, Mat B) {
    Mat result;
    int rowa = A.row, rowb = B.row, cola = A.col, colb = B.col;
    double* C = (double*)malloc(rowa * colb * sizeof(double));
    if (cola != rowb) { printf("���󲻿ɳ�"); return result; }
    int i, j, k, t;
    for (i = 0; i < rowa; i++)
        for (j = 0; j < colb; j++) {
            for (k = i * cola, t = j; k < i * cola + cola; k++, t += colb)
                C[i * colb + j] += A.mat[k] * B.mat[t];
        }
    result.mat = C;
    result.row = rowa;
    result.col = colb;
    return result;
}


//����ת�ü�������
void matT(double* A, int rowa, int cola, double* AT) {
    int i, j;
    for (i = 0; i < cola; i++)
        for (j = 0; j < rowa; j++)
            AT[i * rowa + j] = A[j * cola + i];
}
Mat matT(Mat A) {
    Mat result;
    int rowa = A.row;
    int cola = A.col;
    int i, j;
    double* AT = (double*)malloc(rowa * cola * sizeof(double));
    for (i = 0; i < cola; i++)
        for (j = 0; j < rowa; j++)
            AT[i * rowa + j] = A.mat[j * cola + i];
    result.mat = AT;
    result.col = A.row;
    result.row = A.col;
    return result;
}

//������ͼ�������
void matsum(double* A, int rowa, int cola, double* B, int rowb, int colb, double* C) {
    if (rowa != rowb || cola != colb)
        cout << "���󲻿ɼ�" << endl;
    int i;
    for (i = 0; i < rowa * rowb; i++)
        C[i] = A[i] + B[i];
}

Mat matsum(Mat A, Mat B) {
    Mat result;
    int rowa = A.row, rowb = B.row, cola = A.col, colb = B.col;
    if (rowa != rowb || cola != colb)
        cout << "���󲻿ɼ�" << endl;
    double* C = (double*)malloc(rowa * cola * sizeof(double));
    int i;
    for (i = 0; i < rowa * rowb; i++)
        C[i] = A.mat[i] + B.mat[i];
    result.mat = C;
    result.row = rowa;
    result.col = cola;
    return result;
}

//������������
void matmins(double* A, int rowa, int cola, double* B, int rowb, int colb, double* C) {
    if (rowa != rowb || cola != colb)
        cout << "���󲻿ɼ�" << endl;
    int i;
    for (i = 0; i < rowa * rowb; i++)
        C[i] = A[i] - B[i];
}
Mat matmins(Mat A, Mat B) {
    Mat result;
    int rowa = A.row, rowb = B.row, cola = A.col, colb = B.col;
    if (rowa != rowb || cola != colb)
        cout << "���󲻿ɼ�" << endl;
    double* C = (double*)malloc(rowa * cola * sizeof(double));
    int i;
    for (i = 0; i < rowa * rowb; i++)
        C[i] = A.mat[i] - B.mat[i];
    result.mat = C;
    result.row = rowa;
    result.col = cola;
    return result;
}

//��������
void mata(double* A, int rowa, int cola, double b, double* bA) {
    int i;
    for (i = 0; i < cola * rowa; i++)
        bA[i] = b * A[i];
}

//�������
void VerdotX(double* A, int lena, double* B, int lenb, double* C) {
    if (lena != lenb)
        cout << "�������ɵ��" << endl;
    int i;
    for (i = 0; i < lena; i++)
        C[i] = A[i] * B[i];
}

//�������
void VerX(double* A, int lena, double* B, int lenb, double* C) {
    if (lena != 3 || lenb != 3)
        cout << "�������ɲ��" << endl;
    int i;
    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];
}

void mat2file(FILE* fp, Mat T, const char* name) {
    fprintf(fp, name);//д�ļ�ͷ
    fprintf(fp, ": !!");
    if (T.col == 1 || T.row == 1)
        fprintf(fp, "vector\n");
    else
        fprintf(fp, "matrix\n");
    fprintf(fp, "   rows: "); fprintf(fp, "%d\n", T.row);
    fprintf(fp, "   cols: "); fprintf(fp, "%d\n", T.col);
    fprintf(fp, "   dt: d\n");
    fprintf(fp, "   data: [");
    int len = T.col * T.row;
    if (len <= 1) fprintf(fp, " %.16e ]\n", T.mat[0]);
    else fprintf(fp, " %.16e, %.16e,\n", T.mat[0], T.mat[1]);
    for (int i = 2; i < len; i++) {
        if (i % 2 == 1 && i != len - 1) fprintf(fp, " %.16e,\n", T.mat[i]);
        else if (i % 2 == 0 && i != len - 1) fprintf(fp, "       %.16e,", T.mat[i]);
        else if (i % 2 == 1 && i == len - 1) fprintf(fp, " %.16e ]\n", T.mat[i]);
        else if (i % 2 == 0 && i == len - 1) fprintf(fp, "       %.16e ]\n", T.mat[i]);
    }
}
