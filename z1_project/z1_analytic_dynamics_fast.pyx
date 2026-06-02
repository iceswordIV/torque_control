# cython: language_level=3, boundscheck=False, wraparound=False, cdivision=True, initializedcheck=False
"""Compiled analytic Z1 dynamics.

This is the same product-of-exponentials / Lie-bracket dM formulation as
z1_analytic_dynamics.py, implemented with fixed-size C loops.
"""

import numpy as np
cimport numpy as cnp
from libc.math cimport sin, cos

cnp.import_array()

cdef int N = 6
cdef double G_CONST = 9.80665


cdef inline void zero4(double A[4][4]):
    cdef int i, j
    for i in range(4):
        for j in range(4):
            A[i][j] = 0.0


cdef inline void identity4(double A[4][4]):
    cdef int i
    zero4(A)
    for i in range(4):
        A[i][i] = 1.0


cdef inline void zero6x6(double A[6][6]):
    cdef int i, j
    for i in range(6):
        for j in range(6):
            A[i][j] = 0.0


cdef inline void identity6(double A[6][6]):
    cdef int i
    zero6x6(A)
    for i in range(6):
        A[i][i] = 1.0


cdef inline void mat4_mul(double A[4][4], double B[4][4], double C[4][4]):
    cdef int i, j, k
    cdef double s
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += A[i][k] * B[k][j]
            C[i][j] = s


cdef inline void mat6_vec_mul(double A[6][6], double x[6], double y[6]):
    cdef int i, j
    cdef double s
    for i in range(6):
        s = 0.0
        for j in range(6):
            s += A[i][j] * x[j]
        y[i] = s


cdef inline double dot6(double a[6], double b[6]):
    cdef int i
    cdef double s = 0.0
    for i in range(6):
        s += a[i] * b[i]
    return s


cdef inline void bracket_vw(double x1[6], double x2[6], double out[6]):
    cdef double v1x = x1[0]
    cdef double v1y = x1[1]
    cdef double v1z = x1[2]
    cdef double w1x = x1[3]
    cdef double w1y = x1[4]
    cdef double w1z = x1[5]
    cdef double v2x = x2[0]
    cdef double v2y = x2[1]
    cdef double v2z = x2[2]
    cdef double w2x = x2[3]
    cdef double w2y = x2[4]
    cdef double w2z = x2[5]
    out[0] = w1y * v2z - w1z * v2y + v1y * w2z - v1z * w2y
    out[1] = w1z * v2x - w1x * v2z + v1z * w2x - v1x * w2z
    out[2] = w1x * v2y - w1y * v2x + v1x * w2y - v1y * w2x
    out[3] = w1y * w2z - w1z * w2y
    out[4] = w1z * w2x - w1x * w2z
    out[5] = w1x * w2y - w1y * w2x


cdef inline void twist_exp(double w[3], double v[3], double theta, double E[4][4]):
    cdef double wx = w[0]
    cdef double wy = w[1]
    cdef double wz = w[2]
    cdef double s = sin(theta)
    cdef double c = cos(theta)
    cdef double one_c = 1.0 - c
    cdef double W[3][3]
    cdef double W2[3][3]
    cdef double R[3][3]
    cdef double Wv[3]
    cdef double dotwv
    cdef int i, j, k

    W[0][0] = 0.0
    W[0][1] = -wz
    W[0][2] = wy
    W[1][0] = wz
    W[1][1] = 0.0
    W[1][2] = -wx
    W[2][0] = -wy
    W[2][1] = wx
    W[2][2] = 0.0

    for i in range(3):
        for j in range(3):
            W2[i][j] = 0.0
            for k in range(3):
                W2[i][j] += W[i][k] * W[k][j]

    for i in range(3):
        for j in range(3):
            R[i][j] = (1.0 if i == j else 0.0) + W[i][j] * s + W2[i][j] * one_c

    Wv[0] = W[0][0] * v[0] + W[0][1] * v[1] + W[0][2] * v[2]
    Wv[1] = W[1][0] * v[0] + W[1][1] * v[1] + W[1][2] * v[2]
    Wv[2] = W[2][0] * v[0] + W[2][1] * v[1] + W[2][2] * v[2]
    dotwv = wx * v[0] + wy * v[1] + wz * v[2]

    identity4(E)
    for i in range(3):
        for j in range(3):
            E[i][j] = R[i][j]
        E[i][3] = Wv[i]
        for j in range(3):
            E[i][3] -= R[i][j] * Wv[j]
        E[i][3] += w[i] * dotwv * theta


cdef inline void adjoint_inverse_from_g(double g[4][4], double A[6][6]):
    cdef int i, j, k
    cdef double Rt[3][3]
    cdef double p[3]
    cdef double S[3][3]
    cdef double block
    zero6x6(A)
    for i in range(3):
        p[i] = g[i][3]
        for j in range(3):
            Rt[i][j] = g[j][i]
            A[i][j] = Rt[i][j]
            A[i + 3][j + 3] = Rt[i][j]

    S[0][0] = 0.0
    S[0][1] = -p[2]
    S[0][2] = p[1]
    S[1][0] = p[2]
    S[1][1] = 0.0
    S[1][2] = -p[0]
    S[2][0] = -p[1]
    S[2][1] = p[0]
    S[2][2] = 0.0

    for i in range(3):
        for j in range(3):
            block = 0.0
            for k in range(3):
                block -= Rt[i][k] * S[k][j]
            A[i][j + 3] = block


cdef inline void adjoint_inverse_from_translation(double p[3], double A[6][6]):
    zero6x6(A)
    A[0][0] = 1.0
    A[1][1] = 1.0
    A[2][2] = 1.0
    A[3][3] = 1.0
    A[4][4] = 1.0
    A[5][5] = 1.0
    # -skew(p)
    A[0][4] = p[2]
    A[0][5] = -p[1]
    A[1][3] = -p[2]
    A[1][5] = p[0]
    A[2][3] = p[1]
    A[2][4] = -p[0]


cdef inline void adjoint_apply(double T[4][4], double xi[6], double out[6]):
    cdef int i, j
    cdef double Rv[3]
    cdef double Rw[3]
    cdef double p[3]
    for i in range(3):
        p[i] = T[i][3]
        Rv[i] = 0.0
        Rw[i] = 0.0
        for j in range(3):
            Rv[i] += T[i][j] * xi[j]
            Rw[i] += T[i][j] * xi[j + 3]
    out[0] = Rv[0] + p[1] * Rw[2] - p[2] * Rw[1]
    out[1] = Rv[1] + p[2] * Rw[0] - p[0] * Rw[2]
    out[2] = Rv[2] + p[0] * Rw[1] - p[1] * Rw[0]
    out[3] = Rw[0]
    out[4] = Rw[1]
    out[5] = Rw[2]


cdef inline void init_constants(
    double m[6],
    double I[6][3][3],
    double w[6][3],
    double q_axis[6][3],
    double c[6][3],
    double xi[6][6],
    double gsl0_p[6][3],
):
    cdef int i, a
    cdef double v[3]

    m[0] = 0.67332551
    m[1] = 1.19132258
    m[2] = 0.83940874
    m[3] = 0.56404563
    m[4] = 0.38938492
    m[5] = 1.09473306147355

    for i in range(6):
        for a in range(3):
            w[i][a] = 0.0
            q_axis[i][a] = 0.0
            c[i][a] = 0.0
        for a in range(6):
            xi[i][a] = 0.0

    w[0][2] = 1.0
    w[1][1] = 1.0
    w[2][1] = 1.0
    w[3][1] = 1.0
    w[4][2] = 1.0
    w[5][0] = 1.0

    q_axis[0][0] = 0.0
    q_axis[0][1] = 0.0
    q_axis[0][2] = 0.0585
    q_axis[1][0] = 0.0
    q_axis[1][1] = 0.0
    q_axis[1][2] = 0.1035
    q_axis[2][0] = -0.35
    q_axis[2][1] = 0.0
    q_axis[2][2] = 0.1035
    q_axis[3][0] = -0.132
    q_axis[3][1] = 0.0
    q_axis[3][2] = 0.1605
    q_axis[4][0] = -0.062
    q_axis[4][1] = 0.0
    q_axis[4][2] = 0.1605
    q_axis[5][0] = -0.0128
    q_axis[5][1] = 0.0
    q_axis[5][2] = 0.1605

    c[0][0] = 2.47e-06
    c[0][1] = -0.00025198
    c[0][2] = 0.02317169
    c[1][0] = -0.11012601
    c[1][1] = 0.00240029
    c[1][2] = 0.00158266
    c[2][0] = 0.10609208
    c[2][1] = -0.00541815
    c[2][2] = 0.03476383
    c[3][0] = 0.04366681
    c[3][1] = 0.00364738
    c[3][2] = -0.00170192
    c[4][0] = 0.03121533
    c[4][1] = 0.0
    c[4][2] = 0.00646316
    c[5][0] = 0.023210304542
    c[5][1] = -0.000363250494
    c[5][2] = 0.002026681669

    for i in range(6):
        for a in range(3):
            gsl0_p[i][a] = q_axis[i][a] + c[i][a]

    for i in range(6):
        for a in range(3):
            for b in range(3):
                I[i][a][b] = 0.0

    I[0][0][0] = 0.00128328
    I[0][0][1] = -6e-08
    I[0][0][2] = -4e-07
    I[0][1][0] = -6e-08
    I[0][1][1] = 0.00071931
    I[0][1][2] = 5e-07
    I[0][2][0] = -4e-07
    I[0][2][1] = 5e-07
    I[0][2][2] = 0.00083936

    I[1][0][0] = 0.00102138
    I[1][0][1] = 0.00062358
    I[1][0][2] = 5.13e-06
    I[1][1][0] = 0.00062358
    I[1][1][1] = 0.02429457
    I[1][1][2] = -2.1e-06
    I[1][2][0] = 5.13e-06
    I[1][2][1] = -2.1e-06
    I[1][2][2] = 0.02466114

    I[2][0][0] = 0.00108061
    I[2][0][1] = -8.669e-05
    I[2][0][2] = -0.00208102
    I[2][1][0] = -8.669e-05
    I[2][1][1] = 0.00954238
    I[2][1][2] = -1.332e-05
    I[2][2][0] = -0.00208102
    I[2][2][1] = -1.332e-05
    I[2][2][2] = 0.00886621

    I[3][0][0] = 0.00031576
    I[3][0][1] = 8.13e-05
    I[3][0][2] = 4.091e-05
    I[3][1][0] = 8.13e-05
    I[3][1][1] = 0.00092996
    I[3][1][2] = -5.96e-06
    I[3][2][0] = 4.091e-05
    I[3][2][1] = -5.96e-06
    I[3][2][2] = 0.00097912

    I[4][0][0] = 0.00017605
    I[4][0][1] = 4e-07
    I[4][0][2] = 5.689e-05
    I[4][1][0] = 4e-07
    I[4][1][1] = 0.00055896
    I[4][1][2] = -1.3e-07
    I[4][2][0] = 5.689e-05
    I[4][2][1] = -1.3e-07
    I[4][2][2] = 0.00053860

    I[5][0][0] = 0.003367542789986
    I[5][0][1] = -1.884722331677e-05
    I[5][0][2] = 2.829437074001e-04
    I[5][1][0] = -1.884722331677e-05
    I[5][1][1] = 0.002529152661291
    I[5][1][2] = 2.211249714817e-05
    I[5][2][0] = 2.829437074001e-04
    I[5][2][1] = 2.211249714817e-05
    I[5][2][2] = 0.002713697251443

    for i in range(6):
        v[0] = -(w[i][1] * q_axis[i][2] - w[i][2] * q_axis[i][1])
        v[1] = -(w[i][2] * q_axis[i][0] - w[i][0] * q_axis[i][2])
        v[2] = -(w[i][0] * q_axis[i][1] - w[i][1] * q_axis[i][0])
        xi[i][0] = v[0]
        xi[i][1] = v[1]
        xi[i][2] = v[2]
        xi[i][3] = w[i][0]
        xi[i][4] = w[i][1]
        xi[i][5] = w[i][2]


cdef void compute_A_Gp(
    double q[6],
    double A[6][6][6][6],
    double Gp[6][6][6],
    double xi[6][6],
    double m[6],
    double w[6][3],
    double q_axis[6][3],
    double I[6][3][3],
    double gsl0_p[6][3],
):
    cdef int i, j, k, l, r, a, b, cidx
    cdef double E[6][4][4]
    cdef double g[4][4]
    cdef double tmp4[4][4]
    cdef double A0[6][6]
    cdef double G[6][6]
    cdef double v[3]
    cdef double tmp

    for l in range(6):
        for j in range(6):
            for a in range(6):
                for b in range(6):
                    A[l][j][a][b] = 0.0

    for i in range(6):
        v[0] = xi[i][0]
        v[1] = xi[i][1]
        v[2] = xi[i][2]
        twist_exp(w[i], v, q[i], E[i])

    for l in range(6):
        for j in range(l + 1):
            if l == j:
                for a in range(6):
                    A[l][j][a][a] = 1.0
            else:
                identity4(g)
                for r in range(j + 1, l + 1):
                    mat4_mul(g, E[r], tmp4)
                    for a in range(4):
                        for b in range(4):
                            g[a][b] = tmp4[a][b]
                adjoint_inverse_from_g(g, A[l][j])

    for l in range(6):
        zero6x6(G)
        G[0][0] = m[l]
        G[1][1] = m[l]
        G[2][2] = m[l]
        for a in range(3):
            for b in range(3):
                G[a + 3][b + 3] = I[l][a][b]

        adjoint_inverse_from_translation(gsl0_p[l], A0)
        for i in range(6):
            for j in range(6):
                tmp = 0.0
                for a in range(6):
                    for b in range(6):
                        tmp += A0[a][i] * G[a][b] * A0[b][j]
                Gp[l][i][j] = tmp


cdef void compute_mass_dM_core(double q[6], double M[6][6], double dM[6][6][6]):
    cdef double m[6]
    cdef double I[6][3][3]
    cdef double w[6][3]
    cdef double q_axis[6][3]
    cdef double c[6][3]
    cdef double xi[6][6]
    cdef double gsl0_p[6][3]
    cdef double A[6][6][6][6]
    cdef double Gp[6][6][6]
    cdef double J[6][6][6]
    cdef double GJ[6][6][6]
    cdef double dJ[6][6][6][6]
    cdef double GdJ[6][6][6][6]
    cdef double x1[6]
    cdef double br[6]
    cdef double val
    cdef int i, j, k, l, a, b

    init_constants(m, I, w, q_axis, c, xi, gsl0_p)
    compute_A_Gp(q, A, Gp, xi, m, w, q_axis, I, gsl0_p)

    for l in range(6):
        for j in range(6):
            for a in range(6):
                J[l][j][a] = 0.0
                GJ[l][j][a] = 0.0
                for k in range(6):
                    dJ[l][j][k][a] = 0.0
                    GdJ[l][j][k][a] = 0.0

    for l in range(6):
        for j in range(l + 1):
            mat6_vec_mul(A[l][j], xi[j], J[l][j])

    for l in range(6):
        for j in range(l + 1):
            mat6_vec_mul(Gp[l], J[l][j], GJ[l][j])

    for i in range(6):
        for j in range(6):
            val = 0.0
            for l in range(i if i > j else j, 6):
                val += dot6(J[l][i], GJ[l][j])
            M[i][j] = val
    for i in range(6):
        for j in range(i + 1, 6):
            val = 0.5 * (M[i][j] + M[j][i])
            M[i][j] = val
            M[j][i] = val

    for l in range(6):
        for j in range(l + 1):
            for k in range(j, l + 1):
                mat6_vec_mul(A[k][j], xi[j], x1)
                bracket_vw(x1, xi[k], br)
                mat6_vec_mul(A[l][k], br, dJ[l][j][k])

    for l in range(6):
        for j in range(l + 1):
            for k in range(j, l + 1):
                mat6_vec_mul(Gp[l], dJ[l][j][k], GdJ[l][j][k])

    for i in range(6):
        for j in range(6):
            for k in range(6):
                val = 0.0
                for l in range(i if i > j else j, 6):
                    val += dot6(dJ[l][i][k], GJ[l][j]) + dot6(J[l][i], GdJ[l][j][k])
                dM[i][j][k] = val


cdef void compute_gravity_core(double q[6], double Nvec[6]):
    cdef double m[6]
    cdef double I[6][3][3]
    cdef double w[6][3]
    cdef double q_axis[6][3]
    cdef double c[6][3]
    cdef double xi[6][6]
    cdef double gsl0_p[6][3]
    cdef double E[6][4][4]
    cdef double g[4][4]
    cdef double tmp4[4][4]
    cdef double p_com[6][3]
    cdef double J_space[6][6]
    cdef double v[3]
    cdef double dp_z
    cdef int i, j, l, a, b, r

    init_constants(m, I, w, q_axis, c, xi, gsl0_p)
    for i in range(6):
        v[0] = xi[i][0]
        v[1] = xi[i][1]
        v[2] = xi[i][2]
        twist_exp(w[i], v, q[i], E[i])

    for l in range(6):
        identity4(g)
        for r in range(l + 1):
            mat4_mul(g, E[r], tmp4)
            for a in range(4):
                for b in range(4):
                    g[a][b] = tmp4[a][b]
        for a in range(3):
            p_com[l][a] = g[a][3]
            for b in range(3):
                p_com[l][a] += g[a][b] * gsl0_p[l][b]

    identity4(g)
    for j in range(6):
        adjoint_apply(g, xi[j], J_space[j])
        mat4_mul(g, E[j], tmp4)
        for a in range(4):
            for b in range(4):
                g[a][b] = tmp4[a][b]

    for j in range(6):
        Nvec[j] = 0.0
        for l in range(j, 6):
            dp_z = J_space[j][2] + J_space[j][3] * p_com[l][1] - J_space[j][4] * p_com[l][0]
            Nvec[j] += m[l] * G_CONST * dp_z


cpdef tuple mass_and_dM(object q_input):
    cdef cnp.ndarray[cnp.double_t, ndim=1, mode="c"] q_arr = np.ascontiguousarray(q_input, dtype=np.float64).reshape((6,))
    cdef cnp.ndarray[cnp.double_t, ndim=2, mode="c"] M_out = np.empty((6, 6), dtype=np.float64)
    cdef cnp.ndarray[cnp.double_t, ndim=3, mode="c"] dM_out = np.empty((6, 6, 6), dtype=np.float64)
    cdef double q[6]
    cdef double M[6][6]
    cdef double dM[6][6][6]
    cdef int i, j, k
    for i in range(6):
        q[i] = q_arr[i]
    compute_mass_dM_core(q, M, dM)
    for i in range(6):
        for j in range(6):
            M_out[i, j] = M[i][j]
            for k in range(6):
                dM_out[i, j, k] = dM[i][j][k]
    return M_out, dM_out


cpdef cnp.ndarray gravity_vector(object q_input):
    cdef cnp.ndarray[cnp.double_t, ndim=1, mode="c"] q_arr = np.ascontiguousarray(q_input, dtype=np.float64).reshape((6,))
    cdef cnp.ndarray[cnp.double_t, ndim=1, mode="c"] N_out = np.empty((6,), dtype=np.float64)
    cdef double q[6]
    cdef double Nvec[6]
    cdef int i
    for i in range(6):
        q[i] = q_arr[i]
    compute_gravity_core(q, Nvec)
    for i in range(6):
        N_out[i] = Nvec[i]
    return N_out


cpdef tuple dynamics(object q_input, object dq_input):
    cdef cnp.ndarray[cnp.double_t, ndim=1, mode="c"] q_arr = np.ascontiguousarray(q_input, dtype=np.float64).reshape((6,))
    cdef cnp.ndarray[cnp.double_t, ndim=1, mode="c"] dq_arr = np.ascontiguousarray(dq_input, dtype=np.float64).reshape((6,))
    cdef cnp.ndarray[cnp.double_t, ndim=2, mode="c"] M_out = np.empty((6, 6), dtype=np.float64)
    cdef cnp.ndarray[cnp.double_t, ndim=2, mode="c"] C_out = np.empty((6, 6), dtype=np.float64)
    cdef cnp.ndarray[cnp.double_t, ndim=1, mode="c"] N_out = np.empty((6,), dtype=np.float64)
    cdef cnp.ndarray[cnp.double_t, ndim=3, mode="c"] dM_out = np.empty((6, 6, 6), dtype=np.float64)
    cdef double q[6]
    cdef double dq[6]
    cdef double M[6][6]
    cdef double dM[6][6][6]
    cdef double Nvec[6]
    cdef double cval
    cdef int i, j, k
    for i in range(6):
        q[i] = q_arr[i]
        dq[i] = dq_arr[i]
    compute_mass_dM_core(q, M, dM)
    compute_gravity_core(q, Nvec)
    for i in range(6):
        N_out[i] = Nvec[i]
        for j in range(6):
            M_out[i, j] = M[i][j]
            cval = 0.0
            for k in range(6):
                cval += (dM[i][j][k] + dM[i][k][j] - dM[k][j][i]) * dq[k]
                dM_out[i, j, k] = dM[i][j][k]
            C_out[i, j] = 0.5 * cval
    return M_out, C_out, N_out, dM_out
