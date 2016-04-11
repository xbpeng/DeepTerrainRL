/* QL.F -- translated by f2c (version 20100827).
   You must link the resulting object file with libf2c:
    on Microsoft Windows system, link with libf2c.lib;
    on Linux or Unix systems, link with .../path/to/libf2c.a -lm
    or, if you install libf2c.a in a standard place, with -lf2c -lm
    -- in that order, at the end of the command line, as in
        cc *.o -lf2c -lm
    Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

        http://www.netlib.org/f2c/libf2c.zip
*/

#include "f2c.h"

/* Table of constant values */

static integer c__1 = 1;


/* Subroutine */ int ql_(integer *m, integer *me, integer *mmax, integer *n, 
    integer *nmax, integer *mnn, doublereal *c__, doublereal *d__, 
    doublereal *a, doublereal *b, doublereal *xl, doublereal *xu, 
    doublereal *x, doublereal *u, doublereal *eps, integer *mode, integer 
    *iout, integer *ifail, integer *iprint, doublereal *war, integer *
    lwar, integer *iwar, integer *liwar)
{
    /* Format strings */
    static char fmt_1000[] = "(/8x,\002*** ERROR (QL): Matrix G was enlarge"
        "d\002,i3,\002-times by unit matrix!\002)";
    static char fmt_1100[] = "(/8x,\002*** ERROR (QL): Constraint \002,i5"
        ",\002 not consistent to \002,/,(10x,10i5))";
    static char fmt_1200[] = "(/8x,\002*** ERROR (QL): LWAR too small! Shoul"
        "d be at least \002,i8,\002, but only \002,i8,\002 is available"
        ".\002)";
    static char fmt_1210[] = "(/8x,\002*** ERROR (QL): LIWAR too small!\002)";
    static char fmt_1220[] = "(/8x,\002*** ERROR (QL): MNN too small!\002)";
    static char fmt_1230[] = "(/8x,\002*** ERROR (QL): Internal error, divis"
        "ion by zero!\002)";
    static char fmt_1240[] = "(/8x,\002*** ERROR (QL): EPS not positive!\002)"
        ;
    static char fmt_1300[] = "(/8x,\002*** ERROR (QL): Too many iterations ("
        "more than\002,i6,\002)\002)";
    static char fmt_1400[] = "(/8x,\002*** ERROR (QL): Accuracy insufficient"
        " to attain \002,\002convergence!\002)";
    static char fmt_1500[] = "(/8x,\002*** ERROR (QL): Accuracy too small to"
        " detect \002,\002feasibility! Restart with tolerance \002,d12.4"
        ",\002 in WA(1)!\002)";

    /* System generated locals */
    integer c_dim1, c_offset, a_dim1, a_offset, i__1, i__2;

    /* Builtin functions */
    integer s_wsfe(cilist *), do_fio(integer *, char *, ftnlen), e_wsfe(void);

    /* Local variables */
    static integer i__, j, in, mn, lw;
    static logical lql;
    static integer inw1, inw2;
    static doublereal diag;
    extern /* Subroutine */ int ql0002_(integer *, integer *, integer *, 
        integer *, integer *, integer *, logical *, doublereal *, 
        doublereal *, doublereal *, doublereal *, doublereal *, 
        doublereal *, doublereal *, integer *, integer *, integer *, 
        doublereal *, integer *, doublereal *, doublereal *, integer *);
    static integer nact, info;
    static doublereal zero;
    static integer idiag, maxit;
    static doublereal qpeps;

    /* Fortran I/O blocks */
    static cilist io___16 = { 0, 0, 0, fmt_1000, 0 };
    static cilist io___17 = { 0, 0, 0, fmt_1100, 0 };
    static cilist io___18 = { 0, 0, 0, fmt_1200, 0 };
    static cilist io___19 = { 0, 0, 0, fmt_1210, 0 };
    static cilist io___20 = { 0, 0, 0, fmt_1220, 0 };
    static cilist io___21 = { 0, 0, 0, fmt_1230, 0 };
    static cilist io___22 = { 0, 0, 0, fmt_1240, 0 };
    static cilist io___23 = { 0, 0, 0, fmt_1300, 0 };
    static cilist io___24 = { 0, 0, 0, fmt_1400, 0 };
    static cilist io___25 = { 0, 0, 0, fmt_1500, 0 };



/* ********************************************************************* */


/*         AN IMPLEMENTATION OF A PRIMAL-DUAL QUADRATIC PROGRAMMING */

/*                                METHOD */


/*   The Problem: */

/*   The code solves the strictly convex quadratic program */

/*             minimize      1/2 x^ C x + c^x */
/*             subject to    a_j^x + b_j  =  0  ,  j=1,...,m_e */
/*                           a_j^x + b_j  >= 0  ,  j=m+1,...,m */
/*                           x_l <= x <= x_u */

/*   with an n by n positive definite matrix C, an n-dimensional vector d, */
/*   an m by n matrix A=(a_1,...,a_m)^, and an m-vector b. */



/*   The Numerical Algorithm: */

/*   The subroutine reorganizes some data to solve the quadratic program */
/*   by a modification or a code going back to Powell (1983). The numerical */
/*   algorithm is the primal-dual method of Goldfarb and Idnani (1983). */
/*   First, a solution of the unconstrained quadratic program is found */
/*   proceeding from a Cholesky decomposition of C. Subsequently, violated */
/*   constraints are added successively. Constraints no longer considered */
/*   as active ones, are dropped. Numerically stable orthogonal */
/*   decomposition are applied to find intermediate minima on hyperplane */
/*   of the active constraints. */



/*   Usage: */

/*      CALL QL (      M,     ME,   MMAX,      N,   NMAX, */
/*     /             MNN,      C,      D,      A,      B, */
/*     /              XL,     XU,      X,      U,    EPS, */
/*     /            MODE,   IOUT,  IFAIL, IPRINT,    WAR, */
/*     /            LWAR,   IWAR,  LIWAR) */



/*   Definition of the parameters: */

/*   M :       Number of constraints. */
/*   ME :      Number of equality constraints. */
/*   MMAX :    Row dimension of array A containing linear constraints. */
/*             MMAX must be at least one and greater or equal to M. */
/*   N :       Number of optimization variables. */
/*   NMAX :    Row dimension of C. NMAX must be at least one and greater */
/*             or equal to N. */
/*   MNN :     Must be equal to M+N+N when calling QL, dimension of U. */
/*   C(NMAX,NMAX): Objective function matrix which should be symmetric */
/*             and positive definite. If MODE=0, C is supposed to be the */
/*             upper triangular factor of a Cholesky decomposition of C. */
/*   D(NMAX) : Contains the constant vector of the quadratic objective */
/*             function. */
/*   A(MMAX,NMAX): Matrix of the linear constraints, first ME rows for */
/*             equality, then M-ME rows for inequality constraints. */
/*   B(MMAX) : Constant values of linear constraints in the same order. */
/*   XL(N),XU(N) : On input, the one-dimensional arrays XL and XU must */
/*             contain the upper and lower bounds of the variables. */
/*   X(N) :    On return, X contains the optimal solution. */
/*   U(MNN) :  On return, U contains the multipliers subject to the */
/*             linear constraints and bounds. The first M locations */
/*             contain the multipliers of the M linear constraints, the */
/*             subsequent N locations the multipliers of the lower */
/*             bounds, and the final N locations the multipliers of the */
/*             upper bounds. At the optimal solution, all multipliers */
/*             with respect to inequality constraints should be */
/*             nonnegative. */
/*   EPS :     The user has to specify the desired final accuracy */
/*             (e.g. 1.0D-12). The parameter value should not be smaller */
/*             than the underlying machine precision. */
/*   MODE :    MODE=0 - The user provides an initial Cholesky factorization */
/*                      of C, stored in the upper triangular part of the */
/*                      array C. */
/*             MODE=1 - A Cholesky decomposition to get the first */
/*                      unconstrained minimizer, is computed internally. */
/*   IOUT :    Integer indicating the desired output unit number, i.e. all */
/*             write-statements start with 'WRITE(IOUT,... '. */
/*   IFAIL :   The parameter shows the reason for terminating a solution */
/*             process. On return, IFAIL could get the following values: */
/*             IFAIL=0  : The optimality conditions are satisfied. */
/*             IFAIL=1  : The algorithm has been stopped after too many */
/*                        MAXIT iterations (40*(N+M)). */
/*             IFAIL=2  : Termination accuracy insufficient to satisfy */
/*                        convergence criterion. */
/*             IFAIL=3  : Internal inconsistency of QL, division by zero. */
/*             IFAIL=4  : Numerical instability prevents successful termination. */
/*                        Use tolerance specified in WAR(1) for a restart. */
/*             IFAIL=5  : LWAR, LIWAR, MNN, or EPS incorrect. */
/*             IFAIL>100: Constraints are inconsistent and IFAIL=100+ICON, */
/*                        where ICON denotes a constraint causing the conflict. */
/*   IPRINT :  Specification of the desired output level. */
/*             IPRINT=0 :  No output of the program. */
/*             IPRINT=1 :  Only a final error message is given. */
/*   WAR(LWAR),LWAR : WAR is a real working array of length LWAR. LWAR */
/*             must be at least 3*NMAX*NMAX/2 + 10*NMAX + MMAX + M + 1. */
/*   IWAR(LIWAR),LIWAR : The user has to provide working space for an */
/*             integer array. LIWAR must be at least N. */



/*   Copyright(C):  Klaus Schittkowski, Department of Computer Science, */
/*                  University of Bayreuth, D-95440 Bayreuth, Germany, */
/*                  (1987-2010) */



/*   Reference:     M.J.D. Powell: ZQPCVX, A FORTRAN Subroutine for Convex */
/*                  Programming, Report DAMTP/1983/NA17, University of */
/*                  Cambridge, England, 1983 */

/*                  D. Goldfarb, A. Idnani (1983): A numerically stable */
/*                  method for solving strictly convex quadratic programs, */
/*                  Mathematical Programming, Vol. 27, 1-33 */

/*                  K. Schittkowski (2007): QL: A Fortran code for convex */
/*                  quadratic programming - user's guide, Report, Department */
/*                  of Computer Science, University of Bayreuth */



/*   Version:       1.0   (Mar, 1987) - first implementation */
/*                  1.8   (Oct, 2002) - new tolerances */
/*                  2.0   (Apr, 2003) - parameter list, documentation */
/*                  2.1   (Sep, 2004) - new error message */
/*                  2.1.1 (Jul, 2005) - error message (LWAR) */
/*                  2.1.2 (Jul, 2007) - parameter declarations */
/*                  2.1.3 (Sep, 2007) - test for division by zero */
/*                  2.1.4 (Feb, 2009) - some if-statements changed */
/*                  2.1.5 (May, 2009) - comments */
/*                  2.1.6 (Jun, 2009) - IFAIL=4 introduced */
/*                  2.1.7 (Apr, 2010) - comments */
/*                  2.2   (Sep, 2010) - division by zero */
/*                  3.0   (Nov, 2010) - warnings removed */
/*                  3.1   (Nov, 2010) - underflow for zero checks */
/*                  3.2   (Feb, 2011) - input checks, overflow near */
/*                                      label 770 */

/* ********************************************************************* */


/*     CONSTANT DATA */

    /* Parameter adjustments */
    --b;
    --x;
    --xu;
    --xl;
    a_dim1 = *mmax;
    a_offset = 1 + a_dim1;
    a -= a_offset;
    --d__;
    c_dim1 = *nmax;
    c_offset = 1 + c_dim1;
    c__ -= c_offset;
    --u;
    --war;
    --iwar;

    /* Function Body */
    lql = FALSE_;
    if (*mode == 1) {
    lql = TRUE_;
    }
    zero = 0.;
    maxit = (*m + *n) * 40;
    qpeps = *eps;
    inw1 = 1;
    inw2 = inw1 + *mmax;

/*     PREPARE PROBLEM DATA FOR EXECUTION */

    if (*m > 0) {
    in = inw1;
    i__1 = *m;
    for (j = 1; j <= i__1; ++j) {
        war[in] = -b[j];
        ++in;
    }
    }
    lw = *nmax * 3 * *nmax / 2 + *nmax * 10 + *m;
    if (inw2 + lw > *lwar) {
    goto L80;
    }
    if (*liwar < *n) {
    goto L81;
    }
    if (*mnn < *m + *n + *n) {
    goto L82;
    }
    if (*eps <= 0.) {
    goto L84;
    }
    mn = *m + *n;

/*     CALL OF QL0002 */

    ql0002_(n, m, me, mmax, &mn, nmax, &lql, &a[a_offset], &war[inw1], &d__[1]
        , &c__[c_offset], &xl[1], &xu[1], &x[1], &nact, &iwar[1], &maxit, 
        &qpeps, &info, &diag, &war[inw2], &lw);

/*     TEST OF MATRIX CORRECTIONS */

    if (info == 3 || info == 4) {
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
        x[i__] = 0.;
    }
    }
    *ifail = 0;
    if (info == 1) {
    goto L40;
    }
    if (info == 2) {
    goto L90;
    }
    if (info == 3) {
    goto L83;
    }
    if (info == 4) {
    goto L95;
    }
    idiag = 0;
    if (diag > zero && diag < 1e3) {
    idiag = (integer) diag;
    }
    if (*iprint > 0 && idiag > 0) {
    io___16.ciunit = *iout;
    s_wsfe(&io___16);
    do_fio(&c__1, (char *)&idiag, (ftnlen)sizeof(integer));
    e_wsfe();
    }
    if (info < 0) {
    goto L70;
    }

/*     REORDER MULTIPLIER */

    i__1 = *mnn;
    for (j = 1; j <= i__1; ++j) {
    u[j] = zero;
    }
    in = inw2 - 1;
    if (nact == 0) {
    goto L30;
    }
    i__1 = nact;
    for (i__ = 1; i__ <= i__1; ++i__) {
    j = iwar[i__];
    u[j] = war[in + i__];
    }
L30:
    return 0;

/*     ERROR MESSAGES */

L70:
    *ifail = -info + 100;
    if (*iprint > 0 && nact > 0) {
    io___17.ciunit = *iout;
    s_wsfe(&io___17);
    i__1 = -info;
    do_fio(&c__1, (char *)&i__1, (ftnlen)sizeof(integer));
    i__2 = nact;
    for (i__ = 1; i__ <= i__2; ++i__) {
        do_fio(&c__1, (char *)&iwar[i__], (ftnlen)sizeof(integer));
    }
    e_wsfe();
    }
    return 0;
L80:
    *ifail = 5;
    if (*iprint > 0) {
    io___18.ciunit = *iout;
    s_wsfe(&io___18);
    i__1 = inw2 + lw;
    do_fio(&c__1, (char *)&i__1, (ftnlen)sizeof(integer));
    do_fio(&c__1, (char *)&(*lwar), (ftnlen)sizeof(integer));
    e_wsfe();
    }
    return 0;
L81:
    *ifail = 5;
    if (*iprint > 0) {
    io___19.ciunit = *iout;
    s_wsfe(&io___19);
    e_wsfe();
    }
    return 0;
L82:
    *ifail = 5;
    if (*iprint > 0) {
    io___20.ciunit = *iout;
    s_wsfe(&io___20);
    e_wsfe();
    }
    return 0;
L83:
    *ifail = 3;
    if (*iprint > 0) {
    io___21.ciunit = *iout;
    s_wsfe(&io___21);
    e_wsfe();
    }
    return 0;
L84:
    *ifail = 5;
    if (*iprint > 0) {
    io___22.ciunit = *iout;
    s_wsfe(&io___22);
    e_wsfe();
    }
    return 0;
L40:
    *ifail = 1;
    if (*iprint > 0) {
    io___23.ciunit = *iout;
    s_wsfe(&io___23);
    do_fio(&c__1, (char *)&maxit, (ftnlen)sizeof(integer));
    e_wsfe();
    }
    return 0;
L90:
    *ifail = 2;
    if (*iprint > 0) {
    io___24.ciunit = *iout;
    s_wsfe(&io___24);
    e_wsfe();
    }
    return 0;
L95:
    *ifail = 4;
    if (*iprint > 0) {
    io___25.ciunit = *iout;
    s_wsfe(&io___25);
    do_fio(&c__1, (char *)&war[1], (ftnlen)sizeof(doublereal));
    e_wsfe();
    }
    return 0;

/*     FORMAT-INSTRUCTIONS */

} /* ql_ */


/* Subroutine */ int ql0002_(integer *n, integer *m, integer *meq, integer *
    mmax, integer *mn, integer *nmax, logical *lql, doublereal *a, 
    doublereal *b, doublereal *grad, doublereal *g, doublereal *xl, 
    doublereal *xu, doublereal *x, integer *nact, integer *iact, integer *
    maxit, doublereal *vsmall, integer *info, doublereal *diag, 
    doublereal *w, integer *lw)
{
    /* System generated locals */
    integer a_dim1, a_offset, g_dim1, g_offset, i__1, i__2, i__3, i__4;
    doublereal d__1, d__2, d__3, d__4;

    /* Builtin functions */
    double pow_di(doublereal *, integer *), sqrt(doublereal);

    /* Local variables */
    static integer i__, j, k, k1;
    static doublereal ga, gb;
    static integer ia, id, ii, il, kk, jl, ir, nm, is, iu, iw, ju, ix, iz, nu,
         iy, ira, irb, iwa;
    static doublereal one;
    static integer iwd, iza;
    static doublereal ufl, res;
    static integer iwr, iws;
    static doublereal sum;
    static integer iww, iwx, iwy;
    static doublereal two;
    static integer iwz;
    static doublereal onha, xmag, suma, sumb, sumc, temp, step, zero;
    static integer iwwn;
    static doublereal sumx, sumy, fdiff;
    static integer iflag, jflag, kflag, lflag;
    static doublereal diagr;
    static integer nflag, ifinc, jfinc, kfinc, mflag;
    static doublereal vfact, tempa;
    static integer iterc, itref;
    static doublereal mincv, cvmax, ratio, xmagr;
    static integer kdrop;
    static logical lower;
    static integer knext;
    static doublereal fdiffa, parinc, parnew;


/* ************************************************************************** */


/*   THIS SUBROUTINE SOLVES THE QUADRATIC PROGRAMMING PROBLEM */

/*       MINIMIZE      GRAD'*X  +  0.5 * X*G*X */
/*       SUBJECT TO    A(K)*X  =  B(K)   K=1,2,...,MEQ, */
/*                     A(K)*X >=  B(K)   K=MEQ+1,...,M, */
/*                     XL  <=  X  <=  XU */

/*   THE QUADRATIC PROGRAMMING METHOD PROCEEDS FROM AN INITIAL CHOLESKY- */
/*   DECOMPOSITION OF THE OBJECTIVE FUNCTION MATRIX, TO CALCULATE THE */
/*   UNIQUELY DETERMINED MINIMIZER OF THE UNCONSTRAINED PROBLEM. */
/*   SUCCESSIVELY ALL VIOLATED CONSTRAINTS ARE ADDED TO A WORKING SET */
/*   AND A MINIMIZER OF THE OBJECTIVE FUNCTION SUBJECT TO ALL CONSTRAINTS */
/*   IN THIS WORKING SET IS COMPUTED. IT IS POSSIBLE THAT CONSTRAINTS */
/*   HAVE TO LEAVE THE WORKING SET. */


/*   DESCRIPTION OF PARAMETERS: */

/*     N        : IS THE NUMBER OF VARIABLES. */
/*     M        : TOTAL NUMBER OF CONSTRAINTS. */
/*     MEQ      : NUMBER OF EQUALITY CONTRAINTS. */
/*     MMAX     : ROW DIMENSION OF A, DIMENSION OF B. MMAX MUST BE AT */
/*                LEAST ONE AND GREATER OR EQUAL TO M. */
/*     MN       : MUST BE EQUAL M + N. */
/*     NMAX     : ROW DIEMSION OF G. MUST BE AT LEAST N. */
/*     LQL      : DETERMINES INITIAL DECOMPOSITION. */
/*        LQL = .FALSE.  : THE UPPER TRIANGULAR PART OF THE MATRIX G */
/*                         CONTAINS INITIALLY THE CHOLESKY-FACTOR OF A SUITABLE */
/*                         DECOMPOSITION. */
/*        LQL = .TRUE.   : THE INITIAL CHOLESKY-FACTORISATION OF G IS TO BE */
/*                         PERFORMED BY THE ALGORITHM. */
/*     A(MMAX,NMAX) : A IS A MATRIX WHOSE COLUMNS ARE THE CONSTRAINTS NORMALS. */
/*     B(MMAX)  : CONTAINS THE RIGHT HAND SIDES OF THE CONSTRAINTS. */
/*     GRAD(N)  : CONTAINS THE OBJECTIVE FUNCTION VECTOR GRAD. */
/*     G(NMAX,N): CONTAINS THE SYMMETRIC OBJECTIVE FUNCTION MATRIX. */
/*     XL(N), XU(N): CONTAIN THE LOWER AND UPPER BOUNDS FOR X. */
/*     X(N)     : VECTOR OF VARIABLES. */
/*     NACT     : FINAL NUMBER OF ACTIVE CONSTRAINTS. */
/*     IACT(K) (K=1,2,...,NACT): INDICES OF THE FINAL ACTIVE CONSTRAINTS. */
/*     INFO     : REASON FOR THE RETURN FROM THE SUBROUTINE. */
/*         INFO = 0 : CALCULATION WAS TERMINATED SUCCESSFULLY. */
/*         INFO = 1 : MAXIMUM NUMBER OF ITERATIONS ATTAINED. */
/*         INFO = 2 : ACCURACY IS INSUFFICIENT TO MAINTAIN INCREASING */
/*                    FUNCTION VALUES. */
/*         INFO = 3 : INTERNAL INCONSISTENCY OF QP, DIVISION BY ZERO. */
/*         INFO = 4 : ACCURACY TOO SMALL FOR SUCESSFUL TERMINATION. */
/*         INFO < 0 : THE CONSTRAINT WITH INDEX ABS(INFO) AND THE CON- */
/*                    STRAINTS WHOSE INDICES ARE IACT(K), K=1,2,...,NACT, */
/*                    ARE INCONSISTENT. */
/*     MAXIT    : MAXIMUM NUMBER OF ITERATIONS. */
/*     VSMALL   : REQUIRED ACCURACY TO BE ACHIEVED (E.G. IN THE ORDER OF THE */
/*                MACHINE PRECISION FOR SMALL AND WELL-CONDITIONED PROBLEMS). */
/*     DIAG     : ON RETURN DIAG IS EQUAL TO THE MULTIPLE OF THE UNIT MATRIX */
/*                THAT WAS ADDED TO G TO ACHIEVE POSITIVE DEFINITENESS. */
/*     W(LW)    : THE ELEMENTS OF W(.) ARE USED FOR WORKING SPACE. THE LENGTH */
/*                OF W MUST NOT BE LESS THAN (1.5*NMAX*NMAX + 10*NMAX + M). */
/*                WHEN INFO = 0 ON RETURN, THE LAGRANGE MULTIPLIERS OF THE */
/*                FINAL ACTIVE CONSTRAINTS ARE HELD IN W(K), K=1,2,...,NACT. */
/*   THE VALUES OF N, M, MEQ, MMAX, MN, AND NMAX AND THE ELEMENTS OF */
/*   A, B, GRAD AND G ARE NOT ALTERED. */

/*   THE FOLLOWING INTEGERS ARE USED TO PARTITION W: */
/*     THE FIRST N ELEMENTS OF W HOLD LAGRANGE MULTIPLIER ESTIMATES. */
/*     W(IWZ+I+(N-1)*J) HOLDS THE MATRIX ELEMENT Z(I,J). */
/*     W(IWR+I+0.5*J*(J-1)) HOLDS THE UPPER TRIANGULAR MATRIX */
/*       ELEMENT R(I,J). THE SUBSEQUENT N COMPONENTS OF W MAY BE */
/*       TREATED AS AN EXTRA COLUMN OF R(.,.). */
/*     W(IWW-N+I) (I=1,2,...,N) ARE USED FOR TEMPORARY STORAGE. */
/*     W(IWW+I) (I=1,2,...,N) ARE USED FOR TEMPORARY STORAGE. */
/*     W(IWD+I) (I=1,2,...,N) HOLDS G(I,I) DURING THE CALCULATION. */
/*     W(IWX+I) (I=1,2,...,N) HOLDS VARIABLES THAT WILL BE USED TO */
/*       TEST THAT THE ITERATIONS INCREASE THE OBJECTIVE FUNCTION. */
/*     W(IWA+K) (K=1,2,...,M) USUALLY HOLDS THE RECIPROCAL OF THE */
/*       LENGTH OF THE K-TH CONSTRAINT, BUT ITS SIGN INDICATES */
/*       WHETHER THE CONSTRAINT IS ACTIVE. */


/*   AUTHOR:    K. SCHITTKOWSKI, */
/*              MATHEMATISCHES INSTITUT, */
/*              UNIVERSITAET BAYREUTH, */
/*              8580 BAYREUTH, */
/*              GERMANY, F.R. */

/*   AUTHOR OF ORIGINAL VERSION: */
/*              M.J.D. POWELL, DAMTP, */
/*              UNIVERSITY OF CAMBRIDGE, SILVER STREET */
/*              CAMBRIDGE, */
/*              ENGLAND */


/*   REFERENCE: M.J.D. POWELL: ZQPCVX, A FORTRAN SUBROUTINE FOR CONVEX */
/*              PROGRAMMING, REPORT DAMTP/1983/NA17, UNIVERSITY OF */
/*              CAMBRIDGE, ENGLAND, 1983. */


/*   VERSION :  2.0 (MARCH, 1987) */


/* ************************************************************************* */

/*      DOUBLE PRECISION DMAX1,DSQRT,DABS,DMIN1 */
/*      INTEGER   MAX0,MIN0 */

/*     INITIALIZE VARIABLES THAT MAY BE UNINITIALIZED OTHERWISE */

    /* Parameter adjustments */
    --iact;
    --x;
    --xu;
    --xl;
    --grad;
    --b;
    a_dim1 = *mmax;
    a_offset = 1 + a_dim1;
    a -= a_offset;
    g_dim1 = *nmax;
    g_offset = 1 + g_dim1;
    g -= g_offset;
    --w;

    /* Function Body */
    nflag = 0;
    parinc = 0.;
    parnew = 0.;
    ratio = 0.;
    res = 0.;
    step = 0.;
    sumy = 0.;
    temp = 0.;
    j = 0;
    jflag = 0;
    kdrop = 0;
    nu = 0;
    mflag = 0;
    knext = 0;
/*      UFL  = VSMALL**6 */
/*      IF (UFL.LE.0.0D0) UFL = VSMALL**5 */
/*      IF (UFL.LE.0.0D0) UFL = VSMALL**4 */
/*      IF (UFL.LE.0.0D0) UFL = VSMALL**3 */
/*      IF (UFL.LE.0.0D0) UFL = VSMALL**2 */
/*      IF (UFL.LE.0.0D0) UFL = VSMALL */
/*      UFL = UFL*1.0D1 */
/* Computing 10th power */
    d__1 = *vsmall, d__1 *= d__1, d__2 = d__1, d__1 *= d__1;
    ufl = d__2 * (d__1 * d__1);
    if (ufl <= 0.) {
    for (i__ = 1; i__ <= 8; ++i__) {
        if (ufl <= 0.) {
        i__1 = 10 - i__;
        ufl = pow_di(vsmall, &i__1);
        }
    }
    }
/*      IF (UFL.GT.VSMALL) UFL = VSMALL */

/*   INITIAL ADDRESSES */

    iwz = *nmax;
    iwr = iwz + *nmax * *nmax;
    iww = iwr + *nmax * (*nmax + 3) / 2;
    iwd = iww + *nmax;
    iwx = iwd + *nmax;
    iwa = iwx + *nmax;

/*     SET SOME CONSTANTS. */

    zero = 0.;
    one = 1.;
    two = 2.;
    onha = 1.5;
    vfact = 1.;
    mincv = 1e30;

/*     SET SOME PARAMETERS. */
/*     NUMBER LESS THAN VSMALL ARE ASSUMED TO BE NEGLIGIBLE. */
/*     THE MULTIPLE OF I THAT IS ADDED TO G IS AT MOST DIAGR TIMES */
/*       THE LEAST MULTIPLE OF I THAT GIVES POSITIVE DEFINITENESS. */
/*     X IS RE-INITIALISED IF ITS MAGNITUDE IS REDUCED BY THE */
/*       FACTOR XMAGR. */
/*     A CHECK IS MADE FOR AN INCREASE IN F EVERY IFINC ITERATIONS, */
/*       AFTER KFINC ITERATIONS ARE COMPLETED. */

    diagr = two;
    *diag = zero;
    xmagr = .01;
    ifinc = 3;
    kfinc = max(10,*n);

/*     FIND THE RECIPROCALS OF THE LENGTHS OF THE CONSTRAINT NORMALS. */
/*     RETURN IF A CONSTRAINT IS INFEASIBLE DUE TO A ZERO NORMAL. */

    *nact = 0;
    if (*m <= 0) {
    goto L45;
    }
    i__1 = *m;
    for (k = 1; k <= i__1; ++k) {
    sum = zero;
    i__2 = *n;
    for (i__ = 1; i__ <= i__2; ++i__) {
/* Computing 2nd power */
        d__1 = a[k + i__ * a_dim1];
        sum += d__1 * d__1;
    }
    if (sum > zero) {
        goto L20;
    }
    if (b[k] == zero) {
        goto L30;
    }
    *info = -k;
    if (k <= *meq) {
        goto L730;
    }
/*      IF (B(K)) 30,30,730 */
    if (b[k] <= zero) {
        goto L30;
    } else {
        goto L730;
    }
L20:
    sum = one / sqrt(sum);
L30:
    ia = iwa + k;
    w[ia] = sum;
/* L40: */
    }
L45:
    i__1 = *n;
    for (k = 1; k <= i__1; ++k) {
    ia = iwa + *m + k;
    w[ia] = one;
    }

/*     IF NECESSARY INCREASE THE DIAGONAL ELEMENTS OF G. */

    if (! (*lql)) {
    goto L165;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    id = iwd + i__;
    w[id] = g[i__ + i__ * g_dim1];
/* Computing MAX */
    d__1 = *diag, d__2 = *vsmall - w[id];
    *diag = max(d__1,d__2);
    if (i__ == *n) {
        goto L60;
    }
    ii = i__ + 1;
    i__2 = *n;
    for (j = ii; j <= i__2; ++j) {
/* Computing MIN */
        d__1 = w[id], d__2 = g[j + j * g_dim1];
        ga = -min(d__1,d__2);
        gb = (d__1 = w[id] - g[j + j * g_dim1], abs(d__1)) + (d__2 = g[
            i__ + j * g_dim1], abs(d__2));
        if (gb > zero) {
/* Computing 2nd power */
        d__1 = g[i__ + j * g_dim1];
        ga += d__1 * d__1 / gb;
        }
        *diag = max(*diag,ga);
    }
L60:
    ;
    }
    if (*diag <= zero) {
    goto L90;
    }
L70:
    *diag = diagr * *diag;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    id = iwd + i__;
    g[i__ + i__ * g_dim1] = *diag + w[id];
    }

/*     FORM THE CHOLESKY FACTORISATION OF G. THE TRANSPOSE */
/*     OF THE FACTOR WILL BE PLACED IN THE R-PARTITION OF W. */

L90:
    ir = iwr;
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
    ira = iwr;
    irb = ir + 1;
    i__2 = j;
    for (i__ = 1; i__ <= i__2; ++i__) {
        temp = g[i__ + j * g_dim1];
        if (i__ == 1) {
        goto L110;
        }
        i__3 = ir;
        for (k = irb; k <= i__3; ++k) {
        ++ira;
        temp -= w[k] * w[ira];
        }
L110:
        ++ir;
        ++ira;
        if (i__ < j) {
        w[ir] = temp / w[ira];
        }
/* L120: */
    }
    if (temp < *vsmall) {
        goto L140;
    }
    w[ir] = sqrt(temp);
/* L130: */
    }
    goto L170;

/*     INCREASE FURTHER THE DIAGONAL ELEMENT OF G. */

L140:
    w[j] = one;
    sumx = one;
    k = j;
L150:
    sum = zero;
    ira = ir - 1;
    i__1 = j;
    for (i__ = k; i__ <= i__1; ++i__) {
    sum -= w[ira] * w[i__];
    ira += i__;
    }
    ir -= k;
    --k;
    if (k < 1) {
    goto L165;
    }
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    if (w[ir] != 0.) {
    w[k] = sum / w[ir];
    } else {
    w[k] = 1e30;
    }
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
/* Computing 2nd power */
    d__1 = w[k];
    sumx += d__1 * d__1;
    if (k >= 2) {
    goto L150;
    }
    *diag = *diag + *vsmall - temp / sumx;
    goto L70;

/*     STORE THE CHOLESKY FACTORISATION IN THE R-PARTITION */
/*     OF W. */

L165:
    ir = iwr;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    i__2 = i__;
    for (j = 1; j <= i__2; ++j) {
        ++ir;
        w[ir] = g[j + i__ * g_dim1];
    }
    }

/*     SET Z THE INVERSE OF THE MATRIX IN R. */

L170:
    nm = *n - 1;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    iz = iwz + i__;
    if (i__ > 1) {
        i__2 = i__;
        for (j = 2; j <= i__2; ++j) {
        w[iz] = zero;
        iz += *n;
        }
    }
    ir = iwr + (i__ + i__ * i__) / 2;
    w[iz] = one / w[ir];
    if (i__ == *n) {
        goto L220;
    }
    iza = iz;
    i__2 = nm;
    for (j = i__; j <= i__2; ++j) {
        ir += i__;
        sum = zero;
        i__3 = iz;
        i__4 = *n;
        for (k = iza; i__4 < 0 ? k >= i__3 : k <= i__3; k += i__4) {
        sum += w[k] * w[ir];
        ++ir;
        }
        iz += *n;
        w[iz] = -sum / w[ir];
    }
L220:
    ;
    }

/*     SET THE INITIAL VALUES OF SOME VARIABLES. */
/*     ITERC COUNTS THE NUMBER OF ITERATIONS. */
/*     ITREF IS SET TO ONE WHEN ITERATIVE REFINEMENT IS REQUIRED. */
/*     JFINC INDICATES WHEN TO TEST FOR AN INCREASE IN F. */

    iterc = 1;
    itref = 0;
    jfinc = -kfinc;

/*     SET X TO ZERO AND SET THE CORRESPONDING RESIDUALS OF THE */
/*     KUHN-TUCKER CONDITIONS. */

L230:
    iflag = 1;
    iws = iww - *n;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    x[i__] = zero;
    iw = iww + i__;
    w[iw] = grad[i__];
    if (i__ > *nact) {
        goto L240;
    }
    w[i__] = zero;
    is = iws + i__;
    k = iact[i__];
    if (k <= *m) {
        goto L235;
    }
    if (k > *mn) {
        goto L234;
    }
    k1 = k - *m;
    w[is] = xl[k1];
    goto L240;
L234:
    k1 = k - *mn;
    w[is] = -xu[k1];
    goto L240;
L235:
    w[is] = b[k];
L240:
    ;
    }
    xmag = zero;
    vfact = 1.;
/*      IF (NACT) 340,340,280 */
    if (*nact <= 0) {
    goto L340;
    } else {
    goto L280;
    }

/*     SET THE RESIDUALS OF THE KUHN-TUCKER CONDITIONS FOR GENERAL X. */

L250:
    iflag = 2;
    iws = iww - *n;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    iw = iww + i__;
    w[iw] = grad[i__];
    if (*lql) {
        goto L259;
    }
    id = iwd + i__;
    w[id] = zero;
    i__2 = *n;
    for (j = i__; j <= i__2; ++j) {
        w[id] += g[i__ + j * g_dim1] * x[j];
    }
    i__2 = i__;
    for (j = 1; j <= i__2; ++j) {
        id = iwd + j;
        w[iw] += g[j + i__ * g_dim1] * w[id];
    }
    goto L260;
L259:
    i__2 = *n;
    for (j = 1; j <= i__2; ++j) {
        w[iw] += g[i__ + j * g_dim1] * x[j];
    }
L260:
    ;
    }
    if (*nact == 0) {
    goto L340;
    }
    i__1 = *nact;
    for (k = 1; k <= i__1; ++k) {
    kk = iact[k];
    is = iws + k;
    if (kk > *m) {
        goto L265;
    }
    w[is] = b[kk];
    i__2 = *n;
    for (i__ = 1; i__ <= i__2; ++i__) {
        iw = iww + i__;
        w[iw] -= w[k] * a[kk + i__ * a_dim1];
        w[is] -= x[i__] * a[kk + i__ * a_dim1];
    }
    goto L270;
L265:
    if (kk > *mn) {
        goto L266;
    }
    k1 = kk - *m;
    iw = iww + k1;
    w[iw] -= w[k];
    w[is] = xl[k1] - x[k1];
    goto L270;
L266:
    k1 = kk - *mn;
    iw = iww + k1;
    w[iw] += w[k];
    w[is] = -xu[k1] + x[k1];
L270:
    ;
    }

/*     PRE-MULTIPLY THE VECTOR IN THE S-PARTITION OF W BY THE */
/*     INVERS OF R TRANSPOSE. */

L280:
    ir = iwr;
/*      IP=IWW+1 */
/*      IPP=IWW+N */
    il = iws + 1;
    iu = iws + *nact;
    i__1 = iu;
    for (i__ = il; i__ <= i__1; ++i__) {
    sum = zero;
    if (i__ == il) {
        goto L300;
    }
    ju = i__ - 1;
    i__2 = ju;
    for (j = il; j <= i__2; ++j) {
        ++ir;
        sum += w[ir] * w[j];
    }
L300:
    ++ir;
    w[i__] = (w[i__] - sum) / w[ir];
/* L310: */
    }

/*     SHIFT X TO SATISFY THE ACTIVE CONSTRAINTS AND MAKE THE */
/*     CORRESPONDING CHANGE TO THE GRADIENT RESIDUALS. */

    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    iz = iwz + i__;
    sum = zero;
    i__2 = iu;
    for (j = il; j <= i__2; ++j) {
        sum += w[j] * w[iz];
        iz += *n;
    }
    x[i__] += sum;
    if (*lql) {
        goto L329;
    }
    id = iwd + i__;
    w[id] = zero;
    i__2 = *n;
    for (j = i__; j <= i__2; ++j) {
        w[id] += g[i__ + j * g_dim1] * sum;
    }
    iw = iww + i__;
    i__2 = i__;
    for (j = 1; j <= i__2; ++j) {
        id = iwd + j;
        w[iw] += g[j + i__ * g_dim1] * w[id];
    }
    goto L330;
L329:
    i__2 = *n;
    for (j = 1; j <= i__2; ++j) {
        iw = iww + j;
        w[iw] += sum * g[i__ + j * g_dim1];
    }
L330:
    ;
    }

/*     FORM THE SCALAR PRODUCT OF THE CURRENT GRADIENT RESIDUALS */
/*     WITH EACH COLUMN OF Z. */

L340:
    kflag = 1;
    goto L930;
L350:
    if (*nact == *n) {
    goto L380;
    }

/*     SHIFT X SO THAT IT SATISFIES THE REMAINING KUHN-TUCKER */
/*     CONDITIONS. */

    il = iws + *nact + 1;
    iza = iwz + *nact * *n;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    sum = zero;
    iz = iza + i__;
    i__2 = iww;
    for (j = il; j <= i__2; ++j) {
        sum += w[iz] * w[j];
        iz += *n;
    }
    x[i__] -= sum;
/* L370: */
    }
    *info = 0;
    if (*nact == 0) {
    goto L410;
    }

/*     UPDATE THE LAGRANGE MULTIPLIERS. */

L380:
    lflag = 3;
    goto L740;
L390:
    i__1 = *nact;
    for (k = 1; k <= i__1; ++k) {
    iw = iww + k;
    w[k] += w[iw];
    }

/*     REVISE THE VALUES OF XMAG. */
/*     BRANCH IF ITERATIVE REFINEMENT IS REQUIRED. */

L410:
    jflag = 1;
    goto L910;
L420:
    if (iflag == itref) {
    goto L250;
    }

/*     DELETE A CONSTRAINT IF A LAGRANGE MULTIPLIER OF AN */
/*     INEQUALITY CONSTRAINT IS NEGATIVE. */

    kdrop = 0;
    goto L440;
L430:
    ++kdrop;
    if (w[kdrop] >= zero) {
    goto L440;
    }
    if (iact[kdrop] <= *meq) {
    goto L440;
    }
    nu = *nact;
    mflag = 1;
    goto L800;
L440:
    if (kdrop < *nact) {
    goto L430;
    }

/*     SEEK THE GREATEAST NORMALISED CONSTRAINT VIOLATION, DISREGARDING */
/*     ANY THAT MAY BE DUE TO COMPUTER ROUNDING ERRORS. */

L450:
    cvmax = zero;
    if (*m <= 0) {
    goto L481;
    }
    i__1 = *m;
    for (k = 1; k <= i__1; ++k) {
    ia = iwa + k;
    if (w[ia] <= zero) {
        goto L480;
    }
    sum = -b[k];
    i__2 = *n;
    for (i__ = 1; i__ <= i__2; ++i__) {
        sum += x[i__] * a[k + i__ * a_dim1];
    }
    sumx = -sum * w[ia];
    if (k <= *meq) {
        sumx = abs(sumx);
    }
    if (sumx <= cvmax) {
        goto L480;
    }
    temp = (d__1 = b[k], abs(d__1));
    i__2 = *n;
    for (i__ = 1; i__ <= i__2; ++i__) {
        temp += (d__1 = x[i__] * a[k + i__ * a_dim1], abs(d__1));
    }
    tempa = temp + abs(sum);
    if (tempa <= temp) {
        goto L480;
    }
    temp += onha * abs(sum);
    if (temp <= tempa) {
        goto L480;
    }
    cvmax = sumx;
    res = sum;
    knext = k;
L480:
    ;
    }
L481:
    i__1 = *n;
    for (k = 1; k <= i__1; ++k) {
    lower = TRUE_;
    ia = iwa + *m + k;
    if (w[ia] <= zero) {
        goto L485;
    }
    sum = xl[k] - x[k];
/*      IF (SUM) 482,485,483 */
    if (sum < zero) {
        goto L482;
    } else {
        if (sum == zero) {
        goto L485;
        } else {
        goto L483;
        }
    }
L482:
    sum = x[k] - xu[k];
    lower = FALSE_;
L483:
    if (sum <= cvmax) {
        goto L485;
    }
    cvmax = sum;
    res = -sum;
    knext = k + *m;
    if (lower) {
        goto L485;
    }
    knext = k + *mn;
L485:
    ;
    }
    if (cvmax < mincv) {
    mincv = cvmax;
    }

/*     TEST FOR CONVERGENCE */

    *info = 0;
    if (cvmax <= *vsmall) {
    goto L700;
    }

/*     RETURN IF, DUE TO ROUNDING ERRORS, THE ACTUAL CHANGE IN */
/*     X MAY NOT INCREASE THE OBJECTIVE FUNCTION */

    ++jfinc;
    if (jfinc == 0) {
    goto L510;
    }
    if (jfinc != ifinc) {
    goto L530;
    }
    fdiff = zero;
    fdiffa = zero;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    sum = two * grad[i__];
    sumx = abs(sum);
    if (*lql) {
        goto L489;
    }
    id = iwd + i__;
    w[id] = zero;
    i__2 = *n;
    for (j = i__; j <= i__2; ++j) {
        ix = iwx + j;
        w[id] += g[i__ + j * g_dim1] * (w[ix] + x[j]);
    }
    i__2 = i__;
    for (j = 1; j <= i__2; ++j) {
        id = iwd + j;
        temp = g[j + i__ * g_dim1] * w[id];
        sum += temp;
        sumx += abs(temp);
    }
    goto L495;
L489:
    i__2 = *n;
    for (j = 1; j <= i__2; ++j) {
        ix = iwx + j;
        temp = g[i__ + j * g_dim1] * (w[ix] + x[j]);
        sum += temp;
        sumx += abs(temp);
    }
L495:
    ix = iwx + i__;
    fdiff += sum * (x[i__] - w[ix]);
    fdiffa += sumx * (d__1 = x[i__] - w[ix], abs(d__1));
/* L500: */
    }
    *info = 2;
    sum = fdiffa + fdiff;
    if (sum <= fdiffa) {
    goto L700;
    }
    temp = fdiffa + onha * fdiff;
    if (temp <= sum) {
    goto L700;
    }
    jfinc = 0;
    *info = 0;
L510:
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    ix = iwx + i__;
    w[ix] = x[i__];
    }

/*     FORM THE SCALAR PRODUCT OF THE NEW CONSTRAINT NORMAL WITH EACH */
/*     COLUMN OF Z. PARNEW WILL BECOME THE LAGRANGE MULTIPLIER OF */
/*     THE NEW CONSTRAINT. */

L530:
    ++iterc;
    if (iterc <= *maxit) {
    goto L531;
    }
    *info = 1;
    goto L710;
L531:
    iws = iwr + (*nact + *nact * *nact) / 2;
    if (knext > *m) {
    goto L541;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    iw = iww + i__;
    w[iw] = a[knext + i__ * a_dim1];
    }
    goto L549;
L541:
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    iw = iww + i__;
    w[iw] = zero;
    }
    k1 = knext - *m;
    if (k1 > *n) {
    goto L545;
    }
    iw = iww + k1;
    w[iw] = one;
    iz = iwz + k1;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    is = iws + i__;
    w[is] = w[iz];
    iz += *n;
    }
    goto L550;
L545:
    k1 = knext - *mn;
    iw = iww + k1;
    w[iw] = -one;
    iz = iwz + k1;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    is = iws + i__;
    w[is] = -w[iz];
    iz += *n;
    }
    goto L550;
L549:
    kflag = 2;
    goto L930;
L550:
    parnew = zero;

/*     APPLY GIVENS ROTATIONS TO MAKE THE LAST (N-NACT-2) SCALAR */
/*     PRODUCTS EQUAL TO ZERO. */

    if (*nact == *n) {
    goto L570;
    }
    nu = *n;
    nflag = 1;
    goto L860;

/*     BRANCH IF THERE IS NO NEED TO DELETE A CONSTRAINT. */

L560:
    is = iws + *nact;
    if (*nact == 0) {
    goto L640;
    }
    suma = zero;
    sumb = zero;
    sumc = zero;
    iz = iwz + *nact * *n;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    ++iz;
    iw = iww + i__;
    suma += w[iw] * w[iz];
    sumb += (d__1 = w[iw] * w[iz], abs(d__1));
/* Computing 2nd power */
    d__1 = w[iz];
    sumc += d__1 * d__1;
/* L563: */
    }
    temp = sumb + abs(suma) * .1;
    tempa = sumb + abs(suma) * .2;
    if (temp <= sumb) {
    goto L570;
    }
    if (tempa <= temp) {
    goto L570;
    }
    if (sumb > *vsmall) {
    goto L5;
    }
    goto L570;
L5:
    sumc = sqrt(sumc);
    ia = iwa + knext;
    if (knext <= *m) {
    sumc /= w[ia];
    }
    temp = sumc + abs(suma) * .1;
    tempa = sumc + abs(suma) * .2;
    if (temp <= sumc) {
    goto L567;
    }
    if (tempa <= temp) {
    goto L567;
    }
    goto L640;

/*     CALCULATE THE MULTIPLIERS FOR THE NEW CONSTRAINT NORMAL */
/*     EXPRESSED IN TERMS OF THE ACTIVE CONSTRAINT NORMALS. */
/*     THEN WORK OUT WHICH CONTRAINT TO DROP. */

L567:
    lflag = 4;
    goto L740;
L570:
    lflag = 1;
    goto L740;

/*     COMPLETE THE TEST FOR LINEARLY DEPENDENT CONSTRAINTS. */

L571:
    if (knext > *m) {
    goto L574;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    suma = a[knext + i__ * a_dim1];
    sumb = abs(suma);
    if (*nact == 0) {
        goto L581;
    }
    i__2 = *nact;
    for (k = 1; k <= i__2; ++k) {
        kk = iact[k];
        if (kk <= *m) {
        goto L568;
        }
        kk -= *m;
        temp = zero;
        if (kk == i__) {
        temp = w[iww + kk];
        }
        kk -= *n;
        if (kk == i__) {
        temp = -w[iww + kk];
        }
        goto L569;
L568:
        iw = iww + k;
        temp = w[iw] * a[kk + i__ * a_dim1];
L569:
        suma -= temp;
        sumb += abs(temp);
/* L572: */
    }
L581:
    if (suma <= *vsmall) {
        goto L573;
    }
    temp = sumb + abs(suma) * .1;
    tempa = sumb + abs(suma) * .2;
    if (temp <= sumb) {
        goto L573;
    }
    if (tempa <= temp) {
        goto L573;
    }
    goto L630;
L573:
    ;
    }
    lflag = 1;
    goto L775;
L574:
    k1 = knext - *m;
    if (k1 > *n) {
    k1 -= *n;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    suma = zero;
    if (i__ != k1) {
        goto L575;
    }
    suma = one;
    if (knext > *mn) {
        suma = -one;
    }
L575:
    sumb = abs(suma);
    if (*nact == 0) {
        goto L582;
    }
    i__2 = *nact;
    for (k = 1; k <= i__2; ++k) {
        kk = iact[k];
        if (kk <= *m) {
        goto L579;
        }
        kk -= *m;
        temp = zero;
        if (kk == i__) {
        temp = w[iww + kk];
        }
        kk -= *n;
        if (kk == i__) {
        temp = -w[iww + kk];
        }
        goto L576;
L579:
        iw = iww + k;
        temp = w[iw] * a[kk + i__ * a_dim1];
L576:
        suma -= temp;
        sumb += abs(temp);
/* L577: */
    }
L582:
    temp = sumb + abs(suma) * .1;
    tempa = sumb + abs(suma) * .2;
    if (temp <= sumb) {
        goto L578;
    }
    if (tempa <= temp) {
        goto L578;
    }
    goto L630;
L578:
    ;
    }
    lflag = 1;
    goto L775;

/*     BRANCH IF THE CONTRAINTS ARE INCONSISTENT. */

L580:
    *info = -knext;
    if (kdrop == 0) {
    goto L700;
    }
    parinc = ratio;
    parnew = parinc;

/*     REVISE THE LAGRANGE MULTIPLIERS OF THE ACTIVE CONSTRAINTS. */

L590:
    if (*nact == 0) {
    goto L601;
    }
    i__1 = *nact;
    for (k = 1; k <= i__1; ++k) {
    iw = iww + k;
    w[k] -= parinc * w[iw];
    if (iact[k] > *meq) {
/* Computing MAX */
        d__1 = zero, d__2 = w[k];
        w[k] = max(d__1,d__2);
    }
/* L600: */
    }
L601:
    if (kdrop == 0) {
    goto L680;
    }

/*     DELETE THE CONSTRAINT TO BE DROPPED. */
/*     SHIFT THE VECTOR OF SCALAR PRODUCTS. */
/*     THEN, IF APPROPRIATE, MAKE ONE MORE SCALAR PRODUCT ZERO. */

    nu = *nact + 1;
    mflag = 2;
    goto L800;
L610:
    iws = iws - *nact - 1;
    nu = min(*n,nu);
    i__1 = nu;
    for (i__ = 1; i__ <= i__1; ++i__) {
    is = iws + i__;
    j = is + *nact;
    w[is] = w[j + 1];
    }
    nflag = 2;
    goto L860;

/*     CALCULATE THE STEP TO THE VIOLATED CONSTRAINT. */

L630:
    is = iws + *nact;
L640:
    sumy = w[is + 1];
/*      IF (SUMY.EQ.0.0D0) THEN */
    if (abs(sumy) < ufl) {
    *info = 3;
    return 0;
    }
    step = -res / sumy;
    parinc = step / sumy;
    if (*nact == 0) {
    goto L660;
    }

/*     CALCULATE THE CHANGES TO THE LAGRANGE MULTIPLIERS, AND REDUCE */
/*     THE STEP ALONG THE NEW SEARCH DIRECTION IF NECESSARY. */

    lflag = 2;
    goto L740;
L650:
    if (kdrop == 0) {
    goto L660;
    }
    temp = one - ratio / parinc;
    if (temp <= zero) {
    kdrop = 0;
    }
    if (kdrop == 0) {
    goto L660;
    }
    step = ratio * sumy;
    parinc = ratio;
    res = temp * res;

/*     UPDATE X AND THE LAGRANGE MULTIPIERS. */
/*     DROP A CONSTRAINT IF THE FULL STEP IS NOT TAKEN. */

L660:
    iwy = iwz + *nact * *n;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    iy = iwy + i__;
    x[i__] += step * w[iy];
    }
    parnew += parinc;
    if (*nact >= 1) {
    goto L590;
    }

/*     ADD THE NEW CONSTRAINT TO THE ACTIVE SET. */

L680:
    ++(*nact);
    w[*nact] = parnew;
    iact[*nact] = knext;
    ia = iwa + knext;
    if (knext > *mn) {
    ia -= *n;
    }
    w[ia] = -w[ia];

/*     ESTIMATE THE MAGNITUDE OF X. THEN BEGIN A NEW ITERATION, */
/*     RE-INITILISING X IF THIS MAGNITUDE IS SMALL. */

    jflag = 2;
    goto L910;
L690:
    if (sum < xmagr * xmag) {
    goto L230;
    }
/*      IF (ITREF) 450,450,250 */
    if (itref <= 0) {
    goto L450;
    } else {
    goto L250;
    }

/*     INITIATE ITERATIVE REFINEMENT IF IT HAS NOT YET BEEN USED, */
/*     OR RETURN AFTER RESTORING THE DIAGONAL ELEMENTS OF G. */

L700:
    if (iterc == 0) {
    goto L710;
    }
    ++itref;
    jfinc = -1;
    if (itref == 1) {
    goto L250;
    }
/* TERMINATION ACCURACY CANNOT BE REACHED DUE TO NUMERICAL INSTABILITIES */
/*  710 IF (INFO .LT. 0 .AND. MINCV .LT. 1D-8) THEN */
L710:
    if (*info < 0 && mincv < *vsmall) {
    *info = 4;
    b[1] = mincv + *vsmall;
    }
    if (! (*lql)) {
    return 0;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    id = iwd + i__;
    g[i__ + i__ * g_dim1] = w[id];
    }
L730:
    return 0;


/*     THE REMAINING INSTRUCTIONS ARE USED AS SUBROUTINES. */


/* ******************************************************************** */


/*     CALCULATE THE LAGRANGE MULTIPLIERS BY PRE-MULTIPLYING THE */
/*     VECTOR IN THE S-PARTITION OF W BY THE INVERSE OF R. */

L740:
    ir = iwr + (*nact + *nact * *nact) / 2;
    i__ = *nact;
    sum = zero;
    goto L770;
L750:
    ira = ir - 1;
    sum = zero;
    if (*nact == 0) {
    goto L761;
    }
    i__1 = *nact;
    for (j = i__; j <= i__1; ++j) {
    iw = iww + j;
    sum += w[ira] * w[iw];
    ira += j;
    }
L761:
    ir -= i__;
    --i__;
L770:
    iw = iww + i__;
    is = iws + i__;
    if ((d__1 = w[ir], abs(d__1)) == 0. || (d__2 = w[ir], abs(d__2)) < sqrt(
        ufl)) {
    *info = 3;
    goto L710;
    }
    w[iw] = (w[is] - sum) / w[ir];
    if (i__ > 1) {
    goto L750;
    }
    if (lflag == 3) {
    goto L390;
    }
    if (lflag == 4) {
    goto L571;
    }

/*     CALCULATE THE NEXT CONSTRAINT TO DROP. */

/*  775 IP=IWW+1 */
/*      IPP=IWW+NACT */
L775:
    kdrop = 0;
    if (*nact == 0) {
    goto L791;
    }
    i__1 = *nact;
    for (k = 1; k <= i__1; ++k) {
    if (iact[k] <= *meq) {
        goto L790;
    }
    iw = iww + k;
    if (res * w[iw] >= zero) {
        goto L790;
    }
/*      IF (W(IW).EQ.0.0D0) THEN */
    if ((d__1 = w[iw], abs(d__1)) < ufl) {
        *info = 3;
        return 0;
    }
    temp = w[k] / w[iw];
    if (kdrop == 0) {
        goto L780;
    }
    if (abs(temp) >= abs(ratio)) {
        goto L790;
    }
L780:
    kdrop = k;
    ratio = temp;
L790:
    ;
    }
L791:
    if (lflag == 1) {
    goto L580;
    }
    if (lflag == 2) {
    goto L650;
    }


/* ******************************************************************** */


/*     DROP THE CONSTRAINT IN POSITION KDROP IN THE ACTIVE SET. */

L800:
    ia = iwa + iact[kdrop];
    if (iact[kdrop] > *mn) {
    ia -= *n;
    }
    w[ia] = -w[ia];
    if (kdrop == *nact) {
    goto L850;
    }

/*     SET SOME INDICES AND CALCULATE THE ELEMENTS OF THE NEXT */
/*     GIVENS ROTATION. */

    iz = iwz + kdrop * *n;
    ir = iwr + (kdrop + kdrop * kdrop) / 2;
L810:
    ira = ir;
    ir = ir + kdrop + 1;
/* Computing MAX */
    d__3 = (d__1 = w[ir - 1], abs(d__1)), d__4 = (d__2 = w[ir], abs(d__2));
    temp = max(d__3,d__4);
    if (temp > *vsmall) {
/* Computing 2nd power */
    d__1 = w[ir - 1] / temp;
/* Computing 2nd power */
    d__2 = w[ir] / temp;
    sum = temp * sqrt(d__1 * d__1 + d__2 * d__2);
    } else {
    sum = *vsmall;
    }
    ga = w[ir - 1] / sum;
    gb = w[ir] / sum;

/*     EXCHANGE THE COLUMNS OF R. */

    i__1 = kdrop;
    for (i__ = 1; i__ <= i__1; ++i__) {
    ++ira;
    j = ira - kdrop;
    temp = w[ira];
    w[ira] = w[j];
    w[j] = temp;
    }
    w[ir] = zero;

/*     APPLY THE ROTATION TO THE ROWS OF R. */

    w[j] = sum;
    ++kdrop;
    i__1 = nu;
    for (i__ = kdrop; i__ <= i__1; ++i__) {
    temp = ga * w[ira] + gb * w[ira + 1];
    w[ira + 1] = ga * w[ira + 1] - gb * w[ira];
    w[ira] = temp;
    ira += i__;
    }

/*     APPLY THE ROTATION TO THE COLUMNS OF Z. */

    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    ++iz;
    j = iz - *n;
    temp = ga * w[j] + gb * w[iz];
    w[iz] = ga * w[iz] - gb * w[j];
    w[j] = temp;
    }

/*     REVISE IACT AND THE LAGRANGE MULTIPLIERS. */

    iact[kdrop - 1] = iact[kdrop];
    w[kdrop - 1] = w[kdrop];
    if (kdrop < *nact) {
    goto L810;
    }
L850:
    --(*nact);
    if (mflag == 1) {
    goto L250;
    }
    if (mflag == 2) {
    goto L610;
    }


/* ******************************************************************** */


/*     APPLY GIVENS ROTATION TO REDUCE SOME OF THE SCALAR */
/*     PRODUCTS IN THE S-PARTITION OF W TO ZERO. */

L860:
    iz = iwz + nu * *n;
L870:
    iz -= *n;
L880:
    is = iws + nu;
    --nu;
    if (nu == *nact) {
    goto L900;
    }
    if (w[is] == zero) {
    goto L870;
    }
/* Computing MAX */
    d__3 = (d__1 = w[is - 1], abs(d__1)), d__4 = (d__2 = w[is], abs(d__2));
    temp = max(d__3,d__4);
/* Computing 2nd power */
    d__1 = w[is - 1] / temp;
/* Computing 2nd power */
    d__2 = w[is] / temp;
    sum = temp * sqrt(d__1 * d__1 + d__2 * d__2);
    ga = w[is - 1] / sum;
    gb = w[is] / sum;
    w[is - 1] = sum;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    k = iz + *n;
    temp = ga * w[iz] + gb * w[k];
    w[k] = ga * w[k] - gb * w[iz];
    w[iz] = temp;
    --iz;
    }
    goto L880;
L900:
    if (nflag == 1) {
    goto L560;
    }
    if (nflag == 2) {
    goto L630;
    }


/* ******************************************************************** */


/*     CALCULATE THE MAGNITUDE OF X AN REVISE XMAG. */

L910:
    sum = zero;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    sum += (d__1 = x[i__], abs(d__1)) * vfact * ((d__2 = grad[i__], abs(
        d__2)) + (d__3 = g[i__ + i__ * g_dim1] * x[i__], abs(d__3)));
    if (*lql) {
        goto L920;
    }
    if (sum < 1e-30) {
        goto L920;
    }
    vfact *= 1e-5;
    sum *= 1e-5;
    xmag *= 1e-5;
L920:
    ;
    }
    xmag = max(xmag,sum);
    if (jflag == 1) {
    goto L420;
    }
    if (jflag == 2) {
    goto L690;
    }


/* ******************************************************************** */


/*     PRE-MULTIPLY THE VECTOR IN THE W-PARTITION OF W BY Z TRANSPOSE. */

L930:
    jl = iww + 1;
    iz = iwz;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
    is = iws + i__;
    w[is] = zero;
    iwwn = iww + *n;
    i__2 = iwwn;
    for (j = jl; j <= i__2; ++j) {
        ++iz;
        w[is] += w[iz] * w[j];
    }
/* L940: */
    }
    if (kflag == 1) {
    goto L350;
    }
    if (kflag == 2) {
    goto L550;
    }
    return 0;
} /* ql0002_ */

