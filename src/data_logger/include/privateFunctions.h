#ifndef PRIVATEFUNCTIONS
#define PRIVATEFUNCTIONS

#include <TNT/tnt.h>
#include <TNT/jama_svd.h>
#include <TNT/jama_lu.h>
#include <TNT/jama_eig.h>
#include <iostream>
#include <vector>

/// numeric constants
#define RAD_TO_DEG       180.0d/3.14159265358979323846264d
#define DEG_TO_RAD       3.14159265358979323846264d/180.0d
#define RPM_TO_RADSEC    (2.0d*3.14159265358979323846264d)/60.0d
#define RADSEC_TO_RPM    60.0d/(2.0d*3.14159265358979323846264d)
#define HSS_PI           3.14159265358979323846264d
inline double rad2deg(double rad)
{
    return rad*(180.0/3.14159265358979323846264);
}
inline double deg2rad(double deg)
{
    return deg*(3.14159265358979323846264/180.0);
}

using namespace TNT;

namespace hss
{

inline Array2D<double> operator - (Array2D<double> const &A, double const b)
{
    int row = A.dim1();
    int col = A.dim2();

    Array2D<double> C(row, col, 0.0);
    for( int i = 0; i < col; i++)
    {
        for( int j = 0; j < row; j++)
            C[j][i] = A[j][i] - b;
    }
    return C;
}
inline Array2D<double> cosine( Array2D<double> const & A )
{
    int row = A.dim1();
    int col = A.dim2();

    Array2D<double> C(row, col, 0.0);
    for( int i = 0; i < col; i++)
    {
        for( int j = 0; j < row; j++)
            C[j][i] = cos( A[j][i] );
    }
    return C;
}
inline Array2D<double> sine( Array2D<double> const & A )
{
    int row = A.dim1();
    int col = A.dim2();

    Array2D<double> C(row, col, 0.0);
    for( int i = 0; i < col; i++)
    {
        for( int j = 0; j < row; j++)
            C[j][i] = sin( A[j][i] );
    }
    return C;
}

inline double norm2(Array2D<double> const &mat)
{
    assert(mat.dim1() == 1 || mat.dim2() == 1);
    double result = 0;
    if(mat.dim1()==1)
        for(int i=0; i<mat.dim2(); i++)
            result += pow(mat[0][i],2);
    else
        for(int i=0; i<mat.dim1(); i++)
            result += pow(mat[i][0],2);

    return sqrt(result);
}
inline Array2D<double> Identity(const int n)
{
    Array2D<double> I(n,n,0.0);
    for(int i=0; i<n; i++)
        I[i][i] = 1.0;
    return I;
}
inline Array2D<double> Ones(int row, int col)
{
    Array2D<double> result(row,col,1.0);
    return result;
}
inline Array2D<double> Zeros(int row, int col)
{
    Array2D<double> result(row,col,0.0);
    return result;
}
inline Array2D<double> transpose(Array2D<double> const &ar)
{
    int numRow = ar.dim1();
    int numCol = ar.dim2();

    Array2D<double> result(numCol,numRow);

    int i,j;
    for( i=0; i<numRow; i++)
        for(j=0; j<numCol; j++)
            result[j][i] = ar[i][j];

    return result;
}
inline Array2D<double> submat(Array2D<double> const &mat, int rowStart, int rowStop, int colStart, int colStop)
{
    assert(rowStart >= 0 && rowStop <= mat.dim1() && colStart >= 0 && colStop <= mat.dim2());

    Array2D<double> result(rowStop-rowStart+1, colStop-colStart+1);
    for(int i=0; i<=rowStop-rowStart; i++)
        for(int j=0; j<=colStop-colStart; j++)
            result[i][j] = mat[i+rowStart][j+colStart];

    return result;
}
inline void assignRows(Array2D<double> &mat, int rowStart, int rowStop, Array2D<double> const &rows)
{
    assert(rowStart >= 0 && rowStop < mat.dim1() && rows.dim1() == rowStop-rowStart+1 && rows.dim2() == mat.dim2());

    for(int r=rowStart; r<=rowStop; r++)
        for(int c=0; c<mat.dim2(); c++)
            mat[r][c] = rows[r-rowStart][c];
}
inline void assignRows(Array2D<double> &mat, int rowStart, int rowStop, double val)
{
    assert(rowStart >= 0 && rowStop < mat.dim1());

    for(int r=rowStart; r<=rowStop; r++)
        for(int c=0; c<mat.dim2(); c++)
            mat[r][c] = val;
}
inline void assignColumns(Array2D<double> &mat, int colStart, int colStop, Array2D<double> const &cols)
{
    assert(colStart >= 0 && colStop < mat.dim2() && cols.dim2() == colStop-colStart+1 && cols.dim1() == mat.dim1());

    for(int r=0; r<mat.dim1(); r++)
        for(int c=colStart; c<=colStop; c++)
            mat[r][c] = cols[r][c-colStart];
}
inline void assignColumns(Array2D<double> &mat, int colStart, int colStop, double val)
{
    assert(colStart >= 0 && colStop < mat.dim2());

    for(int r=0; r<mat.dim1(); r++)
        for(int c=colStart; c<=colStop; c++)
            mat[r][c] = val;
}
inline Array2D<double> stackVertical(Array2D<double> const &matTop, Array2D<double> const &matBottom)
{
    assert(matTop.dim2() == matBottom.dim2());

    Array2D<double> matNew(matTop.dim1()+matBottom.dim1(), matTop.dim2());
    hss::assignRows(matNew,0,matTop.dim1()-1,matTop);
    hss::assignRows(matNew,matTop.dim1(),matNew.dim1()-1,matBottom);

    return matNew;
}
inline Array2D<double> stackHorizontal(Array2D<double> const &matLeft, Array2D<double> const &matRight)
{
    assert(matLeft.dim1() == matRight.dim1());

    Array2D<double> matNew(matLeft.dim1(), matLeft.dim2()+matRight.dim2());
    hss::assignColumns(matNew, 0, matLeft.dim2()-1, matLeft);
    hss::assignColumns(matNew, matLeft.dim2(), matNew.dim2()-1, matRight);

    return matNew;
}
inline Array2D<double> repmat(Array2D<double> const &mat, int numRows, int numCols)
{

    Array2D<double> result = mat.copy();
    Array2D<double> tempHor;
    Array2D<double> tempVer;

    if(numCols > 1)
    {
        for(int j=0; j<numCols-1; j++)
        {
            tempHor = hss::stackHorizontal(result, mat);
            result = tempHor;
        }
    }
    else
        tempHor = result;

    for(int i=0; i<numRows-1; i++)
    {
        tempVer = hss::stackVertical(result, tempHor);
        result = tempVer;
    }
    return result;
}
inline void injectArray( Array2D<double> &mat, int rowStart, int colStart, Array2D<double> const &subarray)
{

    assert(rowStart + subarray.dim1() <= mat.dim1() &&  colStart + subarray.dim2() <= mat.dim2());

    for(int i=rowStart; i<rowStart+subarray.dim1(); i++)
        for(int j=colStart; j<colStart+subarray.dim2(); j++)
            mat[i][j] = subarray[i-rowStart][j-colStart];
}
inline Array2D<double> vectorize( Array2D<double> const &mat )
{

    int N = mat.dim1();
    int M = mat.dim2();
    Array2D<double> result(N*M, 1);
    for(int j=0; j<M; j++)
        injectArray( result, j*N, 0, submat(mat, 0, N-1, j, j) );
    return result;

}
inline Array2D<double> reshape( Array2D<double> const &mat, int newRows, int newCols )
{
    assert( mat.dim1()*mat.dim2() == newRows*newCols );

    Array2D<double> vec = vectorize(mat);
    Array2D<double> result(newRows, newCols, 0.0);
    for(int j=0; j<newCols; j++)
        injectArray( result, 0, j, submat(vec, j*newRows, (j+1)*newRows-1, 0, 0) );
    return result;
}
inline Array2D<double> cross(Array2D<double> const &matLeft, Array2D<double> const &matRight)
{
    assert(matLeft.dim1() == 3 && matRight.dim1() == 3 && matLeft.dim2() == 1 && matRight.dim2() == 1);

    Array2D<double> matNew(3,1);
    matNew[0][0] = matLeft[1][0]*matRight[2][0]-matLeft[2][0]*matRight[1][0];
    matNew[1][0] = matLeft[2][0]*matRight[0][0]-matLeft[0][0]*matRight[2][0];
    matNew[2][0] = matLeft[0][0]*matRight[1][0]-matLeft[1][0]*matRight[0][0];

    return matNew;
}
inline double dot( Array2D<double> const &matLeft, Array2D<double> const &matRight )
{
    assert(matLeft.dim2() == 1 && matRight.dim2() == 1 && matLeft.dim1() == matRight.dim1() );

    double res = 0;
    for(int i=0; i<matLeft.dim1(); i++)
        res += matLeft[i][0]*matRight[i][0];

    return res;
}
inline std::vector<double> array2vector( Array2D<double> const &mat )
{
    int NM = mat.dim1()*mat.dim2();
    Array2D<double> temp = vectorize( mat );
    double** d = temp;
    std::vector<double> mat_vec( (*d), (*d+NM) );
    return mat_vec;
}
inline Array2D<double> vector2array( std::vector<double> const &vec )
{
    int N = vec.size();
    Array2D<double> vec_mat(N,1,0.0);
    for(int i=0; i<N; i++)
        vec_mat[i][0] = vec[i];
    return vec_mat;
}
inline Array2D<double> vector2array( std::vector<Array2D<double>> const &vec )
{
    assert( vec[0].dim2() == 1 );

    int col = vec.size();
    int row = vec[0].dim1();

    Array2D<double> vec_mat(row, col, 0.0);
    for( int i = 0; i < col; i++ )
    {
        Array2D<double> tmp = vec[i];
        for( int j = 0; j < row; j++ )
        {
            vec_mat[j][i] = tmp[j][0];
        }
    }
    return vec_mat;
}
inline std::vector<int> setdiff( std::vector<int> &A, std::vector<int> &B )
{
    // A- B
    //assert( A.size() > B.size() );
    int nA = A.size();
    int nB = B.size();

    std::vector<int> result(nA + nB);
    std::sort( A.begin(), A.end() );
    std::sort( B.begin(), B.end() );

    std::vector<int>::iterator it = std::set_difference(A.begin(), A.end(), B.begin(), B.end(), result.begin());
    //std::set_symmetric_difference(A.begin(), A.end(), B.begin(), B.end(), result.begin());

    result.resize( it - result.begin() );
    return result;
}

inline void printMatrix(Array2D<double> const &T)
{

    int row = T.dim1();
    int col = T.dim2();
    for( int i = 0; i < row; i++ )
    {
        std::cout << "[ ";
        for( int j = 0; j < col; j++ )
        {
            std::cout << T[i][j] << " ";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << std::endl;
}
inline Array2D<double> inv( Array2D<double> const& mat )
{
    assert( mat.dim1() == mat.dim2() );
    int N = mat.dim1();
    Array2D<double> invMat = Zeros(N,N);

    JAMA::LU<double> lu(mat);
    invMat = lu.solve( Identity(N) );
    return invMat;
}
inline Array2D<double> pinv( Array2D<double> const& mat )
{
    int N = mat.dim1();
    int M = mat.dim2();
    JAMA::SVD<double> svd(mat);
    Array2D<double> U,S,V;
    svd.getS(S);
    svd.getV(V);
    svd.getU(U);
    for( int i=0; i<S.dim1(); i++)
        S[i][i] = 1.0/S[i][i];

    //Array2D<double> pmat = V*S*transpose(U);
    Array2D<double> pmat = matmult(V, matmult(S,transpose(U)));

    assert( pmat.dim1() == M && pmat.dim2() == N );
    return pmat;
}
inline Array2D<double> normalize( Array2D<double> const& mat )
{
    assert( mat.dim2() == 1 );

    double n = norm2(mat);
    return (1.0/n)*mat;
}
inline double quadprod( Array2D<double> const &x, Array2D<double> const &Q )
{
    assert( x.dim1() == Q.dim2() );
    assert( Q.dim1() == Q.dim2() );

    Array2D<double> Qx = matmult(Q, x);
    double n = dot(x, Qx);
    return n;
}
inline Array2D<double> linspace(Array2D<double> const &P1, Array2D<double> const &P2, int n )
{
    assert( P1.dim2() == 1 );
    assert( P2.dim2() == 1 );
    assert( P1.dim1() == P2.dim1() );

    int dim1 = P1.dim1();

    Array2D<double> res;
    res = Zeros(dim1, n);

    for(int i=0; i<n; i++)
        injectArray(res, 0, i, P1+( (double)i/(double)(n-1) )*(P2-P1));

    return res;
}
inline Array2D<double> linspace(double p1, double p2, int n)
{
    Array2D<double> res = Zeros(1,n);
    for( int i = 0; i < n; i++)
        res[0][i] = p1+( (double)i/(double)(n-1) )*(p2-p1);

    return res;
}
inline void kalmanFilter(Array2D<double> &x,
                    Array2D<double> &X,
                    Array2D<double> const &z,
                    Array2D<double> const &H,
                    Array2D<double> const &V,
                    Array2D<double> const &F,
                    Array2D<double> const &W)
{
    /// x : filtered state
    /// X : covariance
    /// z : measurement (const)
    /// H : measurement matrix (const)
    /// V : measurement noise (const)
    /// F : state-transition matrix (const)
    /// W : state-transition noise (const)

    assert( x.dim1() == X.dim2() );

    Array2D<double> K = matmult(matmult(inv(X) + matmult(matmult(transpose(H),inv(V)),H) ,transpose(H)),inv(V)); // kalman gain
    // measurement update
    x = x + matmult(K, (z-matmult(H,x)));
    X = inv( inv(X) + matmult(matmult(transpose(H),inv(V)),H ));
    // time update
    x = matmult(F,x);
    X = matmult(matmult(F,X),transpose(F)) + W;
}

inline Array2D<double> arrayDiv(const Array2D<double> &A, const Array2D<double> &B)
{
	int m = A.dim1();
	int n = A.dim2();

	if( B.dim1() != m || B.dim2() != n)
		return Array2D<double>();
	else
	{
		Array2D<double> C(m,n);
		for(int i=0; i<m; i++)
		{
			for(int j=0; j<n; j++)
				C[i][j] = A[i][j] / B[i][j];
		}
		return C;
	}
}

namespace so3
{
inline Array2D<double> Rz( double theta )
{
    Array2D<double> R(3,3,0.0);
    R[0][0] = cos(theta);
    R[0][1] = -sin(theta);
    R[1][0] = sin(theta);
    R[1][1] = cos(theta);
    R[2][2] = 1.0;
    return R;
}
inline Array2D<double> Rx( double theta )
{
    Array2D<double> R(3,3,0.0);
    R[1][1] = cos(theta);
    R[1][2] = -sin(theta);
    R[2][1] = sin(theta);
    R[2][2] = cos(theta);
    R[0][0] = 1.0;
    return R;
}
inline Array2D<double> Ry( double theta )
{
    Array2D<double> R(3,3,0.0);
    R[0][0] = cos(theta);
    R[2][0] = -sin(theta);
    R[0][2] = sin(theta);
    R[2][2] = cos(theta);
    R[1][1] = 1.0;
    return R;
}
inline Array2D<double> rpy2R( double phi, double theta, double psi )
{
    Array2D<double> R(3,3,0.0);
    R = matmult(Rz(psi),matmult(Ry(theta),Rx(phi)));
    return R;
}
inline Array2D<double> rpy2R( Array2D<double> const &euler )
{
    double phi = euler[0][0];
    double theta = euler[1][0];
    double psi = euler[2][0];
    return rpy2R(phi, theta, psi);
}
inline Array2D<double> R2rpy( Array2D<double> const &R )
{
    double r = atan2(R[2][1], R[2][2]);
    double p = atan2(-R[2][0], sqrt( pow(R[2][1],2) + pow(R[2][2],2) ));
    double y = atan2(R[1][0], R[0][0]);
    Array2D<double> euler(3,1,0.0);
    euler[0][0] = r;
    euler[1][0] = p;
    euler[2][0] = y;
    return euler;
}
inline Array2D<double> q2R( Array2D<double> const &q )
{
    double q0 = q[0][0];
    double q1 = q[1][0];
    double q2 = q[2][0];
    double q3 = q[3][0];
    Array2D<double> R(3,3,0.0);
    R[0][0] = pow(q0,2)+pow(q1,2)-pow(q2,2)-pow(q3,2);
    R[0][1] = 2*(q1*q2-q0*q3);
    R[0][2] = 2*(q1*q3+q0*q2);
    R[1][0] = 2*(q1*q2+q0*q3);
    R[1][1] = pow(q0,2)-pow(q1,2)+pow(q2,2)-pow(q3,2);
    R[1][2] = 2*(q2*q3-q0*q1);
    R[2][0] = 2*(q1*q3-q0*q2);
    R[2][1] = 2*(q2*q3+q0*q1);
    R[2][2] = pow(q0,2)-pow(q1,2)-pow(q2,2)+pow(q3,2);
    return R;
}
inline Array2D<double> R2q( Array2D<double> const &R )
{
    double R11 = R[0][0];
    double R12 = R[0][1];
    double R13 = R[0][2];
    double R21 = R[1][0];
    double R22 = R[1][1];
    double R23 = R[1][2];
    double R31 = R[2][0];
    double R32 = R[2][1];
    double R33 = R[2][2];

    Array2D<double> T(4,4,1.0);
    T[1][1] = -1.0;
    T[1][2] = -1.0;
    T[2][0] = -1.0;
    T[2][2] = -1.0;
    T[3][0] = -1.0;
    T[3][1] = -1.0;

    Array2D<double> diag(4,1,0.0);
    diag[0][0] = R11;
    diag[1][0] = R22;
    diag[2][0] = R33;
    diag[3][0] = 1.0;


    Array2D<double> sq = 0.25*matmult(T,diag);
    double q0 = sqrt(sq[0][0]);
    double q1 = sqrt(sq[1][0]);
    double q2 = sqrt(sq[2][0]);
    double q3 = sqrt(sq[3][0]);

    if( (q0 >= q1) && (q0 >= q2) && (q0 >= q3) )
    {
        q1 = copysign(q1, R32-R23);
        q2 = copysign(q2, R13-R31);
        q3 = copysign(q3, R21-R12);
    }
    else if( (q1 >= q0) && (q1 >= q2) && (q1 >= q3) )
    {
        q0 = copysign(q0, R32-R23);
        q2 = copysign(q2, R21+R12);
        q3 = copysign(q3, R13+R31);
    }
    else if( (q2 >= q0) && (q2 >= q1) && (q2 >= q3) )
    {
        q0 = copysign(q0, R13-R31);
        q1 = copysign(q1, R21+R12);
        q3 = copysign(q3, R32+R23);
    }
    else if( (q3 >= q0) && (q3 >= q1) && (q3 >= q2) )
    {
        q0 = copysign(q0, R21-R12);
        q1 = copysign(q1, R31+R13);
        q2 = copysign(q2, R32+R23);
    }

    Array2D<double> q(4,1,0.0);
    q[0][0] = q0;
    q[1][0] = q1;
    q[2][0] = q2;
    q[3][0] = q3;

    double n = norm2(q);
    q = (1/n)*q;
    return q;
}
inline Array2D<double> hat( Array2D<double> const &w )
{
    Array2D<double> W(3,3,0.0);
    W[0][1] = -w[2][0];
    W[0][2] = w[1][0];
    W[1][0] = w[2][0];
    W[1][2] = -w[0][0];
    W[2][0] = -w[1][0];
    W[2][1] = w[0][0];
    return W;
}
inline Array2D<double> exp( Array2D<double> const &w, double theta )
{
    Array2D<double> A(3,3,0.0);

    double n = norm2(w);
    if( n < 1e-8 )
    {
        A[0][0] = 1.0;
        A[1][1] = 1.0;
        A[2][2] = 1.0;
        return A;
    }
    else
    {
        Array2D<double> w_n = (1/n)*w;
        Array2D<double> W(3,3,0.0);
        W = hat(w_n);
        A = Identity(3) + sin(n*theta)*W + (1-cos(n*theta))*matmult(W,W);
        return A;
    }
}
}

namespace se3
{
inline Array2D<double> e2h( Array2D<double> const &e )
{
    int col = e.dim2();
    Array2D<double> ones(1,col,1.0);
    Array2D<double> h = hss::stackVertical(e, ones);

    return h;
}
inline Array2D<double> h2e( Array2D<double> const &h )
{
    int row = h.dim1();
    int col = h.dim2();

    Array2D<double> e(row-1, col, 0.0);
    Array2D<double> Q = submat(h, 0, row-2, 0, col-1);
    Array2D<double> P = repmat( submat(h,row-1,row-1,0,col-1), row-1, 1 );
    e = arrayDiv(Q,P);
    return e;
}
inline Array2D<double> trans( double x, double y, double z )
{
    Array2D<double> A = Identity(4);
    A[0][3] = x;
    A[1][3] = y;
    A[2][3] = z;
    return A;
}
inline Array2D<double> Rz( double theta )
{
    Array2D<double> R = Identity(4);
    injectArray(R, 0, 0, so3::Rz( theta ));
    return R;
}
inline Array2D<double> Ry( double theta )
{
    Array2D<double> R = Identity(4);
    injectArray(R, 0, 0, so3::Ry( theta ));
    return R;
}
inline Array2D<double> Rx( double theta )
{
    Array2D<double> R = Identity(4);
    injectArray(R, 0, 0, so3::Rx( theta ));
    return R;
}
inline Array2D<double> g2transRPY( Array2D<double> const &g )
{
    Array2D<double> xi;
    xi = hss::stackVertical( submat(g, 0, 2, 3, 3), so3::R2rpy( submat(g, 0, 2, 0, 2) ) );
    return xi;
}
inline Array2D<double> exp( Array2D<double> const &xi, double theta )
{
    Array2D<double> dg = Identity(4);
    Array2D<double> w = submat(xi, 3,5,0,0);
    Array2D<double> v = submat(xi, 0,2,0,0);
    double n = norm2(w);
    if( n < 1e-8 )
    {
        injectArray(dg, 0, 3, theta*v);
        return dg;
    }
    else
    {
        Array2D<double> w_n = (1/n)*w;
        Array2D<double> W = so3::exp(w_n, theta*n);
        Array2D<double> p = matmult((Identity(3)-W),(hss::cross(w_n,v))) + theta*(dot(w_n,v))*w_n;
        injectArray(dg, 0, 0, W);
        injectArray(dg, 0, 3, p);
        return dg;
    }
}
inline Array2D<double> dg( Array2D<double> const &xi, double dt )
{
    Array2D<double> dg = Identity(4);
    injectArray(dg, 0, 0, so3::exp(submat(xi,3,5,0,0), dt));
    injectArray(dg, 0, 3, dt*submat(xi,0,2,0,0));
    return dg;
}
inline Array2D<double> adjoint( Array2D<double> const &g )
{
    Array2D<double> Adj(6,6,0.0);

    Array2D<double> R = submat(g, 0,2,0,2);
    Array2D<double> p = submat(g, 0,2,3,3);
    Array2D<double> P = so3::hat(p);

    injectArray(Adj, 0,0, R);
    injectArray(Adj, 0,3, matmult(P,R));
    injectArray(Adj, 3,3, R);
    return Adj;
}
inline Array2D<double> inv( Array2D<double> const &G )
{
    Array2D<double> invG = Identity(4);
    injectArray(invG, 0,0, transpose(submat(G,0,2,0,2)));
    injectArray(invG, 0,3, Zeros(3,1)-matmult(transpose(submat(G,0,2,0,2)),submat(G,0,2,3,3)) );
    return invG;
}
inline Array2D<double> adjointInv( Array2D<double> const &G )
{
    Array2D<double> invAdj = Identity(6);
    Array2D<double> R = submat(G, 0,2,0,2);
    Array2D<double> p = submat(G, 0,2,3,3);
    Array2D<double> P = so3::hat(p);
    injectArray(invAdj, 0,0, transpose(R));
    injectArray(invAdj, 0,3, (Zeros(3,3)-matmult(transpose(R),P)));
    injectArray(invAdj, 3,3, transpose(R));
    return invAdj;
}
}

} // hss

#endif // PRIVATE2
