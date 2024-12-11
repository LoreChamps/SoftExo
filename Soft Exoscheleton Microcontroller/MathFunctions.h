#ifndef MathFunctions_header
#define MathFunctions_header

#include <math.h>

#define PI 3.1415926535897932384626433832795


class Matrix
{
  public:

    float v33[3][3];  	// (3x3)
    float v31[3];		// (3x1 OR 1x3)
    int n; 				// num of columns

    // constructors
    Matrix();
    Matrix(float a11, float a12, float a13, float a21, float a22, float a23, float a31, float a32, float a33); // 3x3 Matrix
    Matrix(float a11, float a21, float a31); // 3x1 Col-Matrix
    Matrix(int ang, float val); // rotation matrix around ang (x=1, y=2, z=3) of value val

    // methods
    void zeros(int num); // zeros matrix - num = 3 (3x3), else (3x1)
    void eye();
    Matrix transpose(); // transpose (only for 3x3)
    Matrix get_col(int num); // Extract column num 

    // debug
    void plotMatrix(); // plot matrix
  
};

class AxAng
{
  public:
    float ax1;
    float ax2;
    float ax3;
    float ang;

    AxAng();
};

class YPR
{
  public:
    float yaw;
    float pitch;
    float roll;

    YPR();
};

float D2R(float deg); // Degree2Radians
float R2D(float rad); // Radians2Degree
Matrix dot_product(const Matrix &A, const Matrix &B); // Matrix multiplication  (3x3 * 3x3 OR 3x3 * 3x1)
float dot_product2(const Matrix &A, const Matrix &B); // Matrix multiplication  (1x3 * 3x1)
Matrix eul2rotm(float roll, float pitch, float yaw); // Euler Angle 2 Rotation Matrix
Matrix rotm2eul(const Matrix &M); // Rotation Matrix 2 Euler Angle
float within_angle(const Matrix &A, const Matrix &B); // Angle between two vectors
Matrix matrix_sum(const Matrix &A, const Matrix &B); // sum between matrix
void eul2axang(float yaw, float pitch, float roll, AxAng &axang); // Euler Angle 2 Axis Angle


#endif

// NOTE: const Var &VarName is needed when the VarName may be the result of a method and not a named-var (e.g., after a Var.transpose()).
// 		 Var VarName would be sufficient if passing two vars, problem is, the called function is rebuilding the struct which may be expensive!