#include "MathFunctions.h"
using namespace std;


// ------------------------------------- Class Matrix methods

Matrix::Matrix()
{
}

Matrix::Matrix(float v00, float v01, float v02) // constructor 3x1
{
    this->v31[0] = v00;
    this->v31[1] = v01;
    this->v31[2] = v02;
    this->n = 1;
}

Matrix::Matrix(float v00, float v01, float v02, float v10, float v11, float v12, float v20, float v21, float v22) // constructor 3x3
{
    this->v33[0][0] = v00;
    this->v33[0][1] = v01;
    this->v33[0][2] = v02;
    this->v33[1][0] = v10;
    this->v33[1][1] = v11;
    this->v33[1][2] = v12;
    this->v33[2][0] = v20;
    this->v33[2][1] = v21;
    this->v33[2][2] = v22;
    this->n = 3;
}

Matrix::Matrix(int ang, float val) // rotation matrix around ang (x=1, y=2, z=3) of value val
{
  val = D2R(val);

  this->n = 3;

  if (ang==1) // rotation around x
  {
    this->v33[0][0] = 1;
      this->v33[0][1] = 0;
      this->v33[0][2] = 0;
      this->v33[1][0] = 0;
      this->v33[1][1] = cos(val);
      this->v33[1][2] = -sin(val);
      this->v33[2][0] = 0;
      this->v33[2][1] = sin(val);
      this->v33[2][2] = cos(val);
  }
  else if(ang==2) // rotation around y
  {
    this->v33[0][0] = cos(val);
      this->v33[0][1] = 0;
      this->v33[0][2] = sin(val);
      this->v33[1][0] = 0;
      this->v33[1][1] = 1;
      this->v33[1][2] = 0;
      this->v33[2][0] = -sin(val);
      this->v33[2][1] = 0;
      this->v33[2][2] = cos(val);
  }
  else  // rotation around z
  {
    this->v33[0][0] = cos(val);
      this->v33[0][1] = -sin(val);
      this->v33[0][2] = 0;
      this->v33[1][0] = sin(val);
      this->v33[1][1] = cos(val);
      this->v33[1][2] = 0;
      this->v33[2][0] = 0;
      this->v33[2][1] = 0;
      this->v33[2][2] = 1;
  }
}


void Matrix::zeros(int num)
{
  if(num==3){
    this->v33[0][0] = 0;
      this->v33[0][1] = 0;
      this->v33[0][2] = 0;
      this->v33[1][0] = 0;
      this->v33[1][1] = 0;
      this->v33[1][2] = 0;
      this->v33[2][0] = 0;
      this->v33[2][1] = 0;
      this->v33[2][2] = 0;
      this->n = 3;
  }
  else{
    this->v31[0] = 0;
      this->v31[1] = 0;
      this->v31[2] = 0;
      this->n = 1;
  }
}

void Matrix::eye()
{
  this->v33[0][0] = 1;
    this->v33[0][1] = 0;
    this->v33[0][2] = 0;
    this->v33[1][0] = 0;
    this->v33[1][1] = 1;
    this->v33[1][2] = 0;
    this->v33[2][0] = 0;
    this->v33[2][1] = 0;
    this->v33[2][2] = 1;
    this->n = 3;
}

Matrix Matrix::transpose() // Matrix transpose (only 3x3)
{
  Matrix tmp;
  
  tmp.n = 3;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++) 
        {
            tmp.v33[j][i] = this->v33[i][j];
        }    
    }
    return tmp;
}

Matrix Matrix::get_col(int num)
{
  Matrix tmp;
  tmp.n = 1;

  for (int i = 0; i < 3; ++i)
  {
    tmp.v31[i] = this->v33[i][num];
  }

  return tmp;
}


// void Matrix::plotMatrix()
// {
//     if (this->n == 3)
//     {
//         // cout << "3x3 Matrix" << endl;
//         for(int i = 0; i < 3; ++i)
//         {
//             for(int j = 0; j < 3; ++j)
//             {
//                 cout << this->v33[i][j] << " ";
//             }
//             cout << endl;
//         }

//     }
//     else
//     {
//         // cout << "3x1 Matrix" << endl;
//         for(int i = 0; i < 3; ++i)
//         {
//             cout << this->v31[i] << endl;
//         }
//     }
// }

// ---------------------------------- class AxAng methods

AxAng::AxAng()
{
  this->ax1 = 0;
  this->ax2 = 0;
  this->ax3 = 0;
  this->ang = 0;
}

YPR::YPR()
{
  this->yaw = 0;
  this->pitch = 0;
  this->roll = 0;
}

void eul2axang(float yaw, float pitch, float roll, AxAng &axang)
{
  float c1 = cos(yaw/2);
  float s1 = sin(yaw/2);
  float c2 = cos(pitch/2);
  float s2 = sin(pitch/2);
  float c3 = cos(roll/2);
  float s3 = sin(roll/2);
  float w = c1*c2*c3 - s1*s2*s3;
  float x = c1*c2*s3 + s1*s2*c3;
  float y = s1*c2*c3 + c1*s2*s3;
  float z = c1*s2*c3 - s1*c2*s3;
  float norm = x*x + y*y + z*z;

  axang.ang = 2*acos(w);
  axang.ax1 = x;
  axang.ax2 = y;
  axang.ax3 = z;
  
  if (norm < 0.001) { // when all euler angles are zero angle we can set axis to anything to avoid divide by zero
    axang.ax1 = 1;
    axang.ax2 = 0;
    axang.ax3 = 0;
  } 
  else{
    norm = sqrt(norm);
      axang.ax1 /= norm;
      axang.ax2 /= norm;
      axang.ax3 /= norm;
    }
}

// ---------------------------------- Not class Matrix methods

float D2R(float d)
{
    return d*(PI/180);
}

float R2D(float r)
{
    return r*(180/PI);
}

Matrix dot_product(const Matrix &A, const Matrix &B) // Matrix multiplication (3x3 * 3x3 OR 3x3 * 3x1)
{
  Matrix tmp;

  if(B.n == 3){
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        tmp.v33[i][j] = 0;
        for (int k = 0; k < 3; ++k)
        {
          tmp.v33[i][j] += A.v33[i][k] * B.v33[k][j];
        }
      }
    }
    tmp.n = 3;
    return tmp;
  }
  else{
    for (int i = 0; i < 3; ++i)
    {
      tmp.v31[i] = 0;
      for (int j = 0; j < 3; ++j)
      {
        tmp.v31[i] += A.v33[i][j] * B.v31[j];
      }
    }
    tmp.n = 1;
    return tmp;
  }

}

float dot_product2(const Matrix &A, const Matrix &B) // Matrix multiplication (1x3 * 3x1)
{
  float tmp = 0;
  
  for (int i = 0; i < 3; ++i)
  {
    tmp += A.v31[i] * B.v31[i];
  }
        
  return tmp;
}

Matrix eul2rotm(float r, float p, float y)
{
  Matrix tmp;
  tmp.n = 3;

  tmp.v33[0][0] = cos(p)*cos(r);
  tmp.v33[1][0] = cos(p)*sin(r);
  tmp.v33[2][0] = -sin(p);

  tmp.v33[0][1] = sin(p)*sin(y)*cos(r) - sin(r)*cos(y);
  tmp.v33[1][1] = sin(p)*sin(y)*sin(r) + cos(r)*cos(y);
  tmp.v33[2][1] = cos(p)*sin(y);

  tmp.v33[0][2] = sin(p)*cos(y)*cos(r) + sin(r)*sin(y);
  tmp.v33[1][2] = sin(p)*cos(y)*sin(r) - cos(r)*sin(y);
  tmp.v33[2][2] = cos(p)*cos(y);

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      if(fabs(tmp.v33[i][j]) < 0.00001){tmp.v33[i][j] = 0;}
    }
  }

  return tmp;
}

Matrix rotm2eul(const Matrix &M) // Rotation Matrix 2 Euler Angle
{
  Matrix tmp;
  tmp.n = 1;

  tmp.v31[0] = atan2(M.v33[1][0],M.v33[0][0]);
  tmp.v31[1] = atan2(-M.v33[2][0], sqrt(pow(M.v33[2][1],2) + pow(M.v33[2][2],2)));
  tmp.v31[2] = atan2(M.v33[2][1],M.v33[2][2]);

  for (int i = 0; i < 3; ++i)
  {
    if(fabs(tmp.v31[i]) < 0.00001){tmp.v31[i] = 0;}
  }

  return tmp;
}

float within_angle(const Matrix &A, const Matrix &B)
{
  float tmp;

  float dot_prod = 0;
  float mod_a = 0;
  float mod_b = 0;
  
  for (int i = 0; i < 3; ++i)
  {
    dot_prod += A.v31[i] * B.v31[i];
  }
  mod_a = sqrt(pow(A.v31[0],2) + pow(A.v31[1],2) + pow(A.v31[2],2));
  mod_b = sqrt(pow(B.v31[0],2) + pow(B.v31[1],2) + pow(B.v31[2],2));

  tmp = acos(dot_prod/(mod_a * mod_b));

  return tmp;
}

Matrix matrix_sum(const Matrix &A, const Matrix &B)
{
  Matrix tmp;
  tmp.n = 1;

  tmp.v31[0] = A.v31[0] + B.v31[0];
  tmp.v31[1] = A.v31[1] + B.v31[1];
  tmp.v31[2] = A.v31[2] + B.v31[2]; 

  return tmp;
}














// // testing matrix 
// int main(int argc, char const *argv[])
// {

  // Matrix M(1,2,3,4,5,6,7,8,9);
  // M.plotMatrix();

  // Matrix A(2,4,6);
  // A.plotMatrix();

  // Matrix C;
  // C = Matrix::mat_mult(M,A);

  // C.plotMatrix();

  // float tmp = C.v31[1];

  // cout << tmp << endl;

  // M.get_col(1).plotMatrix();

  // Matrix A;
  // A = Matrix::eul2rotm(pi/2,pi/4,pi);
  // A.plotMatrix();
  
  // cout << "\n";

  // Matrix::rotm2eul(A).plotMatrix();

  // 	Matrix A(1,2,3);
  // 	cout << Matrix::dot_product2(A,A) << endl;

  // 	return 0;
// }