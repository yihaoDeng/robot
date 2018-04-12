#ifndef _VECTOR_TOOL_H_
#define _VECTOR_TOOL_H_

#include <math.h>
#include <string>
#include <vector>

const int kServerPort = 8089;
const double kAlpha = static_cast<double>(0.000001);
const size_t kVecDimension = 3;

std::string Formate(std::vector<double> &input); 

 
bool CalcDotMultiply(std::vector<double> &left, 
        std::vector<double> &right, double *result); 
bool CalcVecMagnitude(std::vector<double> &vec, double *result);
bool NormalizeVec(std::vector<double> *vec);
bool ComputeQuadruple(std::vector<double> &left, 
        std::vector<double> &right, 
        std::vector<double> *result); 
#endif
