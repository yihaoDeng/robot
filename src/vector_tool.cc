#include "vector_tool.h"

std::string Formate(std::vector<double> &input) {
    std::string out;
    for (auto t: input) {
        out += std::to_string(t);
        out += ',';
    }
    out += "\r";  
    return out;
}

bool CalcDotMultiply(std::vector<double> &left, std::vector<double> &right, 
        double *result) {
    if (left.size() != right.size() || left.size() != kVecDimension) {
        return false; 
    } 

    *result = left[0]*right[0] + left[1]*right[1] + left[2]*right[2];
    return true;
}
bool CalcVecMagnitude(std::vector<double> &vec, double *result) {
    bool ret = false;
    if (vec.size() != kVecDimension) {
        return ret;
    }
    double temp;
    if (true == (ret = CalcDotMultiply(vec, vec, &temp))) {
        *result = sqrt(temp);  
    } 
    
    return ret;
}

bool NormalizeVec(std::vector<double> *vec) {
    bool ret = false;
    if (vec->size() != kVecDimension) {
        return ret;
    }
    //TODO(dengyihao): should not compare with 'kAlpha'
    double magnitude; 
    if ((ret = CalcVecMagnitude(*vec, &magnitude))
            && (fabs(magnitude) > kAlpha)) {
        (*vec)[0] = (*vec)[0]/magnitude;
        (*vec)[1] = (*vec)[1]/magnitude;
        (*vec)[2] = (*vec)[2]/magnitude;
    }
    return ret;
}
bool ComputeQuadruple(std::vector<double> &left, 
        std::vector<double> &right, 
        std::vector<double> *result) {
    bool ret = false;
    if (left.size() != right.size() 
            || left.size() != kVecDimension) {
        return ret; 
    } 

    std::vector<double> axis; 
    axis.push_back(left[1] * right[2] - left[2] * right[1]); 
    axis.push_back(left[2] * right[0] - left[0] * right[2]); 
    axis.push_back(left[0] * right[1] - left[1] * right[0]); 

    double radian, dot_sum, left_Magnitude, right_Magnitude; 
    ret = NormalizeVec(&axis) 
        && CalcVecMagnitude(left, &left_Magnitude)
        && CalcVecMagnitude(right, &right_Magnitude) 
        && CalcDotMultiply(left, right, &dot_sum); 

    if (!ret) { return ret; }

    double angle = dot_sum/(left_Magnitude * right_Magnitude);  
    if (fabs(angle - static_cast<double>(1.0)) < kAlpha) {
        radian = acos(1.0);
    } else {
        radian = acos(angle);
    }

    result->push_back(cos(radian/2));
    result->push_back(axis[0] * sin(radian/2));
    result->push_back(axis[1] * sin(radian/2));
    result->push_back(axis[2] * sin(radian/2));

    return ret;
}
