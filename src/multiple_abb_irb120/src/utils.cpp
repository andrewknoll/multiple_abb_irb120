#include "utils.hpp"

float normalize(geometry_msgs::Point point) {
    return sqrtf(point.x * point.x + point.y * point.y + point.z * point.z);
}

geometry_msgs::Point operator+ (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
    geometry_msgs::Point result;
    result.x = p1.x + p2.x;
    result.y = p1.y + p2.y;
    result.z = p1.z + p2.z;
    return result;
}

geometry_msgs::Point operator- (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
    geometry_msgs::Point result;
    result.x = p1.x - p2.x;
    result.y = p1.y - p2.y;
    result.z = p1.z - p2.z;
    return result;
}

geometry_msgs::Point operator* (const geometry_msgs::Point& p, double f){
    geometry_msgs::Point result;
    result.x = p.x * f;
    result.y = p.y * f;
    result.z = p.z * f;
    return result;
}

geometry_msgs::Point operator* (double f, const geometry_msgs::Point& p){
    return p * f;
}

geometry_msgs::Point operator/ (const geometry_msgs::Point& p, double f){
    geometry_msgs::Point result;
    result.x = p.x / f;
    result.y = p.y / f;
    result.z = p.z / f;
    return result;
}

Eigen::Matrix<float, 1, 3> toRow(const geometry_msgs::Point& p){
    Eigen::Matrix<float, 1, 3> m;
    m(0,0) = p.x;
    m(0,1) = p.y;
    m(0,2) = p.z;
    return m;
}

geometry_msgs::Point toPoint(const Eigen::Matrix<float, 1, 3> m){
    geometry_msgs::Point p;
    p.x = m(0,0);
    p.y = m(0,1);
    p.z = m(0,2);
    return p;
}