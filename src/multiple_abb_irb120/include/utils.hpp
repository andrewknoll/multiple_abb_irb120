#include <geometry_msgs/Point.h>
#include <Eigen/Core>

float normalize(geometry_msgs::Point point);
geometry_msgs::Point operator+ (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
geometry_msgs::Point operator- (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
geometry_msgs::Point operator* (const geometry_msgs::Point& p, double f);
geometry_msgs::Point operator* (double f, const geometry_msgs::Point& p);
geometry_msgs::Point operator/ (const geometry_msgs::Point& p, double f);

Eigen::Matrix<float, 1, 3> toRow(const geometry_msgs::Point& p);
geometry_msgs::Point toPoint(const Eigen::Matrix<float, 1, 3> m);