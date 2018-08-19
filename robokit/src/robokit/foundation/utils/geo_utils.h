#ifndef _GEO_UTILS_H_
#define _GEO_UTILS_H_

#include <robokit/foundation/utils/geometry.h>
#include <string>
#include <vector>

namespace rbk
{
    namespace foundation
    {
        struct Range;
        namespace math {
            extern double PI;
            extern double Epsilon;

            double round(double r);
            double roundTo(double r, int digit = 3);
			int sgn(double a);
        }
        namespace utils {
            double Normalize(double theta);
            double NormalizeDegree(double angle);
            double AverageAngle(double theta1, double theta2);
            Vector Polar2Vector(double dist, double angle);
            double Deg2Rad(double x);
            double Rad2Deg(double x);
            int distToEncoder(double dist, double r, int pos_rate);
            int QuadraticEquation(const double &a, const double &b,
                const double &c, double &s1, double &s2);
            double GaussFunc(double c, double x);
            int RangeRandom(int lb, int ub);
            double StringToDouble(std::string);
            int StringToInt(std::string);
            std::string IntToString(int);
            std::string DoubleToString(double);
            bool StringToBool(std::string str);
//            double getYaw(const tf::Quaternion& bt_q);
            bool pointInPolygon(std::vector<GeoPoint> polygon, GeoPoint p);
            bool areIntersecting(double v1x1, double v1y1, double v1x2, double v1y2,
                double v2x1, double v2y1, double v2x2, double v2y2);
        }
    }
}
#endif	// ~_GEO_UTILS_H_
