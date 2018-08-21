#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include <iostream>
#include <iomanip>
#include <list>
#include <cmath>
#include <algorithm>

namespace rbk
{
    namespace foundation
    {
        class Vector {
        public:
            Vector() : _x(0), _y(0) {
            }

            Vector(double x, double y) : _x(x), _y(y) {
            }

            Vector(const Vector& v) : _x(v.x()), _y(v.y()) {
            }
            //rad
            Vector rotate(double angle) {
                double mag = std::sqrt(_x * _x + _y * _y);
                double new_angle = std::atan2(y(), x()) + angle;
                _x = mag * cos(new_angle);
                _y = mag * sin(new_angle);
                return Vector(_x, _y);
            }

            double mod() const {
                return std::sqrt(_x * _x + _y * _y);
            }

            double mod2() const {
                return (_x * _x + _y * _y);
            }

            double dir() const {
                return std::atan2(y(), x());
            }

            double x() const {
                return _x;
            }

            double y() const {
                return _y;
            }

            double value(double angle) const {
                return mod() * std::cos(dir() - angle);
            }

            Vector operator +(const Vector& v) const {
                return Vector(_x + v.x(), _y + v.y());
            }

            Vector operator -(const Vector& v) const {
                return Vector(_x - v.x(), _y - v.y());
            }

            Vector operator *(double a) const {
                return Vector(_x * a, _y * a);
            }

            Vector operator /(double a) const {
                return Vector(_x / a, _y / a);
            }

            Vector operator +=(const Vector& v) {
                _x = _x + v.x();
                _y = _y + v.y();
                return *this;
            }

            Vector operator -=(const Vector& v) {
                _x = _x - v.x();
                _y = _y - v.y();
                return *this;
            }

            Vector operator *=(double a) {
                _x = _x * a;
                _y = _y * a;
                return *this;
            }

            Vector operator /=(double a) {
                _x = _x / a;
                _y = _y / a;
                return *this;
            }

            Vector operator -() const {
                return Vector(-1 * _x, -1 * _y);
            }

            friend std::ostream& operator <<(std::ostream& os, const Vector& v) {
                return os << "(" << v.x() << ":" << v.y() << ")";
            }

        private:
            double _x, _y;
        };

        /************************************************************************/
        /*                       NGeoPoint                                      */
        /************************************************************************/
        class GeoPoint {
        public:
            GeoPoint() : _x(0), _y(0) {
            }

            GeoPoint(double x, double y) : _x(x), _y(y) {
            }

            GeoPoint(const GeoPoint& p) : _x(p.x()), _y(p.y()) {
            }

            bool operator==(const GeoPoint& rhs) const {
                return ((this->x() == rhs.x()) && (this->y() == rhs.y()));
            }

            bool operator<(const GeoPoint& rhs) const {
                return (this->x() < rhs.x());
            }

            bool operator>(const GeoPoint& rhs) const {
                return (this->x() > rhs.x());
            }

            double x() const {
                return _x;
            }

            double y() const {
                return _y;
            }

            double dist(const GeoPoint& p) const {
                return Vector(p - GeoPoint(_x, _y)).mod();
            }

            GeoPoint operator+(const Vector& v) const {
                return GeoPoint(_x + v.x(), _y + v.y());
            }

            GeoPoint operator*(const double& a) const {
                return GeoPoint(_x * a, _y * a);
            }

            Vector operator-(const GeoPoint& p) const {
                return Vector(_x - p.x(), _y - p.y());
            }

            friend std::ostream& operator <<(std::ostream& os, const GeoPoint& v) {
                return os << "(" << v.x() << ":" << v.y() << ")";
            }

        private:
            double _x, _y;
        };

        //class GeoOrientPos{
        //public:
        //	GeoPoint pos;
        //	double angle;

        //	GeoOrientPos(double x = 0.0 , double y = 0.0 , double a = 0.0):pos(x,y),angle(a) {}
        //};

        /************************************************************************/
        /*                        NGeoLine                                      */
        /************************************************************************/
        class GeoLine {
        public:
            GeoLine() {}
            GeoLine(const GeoPoint& p1, const GeoPoint& p2) : _p1(p1), _p2(p2) {
                calABC();
            }
            GeoLine(const GeoPoint& p, double angle) : _p1(p), _p2(p.x() + std::cos(angle), p.y() + std::sin(angle)) {
                calABC();
            }
            void calABC() {
                if (_p1.y() == _p2.y()) {
                    _a = 0;
                    _b = 1;
                    _c = -1.0 * _p1.y();
                }
                else {
                    _a = 1;
                    _b = -1.0 * (_p1.x() - _p2.x()) / (_p1.y() - _p2.y());
                    _c = (_p1.x()*_p2.y() - _p1.y()*_p2.x()) / (_p1.y() - _p2.y());
                }
            }
            GeoPoint projection(const GeoPoint& p) const {
                if (_p2.x() == _p1.x()) {
                    return GeoPoint(_p1.x(), p.y());
                }
                else {
                    // 如果该线段不平行于X轴也不平行于Y轴，则斜率存在且不为0。设线段的两端点为pt1和pt2，斜率为：
                    double k = (_p2.y() - _p1.y()) / (_p2.x() - _p1.x());
                    // 该直线方程为:					y = k* ( x - pt1.x) + pt1.y
                    // 其垂线的斜率为 -1/k,垂线方程为:	y = (-1/k) * (x - point.x) + point.y
                    // 联立两直线方程解得:
                    double x = (k * k * _p1.x() + k * (p.y() - _p1.y()) + p.x()) / (k * k + 1);
                    double y = k * (x - _p1.x()) + _p1.y();
                    return GeoPoint(x, y);
                }
            }
            GeoPoint p1() const { return _p1; }
            GeoPoint p2() const { return _p2; }
            bool operator==(const GeoLine& rhs)
            {
                return ((this->p1().x() == rhs.p1().x()) && (this->p1().y() == rhs.p1().y())
                    && (this->p2().x() == rhs.p2().x()) && (this->p2().y() == rhs.p2().y()));
            }
            const double& a() const { return _a; }
            const double& b() const { return _b; }
            const double& c() const { return _c; }
        private:
            GeoPoint _p1;
            GeoPoint _p2;

            // 直线的解析方程 a*x+b*y+c=0 为统一表示，约定 a>= 0
            double _a;
            double _b;
            double _c;
        };

        /************************************************************************/
        /*                       NGeoSegment									*/
        /************************************************************************/
        class GeoSegment : public GeoLine {
        public:
            GeoSegment() {}
            GeoSegment(const GeoPoint& p1, const GeoPoint& p2) : GeoLine(p1, p2), _start(p1), _end(p2) {
                _compareX = std::fabs(p1.x() - p2.x()) > std::fabs(p1.y() - p2.y());
            }
            bool IsPointOnLineOnSegment(const GeoPoint& p) const // 直线上的点是否在线段上
            {
                if (_compareX) {
                    return p.x() > (std::min)(_start.x(), _end.x()) && p.x() < (std::max)(_start.x(), _end.x());
                }
                return p.y() > (std::min)(_start.y(), _end.y()) && p.y() < (std::max)(_start.y(), _end.y());
            }
            const GeoPoint& start() const { return _start; }
            const GeoPoint& end() const { return _end; }

        private:
            GeoPoint _start;
            GeoPoint _end;
            bool _compareX;
        };

        class GeoLineLineIntersection {
        public:
            GeoLineLineIntersection(const GeoLine& line_1, const GeoLine& line_2) {
                double d = line_1.a() * line_2.b() - line_1.b() * line_2.a();
                if (std::fabs(d) < 0.0001) {
                    _intersectant = false;
                }
                else {
                    double px = (line_1.b() * line_2.c() - line_1.c() * line_2.b()) / d;
                    double py = (line_1.c() * line_2.a() - line_1.a() * line_2.c()) / d;
                    _point = GeoPoint(px, py);
                    _intersectant = true;
                }
            }
            bool Intersectant() const { return _intersectant; }
            const GeoPoint& IntersectPoint() const { return _point; }
        private:
            bool _intersectant;
            GeoPoint _point;
        };
    }
}
#endif // ~_GEOMETRY_H_
