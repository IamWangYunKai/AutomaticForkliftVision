#ifndef _VELOCITY_TRANSFORMATION_H_
#define _VELOCITY_TRANSFORMATION_H_

namespace rbk
{
    namespace foundation
    {
        //////////////////////////////////////////////////////////////////////////
        /// wheels group
        struct WheelVel {
            double w1;
            double w2;
        };

        struct WheelPos {
            double p1;
            double p2;
        };

        //////////////////////////////////////////////////////////////////////////
        /// planar
        struct PlanarVel {
            double vx;
            double vy;
            double w;
        };

        struct PlanarPos {
            double x;
            double y;
            double theta;
        };

        class VelTrans {
        public:
            /// Constructor
            VelTrans();

            /// Destructor
            ~VelTrans();

            /// Initial
            void initialze(double e, double d, double r, int vel_reduce_rate, int pos_reduce_rate);

            /// Forward Kinematics
            void forwardKinematicsTrans(const WheelVel& wheelVel, PlanarVel& planarVel);

            /// Forward Kinematics
            void forwardKinematicsTrans(const WheelPos& wheelPos, PlanarPos& planarPos);

            /// Inverse Kinematics
            void inverseKinematicsTrans(const PlanarVel& planarVel, WheelVel& wheelVel);

        private:
            /// Forward Kinematics - Diff
            void forwardKinematicsTransDiff(const WheelVel& wheelVel, PlanarVel& planarVel);

            /// Forward Kinematics - Diff
            void forwardKinematicsTransDiff(const WheelPos& wheelPos, PlanarPos& planarPos);

            /// Inverse Kinematics - Diff
            void inverseKinematicsTransDiff(const PlanarVel& planarVel, WheelVel& wheelVel);

        private:
            /// Print for test
            void debugPrint(const PlanarVel& planarVel, const WheelVel& wheelVel, bool forward = true);

        private:
            /// Distance from left to right
            double 					m_e;

            /// Distance from front to back
            double 					m_d;

            /// Radius of the single wheel
            double 					m_r;

            /// PI
            double 					m_pi;

            /// Arc length
            double					m_scale;

            bool print_debug;
            double m_pos_reduce_rate;
            double m_vel_reduce_rate;
        };

        /// Singleton class
        //typedef NormalSingleton < CVelTrans > VelTrans;
    }
}
#endif	// ~_VELOCITY_TRANSFORMATION_H_
