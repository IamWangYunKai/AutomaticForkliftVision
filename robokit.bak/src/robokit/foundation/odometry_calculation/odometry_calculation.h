#ifndef _ODOMETRY_CALCULATION_H_
#define _ODOMETRY_CALCULATION_H_

#include <robokit/foundation/velocity_transformation/velocity_transformation.h>

namespace rbk
{
    namespace foundation
    {
        class OdoCal {
        public:
            OdoCal(VelTrans* vel_trans);
            ~OdoCal();
            void initialize();
            void onTimerCal2(int p1, int p2, int max_value, bool reset = false);
            void onTimerCalDiff(double p1, double p2, int max_value);
            void getCurOdo(double& x, double& y, double& angle, bool& is_stop);
            void getCurLocalVel(double& lvx, double& lvy, double& lw);
            void getCurGlobalVel(double& gvx, double& gvy, double& gw);
        private:
            void doFileRecord();

        private:
            /// Record odometer
            double m_x;
            double m_y;
            double m_angle;
            bool m_is_stop;

            /// Record all-wheels
            double m_w1;
            double m_w2;

            /// Record real-vel : local
            double m_lvx;
            double m_lvy;
            double m_lw;

            /// Record real-vel : global
            double m_gvx;
            double m_gvy;
            double m_gw;

            /// Record cal time step : ms
            double m_cal_time_step;
            //rbk::base::Mutex m_is_stop_lock;
            VelTrans* m_vel_trans;
        };

        /// Singleton class
        //typedef NormalSingleton < COdoCal > OdoCal;
    }
}
#endif	// ~_ODOMETRY_CALCULATION_H_
