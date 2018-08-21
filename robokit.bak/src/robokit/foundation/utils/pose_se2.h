#ifndef _POSE_SE2_H_
#define _POSE_SE2_H_

#include <robokit/foundation/utils/geo_utils.h>
namespace rbk
{
    namespace foundation {
        /**
          * @class PoseSE2
          * @brief This class implements a pose in the domain SE2: \f$ \mathbb{R}^2 \times S^1 \f$
          * The pose consist of the position x and y and an orientation given as angle theta [-pi, pi].
          */
        class PoseSE2
        {
        public:

            /** @name Construct PoseSE2 instances */
            ///@{
            /**
              * @brief Default constructor
              */
            PoseSE2()
            {
                setZero();
            }

            /**
              * @brief Construct pose given a position vector and an angle theta
              * @param position 2D position vector
              * @param theta angle given in rad
              */
            PoseSE2(double x, double y, double theta)
            {
                //_position = position;
                _position_x = x;
                _position_y = y;
                _theta = theta;
            }

            /**
              * @brief Construct pose using single components x, y, and the yaw angle
              * @param x x-coordinate
              * @param y y-coordinate
              * @param theta yaw angle in rad
              */
              //PoseSE2(double x, double y, double theta)
              //{
              //    _position.coeffRef(0) = x;
              //    _position.coeffRef(1) = y;
              //    _theta = theta;
              //}

              /**
               * @brief Construct pose using a geometry_msgs::Pose
               * @param pose geometry_msgs::Pose object
               */
               //PoseSE2(const rbk::protocol::Message_Pose& pose)
               //{
               //    _position.coeffRef(0) = pose.position().x();//pose.position.x;
               //    _position.coeffRef(1) = pose.position().y();

                  // tf::Quaternion bt_q;
                  // //quaternionMsgToTF(msg_q, bt_q);
                  // bt_q = tf::Quaternion(pose.orientation().x(), pose.orientation().y(), pose.orientation().z(), pose.orientation().w());
                  // if (fabs(bt_q.length2() - 1 ) > 0.1)
                  // {
                     //  LogWarn("MSG to TF: Quaternion Not Properly Normalized");
                     //  bt_q.normalize();
                  // }

                  // tfScalar useless_pitch, useless_roll, yaw;
                  // tf::Matrix3x3(bt_q).getRPY( useless_roll, useless_pitch,yaw);
                  //

               //    _theta = yaw;
                  // //_theta = tf::getYaw( pose.orientation );
               //}

               /**
                * @brief Construct pose using a tf::Pose
                * @param pose tf::Pose object
                */
                //PoseSE2(const tf::Pose& pose)
                //{
                //    _position.coeffRef(0) = pose.getOrigin().getX();
                //    _position.coeffRef(1) = pose.getOrigin().getY();
                //    _theta = tf::getYaw( pose.getRotation() );
                //}

                /**
                  * @brief Copy constructor
                  * @param pose PoseSE2 instance
                  */
                  //PoseSE2(const PoseSE2& pose)
                  //{
                  //    _position = pose._position;
                  //    _theta = pose._theta;
                  //}

                  ///@}

                  /**
                    * @brief Destructs the PoseSE2
                    */
            ~PoseSE2() {}

            /** @name Access and modify values */
            ///@{
            /**
              * @brief Access the 2D position part
              * @see estimate
              * @return reference to the 2D position part
              */
              //Eigen::Vector2d& position() {return _position;}

              ///**
              //  * @brief Access the 2D position part (read-only)
              //  * @see estimate
              //  * @return const reference to the 2D position part
              //  */
              //const Eigen::Vector2d& position() const {return _position;}

              /**
                * @brief Access the x-coordinate the pose
                * @return reference to the x-coordinate
                */
            double x() { return _position_x/*_position.coeffRef(0)*/; }

            /**
              * @brief Access the x-coordinate the pose (read-only)
              * @return const reference to the x-coordinate
              */
            const double x() const { return _position_x/*_position.coeffRef(0)*/; }

            /**
              * @brief Access the y-coordinate the pose
              * @return reference to the y-coordinate
              */
            double y() { return _position_y/* _position.coeffRef(1)*/; }

            /**
              * @brief Access the y-coordinate the pose (read-only)
              * @return const reference to the y-coordinate
              */
            const double y() const { return _position_y/*_position.coeffRef(1)*/; }

            /**
              * @brief Access the orientation part (yaw angle) of the pose
              * @return reference to the yaw angle
              */
            double theta() { return _theta; }

            /**
              * @brief Access the orientation part (yaw angle) of the pose (read-only)
              * @return const reference to the yaw angle
              */
            const double theta() const { return _theta; }

            /**
              * @brief Set pose to [0,0,0]
              */
            void setZero()
            {
                //_position.setZero();
                _position_x = 0;
                _position_y = 0;
                _theta = 0;
            }

            /**
             * @brief Convert PoseSE2 to a geometry_msgs::Pose
             * @param[out] pose Pose message
             */
             // void toPoseMsg(rbk::protocol::Message_Pose& pose) const
             // {
             //   pose.mutable_position()->set_x(_position.x());
             //   pose.mutable_position()->set_y( _position.y());
             //   pose.mutable_position()->set_z(0);
                //tf::Quaternion q;
                //q.setRPY(0.0, 0.0, _theta);
                ////rbk::Quaternion q_msg;
                ////tf::quaternionTFToMsg(q, q_msg);
                //if (fabs(q.length2() - 1 ) > 0.1) {
                //	LogWarn("TF to MSG: Quaternion Not Properly Normalized");
                //	tf::Quaternion bt_temp = q;
                //	bt_temp.normalize();
                //	pose.mutable_orientation()->set_x( bt_temp.x());
                //	pose.mutable_orientation()->set_y(bt_temp.y());
                //	pose.mutable_orientation()->set_z(bt_temp.z());
                //	pose.mutable_orientation()->set_w( bt_temp.w());
                //}else{
                //	/*msg.x = q.x(); msg.y = q.y(); msg.z = q.z();  msg.w = q.w();*/
                //	pose.mutable_orientation()->set_x( q.x());
                //	pose.mutable_orientation()->set_y(q.y());
                //	pose.mutable_orientation()->set_z(q.z());
                //	pose.mutable_orientation()->set_w( q.w());
                //}

             //   //pose.mutable_orientation()->set = tf::createQuaternionMsgFromYaw(_theta);
             // }

              /**
               * @brief Return the unit vector of the current orientation
               * @returns [cos(theta), sin(theta))]^T
               */
               //Eigen::Vector2d orientationUnitVec() const {return Eigen::Vector2d(std::cos(_theta), std::sin(_theta));}

               ///@}

               /** @name Arithmetic operations for which operators are not always reasonable */
               ///@{
               /**
                 * @brief Scale all SE2 components (x,y,theta) and normalize theta afterwards to [-pi, pi]
                 * @param factor scale factor
                 */
            void scale(double factor);
            /*  {
            _position_x *= factor;
            _position_y *= factor;
            _theta = utils::Normalize( _theta*factor );
            }*/

            /**
              * @brief Increment the pose by adding a double[3] array
              * The angle is normalized afterwards
              * @param pose_as_array 3D double array [x, y, theta]
              */
            void plus(const double* pose_as_array);
            /*  {
            _position_x += pose_as_array[0];
            _position_y += pose_as_array[1];
            _theta = utils::Normalize( _theta + pose_as_array[2] );
            }
            */
            /**
              * @brief Get the mean / average of two poses and store it in the caller class
              * For the position part: 0.5*(x1+x2)
              * For the angle: take the angle of the mean direction vector
              * @param pose1 first pose to consider
              * @param pose2 second pose to consider
              */
            void averageInPlace(const PoseSE2& pose1, const PoseSE2& pose2);
            /* {
            _position_x = (pose1.x() + pose2.x())/2;
            _position_y = (pose1.y() + pose2.y())/2;
            _theta = utils::AverageAngle(pose1.theta(), pose2.theta());
            }*/

            /**
              * @brief Get the mean / average of two poses and return the result (static)
              * For the position part: 0.5*(x1+x2)
              * For the angle: take the angle of the mean direction vector
              * @param pose1 first pose to consider
              * @param pose2 second pose to consider
              * @return mean / average of \c pose1 and \c pose2
              */
              /*	  static PoseSE2 average(const PoseSE2& pose1, const PoseSE2& pose2)
                    {
                      return PoseSE2( (pose1.x() + pose2.x())/2.0 ,(pose1.y() + pose2.y())/2.0  ,utils::AverageAngle(pose1.theta(), pose2.theta()) );
                    }*/

                    ///@}

                    /** @name Operator overloads / Allow some arithmetic operations */
                    ///@{
                    /**
                      * @brief Asignment operator
                      * @param rhs PoseSE2 instance
                      * @todo exception safe version of the assignment operator
                      */
            PoseSE2& operator=(const PoseSE2& rhs)
            {
                if (&rhs != this)
                {
                    _position_x = rhs.x();
                    _position_y = rhs.y();
                    _theta = rhs.theta();
                }
                return *this;
            }

            /**
              * @brief Compound assignment operator (addition)
              * @param rhs addend
              */
            PoseSE2& operator+=(const PoseSE2& rhs)
            {
                _position_x += rhs.x();
                _position_y += rhs.y();
                _theta = utils::Normalize(_theta + rhs.theta());
                return *this;
            }

            /**
            * @brief Arithmetic operator overload for additions
            * @param lhs First addend
            * @param rhs Second addend
            */
            friend PoseSE2 operator+(PoseSE2 lhs, const PoseSE2& rhs)
            {
                return lhs += rhs;
            }

            /**
              * @brief Compound assignment operator (subtraction)
              * @param rhs value to subtract
              */
            PoseSE2& operator-=(const PoseSE2& rhs)
            {
                _position_x -= rhs.x();
                _position_y -= rhs.y();
                _theta = utils::Normalize(_theta - rhs.theta());
                return *this;
            }

            /**
            * @brief Arithmetic operator overload for subtractions
            * @param lhs First term
            * @param rhs Second term
            */
            friend PoseSE2 operator-(PoseSE2 lhs, const PoseSE2& rhs)
            {
                return lhs -= rhs;
            }

            /**
              * @brief Multiply pose with scalar and return copy without normalizing theta
              * This operator is useful for calculating velocities ...
              * @param pose pose to scale
              * @param scalar factor to multiply with
              * @warning theta is not normalized after multiplying
              */
            friend PoseSE2 operator*(PoseSE2 pose, double scalar)
            {
                pose._position_x *= scalar;
                pose._position_y *= scalar;
                pose._theta *= scalar;
                return pose;
            }

            /**
              * @brief Multiply pose with scalar and return copy without normalizing theta
              * This operator is useful for calculating velocities ...
              * @param scalar factor to multiply with
              * @param pose pose to scale
              * @warning theta is not normalized after multiplying
              */
            friend PoseSE2 operator*(double scalar, PoseSE2 pose)
            {
                pose._position_x *= scalar;
                pose._position_y *= scalar;
                pose._theta *= scalar;
                return pose;
            }

            /**
               * @brief Output stream operator
               * @param stream output stream
               * @param pose to be used
               */
            friend std::ostream& operator<< (std::ostream& stream, const PoseSE2& pose)
            {
                stream << "x: " << pose._position_x << " y: " << pose._position_y << " theta: " << pose._theta;
                return stream;
            }

            ///@}

        public:

            //Eigen::Vector2d _position;
            double _position_x;
            double _position_y;
            double _theta;

            //public:
              //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    } // namespace foundation
}

#endif
