// ----------------------------------------------------------------------------
// Copyright 2015 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
// Set of transformation functions
// ----------------------------------------------------------------------------

#ifndef UTILS_H_
#define UTILS_H_

#include <tf/tf.h>

namespace ur_driver
{
    //=================================================================
    // JointValue
    //=================================================================
    /**
     * Use JointPosition, JointVelocity or JointAcceleration
     */
    class JointValue
    {
        private:
            std::vector<double> values;

        public:
            JointValue();
            JointValue(int jointCount);

            const std::vector<double>& getValues();
            void setValues(int jointCount, ...);

            std::string toString();

            double& operator[](const size_t index);
    };

    /**
     * joint position in [rad]
     */
    typedef JointValue JointPosition;

    /**
     * joint velocity in [rad/s]
     */
    typedef JointValue JointVelocity;

    /**
     * joint acceleration in [rad/s^2]
     */
    typedef JointValue JointAcceleration;

    //=================================================================
    // CartesianValue
    //=================================================================
    /**
     * Use CartesianPosition, CartesianVelocity or CartesianAcceleration
     */
    class CartesianValue
    {
        private:
            std::vector<double> values;

        public:
            CartesianValue();

            const std::vector<double>& getValues();
            void setValues(double x, double y, double z, double roll, double pitch, double yaw);

            double& x();
            double& y();
            double& z();
            double& rx();
            double& ry();
            double& rz();

            double& operator[](const size_t index);

            std::string toString();
    };

    /**
     * Cartesian position in [m] and [rad]
     */
    typedef CartesianValue CartesianPosition;

    /**
     * Cartesian velocity in [m/s] and [rad/s]
     */
    typedef CartesianValue CartesianVelocity;

    /**
     * Cartesian accelerations in [m/s^2] and [rad/s^2]
     */
    typedef CartesianValue CartesianAcceleration;

    //=================================================================
    // conversion functions
    //=================================================================
    /**
     * Convert roll, pitch, yaw into axis angle representation.
     * @param roll
     * @param pitch
     * @param yaw
     * @return
     */
    tf::Vector3 rpyToAxis(double roll, double pitch, double yaw);

    /**
     * Convert roll, pitch, yaw into quaternion.
     * @param roll
     * @param pitch
     * @param yaw
     * @return
     */
    tf::Vector3 rpyToQuaternion(double roll, double pitch, double yaw);

    /**
     * Convert axis angle representation into roll, pitch, yaw.
     * @param rx
     * @param ry
     * @param rz
     * @return
     */
    tf::Vector3 axisToRpy(double rx, double ry, double rz);

    /**
     * Convert axis angle representation into quaternion.
     * @param rx
     * @param ry
     * @param rz
     * @return
     */
    tf::Vector3 axisToQuaternion(double rx, double ry, double rz);

    /**
     * Convert quaternion into axis angle representation.
     * @param x
     * @param y
     * @param z
     * @param w
     * @return
     */
    tf::Vector3 quaternionToAxis(double x, double y, double z, double w);

    /**
     * Convert quaternion into roll, pitch, yaw.
     * @param x
     * @param y
     * @param z
     * @param w
     * @return
     */
    tf::Vector3 quaternionToRpy(double x, double y, double z, double w);
}

#endif
