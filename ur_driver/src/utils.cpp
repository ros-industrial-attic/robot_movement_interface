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

#include <utils.h>

using namespace ur_driver;

//=================================================================
// JointValue
//=================================================================
JointValue::JointValue()
{

}

JointValue::JointValue(int jointCount)
{
    values.resize(jointCount, 0);
}

const std::vector<double>& JointValue::getValues()
{
    return values;
}

void JointValue::setValues(int jointCount, ...)
{
    values.resize(jointCount, 0);

    va_list arguments;
    va_start(arguments, jointCount);
    for (int i = 0; i < jointCount; i++)
    {
        values[i] = va_arg(arguments, double);
    }
    va_end (arguments);
}

std::string JointValue::toString()
{
    std::ostringstream os;
    os <<"[";
    for (int i = 0; i < values.size(); i++)
    {
        if (i != 0)
        {
            os <<", ";
        }

        os <<values[i];
    }
    os <<"]";
    return os.str();
}

double& JointValue::operator[](const size_t index)
{
    return values[index];
}

//=================================================================
// CartesianValue
//=================================================================
CartesianValue::CartesianValue()
{
    values.resize(6, 0);
}

const std::vector<double>& CartesianValue::getValues()
{
    return values;
}

void CartesianValue::setValues(double x, double y, double z, double roll, double pitch, double yaw)
{
    values.resize(6, 0);
    values[0] = x;
    values[1] = y;
    values[2] = z;
    values[3] = roll;
    values[4] = pitch;
    values[5] = yaw;
}

double& CartesianValue::x()
{
    return values[0];
}

double& CartesianValue::y()
{
    return values[1];
}

double& CartesianValue::z()
{
    return values[2];
}

double& CartesianValue::rx()
{
    return values[3];
}

double& CartesianValue::ry()
{
    return values[4];
}

double& CartesianValue::rz()
{
    return values[5];
}

double& CartesianValue::operator[](const size_t index)
{
    return values[index];
}

std::string CartesianValue::toString()
{
    std::ostringstream os;
    os <<"[" <<x() <<", " <<y() <<", " <<z() <<", " <<rx() <<", " <<ry() <<", " <<rz() <<"]";
    return os.str();
}

//=================================================================
// conversion functions
//=================================================================
tf::Vector3 ur_driver::rpyToAxis(double roll, double pitch, double yaw)
{
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll, pitch, yaw);

    return quaternion.getAngle() * quaternion.getAxis();
}

tf::Vector3 ur_driver::rpyToQuaternion(double roll, double pitch, double yaw)
{
    tf::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    tf::Vector3 v(quaternion.x(), quaternion.y(), quaternion.z());
    v.setW(quaternion.w());

    return v;
}

tf::Vector3 ur_driver::axisToRpy(double rx, double ry, double rz)
{
    tf::Vector3 e(rx, ry, rz);

    if (e.isZero())
    {
        e = tf::Vector3(1, 1, 1);
    }

    double l = e.length();
    e = e / l;

    tf::Quaternion quaternion(e, l);
    tf::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    return tf::Vector3(roll, pitch, yaw);
}

tf::Vector3 ur_driver::axisToQuaternion(double rx, double ry, double rz)
{
    tf::Vector3 e(rx, ry, rz);

    if (e.isZero())
    {
        e = tf::Vector3(1, 1, 1);
    }

    double l = e.length();
    e = e / l;

    tf::Quaternion quaternion(e, l);
    tf::Vector3 v(quaternion.x(), quaternion.y(), quaternion.z());
    v.setW(quaternion.w());

    return v;
}

tf::Vector3 ur_driver::quaternionToAxis(double x, double y, double z, double w)
{
    tf::Quaternion quaternion(x, y, z, w);

    return quaternion.getAxis() * quaternion.getAngle();
}

tf::Vector3 ur_driver::quaternionToRpy(double x, double y, double z, double w)
{
    tf::Quaternion quaternion(x, y, z, w);
    tf::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    return tf::Vector3(roll, pitch, yaw);
}
