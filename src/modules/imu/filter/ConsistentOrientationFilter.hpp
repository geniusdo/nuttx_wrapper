// The MIT License (MIT)
// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
#ifndef FILTER_CONSISTENTORIENTATIONFILTER_HPP_
#define FILTER_CONSISTENTORIENTATIONFILTER_HPP_

#include "ErrorStateKalmanFilter.hpp"
#include "stdio.h"
namespace Filter
{

    namespace ConsistentOrientation
    {

        using namespace EmbeddedMath;

        /// @brief error vector [dq]
        using ErrorVec = Vector3f;

        using MeasurementVec = Vector3f;

        /// @brief input vector [dt, ang_vel]
        using ControlVec = Vector4f;

        using NominalState_t = Quaternionf;


        /// @brief
        class ConsistentOrientationFilter : public ErrorStateKalmanFilter<NominalState_t, ErrorVec, MeasurementVec, ControlVec>
        {
        protected:
            NominalState_t updateNominalState(const NominalState_t &state, const ControlVec &u) override
            {
                NominalState_t nxt_state;
                Vector3f dtheta = (u.tail<3>()) * u(0);
                nxt_state = state * quat_Exp(dtheta);
                nxt_state.normalize();
                return nxt_state;
            }

            SystemJacobian updateSysJacobian(const NominalState_t &state, const ErrorVec &x, const ControlVec &u) override
            {
                SystemJacobian F = Matrix3f::Identity();
                F = state.toRotationMatrix() * u(0);
                return F;
            }

            SystemJacobian jacobianOfReset(const ErrorVec &x) override
            {
                SystemJacobian J = Matrix3f::Identity();
                J = Matrix3f::Identity() - 0.5 * skew(Vector3f(x.head<3>()));
                return J;
            }

            NominalState_t correctNominalState(const NominalState_t &state, const ErrorVec &x) override
            {
                NominalState_t corrected_state;
                corrected_state = state * quat_Exp(Vector3f(x.head<3>()));
                corrected_state.normalize();
                return corrected_state;
            }

            MeasurementJacobian updateMeasJacobian(const NominalState_t &state, const ErrorVec &x) override
            {

                MeasurementJacobian H = Matrix3f::Zero();
                Vector3f gravity_inG = Vector3f(0, 0, 1);
                H = state.toRotationMatrix().transpose() * skew(gravity_inG);
                return H;
            }

        private:
            double frequency = 200.0;

        public:
            using ESKF = ErrorStateKalmanFilter<NominalState_t, ErrorVec, MeasurementVec, ControlVec>;
            ConsistentOrientationFilter() : ESKF() {}

            /// @brief set IMU noise and random walk
            /// @param noise unit: rad/s/sqrt(Hz)
            /// @param random_walk  unit: rad/s^2/sqrt(hr)
            /// @note recommend calibrating the IMU noise and random walk using Kalibr Allan deviation calibration
            void setImuParam(const Vector3f &noise, const double frequency = 200.0)
            {
                this->frequency = frequency;
                noiseCov sysNoiseCov = Matrix3f::Zero();
                sysNoiseCov = noise.asDiagonal() * (sqrt(frequency));
                ErrorStateKalmanFilter::setSystemNoiseCovariance(sysNoiseCov);
            }

            /// @brief set measurement noise
            /// @param noise
            void setMeasurementParam(const Vector3f &noise)
            {
                ErrorStateKalmanFilter::setMeasurementNoiseCovariance(noise.asDiagonal());
            }

            void setInitialState(const Quaternionf &q)
            {
                NominalState_t state;
                state = q;
                ErrorStateKalmanFilter::setInitialState(state);
            }
        };
    }; // namespace ConsistentOrientation
}; // namesapce Filter

#endif