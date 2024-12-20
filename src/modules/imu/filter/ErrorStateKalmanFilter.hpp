// The MIT License (MIT)
// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
#ifndef FILTER_ERRORSTATEKALMANFILTER_HPP_
#define FILTER_ERRORSTATEKALMANFILTER_HPP_

#include "KalmanFilterBase.hpp"
#include "stdio.h"
namespace Filter
{

    template <class NominalState, class ErrorState, class Measurement, class Control>
    class ErrorStateKalmanFilter : public KalmanFilterBase<ErrorState>
    {
    public:
        // static_assert(/*Measurement::RowsAtCompileTime == Dynamic ||*/ Measurement::RowsAtCompileTime > 0,
        //               "Measurement vector must contain at least 1 element" /* or be dynamic */);
        // static_assert(Measurement::ColsAtCompileTime == 1, "Measurement type must be a column vector");

        //! Error state base type
        typedef KalmanFilterBase<ErrorState> ErrorStateBase;
        //! Numeric scalar type of base
        using typename ErrorStateBase::T;
        //! Covariance of measurment matrix type
        using measurmentCov = Covariance<T, Measurement::RowsAtCompileTime>;
        //! Noise covariance matrix type
        using noiseCov = Covariance<T, ErrorState::RowsAtCompileTime>;

    protected:
        //! Kalman gain matrix type
        using KalmanGain = EmbeddedCoreType<T, ErrorState::RowsAtCompileTime, Measurement::RowsAtCompileTime>;
        //! System model jacobian matrix type
        using SystemJacobian = EmbeddedCoreType<T, ErrorState::RowsAtCompileTime, ErrorState::RowsAtCompileTime>;
        //! Measurement model jacobian matrix type
        using MeasurementJacobian = EmbeddedCoreType<T, Measurement::RowsAtCompileTime, ErrorState::RowsAtCompileTime>;
        //! Error state
        using ErrorStateBase::x;
        //! Error state covariance
        using ErrorStateBase::cov;

        virtual NominalState updateNominalState(const NominalState &state, const Control &u) = 0;

        virtual NominalState correctNominalState(const NominalState &state, const ErrorState &x) = 0;

        virtual SystemJacobian updateSysJacobian(const NominalState &state, const ErrorState &x, const Control &u) = 0;

        virtual SystemJacobian jacobianOfReset(const ErrorState &x) = 0;

        virtual MeasurementJacobian updateMeasJacobian(const NominalState &state, const ErrorState &x) = 0;

    public:
        const NominalState &predict(const Control &u)
        {
            // update nominal state;
            nominalState = updateNominalState(nominalState, u);

            // calculate transistion matrix F
            F = updateSysJacobian(nominalState, x, u);

            // update error state
            // x = F * x;  // always zero, thus omitted

            // update error state covariance
            cov = (F * cov * F.transpose() + F * systemNoiseCov * F.transpose()).eval(); // TODO: noise modeling

            return nominalState;
        }

        const NominalState &update(const Measurement &z)
        {
            // calculate measurement matrix H
            MeasurementJacobian H = updateMeasJacobian(nominalState, x);

            // calculate Kalman gain
            KalmanGain K = (cov * H.transpose() * (H * cov * H.transpose() + measurementNoiseCov).inverse()).eval();
            
            // update error state
            x += K * (z - H * x); // linearize
            
            // update error state covariance
            cov = (cov - K * H * cov).eval();
            // std::cout<<"[Correct]: Error state covariance: \n"<<cov<<std::endl;

            // update nominal state
            nominalState = correctNominalState(nominalState, x);

            // reproject covariance
            auto G = jacobianOfReset(x);
            cov = (G * cov * G.transpose()).eval();

            // reset error state
            x.setZero();

            return nominalState;
        }

    private:
        //! System model jacobian
        SystemJacobian F;
        //! Nominal state
        NominalState nominalState;
        //! System noise covariance
        noiseCov systemNoiseCov;
        //! Measurement noise covariance
        measurmentCov measurementNoiseCov;

    public:
        ErrorStateKalmanFilter()
        {
            x.setZero();
            cov.setIdentity();
        }

        void setInitialState(const NominalState &state)
        {
            nominalState = NominalState(state);
            return;
        }

        void setSystemNoiseCovariance(const noiseCov &cov)
        {
            systemNoiseCov = noiseCov(cov);
            return;
        }

        void setMeasurementNoiseCovariance(const measurmentCov &cov)
        {
            measurementNoiseCov = measurmentCov(cov);
            return;
        }
    };
}; // namespace filter
#endif