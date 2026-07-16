#include "underwater_navigation/imu/ImuFactorManager.hpp"

#include <iostream>

ImuFactorManager::ImuFactorManager()
{
    imu_params_ = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);

    imu_params_->accelerometerCovariance =
        gtsam::I_3x3 * 0.01 * 0.01;

    imu_params_->gyroscopeCovariance =
        gtsam::I_3x3 * 0.001 * 0.001;

    imu_params_->integrationCovariance =
        gtsam::I_3x3 * 0.0001 * 0.0001;

    imu_params_->biasAccCovariance =
        gtsam::I_3x3 * 0.0001 * 0.0001;

    imu_params_->biasOmegaCovariance =
        gtsam::I_3x3 * 0.00001 * 0.00001;

    imu_params_->biasAccOmegaInt =
        gtsam::Matrix::Identity(6, 6) * 1e-8;

    current_bias_ = gtsam::imuBias::ConstantBias();

    preintegrator_ =
        std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
            imu_params_,
            current_bias_
        );
}

void ImuFactorManager::addMeasurement(
    const gtsam::Vector3& acc,
    const gtsam::Vector3& gyro,
    double dt)
{
    preintegrator_->integrateMeasurement(acc, gyro, dt);
}

void ImuFactorManager::resetIntegration()
{
    preintegrator_->resetIntegration();
}

void ImuFactorManager::resetIntegrationAndSetBias(
    const gtsam::imuBias::ConstantBias& bias)
{
    current_bias_ = bias;
    preintegrator_->resetIntegrationAndSetBias(current_bias_);
}

gtsam::NavState ImuFactorManager::predict(const gtsam::NavState& state) const
{
    return preintegrator_->predict(state, current_bias_);
}

void ImuFactorManager::printPreintegration() const
{
    std::cout << "IMU preintegration result:" << std::endl;

    std::cout << "deltaT = "
              << preintegrator_->deltaTij()
              << std::endl;

    std::cout << "deltaP = "
              << preintegrator_->deltaPij().transpose()
              << std::endl;

    std::cout << "deltaV = "
              << preintegrator_->deltaVij().transpose()
              << std::endl;

    std::cout << "deltaR = " << std::endl;
    preintegrator_->deltaRij().print();
}

const gtsam::imuBias::ConstantBias& ImuFactorManager::currentBias() const
{
    return current_bias_;
}

const gtsam::PreintegratedCombinedMeasurements&
ImuFactorManager::preintegratedMeasurements() const
{
    return *preintegrator_;
}
