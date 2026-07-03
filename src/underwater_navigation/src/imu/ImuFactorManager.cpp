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
