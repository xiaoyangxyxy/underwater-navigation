#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>


#include <memory>

class ImuFactorManager
{
public:

    ImuFactorManager();

    void addMeasurement(
        const gtsam::Vector3& acc,
        const gtsam::Vector3& gyro,
        double dt
    );

    void printPreintegration() const;

    void resetIntegration();
    void resetIntegrationAndSetBias(
        const gtsam::imuBias::ConstantBias& bias
    );

    gtsam::NavState predict(const gtsam::NavState& state) const;

    const gtsam::imuBias::ConstantBias& currentBias() const;

    const gtsam::PreintegratedCombinedMeasurements&
        preintegratedMeasurements() const;

private:

    std::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params_;

    gtsam::imuBias::ConstantBias current_bias_;

    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preintegrator_;
};
