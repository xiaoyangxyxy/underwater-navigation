#pragma once

#include <array>
#include <cmath>

namespace underwater_navigation::simulation
{
struct TrajectoryParameters
{
    double start_x{0.0};
    double start_y{0.0};
    double start_z{0.0};
    double forward_speed{0.2};

    double lateral_amp{0.3};
    double lateral_freq{0.2};
    double depth_amp{0.2};
    double depth_freq{0.15};

    double roll_amp{0.05};
    double roll_freq{0.25};
    double pitch_amp{0.05};
    double pitch_freq{0.2};
    double yaw_amp{0.2};
    double yaw_freq{0.1};
};

struct Vector3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct TrajectoryState
{
    Vector3 position_world;
    Vector3 velocity_world;
    Vector3 acceleration_world;
    Vector3 body_angular_velocity;
    Vector3 body_angular_acceleration;
    std::array<double, 9> rotation_world_body{};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

class TrajectoryModel
{
public:
    explicit TrajectoryModel(TrajectoryParameters parameters)
    : parameters_(parameters)
    {
    }

    TrajectoryState evaluate(double time_s) const
    {
        const AxisState y = sinusoid(
            parameters_.start_y,
            parameters_.lateral_amp,
            parameters_.lateral_freq,
            time_s);
        const AxisState z = sinusoid(
            parameters_.start_z,
            parameters_.depth_amp,
            parameters_.depth_freq,
            time_s);
        const AxisState roll = sinusoid(
            0.0,
            parameters_.roll_amp,
            parameters_.roll_freq,
            time_s);
        const AxisState pitch = sinusoid(
            0.0,
            parameters_.pitch_amp,
            parameters_.pitch_freq,
            time_s);
        const AxisState yaw = sinusoid(
            0.0,
            parameters_.yaw_amp,
            parameters_.yaw_freq,
            time_s);

        TrajectoryState state;
        state.position_world = {
            parameters_.start_x + parameters_.forward_speed * time_s,
            y.position,
            z.position};
        state.velocity_world = {parameters_.forward_speed, y.velocity, z.velocity};
        state.acceleration_world = {0.0, y.acceleration, z.acceleration};
        state.roll = roll.position;
        state.pitch = pitch.position;
        state.yaw = yaw.position;
        state.rotation_world_body = rotationWorldBody(
            state.roll,
            state.pitch,
            state.yaw);
        state.body_angular_velocity = bodyAngularVelocity(
            state.roll,
            state.pitch,
            roll.velocity,
            pitch.velocity,
            yaw.velocity);
        state.body_angular_acceleration = bodyAngularAcceleration(
            state.roll,
            state.pitch,
            roll.velocity,
            pitch.velocity,
            yaw.velocity,
            roll.acceleration,
            pitch.acceleration,
            yaw.acceleration);
        return state;
    }

    static Vector3 worldToBody(
        const std::array<double, 9>& rotation_world_body,
        const Vector3& world_vector)
    {
        return {
            rotation_world_body[0] * world_vector.x +
                rotation_world_body[3] * world_vector.y +
                rotation_world_body[6] * world_vector.z,
            rotation_world_body[1] * world_vector.x +
                rotation_world_body[4] * world_vector.y +
                rotation_world_body[7] * world_vector.z,
            rotation_world_body[2] * world_vector.x +
                rotation_world_body[5] * world_vector.y +
                rotation_world_body[8] * world_vector.z};
    }

    static Vector3 bodyToWorld(
        const std::array<double, 9>& rotation_world_body,
        const Vector3& body_vector)
    {
        return {
            rotation_world_body[0] * body_vector.x +
                rotation_world_body[1] * body_vector.y +
                rotation_world_body[2] * body_vector.z,
            rotation_world_body[3] * body_vector.x +
                rotation_world_body[4] * body_vector.y +
                rotation_world_body[5] * body_vector.z,
            rotation_world_body[6] * body_vector.x +
                rotation_world_body[7] * body_vector.y +
                rotation_world_body[8] * body_vector.z};
    }

    static Vector3 accelerationAtBodyOffset(
        const TrajectoryState& state,
        const Vector3& offset_body)
    {
        const Vector3 tangential = cross(
            state.body_angular_acceleration,
            offset_body);
        const Vector3 centripetal = cross(
            state.body_angular_velocity,
            cross(state.body_angular_velocity, offset_body));
        const Vector3 rotational_acceleration{
            tangential.x + centripetal.x,
            tangential.y + centripetal.y,
            tangential.z + centripetal.z};
        const Vector3 rotational_acceleration_world = bodyToWorld(
            state.rotation_world_body,
            rotational_acceleration);

        return {
            state.acceleration_world.x + rotational_acceleration_world.x,
            state.acceleration_world.y + rotational_acceleration_world.y,
            state.acceleration_world.z + rotational_acceleration_world.z};
    }

private:
    struct AxisState
    {
        double position;
        double velocity;
        double acceleration;
    };

    static AxisState sinusoid(
        double offset,
        double amplitude,
        double frequency,
        double time_s)
    {
        const double phase = frequency * time_s;
        return {
            offset + amplitude * std::sin(phase),
            amplitude * frequency * std::cos(phase),
            -amplitude * frequency * frequency * std::sin(phase)};
    }

    static std::array<double, 9> rotationWorldBody(
        double roll,
        double pitch,
        double yaw)
    {
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);

        return {
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr};
    }

    static Vector3 bodyAngularVelocity(
        double roll,
        double pitch,
        double roll_rate,
        double pitch_rate,
        double yaw_rate)
    {
        const double sin_roll = std::sin(roll);
        const double cos_roll = std::cos(roll);
        const double sin_pitch = std::sin(pitch);
        const double cos_pitch = std::cos(pitch);

        return {
            roll_rate - yaw_rate * sin_pitch,
            pitch_rate * cos_roll + yaw_rate * sin_roll * cos_pitch,
            -pitch_rate * sin_roll + yaw_rate * cos_roll * cos_pitch};
    }

    static Vector3 bodyAngularAcceleration(
        double roll,
        double pitch,
        double roll_rate,
        double pitch_rate,
        double yaw_rate,
        double roll_acceleration,
        double pitch_acceleration,
        double yaw_acceleration)
    {
        const double sin_roll = std::sin(roll);
        const double cos_roll = std::cos(roll);
        const double sin_pitch = std::sin(pitch);
        const double cos_pitch = std::cos(pitch);

        return {
            roll_acceleration - yaw_acceleration * sin_pitch -
                yaw_rate * cos_pitch * pitch_rate,
            pitch_acceleration * cos_roll - pitch_rate * sin_roll * roll_rate +
                yaw_acceleration * sin_roll * cos_pitch +
                yaw_rate * (cos_roll * roll_rate * cos_pitch -
                            sin_roll * sin_pitch * pitch_rate),
            -pitch_acceleration * sin_roll - pitch_rate * cos_roll * roll_rate +
                yaw_acceleration * cos_roll * cos_pitch -
                yaw_rate * (sin_roll * roll_rate * cos_pitch +
                            cos_roll * sin_pitch * pitch_rate)};
    }

    static Vector3 cross(const Vector3& lhs, const Vector3& rhs)
    {
        return {
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
    }

    TrajectoryParameters parameters_;
};
}  // namespace underwater_navigation::simulation
