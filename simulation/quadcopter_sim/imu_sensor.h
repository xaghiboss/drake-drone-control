#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace systems {

class ImuSensor : public LeafSystem<double> {
 public:
  ImuSensor(const multibody::MultibodyPlant<double>* plant,
            const multibody::RigidBody<double>* body,
            double update_rate = 200.0);  // Hz

 private:
  // Periodic discrete update (called at fixed rate)
  void UpdateImuState(const Context<double>& context,
                      DiscreteValues<double>* discrete_state) const;
  
  // Output current IMU reading
  void CalcImuOutput(const Context<double>& context,
                     BasicVector<double>* output) const;

  const multibody::MultibodyPlant<double>* plant_;
  const multibody::RigidBody<double>* body_;
  
  // Noise parameters (realistic IMU characteristics)
  const double accel_noise_stddev_ = 0.02;    // m/s² (MPU6050 spec)
  const double gyro_noise_stddev_ = 0.01;     // rad/s
  const double accel_bias_ = 0.05;            // m/s² (constant bias)
  const double gyro_bias_ = 0.002;            // rad/s
  const double baro_noise_stddev_ = 0.05;     // m (barometer noise)
  const double gravity_ = 9.81;               // m/s² ← ADD THIS LINE!
};

}  // namespace systems
}  // namespace drake