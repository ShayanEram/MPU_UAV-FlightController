#pragma once
#include <cmath>
#include <cstdint>

namespace uav {

class MadgwickAHRS {
public:
    MadgwickAHRS(float beta = 0.1f);
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);
    void getEuler(float& roll, float& pitch, float& yaw) const;

private:
    float q0, q1, q2, q3;
    float _beta;
};
} // namespace uav
