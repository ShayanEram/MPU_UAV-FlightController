#pragma once
#include <cmath>

namespace HW {
    class MadgwickAHRS {
      public:
        MadgwickAHRS(float beta = FILTER_GAIN);
        ~MadgwickAHRS() = default;

        explicit MadgwickAHRS(const MadgwickAHRS& rhs)   = delete;
        explicit MadgwickAHRS(MadgwickAHRS&& rhs)        = delete;
        MadgwickAHRS& operator=(const MadgwickAHRS& rhs) = delete;
        MadgwickAHRS& operator=(MadgwickAHRS&& rhs)      = delete;

        void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
        void GetEuler(float& roll, float& pitch, float& yaw) const;

      private:
        float m_q0, m_q1, m_q2, m_q3;
        float m_beta;

        static constexpr auto FILTER_GAIN = 0.1F;
    };
} // namespace HW