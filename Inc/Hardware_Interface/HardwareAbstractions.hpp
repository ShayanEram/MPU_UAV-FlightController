#pragma once
#include <unistd.h>

#include <cstdint>
#include <vector>

namespace HW {
    /**
     * Generic I2C bus abstraction
     */
    class I2C {
      public:
        I2C()                          = default;
        virtual ~I2C()                 = default;
        explicit I2C(const I2C& rhs)   = delete;
        explicit I2C(I2C&& rhs)        = delete;
        I2C& operator=(const I2C& rhs) = delete;
        I2C& operator=(I2C&& rhs)      = delete;

        virtual bool Read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len)        = 0;
        virtual bool Write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) = 0;
    };

    /**
     *Generic UART port abstraction
     */
    class Uart {
      public:
        Uart()                           = default;
        virtual ~Uart()                  = default;
        explicit Uart(const Uart& rhs)   = delete;
        explicit Uart(Uart&& rhs)        = delete;
        Uart& operator=(const Uart& rhs) = delete;
        Uart& operator=(Uart&& rhs)      = delete;

        virtual bool    Open(const char* device, uint32_t baud)             = 0;
        virtual void    Close()                                             = 0;
        virtual ssize_t Read(uint8_t* buf, size_t len, uint32_t timeout_ms) = 0;
        virtual ssize_t Write(const uint8_t* buf, size_t len)               = 0;
    };

    /**
     *PWM output abstraction for servos/ESCs
     */
    class Pwm {
      public:
        Pwm()                          = default;
        virtual ~Pwm()                 = default;
        explicit Pwm(const Pwm& rhs)   = delete;
        explicit Pwm(Pwm&& rhs)        = delete;
        Pwm& operator=(const Pwm& rhs) = delete;
        Pwm& operator=(Pwm&& rhs)      = delete;

        // channel index 0..N-1, pulseWidth in microseconds (1000..2000 typical)
        virtual bool SetPulseWidth(size_t channel, uint16_t pulse_width_us) = 0;
        virtual bool SetDutyCycle(size_t channel, float duty)               = 0; // 0..1
    };

    /**
     *ADC reader abstraction for direct voltage/current sensing
     */
    class Adc {
      public:
        Adc()                          = default;
        virtual ~Adc()                 = default;
        explicit Adc(const Adc& rhs)   = delete;
        explicit Adc(Adc&& rhs)        = delete;
        Adc& operator=(const Adc& rhs) = delete;
        Adc& operator=(Adc&& rhs)      = delete;

        virtual bool ReadVoltage(float& volts) = 0;
        virtual bool ReadCurrent(float& amps)  = 0;
    };

    /**
     *PPM / SBUS input abstraction for RC receivers
     */
    class Rc {
      public:
        Rc()                         = default;
        virtual ~Rc()                = default;
        explicit Rc(const Rc& rhs)   = delete;
        explicit Rc(Rc&& rhs)        = delete;
        Rc& operator=(const Rc& rhs) = delete;
        Rc& operator=(Rc&& rhs)      = delete;

        // Read raw channel values normalized to -1..1 or 0..1 depending on implementation
        virtual bool ReadChannels(std::vector<float>& channels) = 0;
    };
} // namespace HW