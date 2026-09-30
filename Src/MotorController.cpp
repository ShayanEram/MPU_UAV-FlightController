// #include "MotorController.hpp"

// MotorController::MotorController(SharedResource<MotorData>& motorData) : _motorData(motorData), running(false) {
//     _currentSpeed = 0;
//     _pitchSpeed   = 0;
//     _rollSpeed    = 0;
//     _yawSpeed     = 0;
// }
// MotorController::~MotorController() {
//     stop();
// }
// //------------------------------------------------------------------------------------
// void MotorController::start() {
//     if (!running.load()) {
//         running.store(true);
//         moduleThread = std::thread(&MotorController::runLoop, this);
//     }
// }
// void MotorController::stop() {
//     if (running.load()) {
//         running.store(false);
//         if (moduleThread.joinable()) {
//             moduleThread.join();
//         }
//     }
// }
// //------------------------------------------------------------------------------------
// void MotorController::runLoop() {
//     while (running.load()) {

//         auto startTime = std::chrono::high_resolution_clock::now();

//         MotorDirection direction; // = getUpdatedMotorDirection();
//         setMotorDirection(direction);

//         // Monitor Motor Status
//         for (auto motorId : {MotorID::MOTOR_1, MotorID::MOTOR_2, MotorID::MOTOR_3, MotorID::MOTOR_4}) {
//             int rpm = getMotorStatus(motorId);
//             std::cout << "Motor " << static_cast<int>(motorId) << " RPM: " << rpm << std::endl;

//             // Fail-safe trigger if needed
//             if (rpm < 0) {
//                 handleFailSafe();
//                 return;
//             }
//         }

//         std::cout << "MotorController running... Speed: " << _currentSpeed << std::endl;

//         // Sleep to maintain control loop frequency (~100Hz)
//         auto endTime     = std::chrono::high_resolution_clock::now();
//         auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10) - elapsedTime);
//     }
// }
// //------------------------------------------------------------------------------------
// void MotorController::setMotorDirection(const MotorDirection& direction) {
//     std::cout << "[Motor Control] Setting direction:" << std::endl;

//     // Adjust yaw (Rotation)
//     if (direction.yaw == YawDirection::LEFT) {
//         setMotorSpeed(MotorID::MOTOR_1, -_yawSpeed);
//         setMotorSpeed(MotorID::MOTOR_3, _yawSpeed);
//     }
//     else if (direction.yaw == YawDirection::RIGHT) {
//         setMotorSpeed(MotorID::MOTOR_1, _yawSpeed);
//         setMotorSpeed(MotorID::MOTOR_3, -_yawSpeed);
//     }

//     // Adjust pitch (Forward/Backward)
//     if (direction.pitch == PitchDirection::FORWARD) {
//         setMotorSpeed(MotorID::MOTOR_1, _pitchSpeed);
//         setMotorSpeed(MotorID::MOTOR_2, _pitchSpeed);
//     }
//     else if (direction.pitch == PitchDirection::BACKWARD) {
//         setMotorSpeed(MotorID::MOTOR_1, -_pitchSpeed);
//         setMotorSpeed(MotorID::MOTOR_2, -_pitchSpeed);
//     }

//     // Adjust roll (Left/Right)
//     if (direction.roll == RollDirection::LEFT) {
//         setMotorSpeed(MotorID::MOTOR_2, _rollSpeed);
//         setMotorSpeed(MotorID::MOTOR_4, -_rollSpeed);
//     }
//     else if (direction.roll == RollDirection::RIGHT) {
//         setMotorSpeed(MotorID::MOTOR_2, -_rollSpeed);
//         setMotorSpeed(MotorID::MOTOR_4, _rollSpeed);
//     }

//     std::cout << "Yaw: " << static_cast<int>(direction.yaw) << ", Pitch: " << static_cast<int>(direction.pitch)
//               << ", Roll: " << static_cast<int>(direction.roll) << std::endl;
// }

// int MotorController::setMotorSpeed(const MotorID& id, float throttle) {
//     int dutyCyle = static_cast<int>(throttle * 20000);
//     if (dutyCyle < MIN_SPEED) {
//         dutyCyle = MIN_SPEED;
//     }
//     else if (dutyCyle > MAX_SPEED) {
//         dutyCyle = MAX_SPEED;
//     }

//     std::string dutyCyclePath = PWM_DUTY_CYCLE_PATH + std::to_string(static_cast<int>(id));
//     writeHardware(dutyCyclePath, dutyCyle);

//     _currentSpeed = dutyCyle;

//     return dutyCyle;
// }

// void MotorController::stopMotors() {
//     for (auto motorId : {MotorID::MOTOR_1, MotorID::MOTOR_2, MotorID::MOTOR_3, MotorID::MOTOR_4}) {
//         setMotorSpeed(motorId, 0);
//     }
//     std::cout << "Motors stopped." << std::endl;
// }

// int MotorController::getMotorStatus(const MotorID& id) const {
//     std::fstream fileStream(I2C_DEVICE_PATH, std::ios::in | std::ios::out);
//     if (!fileStream.is_open()) {
//         std::cerr << "Failed to open I2C device: " << I2C_DEVICE_PATH << std::endl;
//         return -1; // or handle error appropriately
//     }

//     char buffer[12] = {0x00};
//     fileStream.write(buffer, sizeof(buffer));
//     fileStream.read(buffer, sizeof(buffer));
//     fileStream.close();

//     int rpm = buffer[0] << 8 | buffer[1];

//     return (rpm > 0) ? rpm : -1;
// }

// void MotorController::handleFailSafe() {
//     stopMotors();
// }

// void MotorController::enableMotors() {
//     writeHardware(PWM_ENABLE_PATH, 1);
// }

// void MotorController::setMPwmPeriod(int period) {
//     writeHardware(PWM_PERIOD_PATH, period);
// }

// void MotorController::writeHardware(const std::string& path, int value) {
//     std::ofstream file(path);
//     if (file.is_open()) {
//         file << value;
//         file.close();
//     }
//     else {
//         std::cerr << "Failed to open file: " << path << std::endl;
//     }
// }