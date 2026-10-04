# MPU Unmanned Aerial Vehicle

This project implements the software for an **Unmanned Aerial Vehicle (UAV)**, focusing on modularity, thread-safe communication, and real-time control. The system is designed to manage various components such as the flight controller, payload manager, sensor manager, and telemetry systems.

This project is a recipe (`uav.bb`) for the [UAV-Yocto](https://github.com/ShayanEram/MPU_UAV-Yocto) project.

---

## **Requirements**

To build and run this project, you will need the following:

- **C++ Compiler**: A C++17-compatible compiler (e.g., GCC, Clang, or MSVC).
- **CMake**: Version 3.15 or higher.
- **Operating System**: Embedded Linux (Custom Yocto build).
- **Development Tools**: Git for version control and a terminal for running commands.

---

## **Installation and Setup**

1. Clone the repository:
   ```sh
   git clone --recursive <repo-url>
   cd MPU_Unmanned-Aerial-Vehicle
   ```

2. Install the required dependencies (if any). Ensure that CMake and a compatible compiler are installed on your system.

3. Configure the project using CMake:
   ```sh
   cmake -S . -B build
   ```

---

## **Building the Application**

### CMD
   ```sh
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   clang-tools --install 18

   conan profile detect
   conan create Inc/Hardware_Interface/third_party/conan_pigpio --build=missing
   conan export Inc/Hardware_Interface/third_party
   conan install . --build=missing
   cmake --preset conan-release
   cmake --build --preset conan-release
   ```
---

## **Expected Results**

When you run the application, the following components will be initialized and executed:

1. **Flight Controller**:
   - Periodically retrieves data from the battery, motor, remote controller, and sensors using asynchronous mechanisms.
   - Logs the status of the UAV to the console.

2. **Payload Manager**:
   - Manages payload operations and notifies the flight controller of updates using the `Observer<PayloadData>` mechanism.
   - Logs payload actions and IDs when updates occur.

3. **Sensor Manager**:
   - Collects and processes data from onboard sensors.
   - Publishes sensor data to a `MessageQueue<SensorData>` for other modules to consume.

4. **Telemetry System**:
   - Sends telemetry data to external systems for monitoring and analysis.

### **Example Output**
When the application is running, you can expect logs similar to the following:

```plaintext
FlightController running
Payload action: Drop
Payload ID: 42
FlightController running
```

---

## **Project Structure**

```
build/
    x64-debug/
CMakeLists.txt
CMakePresets.json
Inc/
    BatteryManager.hpp
    FlightController.hpp
    InterData.hpp
    main.hpp
    MessageQueue.hpp
    MotorController.hpp
    Observer.hpp
    PayloadManager.hpp
    RemoteController.hpp
    SensorManager.hpp
    SharedResource.hpp
    TelemetryManager.hpp
    ThreadManager.hpp
README.md
Src/
    BatteryManager.cpp
    FlightController.cpp
    main.cpp
    MotorController.cpp
    PayloadManager.cpp
    RemoteController.cpp
    SensorManager.cpp
    ...
```

### **Key Components**
- **Flight Controller**: Manages UAV operations, including motor control, battery monitoring, and remote commands.
- **Payload Manager**: Handles payload operations and communicates updates to the flight controller.
- **Sensor Manager**: Collects and processes sensor data for use by other modules.
- **Shared Resources and Communication**:
  - `SharedResource`: Provides thread-safe access to shared data.
  - `MessageQueue`: Implements a thread-safe queue for inter-thread communication.
  - `Observer`: Implements the observer pattern for event-driven communication between modules.
