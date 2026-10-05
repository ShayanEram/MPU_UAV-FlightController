# UAV Flight Controller

This project implements the software for an **Unmanned Aerial Vehicle (UAV)**, focusing on modularity, thread-safe communication, and real-time control. The system is designed to manage various components such as the flight controller, payload manager, sensor manager, and telemetry systems, running on a custom ARINC-653 RTOS library.

This project is a recipe (`uav.bb`) for the [UAV-Yocto](https://github.com/ShayanEram/MPU_UAV-Yocto) project to run on a microporcessor (will start running when the board is powered on by default).

---

## **Requirements**

To build and run this project, you will need the following:

- **C++ Compiler**: A C++17-compatible compiler (e.g., GCC, Clang, or MSVC).
- **CMake**: Version 3.15 or higher.
- **Operating System**: Embedded Linux (Custom Yocto build).
- **Development Tools**: Git for version control and a terminal for running commands.

---

## **Installation and Setup**

1. Clone the repository with the submodules:
   ```sh
   git clone --recursive <repo-url>
   ```

2. Install the required dependencies. Ensure that CMake and a compatible compiler are installed on your system.

3. Configure the project using CMake

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

2. **Sensor Manager**:
   - Collects and processes data from onboard sensors.

3. **Telemetry Manager**:
   - Sends telemetry data to external systems for monitoring and analysis using MAVLINK.

4. **Battery Manager**:
   - Collects and processes data from and for the ESC battery board.

5. **Remote controller**:
   - Sends and receive data to using SBUS.

6. **Motor controller**:
   - Controls actuators (servos) and the pusher motor (propellers).

---

## **NOTE**

- This project also includes libraries for A429, A664 and MILSTD-1553B for future upgrades (if needed).

- The ARINC-653 RTOS library configuration (yaml) file needs to be udpated if more modules are needed to be added to the execution.(rtos_config.yaml).