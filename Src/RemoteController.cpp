// #include "RemoteController.hpp"

// #include <string>

// RemoteController::RemoteController(SharedResource<RemoteData>& remoteData) : _remoteData(remoteData), running(false) {
//     _isConnected = false;
// }
// RemoteController::~RemoteController() {
//     stop();
// }
// //------------------------------------------------------------------------------------
// void RemoteController::start() {
//     if (!running.load()) {
//         running.store(true);
//         moduleThread = std::thread(&RemoteController::runLoop, this);
//     }
// }
// void RemoteController::stop() {
//     if (running.load()) {
//         running.store(false);
//         if (moduleThread.joinable()) {
//             moduleThread.join();
//         }
//     }
// }
// //------------------------------------------------------------------------------------
// void RemoteController::runLoop() {
//     Command command = {0.0f, 0.0f, 0.0f, 0.0f, false};

//     startConnection();

//     while (running.load()) {
//         _clientSocket = accept(_serverSocket, nullptr, nullptr);
//         if (_clientSocket < 0) {
//             perror("Client connection failed");
//             continue;
//         }

//         char buffer[BUFFER_SIZE] = {0};
//         int  bytes_read          = recv(_clientSocket, buffer, sizeof(buffer) - 1, 0);
//         if (bytes_read > 0) {
//             buffer[bytes_read] = '\0';
//             std::cout << "Received command: " << buffer << std::endl;

//             // Process commands
//             command = processCommand(buffer);

//             if (_statusToSend) {
//                 std::string response = "Drone status: Operational\n";
//                 send(_clientSocket, response.c_str(), static_cast<int>(response.length()), 0);
//             }
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(REMOTE_INTERVAL_CHECK_MS));
//         std::cout << "RemoteController running..." << std::endl;
//     }
//     stopConnection();
// }
// //------------------------------------------------------------------------------------
// void RemoteController::startConnection() {
//     // Create Unix domain socket
//     _serverSocket = socket(AF_UNIX, SOCK_STREAM, 0);
//     if (_serverSocket < 0) {
//         perror("Socket creation failed");
//         return;
//     }

//     // Address
//     _serverAddress.sin_family      = AF_INET;
//     _serverAddress.sin_port        = htons(8080);
//     _serverAddress.sin_addr.s_addr = INADDR_ANY;

//     // Bind the socket
//     if (bind(_serverSocket, (struct sockaddr*)&_serverAddress, sizeof(_serverAddress)) < 0) {
//         std::cerr << "Socket bind failed: " << std::endl;
//         return;
//     }

//     // Start listening
//     if (listen(_serverSocket, 5) < 0) {
//         std::cerr << "Socket listen failed: " << std::endl;
//         return;
//     }

//     std::cout << "Drone server started, waiting for commands..." << std::endl;
// }
// void RemoteController::stopConnection() {
//     if (_serverSocket > 0 && _clientSocket > 0) {

//         close(_clientSocket);
//         close(_serverSocket);
//         std::cout << "Drone connection (server) stopped." << std::endl;
//     }
// }
// RemoteController::Command RemoteController::processCommand(const std::string& command) {
//     Command cmd = {0.0f, 0.0f, 0.0f, 0.0f, false};

//     // Parse the command string
//     if (command.find("throttle") != std::string::npos) {
//         cmd.throttle = std::stof(command.substr(command.find("=") + 1));
//     }
//     if (command.find("yaw") != std::string::npos) {
//         cmd.yaw = std::stof(command.substr(command.find("=") + 1));
//     }
//     if (command.find("pitch") != std::string::npos) {
//         cmd.pitch = std::stof(command.substr(command.find("=") + 1));
//     }
//     if (command.find("roll") != std::string::npos) {
//         cmd.roll = std::stof(command.substr(command.find("=") + 1));
//     }
//     if (command.find("releasePayload") != std::string::npos) {
//         cmd.releasePayload = command.find("true") != std::string::npos;
//     }
//     return cmd;
// }