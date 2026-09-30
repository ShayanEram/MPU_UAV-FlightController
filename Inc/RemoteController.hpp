/**
 * @file RemoteController.hpp
 * @brief Header file for the RemoteController class.
 * Remote control via radio signals, Wi-Fi, or satellite links.
 */

#ifndef REMOTE_CONTROLLER_HPP
#define REMOTE_CONTROLLER_HPP

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>

#include "InterData.hpp"

class RemoteController {
  public:
    explicit RemoteController(SharedResource<RemoteData>& _remoteData);
    ~RemoteController();
    void start();
    void stop();

  protected:
    struct Command {
        float throttle;
        float yaw;
        float pitch;
        float roll;
        bool  releasePayload;
    };

    void    startConnection();
    void    stopConnection();
    Command processCommand(const std::string& command);

    static constexpr int BUFFER_SIZE              = 1024;
    static constexpr int REMOTE_INTERVAL_CHECK_MS = 10;

    static constexpr const char* SOCKET_PATH = "/tmp/remote_controller_socket";
    bool                         _isConnected;
    int                          _serverSocket, _clientSocket;
    bool                         _statusToSend;
    sockaddr_in                  _serverAddress;

  private:
    void              runLoop();
    std::thread       moduleThread;
    std::atomic<bool> running;

    SharedResource<RemoteData>& _remoteData;
};

#endif // REMOTE_CONTROLLER_HPP