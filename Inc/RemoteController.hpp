#pragma once
/**
 * @file RemoteController.hpp
 * @brief Header file for the RemoteController class.
 * Remote control via radio signals, Wi-Fi, or satellite links.
 */

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <thread>

#include "InterData.hpp"

class RemoteController {
  public:
    explicit RemoteController();
    ~RemoteController();

    explicit RemoteController(const RemoteController& rhs)   = delete;
    explicit RemoteController(RemoteController&& rhs)        = delete;
    RemoteController& operator=(const RemoteController& rhs) = delete;
    RemoteController& operator=(RemoteController&& rhs)      = delete;

    bool Initialize();

  private:
    void Step();
};