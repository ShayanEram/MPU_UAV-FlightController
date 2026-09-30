#pragma once
/**
 * @file PayloadManager.hpp
 * @brief Header file for the PayloadManager class.
 * This module ensures proper handling and deployment of payloads.
 */

#include <spdlog/spdlog.h>

#include <atomic>
#include <thread>

#include "InterData.hpp"

class PayloadManager {
  public:
    explicit PayloadManager();
    ~PayloadManager();

    explicit PayloadManager(const PayloadManager& rhs)   = delete;
    explicit PayloadManager(PayloadManager&& rhs)        = delete;
    PayloadManager& operator=(const PayloadManager& rhs) = delete;
    PayloadManager& operator=(PayloadManager&& rhs)      = delete;

    static bool Initialize() {
        spdlog::debug("This class not used yet!");
        return true;
    }

  private:
    void Step();
};