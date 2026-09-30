/**
 * @file PayloadManager.hpp
 * @brief Header file for the PayloadManager class.
 * This module ensures proper handling and deployment of payloads.
 */

#ifndef PAYLOADMANAGER_HPP
#define PAYLOADMANAGER_HPP

#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>

#include "InterData.hpp"

class PayloadManager {
  public:
    explicit PayloadManager(Observer<PayloadData>& payloadObserver);
    ~PayloadManager();
    void start();
    void stop();

  protected:
    enum class PayloadState { LOADED, UNLOADED, ERROR };

    void         loadPayload();
    void         unloadPayload();
    PayloadState getPayloadStatus() const;

    bool writeGPIO(const std::string& gpioPath, int value) const;
    int  readGPIO(const std::string& gpioPath) const;

    const std::string PAYLOAD_LOAD_PATH   = "/sys/class/gpio/gpio17/value";
    const std::string PAYLOAD_UNLOAD_PATH = "/sys/class/gpio/gpio18/value";

    static constexpr int PAYLOAD_CHECK_INTERVAL_MS = 1000;

    bool _isPayloadLoaded;
    bool _releasePayloadRequested;

  private:
    void              runLoop();
    std::thread       moduleThread;
    std::atomic<bool> running;

    Observer<PayloadData>& _payloadObserver;
};

#endif // PAYLOADMANAGER_HPP