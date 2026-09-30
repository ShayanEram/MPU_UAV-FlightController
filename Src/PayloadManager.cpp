#include "PayloadManager.hpp"

PayloadManager::PayloadManager(Observer<PayloadData>& payloadObserver) : _payloadObserver(payloadObserver), running(false) {
    _isPayloadLoaded         = false;
    _releasePayloadRequested = false;
}

PayloadManager::~PayloadManager() {
    stop();
}
//------------------------------------------------------------------------------------
void PayloadManager::start() {
    if (!running.load()) {
        running.store(true);
        moduleThread = std::thread(&PayloadManager::runLoop, this);
    }
}
void PayloadManager::stop() {
    if (running.load()) {
        running.store(false);
        if (moduleThread.joinable()) {
            moduleThread.join();
        }
    }
}
//------------------------------------------------------------------------------------
void PayloadManager::runLoop() {
    while (running.load()) {

        PayloadState status = getPayloadStatus();

        if (status == PayloadState::LOADED && !_isPayloadLoaded) {
            loadPayload();
            _isPayloadLoaded = true;
        }
        else if (status == PayloadState::UNLOADED && _isPayloadLoaded) {
            _isPayloadLoaded = false;

            if (_releasePayloadRequested) {
                unloadPayload();
                _releasePayloadRequested = false;
            }
            else {
                std::cout << "Payload is unloaded, but no release requested." << std::endl;
            }
        }
        else if (status == PayloadState::ERROR) {
            std::cerr << "Error in payload operation!" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(PAYLOAD_CHECK_INTERVAL_MS));

        std::cout << "PayloadManager running... Payload operations in progress." << std::endl;
    }
}
//------------------------------------------------------------------------------------
void PayloadManager::loadPayload() {
    if (writeGPIO(PAYLOAD_LOAD_PATH, 1)) {
        std::cout << "Payload loaded successfully!" << std::endl;
    }
    else {
        std::cerr << "Failed to load payload!" << std::endl;
    }
}

void PayloadManager::unloadPayload() {
    if (writeGPIO(PAYLOAD_UNLOAD_PATH, 1)) {
        std::cout << "Payload unloaded successfully!" << std::endl;
    }
    else {
        std::cerr << "Failed to unload payload!" << std::endl;
    }
}

PayloadManager::PayloadState PayloadManager::getPayloadStatus() const {
    int loadState   = readGPIO(PAYLOAD_LOAD_PATH);
    int unloadState = readGPIO(PAYLOAD_UNLOAD_PATH);

    if (loadState == 1) {
        std::cout << "Payload Status: **Loaded**" << std::endl;
        return PayloadState::LOADED;
    }
    else if (unloadState == 1) {
        std::cout << "Payload Status: **Unloaded**" << std::endl;
        return PayloadState::UNLOADED;
    }
    else {
        std::cout << "Payload Status: **Idle**" << std::endl;
        return PayloadState::ERROR; // Assuming error if neither loaded nor unloaded
    }

    return PayloadState::ERROR; // Fallback in case of unexpected state
}

bool PayloadManager::writeGPIO(const std::string& gpioPath, int value) const {
    std::ofstream gpioFile(gpioPath);
    if (gpioFile.is_open()) {
        gpioFile << value;
        gpioFile.close();
        return true;
    }
    return false;
}

int PayloadManager::readGPIO(const std::string& gpioPath) const {
    std::ifstream gpioFile(gpioPath);
    int           value = -1;
    if (gpioFile.is_open()) {
        gpioFile >> value;
        gpioFile.close();
    }
    return value;
}