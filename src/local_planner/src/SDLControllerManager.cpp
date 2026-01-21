#include "SDLControllerManager.h"
#include <sstream>
#include <cmath>

SDLControllerManager::SDLControllerManager() : initialized(false), debugPrint(true) {}

SDLControllerManager::~SDLControllerManager() {
    shutdown();
}

bool SDLControllerManager::init() {
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "Failed to initialize SDL2: " << SDL_GetError() << std::endl;
        return false;
    }

    loadControllerDatabase();
    initialized = true;

    std::cout << "[SDL Controller Manager] SDL2 initialized successfully" << std::endl;
    std::cout << "[SDL Controller Manager] Number of joysticks: " << SDL_NumJoysticks() << std::endl;

    // Automatically add any already connected controllers
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            onControllerAdded(i);
        }
    }

    return true;
}

void SDLControllerManager::shutdown() {
    for (auto controller : controllers) {
        if (controller) {
            SDL_GameControllerClose(controller);
        }
    }
    controllers.clear();
    joysticks.clear();
    currentInputs.clear();

    if (initialized) {
        SDL_Quit();
        initialized = false;
        std::cout << "[SDL Controller Manager] SDL2 shutdown" << std::endl;
    }
}

void SDLControllerManager::loadControllerDatabase() {
    // SDL2 will automatically load the built-in controller database
    // You can also load custom mappings with SDL_GameControllerAddMappingsFromFile()
    int numMappings = SDL_GameControllerNumMappings();
    std::cout << "[SDL Controller Manager] Loaded " << numMappings << " controller mappings" << std::endl;
}

void SDLControllerManager::onControllerAdded(int deviceIndex) {
    if (!SDL_IsGameController(deviceIndex)) {
        std::cerr << "[SDL Controller Manager] Device " << deviceIndex << " is not a game controller" << std::endl;
        return;
    }

    SDL_GameController* controller = SDL_GameControllerOpen(deviceIndex);
    if (!controller) {
        std::cerr << "[SDL Controller Manager] Failed to open controller at index " << deviceIndex 
                  << ": " << SDL_GetError() << std::endl;
        return;
    }

    SDL_Joystick* joystick = SDL_GameControllerGetJoystick(controller);
    controllers.push_back(controller);
    joysticks.push_back(joystick);
    currentInputs.emplace_back();

    std::cout << "[SDL Controller Manager] ✓ Controller connected: " << SDL_GameControllerName(controller) << std::endl;
    std::cout << "  - Mapping: " << SDL_GameControllerMapping(controller) << std::endl;
    std::cout << "  - Joystick ID: " << SDL_JoystickInstanceID(joystick) << std::endl;
}

void SDLControllerManager::onControllerRemoved(SDL_JoystickID joystickId) {
    for (size_t i = 0; i < joysticks.size(); ++i) {
        if (SDL_JoystickInstanceID(joysticks[i]) == joystickId) {
            std::cout << "[SDL Controller Manager] ✗ Controller disconnected" << std::endl;
            SDL_GameControllerClose(controllers[i]);
            controllers.erase(controllers.begin() + i);
            joysticks.erase(joysticks.begin() + i);
            currentInputs.erase(currentInputs.begin() + i);
            break;
        }
    }
}

void SDLControllerManager::update() {
    if (!initialized) return;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_CONTROLLERDEVICEADDED:
                std::cout << "[SDL Controller Manager] Controller device added event" << std::endl;
                onControllerAdded(event.cdevice.which);
                break;

            case SDL_CONTROLLERDEVICEREMOVED:
                std::cout << "[SDL Controller Manager] Controller device removed event" << std::endl;
                onControllerRemoved(event.cdevice.which);
                break;

            case SDL_CONTROLLERAXISMOTION:
                // Handled in getControllerInput()
                break;

            case SDL_CONTROLLERBUTTONDOWN:
            case SDL_CONTROLLERBUTTONUP:
                // Handled in getControllerInput()
                break;

            case SDL_QUIT:
                // Handle quit if needed
                break;

            default:
                break;
        }
    }

    // Update controller input states
    for (size_t i = 0; i < controllers.size(); ++i) {
        SDL_GameController* controller = controllers[i];

        // Update analog sticks and triggers
        currentInputs[i].leftStickX = getAxisValue(SDL_CONTROLLER_AXIS_LEFTX, 0.15f, i);
        currentInputs[i].leftStickY = getAxisValue(SDL_CONTROLLER_AXIS_LEFTY, 0.15f, i);
        currentInputs[i].rightStickX = getAxisValue(SDL_CONTROLLER_AXIS_RIGHTX, 0.15f, i);
        currentInputs[i].rightStickY = getAxisValue(SDL_CONTROLLER_AXIS_RIGHTY, 0.15f, i);

        // Triggers are in range [0, 1] by SDL, we convert to [-1, 1] for consistency
        float rawLT = getAxisValue(SDL_CONTROLLER_AXIS_TRIGGERLEFT, 0.0f, i);
        float rawRT = getAxisValue(SDL_CONTROLLER_AXIS_TRIGGERRIGHT, 0.0f, i);
        currentInputs[i].leftTrigger = rawLT;   // [0, 1]
        currentInputs[i].rightTrigger = rawRT;  // [0, 1]

        // Update button states
        currentInputs[i].buttons.clear();
        const SDL_GameControllerButton buttonList[] = {
            SDL_CONTROLLER_BUTTON_A,
            SDL_CONTROLLER_BUTTON_B,
            SDL_CONTROLLER_BUTTON_X,
            SDL_CONTROLLER_BUTTON_Y,
            SDL_CONTROLLER_BUTTON_BACK,
            SDL_CONTROLLER_BUTTON_GUIDE,
            SDL_CONTROLLER_BUTTON_START,
            SDL_CONTROLLER_BUTTON_LEFTSTICK,
            SDL_CONTROLLER_BUTTON_RIGHTSTICK,
            SDL_CONTROLLER_BUTTON_LEFTSHOULDER,
            SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,
            SDL_CONTROLLER_BUTTON_DPAD_UP,
            SDL_CONTROLLER_BUTTON_DPAD_DOWN,
            SDL_CONTROLLER_BUTTON_DPAD_LEFT,
            SDL_CONTROLLER_BUTTON_DPAD_RIGHT,
        };

        for (auto button : buttonList) {
            currentInputs[i].buttons[button] = SDL_GameControllerGetButton(controller, button) != 0;
        }

        if (debugPrint) {
            printControllerState(i);
        }
    }
}

SDLControllerManager::ControllerInput SDLControllerManager::getControllerInput(int controllerIndex) const {
    if (controllerIndex >= 0 && controllerIndex < static_cast<int>(currentInputs.size())) {
        return currentInputs[controllerIndex];
    }
    return ControllerInput();
}

int SDLControllerManager::getConnectedControllerCount() const {
    return controllers.size();
}

std::string SDLControllerManager::getControllerInfo(int controllerIndex) const {
    if (controllerIndex >= 0 && controllerIndex < static_cast<int>(controllers.size())) {
        std::string name = SDL_GameControllerName(controllers[controllerIndex]);
        return name;
    }
    return "No controller";
}

bool SDLControllerManager::isButtonPressed(SDL_GameControllerButton button, int controllerIndex) const {
    if (controllerIndex >= 0 && controllerIndex < static_cast<int>(currentInputs.size())) {
        auto it = currentInputs[controllerIndex].buttons.find(button);
        if (it != currentInputs[controllerIndex].buttons.end()) {
            return it->second;
        }
    }
    return false;
}

float SDLControllerManager::getAxisValue(SDL_GameControllerAxis axis, float deadband, int controllerIndex) const {
    if (controllerIndex < 0 || controllerIndex >= static_cast<int>(controllers.size())) {
        return 0.0f;
    }

    Sint16 rawValue = SDL_GameControllerGetAxis(controllers[controllerIndex], axis);
    float normalizedValue = rawValue / 32767.0f;  // SDL returns values in [-32768, 32767]

    // Apply deadband
    if (std::abs(normalizedValue) < deadband) {
        return 0.0f;
    }

    // Remove deadband from the value
    if (normalizedValue > 0) {
        return (normalizedValue - deadband) / (1.0f - deadband);
    } else {
        return (normalizedValue + deadband) / (1.0f - deadband);
    }
}

void SDLControllerManager::printControllerState(int controllerIndex) const {
    if (controllerIndex < 0 || controllerIndex >= static_cast<int>(currentInputs.size())) {
        return;
    }

    const auto& input = currentInputs[controllerIndex];

    // Only print if there's any significant input
    bool hasInput = false;
    if (std::abs(input.leftStickX) > 0.05f || std::abs(input.leftStickY) > 0.05f ||
        std::abs(input.rightStickX) > 0.05f || std::abs(input.rightStickY) > 0.05f ||
        input.leftTrigger > 0.1f || input.rightTrigger > 0.1f) {
        hasInput = true;
    }

    for (const auto& btn : input.buttons) {
        if (btn.second) {
            hasInput = true;
            break;
        }
    }

    if (!hasInput) return;

    std::stringstream ss;
    ss << "[Controller " << controllerIndex << "] ";

    // Print analog sticks
    if (std::abs(input.leftStickX) > 0.05f || std::abs(input.leftStickY) > 0.05f) {
        ss << "LS:(" << input.leftStickX << "," << input.leftStickY << ") ";
    }
    if (std::abs(input.rightStickX) > 0.05f || std::abs(input.rightStickY) > 0.05f) {
        ss << "RS:(" << input.rightStickX << "," << input.rightStickY << ") ";
    }

    // Print triggers
    if (input.leftTrigger > 0.1f) {
        ss << "LT:" << input.leftTrigger << " ";
    }
    if (input.rightTrigger > 0.1f) {
        ss << "RT:" << input.rightTrigger << " ";
    }

    // Print pressed buttons
    for (const auto& btn : input.buttons) {
        if (btn.second) {
            ss << getButtonName(btn.first) << " ";
        }
    }

    std::cout << ss.str() << std::endl;
}

std::string SDLControllerManager::getButtonName(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_A: return "A";
        case SDL_CONTROLLER_BUTTON_B: return "B";
        case SDL_CONTROLLER_BUTTON_X: return "X";
        case SDL_CONTROLLER_BUTTON_Y: return "Y";
        case SDL_CONTROLLER_BUTTON_BACK: return "BACK";
        case SDL_CONTROLLER_BUTTON_GUIDE: return "GUIDE";
        case SDL_CONTROLLER_BUTTON_START: return "START";
        case SDL_CONTROLLER_BUTTON_LEFTSTICK: return "LS-BTN";
        case SDL_CONTROLLER_BUTTON_RIGHTSTICK: return "RS-BTN";
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: return "LB";
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: return "RB";
        case SDL_CONTROLLER_BUTTON_DPAD_UP: return "UP";
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: return "DOWN";
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT: return "LEFT";
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: return "RIGHT";
        default: return "UNKNOWN";
    }
}

std::string SDLControllerManager::getAxisName(SDL_GameControllerAxis axis) {
    switch (axis) {
        case SDL_CONTROLLER_AXIS_LEFTX: return "LeftStickX";
        case SDL_CONTROLLER_AXIS_LEFTY: return "LeftStickY";
        case SDL_CONTROLLER_AXIS_RIGHTX: return "RightStickX";
        case SDL_CONTROLLER_AXIS_RIGHTY: return "RightStickY";
        case SDL_CONTROLLER_AXIS_TRIGGERLEFT: return "LeftTrigger";
        case SDL_CONTROLLER_AXIS_TRIGGERRIGHT: return "RightTrigger";
        default: return "UNKNOWN";
    }
}
