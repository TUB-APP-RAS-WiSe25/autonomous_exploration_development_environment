#ifndef SDL_CONTROLLER_MANAGER_H
#define SDL_CONTROLLER_MANAGER_H

#include <SDL2/SDL.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>

/**
 * @class SDLControllerManager
 * @brief Manages SDL2 game controller input with automatic controller detection and mapping
 * 
 * Features:
 * - Automatic controller detection and connection/disconnection handling
 * - SDL game controller database for automatic button/axis mapping
 * - Debug printing of all controller inputs
 * - Support for multiple controllers
 */
class SDLControllerManager {
public:
    struct ControllerInput {
        float leftStickX = 0.0f;
        float leftStickY = 0.0f;
        float rightStickX = 0.0f;
        float rightStickY = 0.0f;
        float leftTrigger = 0.0f;
        float rightTrigger = 0.0f;
        
        std::map<SDL_GameControllerButton, bool> buttons;
    };

    SDLControllerManager();
    ~SDLControllerManager();

    /**
     * Initialize SDL2 controller subsystem
     * @return true if initialization successful
     */
    bool init();

    /**
     * Shutdown SDL2 controller subsystem
     */
    void shutdown();

    /**
     * Update controller input states
     * Must be called frequently in the main loop
     */
    void update();

    /**
     * Get input from the first connected controller (index 0)
     * @return ControllerInput struct with current input state
     */
    ControllerInput getControllerInput(int controllerIndex = 0) const;

    /**
     * Get number of connected controllers
     * @return number of connected game controllers
     */
    int getConnectedControllerCount() const;

    /**
     * Get controller name/info
     * @param controllerIndex index of the controller
     * @return name and info string about the controller
     */
    std::string getControllerInfo(int controllerIndex = 0) const;

    /**
     * Enable/disable debug printing
     * @param enabled if true, prints all controller inputs to console
     */
    void setDebugPrint(bool enabled) { debugPrint = enabled; }

    /**
     * Check if a specific button is pressed
     * @param button SDL_GameControllerButton enum value
     * @param controllerIndex index of the controller
     * @return true if button is pressed
     */
    bool isButtonPressed(SDL_GameControllerButton button, int controllerIndex = 0) const;

    /**
     * Get axis value with deadband applied
     * @param axis SDL_GameControllerAxis enum value
     * @param deadband threshold for analog stick deadband (0.0-1.0)
     * @param controllerIndex index of the controller
     * @return axis value in range [-1.0, 1.0]
     */
    float getAxisValue(SDL_GameControllerAxis axis, float deadband = 0.15f, int controllerIndex = 0) const;

private:
    std::vector<SDL_GameController*> controllers;
    std::vector<SDL_Joystick*> joysticks;
    std::vector<ControllerInput> currentInputs;
    bool initialized = false;
    bool debugPrint = true;

    /**
     * Load SDL game controller database
     */
    void loadControllerDatabase();

    /**
     * Handle a new controller connection
     */
    void onControllerAdded(int deviceIndex);

    /**
     * Handle controller disconnection
     */
    void onControllerRemoved(SDL_JoystickID joystickId);

    /**
     * Print debug information about all controller inputs
     */
    void printControllerState(int controllerIndex) const;

    /**
     * Get button name for debug printing
     */
    static std::string getButtonName(SDL_GameControllerButton button);

    /**
     * Get axis name for debug printing
     */
    static std::string getAxisName(SDL_GameControllerAxis axis);
};

#endif // SDL_CONTROLLER_MANAGER_H
