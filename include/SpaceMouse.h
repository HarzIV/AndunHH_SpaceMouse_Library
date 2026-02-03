/*
Main class to unify all code, into one class.
This class integrates all of the seperate functions within the original code
of AndunHH and makes them into a simply to use class.
*/
#include <Arduino.h>

// The user specific settings, like pin mappings or special configuration variables and sensitivities are stored in config.h.
// Please open config_sample.h, adjust your settings and save it as config.h
#include "config.h"
#include "parameterMenu.h"

// Include inbuilt Arduino HID library by NicoHood: https://github.com/NicoHood/HID
#include "HID.h"

// header file for calibration output and helper routines
#include "calibration.h"

// header to calculate the kinematics of the mouse
#include "kinematics.h"

// header file for reading the keys
#include "spaceKeys.h"

// header for HID emulation of the spacemouse
#include "SpaceMouseHID.h"

// if an encoder wheel is used
#if ROTARY_AXIS > 0 or ROTARY_KEYS > 0
#include "encoderWheel.h"
#endif

#ifdef LEDRING
#include "ledring.h"
#endif

class SpaceMouse
{
public:
    String configPath = "config.h";

    /**
     * @brief Main class function.
     * This function ties together the whole class/spacemouse.
     */
    void main();

private:
#ifdef LEDpin
    void lightSimpleLED(boolean light);
#endif

#ifdef HALLEFFECT
    void setAnalogReferenceVoltage(int dbg);
#endif

    // stores the raw analog values from the joysticks
    int rawReads[8];

    // stores the values from the joysticks after zeroing and mapping
    int centered[8];

    // Centerpoints store the zero position of the joysticks
    int centerPoints[8];

    // Offsets store the drift-compensation values of the joysticks
    int offsets[8];

    // Resulting calculated velocities / movements
    // int16_t to match what the HID protocol expects.
    int16_t velocity[6];

    // global parameters (also stored in EEPROM)
    ParamStorage parStorage;

    ParamData par = {.values = &parStorage,
                     .description = {
                         {PARAM_TYPE_BOOL, "", NULL},                                        // param 0 is unused
                         {PARAM_TYPE_INT, "DEADZONE", &parStorage.deadzone},                 //       1
                         {PARAM_TYPE_FLOAT, "SENS_TX", &parStorage.transX_sensitivity},      //       2
                         {PARAM_TYPE_FLOAT, "SENS_TY", &parStorage.transY_sensitivity},      //       3
                         {PARAM_TYPE_FLOAT, "SENS_PTZ", &parStorage.pos_transZ_sensitivity}, //       4
                         {PARAM_TYPE_FLOAT, "SENS_NTZ", &parStorage.neg_transZ_sensitivity}, //       5
                         {PARAM_TYPE_FLOAT, "GATE_NTZ", &parStorage.gate_neg_transZ},        //       6
                         {PARAM_TYPE_INT, "GATE_RX", &parStorage.gate_rotX},                 //       7
                         {PARAM_TYPE_INT, "GATE_RY", &parStorage.gate_rotY},                 //       8
                         {PARAM_TYPE_INT, "GATE_RZ", &parStorage.gate_rotZ},                 //       9
                         {PARAM_TYPE_FLOAT, "SENS_RX", &parStorage.rotX_sensitivity},        //      10
                         {PARAM_TYPE_FLOAT, "SENS_RY", &parStorage.rotY_sensitivity},        //      11
                         {PARAM_TYPE_FLOAT, "SENS_RZ", &parStorage.rotZ_sensitivity},        //      12
                         {PARAM_TYPE_INT, "MODFUNC", &parStorage.modFunc},                   //      13
                         {PARAM_TYPE_FLOAT, "MOD_A", &parStorage.slope_at_zero},             //      14
                         {PARAM_TYPE_FLOAT, "MOD_B", &parStorage.slope_at_end},              //      15
                         {PARAM_TYPE_BOOL, "INVX", &parStorage.invX},                        //      16
                         {PARAM_TYPE_BOOL, "INVY", &parStorage.invY},                        //      17
                         {PARAM_TYPE_BOOL, "INVZ", &parStorage.invZ},                        //      18
                         {PARAM_TYPE_BOOL, "INVRX", &parStorage.invRX},                      //      19
                         {PARAM_TYPE_BOOL, "INVRY", &parStorage.invRY},                      //      20
                         {PARAM_TYPE_BOOL, "INVRZ", &parStorage.invRZ},                      //      21
                         {PARAM_TYPE_BOOL, "SWITCHXY", &parStorage.switchXY},                //      22
                         {PARAM_TYPE_BOOL, "SWITCHYZ", &parStorage.switchYZ},                //      23
                         {PARAM_TYPE_BOOL, "EXCLUSIVE", &parStorage.exclusiveMode},          //      24
                         {PARAM_TYPE_INT, "EXCL_HYST", &parStorage.exclusiveHysteresis},     //      25
                         {PARAM_TYPE_BOOL, "EXCL_PRIOZ", &parStorage.prioZexclusiveMode},    //      26
                         {PARAM_TYPE_BOOL, "COMP_EN", &parStorage.compEnabled},              //      27
                         {PARAM_TYPE_INT, "COMP_NR", &parStorage.compNoOfPoints},            //      28
                         {PARAM_TYPE_INT, "COMP_WAIT", &parStorage.compWaitTime},            //      29
                         {PARAM_TYPE_INT, "COMP_MDIFF", &parStorage.compMinMaxDiff},         //      30
                         {PARAM_TYPE_INT, "COMP_CDIFF", &parStorage.compCenterDiff},         //      31
                         {PARAM_TYPE_INT, "RAXIS_ECH", &parStorage.rotAxisEchos},            //      32
                         {PARAM_TYPE_INT, "RAXIS_STR", &parStorage.rotAxisSimStrength}       //      33
                     }};

    // store raw value of the keys, without debouncing
    int keyVals[NUMKEYS];

    // key event, after debouncing. It is 1 only for a single sample
    uint8_t keyOut[NUMKEYS];

    // state of the key, which stays 1 as long as the key is pressed
    uint8_t keyState[NUMKEYS];

    /**
     * @brief Similar to the usual void setup();
     * This is the point where every necessary measurement is taken, and every conditinal function, as dictated by the config.h, is run.
     */
    void setup();

#ifdef LEDpin
    void lightSimpleLED(boolean light);
#endif

#ifdef HALLEFFECT
    void setAnalogReferenceVoltage(int dbg);
#endif
};