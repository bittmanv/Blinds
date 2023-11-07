/**
 *  class for generic motor controlling
 * 
 */

#ifndef rollerController_h

#define rollerController_h


#include <Arduino.h>

// class for PID Controlller 
#include <PIDController.h>


#define PID_PERIOD 100
#define PROC_PUB_PERIOD 1000



class RollerControl {

  public:

    // Constructor / desctructor

    RollerControl();

    virtual ~RollerControl();

      
    // TODO:: delete this example
    // typedef void (*cbfunc_t) (libClass *pInstance);

     enum motorState_t { IDDLE, MOPENING, MCLOSING,  HALL_STOP };    
     // button variables 
      enum buttonCommand_t { GO_UP, STOP, GO_DOWN};

    // get counter method to read current state of motor
    typedef int32_t (*getCounterFunc_t) ();

    // function type for setting pwm on motor
    typedef void (*setMotorDirFunc_t) (int16_t pwm);

    // function type for procent publishing call back
    typedef void (*publishPercent_t) (uint8_t percent);

     // function for reading endstops
     typedef int (*getEndStopFunc_t) ();
    

    // method to set motor counter 
    void setGetCounterCb(getCounterFunc_t callbackfunc_new);

    // methods to set motor control functions

    void setMotorUpCb(setMotorDirFunc_t callbackfunc_new);
    void setMotorDownCb(setMotorDirFunc_t callbackfunc_new);
    void setMotorStopCb(setMotorDirFunc_t callbackfunc_new);

    // callback to read end stop
    void setTopEndstopCb(getEndStopFunc_t callbackfunc_new);

    void setPercentMqttCb(publishPercent_t callbackfunc_new);
     
    // method receive command from button
    void buttonPress();

    // main method for controlling activity
    void rollerLoop();

    // command to go for specific percentage
    void gotoPercentage(uint8_t percentage);
    uint8_t getCurrentPercentage();

    void setMaxPwm(int16_t newMaxPwm);

    // set maximum range in motor steps 
    void setMaxRange(int32_t newMaxRange);
    void stopNow();
    
    void init();
    
     // speed calculation  
    int16_t getSpeed(uint32_t currentMillis);

    bool isIddle();
    
    
      
  private :
    PIDController pos_pid;
    uint32_t lastPidCall=0;
    int32_t maxRange=100;
    // requested position
    int32_t requestedPosition=0;
    // last known position for PID Calls 
    int32_t lastKnownPosition=0;
    // max pwm for motor operation
    int16_t maxPwm=255;
    // last calculated pwm
    int16_t lastPwm=0;
    // timestamp of motor stop to preventints durinf SPIFFS ops
    uint32_t stoppedAt;
    
    
    // last calculated speed 
    int16_t motSpeed=0;

     int32_t lastSpeedPos=0;
     uint32_t lastSpeedCalc=millis();
     int32_t currentPosition=0;

     buttonCommand_t  buttonCommand=GO_DOWN;

     uint32_t lastPercPub=0;
    
    motorState_t motorStatus=IDDLE;
    bool getCounterValid=false;
    
    getCounterFunc_t counterCallBackFunc;

    bool motorUpValid=false;
    setMotorDirFunc_t motorUpFunc;

    bool motorDownValid=false;
    setMotorDirFunc_t motorDownFunc;

    bool motorStopValid=false;
    setMotorDirFunc_t motorStopFunc;

     bool getTopEndstopValid=false;
     getEndStopFunc_t getTopEndstopFunc;

     bool publishPercentValid=false;
     publishPercent_t  publishPercentFunc;
     
    void resetSpeed();

    int32_t getCurrentPosition();
    int  getTopEndstop();

    void stopMotor(int pwm);
    void motorUp(int pwm);
    void motorDown(int pwm);

    void publishPercent(uint8_t percent);
    

};


#endif
