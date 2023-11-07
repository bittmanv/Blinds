
#include "RollerControl.h"

RollerControl::RollerControl()
{
    getCounterValid=false;
}


RollerControl::~RollerControl()
{
     // no special activity in destructor
}

void RollerControl::setGetCounterCb(getCounterFunc_t callbackfunc_new)
{
   counterCallBackFunc=callbackfunc_new;
   getCounterValid=true;
}


void RollerControl::setMotorUpCb(setMotorDirFunc_t callbackfunc_new)
{
     motorUpFunc=callbackfunc_new;
     motorUpValid=true;
}

void RollerControl::setMotorDownCb(setMotorDirFunc_t callbackfunc_new)
{
     motorDownFunc=callbackfunc_new;
     motorDownValid=true;
  
}

void RollerControl::setMotorStopCb(setMotorDirFunc_t callbackfunc_new)
{
     motorStopFunc=callbackfunc_new;
     motorStopValid=true;
}

void RollerControl::setPercentMqttCb(publishPercent_t callbackfunc_new)
{
     publishPercentFunc=callbackfunc_new;
     publishPercentValid=true;
}

void RollerControl::setTopEndstopCb(getEndStopFunc_t callbackfunc_new)
{
     getTopEndstopFunc=callbackfunc_new;
     getTopEndstopValid=true;
}

int  RollerControl::getTopEndstop()
{
    if(getTopEndstopValid)
    {
      return getTopEndstopFunc();
    }
    else
      return LOW;
}
void RollerControl::setMaxPwm(int16_t newMaxPwm)
{
  maxPwm=newMaxPwm;
}


int16_t RollerControl::getSpeed(uint32_t currentMillis)
{
    uint16_t retVal=motSpeed;
    // int32_t currPos;
    currentPosition=getCurrentPosition();
    
    if((lastSpeedCalc+PID_PERIOD)<currentMillis)
    {
         
             // currPos=currentPosition;
             // Serial.print(F("s"));
         Serial.print(F(" sCS "));
         Serial.print(lastSpeedPos);
         Serial.print(F(";"));
         Serial.print(currentPosition);
         Serial.print(F(" ")); 
         retVal=abs(currentPosition-lastSpeedPos);
         lastSpeedPos=currentPosition;   
         motSpeed=retVal;
         Serial.print(" ms ");
         Serial.println(motSpeed);
    }
    return retVal;
}

void RollerControl::resetSpeed()
{
        
         lastSpeedPos=currentPosition;
         lastSpeedCalc=millis();
         motSpeed=50;
         
}


void RollerControl::gotoPercentage(uint8_t percentage)
{
   // first refresh current position of motor
   currentPosition=getCurrentPosition();
   Serial.print("Current enc position");
   Serial.println(currentPosition);
   // first map percentage to requested position of motor
   requestedPosition=map(percentage,0,100,0,maxRange);
   Serial.print(" goto ");
   Serial.println(requestedPosition);
   // set properties of PID Controller
   pos_pid.setpoint((double) requestedPosition);
   pos_pid.limit(-maxPwm, maxPwm);
   // now set motor status to proper value
   // in case of opening we are checking end stop  
    resetSpeed();
     if(requestedPosition>currentPosition)
    {
       motorStatus=MCLOSING;
       motorDown(maxPwm);
    }
    else if(requestedPosition<currentPosition) 
    {
      motorStatus=MOPENING;
      motorUp(maxPwm);
    }
    // nowreset speed
    
    
    // current millis in PID Call will allow motor to start
    lastPwm=maxPwm;
    lastPercPub=lastPidCall=millis();
    // now main loop 
    rollerLoop();
}


void RollerControl::setMaxRange(int32_t newMaxRange)
{
    maxRange=newMaxRange;
}


void RollerControl::init()
{
   pos_pid.begin(); 
   pos_pid.tune(1, 2.2, 0);  
   motorStatus=IDDLE;
   
}

int32_t RollerControl::getCurrentPosition()
{
   int32_t tempPos=0;
   if(getCounterValid)
   {
       tempPos=counterCallBackFunc();
   }
   return tempPos;
}

void RollerControl::stopMotor(int pwm)
{
    if(motorStopValid)
    {
       motorStopFunc(pwm);
    }
    motorStatus=IDDLE;
    stoppedAt=millis();
}

void RollerControl::motorUp(int pwm)
{
   if(motorUpValid)
   {
      motorUpFunc(pwm);
   }
  
}

void RollerControl::motorDown(int pwm)
{
   if(motorDownValid)
   {
      motorDownFunc(pwm);
   }
}

void RollerControl::publishPercent(uint8_t percent)
{
    if(publishPercentValid)
    {
       publishPercentFunc(percent);
    }
}

bool RollerControl::isIddle()
{
    switch(motorStatus)
    {
      case IDDLE:
         if(millis()>(stoppedAt+500))
           return true;
          else
            return false;
        break;
      default :
         return false;

    }
  
    // return motorStatus==IDDLE;
}
void RollerControl::rollerLoop()
{
    uint32_t currentMs=millis();
    int16_t motSpeed;

    // in case, when motor is iddle, do nothing
    if(motorStatus==IDDLE)
       return;

    if(currentMs>(lastPercPub+PROC_PUB_PERIOD))
    {
         publishPercent(map(currentPosition,0,maxRange,0,100));
         lastPercPub=currentMs;
    }
    // first handling of endstops
    switch(motorStatus)
    {
       case MOPENING:
        if(getTopEndstop()==LOW)
        {
             stopMotor(0);
             currentPosition=0;
             publishPercent(0);
             // TODO : Reset counter method button management
        }
    }

    // main motor control function
    switch(motorStatus)
    {
        case MOPENING:
        case MCLOSING:
           currentPosition=getCurrentPosition();
           // TODO: Handle call back for publishing current position during movement
           if((lastPidCall+PID_PERIOD)<currentMs)
           {
                // get current motor speed
                motSpeed=getSpeed(currentMs);
                // next pid call after some time
                lastPidCall=currentMs;
                 Serial.print("sp ");
                // motSpeed=getSpeed(lastPIDCall);
                Serial.print(motSpeed);
                
                Serial.print(" curPos ");
                Serial.println(currentPosition);
                // int motorPwm=pos_pid.compute((double)currentPosition, GRAPH, VERBOSE);
                int motorPwm=pos_pid.compute((double)currentPosition);
                // if is there new pwm 
                 Serial.print(" pwm ");
                 Serial.println(motorPwm);

                if(motorPwm!=lastPwm)
                {
                    lastPwm=motorPwm;
                    if(motorPwm>0)
                    {
                       motorDown(abs(motorPwm));
                    }
                    else if(motorPwm<0)
                    {
                        motorUp(abs(motorPwm));
                    }
                    else
                    {
                      stopMotor(motorPwm);
                      motorStatus=IDDLE;
                      publishPercent(map(currentPosition,0,maxRange,0,100));

                      // TODO: Button handling 
                      /*
                       * button command template
                       */
                       
                        if(currentPosition>100)
                            buttonCommand=GO_UP;
                          else
                            buttonCommand=GO_DOWN;
                    }
                }
                // now stop based on speed and requested position
                int posDiff=abs(requestedPosition-currentPosition);

                if((posDiff<20)||(motSpeed<10))
                {
                   stopMotor(0);
                   publishPercent(map(currentPosition,0,maxRange,0,100));
                   motorStatus=IDDLE;
                 
                   // TODO : Button handling
                   
                   if(currentPosition>100)
                      buttonCommand=GO_UP;
                    else
                      buttonCommand=GO_DOWN;
                   
                }   // emd of motor speed stop 
           }

        break;   
    }
   
}

void RollerControl::stopNow()
{
   int32_t curPos=getCurrentPosition();
   pos_pid.setpoint((double) curPos);
   
}

void RollerControl::buttonPress()
{
    if(motorStatus!=IDDLE)
    {
         stopNow();
         int32_t curPos=getCurrentPosition();
        if(curPos>100)
        {
          buttonCommand=GO_UP;
        }
        else
        {
          buttonCommand=GO_DOWN;
        }
    
    }
    else
    {
        switch(buttonCommand)
        {
            case GO_UP:
               gotoPercentage(0);
               break;
            case GO_DOWN:
               gotoPercentage(100);
               break;
        }
    }
}
