/*
ALT-Scann8 UI - Alternative software for T-Scann 8

This tool is a fork of the original user interface application of T-Scann 8

Some additional features of this version include:
- PiCamera 2 integration
- Use of Tkinter instead of Pygame
- Automatic exposure support
- Fast forward support

  Licensed under a MIT LICENSE.
More info in README.md file
*/

#define __author__      "Juan Remirez de Esparza"
#define __copyright__   "Copyright 2022-25, Juan Remirez de Esparza"
#define __credits__     "Juan Remirez de Esparza"
#define __license__     "MIT"
#define __version__     "1.1.12"
#define  __date__       "2025-06-10"
#define  __version_highlight__  "Re-integrated Traction Switch as a fallback for end-of-reel detection."
#define __maintainer__  "Juan Remirez de Esparza"
#define __email__       "jremirez@hotmail.com"
#define __status__      "Development"

#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

const int PHOTODETECT = A0; 
int MaxPT = 0;
int MinPT = 200; 

unsigned int MaxPT_Dynamic = 0;
unsigned int MinPT_Dynamic = 10000;
enum {
    PlotterInfo,
    FrameSteps,
    DebugInfo,
    DebugInfoSingle,
    None
} DebugState = None;
int MaxDebugRepetitions = 3;
#define MAX_DEBUG_REPETITIONS_COUNT 30000

boolean GreenLedOn = false;  
int UI_Command;

#define CMD_VERSION_ID 1
#define CMD_GET_CNT_STATUS 2
#define CMD_RESET_CONTROLLER 3
#define CMD_ADJUST_MIN_FRAME_STEPS 4
#define CMD_START_SCAN 10
#define CMD_TERMINATE 11
#define CMD_GET_NEXT_FRAME 12
#define CMD_STOP_SCAN 13
#define CMD_SET_REGULAR_8 18
#define CMD_SET_SUPER_8 19
#define CMD_SWITCH_REEL_LOCK_STATUS 20
#define CMD_MANUAL_UV_LED 22
#define CMD_FILM_FORWARD 30
#define CMD_FILM_BACKWARD 31
#define CMD_SINGLE_STEP 40
#define CMD_ADVANCE_FRAME 41
#define CMD_ADVANCE_FRAME_FRACTION 42
#define CMD_RUN_FILM_COLLECTION 43
#define CMD_SET_PT_LEVEL 50
#define CMD_SET_MIN_FRAME_STEPS 52
#define CMD_SET_FRAME_FINE_TUNE 54
#define CMD_SET_EXTRA_STEPS 56
#define CMD_SET_UV_LEVEL 58
#define CMD_REWIND 60
#define CMD_FAST_FORWARD 61
#define CMD_INCREASE_WIND_SPEED 62
#define CMD_DECREASE_WIND_SPEED 63
#define CMD_UNCONDITIONAL_REWIND 64
#define CMD_UNCONDITIONAL_FAST_FORWARD 65
#define CMD_SET_SCAN_SPEED 70
#define CMD_SET_STALL_TIME 72
#define CMD_SET_AUTO_STOP 74
#define CMD_REPORT_PLOTTER_INFO 87

#define RSP_VERSION_ID 1
#define RSP_FORCE_INIT 2
#define RSP_FRAME_AVAILABLE 80
#define RSP_SCAN_ERROR 81
#define RSP_REWIND_ERROR 82
#define RSP_FAST_FORWARD_ERROR 83
#define RSP_REWIND_ENDED 84
#define RSP_FAST_FORWARD_ENDED 85
#define RSP_REPORT_AUTO_LEVELS 86
#define RSP_REPORT_PLOTTER_INFO 87
#define RSP_SCAN_ENDED 88
#define RSP_FILM_FORWARD_ENDED 89
#define RSP_ADVANCE_FRAME_FRACTION 90



#define S8_HEIGHT  4.01
#define R8_HEIGHT  3.3
#define NEMA_STEP_DEGREES  1.8
#define NEMA_MICROSTEPS_IN_STEP  16


const int MotorA_Stepper = 2; 
const int MotorA_Neutral = 3;     
const int MotorB_Stepper = 4; 
const int MotorB_Neutral = 5;     
const int MotorC_Stepper = 6; 
const int MotorC_Neutral = 7;     
const int MotorA_Direction = 8; 
const int MotorB_Direction = 9;   
const int MotorC_Direction = 10;  
const int TractionStopPin = 12; 

enum ScanResult{SCAN_NO_FRAME_DETECTED, SCAN_FRAME_DETECTED, SCAN_FRAME_DETECTION_ERROR, SCAN_TERMINATION_REQUESTED};

enum ScanState{
    Sts_Idle,
    Sts_Scan,
    Sts_UnlockReels,
    Sts_Rewind,
    Sts_FastForward,
    Sts_SlowForward,
    Sts_SlowBackward,
    Sts_SingleStep,
    Sts_ManualUvLed
}
ScanState=Sts_Idle;

int UVLedBrightness = 255; 
int ScanSpeed = 10; 
unsigned long BaseScanSpeedDelay = 10; 
unsigned long StepScanSpeedDelay = 100; 
unsigned long ScanSpeedDelay = BaseScanSpeedDelay; 
unsigned long DecreaseScanSpeedDelayStep = 50; 
int RewindSpeed = 4000; 
int TargetRewindSpeedLoop = 200; 
int PerforationMaxLevel = 550; 
int PerforationMinLevel = 50;               
int PerforationThresholdLevelR8 = 180; 
int PerforationThresholdLevelS8 = 90;       
int PerforationThresholdLevel = PerforationThresholdLevelS8; 
int PerforationThresholdAutoLevelRatio = 40; 
float CapstanDiameter = 14.3; 
int MinFrameStepsR8;                  
int MinFrameStepsS8;                  
int MinFrameSteps = MinFrameStepsS8; 
int FrameExtraSteps = 0; 
int FrameDeductSteps = 0; 
int DecreaseSpeedFrameStepsBefore = 3; 
int DecreaseSpeedFrameSteps = MinFrameSteps - DecreaseSpeedFrameStepsBefore; 


boolean ReelsUnlocked = false;
boolean FrameDetected = false;  
boolean UVLedOn = false;
int FilteredSignalLevel = 0;
int OriginalPerforationThresholdLevel = PerforationThresholdLevel; 
int OriginalPerforationThresholdAutoLevelRatio = PerforationThresholdAutoLevelRatio;
int FrameStepsDone = 0;                     


unsigned long OriginalScanSpeedDelay = ScanSpeedDelay; 
int OriginalMinFrameSteps = MinFrameSteps; 

int LastFrameSteps = 0; 
int LastPTLevel = 0; 

boolean IsS8 = true;

boolean TractionSwitchActive = false; 
boolean TractionSwitchActiveLast = false;  

unsigned long StartFrameTime = 0; 
unsigned long StartPictureSaveTime = 0; 
unsigned long FilmDetectedTime = 0; 
bool NoFilmDetected = true;
bool EndScanNotificationSent = false; 
int MaxFilmStallTime = 10000; 

unsigned long lastSwitchChange = 0;
bool switchStateBackup = true;
bool switchTimeoutSent = false;

byte BufferForRPi[9]; 

int PT_SignalLevelRead; 
boolean PT_Level_Auto = true;   

boolean Frame_Steps_Auto = true;
boolean IntegratedPlotter = false;

boolean AutoStopEnabled = false;

boolean VFD_mode_active = false;


int default_collect_timer = 1000;
int collect_timer = default_collect_timer;
int scan_collect_timer = collect_timer;
bool scan_process_ongoing = false;

#define QUEUE_SIZE 40
typedef struct Queue {
    int Data[QUEUE_SIZE];
    int Param[QUEUE_SIZE];
    int Param2[QUEUE_SIZE];
    int in;
    int out;
};

volatile Queue CommandQueue;
volatile Queue ResponseQueue;
void SendToRPi(byte rsp, int param1, int param2)
{
    push_rsp(rsp, param1, param2);
}

void(* resetFunc) (void) = 0;

void setup() {
    
    switchStateBackup = digitalRead(TractionStopPin);
    lastSwitchChange = millis();
    switchTimeoutSent = false;

    
    Serial.begin(1000000); 
  
    Wire.begin(16); 
    Wire.setClock(400000); 
    Wire.onReceive(receiveEvent); 
    Wire.onRequest(sendEvent);
    
    pinMode(MotorA_Stepper, OUTPUT);
    pinMode(MotorA_Direction, OUTPUT);
    pinMode(MotorB_Stepper, OUTPUT);
    pinMode(MotorB_Direction, OUTPUT);
    pinMode(MotorC_Stepper, OUTPUT);
    pinMode(MotorC_Direction, OUTPUT);
    pinMode(TractionStopPin, INPUT);
    pinMode(MotorA_Neutral, OUTPUT);
    pinMode(MotorB_Neutral, OUTPUT);
    pinMode(MotorC_Neutral, OUTPUT);
    
    pinMode(A1, OUTPUT); 
    pinMode(A2, OUTPUT); 
    pinMode(11, OUTPUT); 

    
    digitalWrite(MotorA_Neutral, HIGH);
    
    digitalWrite(MotorA_Direction, LOW);     
    digitalWrite(MotorB_Direction, HIGH); 
    digitalWrite(MotorC_Direction, HIGH); 

    digitalWrite(MotorA_Stepper, LOW);
    digitalWrite(MotorB_Stepper, LOW);
    digitalWrite(MotorC_Stepper, LOW);
    
    CommandQueue.in = 0;
    CommandQueue.out = 0;
    ResponseQueue.in = 0;
    ResponseQueue.out = 0;
    
    SetReelsAsNeutral(HIGH, HIGH, HIGH);
    
    AdjustMinFrameStepsFromCapstanDiameter(CapstanDiameter);
}

void loop() {
    int param;
    int cnt_ver_1 = 0, cnt_ver_2 = 0, cnt_ver_3 = 0;
    char *pt;
    SendToRPi(RSP_FORCE_INIT, 0, 0);  

    while (1) {
        if (dataInCmdQueue())
            UI_Command = pop_cmd(&param); 
        else
            UI_Command = 0;
        ReportPlotterInfo();    

        

        switch (UI_Command) {   
            case CMD_RESET_CONTROLLER:
                resetFunc();
                break;
            case CMD_ADJUST_MIN_FRAME_STEPS:
                DebugPrint(">Adjust MFS", param);
                if (param >= 80 && param <= 300) {   
                    CapstanDiameter = param/10;
                    AdjustMinFrameStepsFromCapstanDiameter(CapstanDiameter);
                    if (IsS8)
                        MinFrameSteps = MinFrameStepsS8;
                    else
                        MinFrameSteps = MinFrameStepsR8;
                    OriginalMinFrameSteps = MinFrameSteps;
                }
                break;
            case CMD_SET_PT_LEVEL:
                DebugPrint(">PTLevel", param);
                if (param >= 0 && param <= 900) {
                    if (param == 0)
                        PT_Level_Auto = true; 
                    else {
                        PT_Level_Auto = false;
                        PerforationThresholdLevel = param;
                        OriginalPerforationThresholdLevel = param;
                    }
                    DebugPrint(">PTLevel",param);
                }
                break;
            case CMD_SET_UV_LEVEL:
                DebugPrint(">UVLevel", param);
                if (param >= 1 && param <= 255) {
                    UVLedBrightness = param;
                    if (UVLedOn) {
                        analogWrite(11, UVLedBrightness); 
                    }
                }
                break;
            case CMD_SET_MIN_FRAME_STEPS:
                DebugPrint(">MinFSteps", param);
                if (param == 0 || param >= 100 && param <= 600) {
                    if (param == 0) {
                        Frame_Steps_Auto = true; 
                        if (IsS8)
                            MinFrameSteps = MinFrameStepsS8;
                        else
                            MinFrameSteps = MinFrameStepsR8;
                        OriginalMinFrameSteps = MinFrameSteps;
                    }
                    else {
                        Frame_Steps_Auto = false;
                        MinFrameSteps = param;
                        OriginalMinFrameSteps = MinFrameSteps;
                        
                        DecreaseSpeedFrameSteps = MinFrameSteps - DecreaseSpeedFrameStepsBefore;
                        DebugPrint(">MinSteps",param);
                    }
                }
                break;
            case CMD_SET_STALL_TIME:
                DebugPrint(">Stall", param);
                
                if (param < 1) param = 1;
                if (param > 12) param = 12;
                MaxFilmStallTime = param * 1000;
                break;
            case CMD_SET_FRAME_FINE_TUNE:       
                DebugPrint(">FineT", param);
                if (param >= 5 and param <= 95)   
                    PerforationThresholdAutoLevelRatio = param;
                break;
            case CMD_SET_EXTRA_STEPS:
                DebugPrint(">BoostPT", param);
                if (param >= 1 && param <= 30)  
                    FrameExtraSteps = param;
                else if (param >= -30 && param <= -1)  
                    FrameDeductSteps = param;
                break;
            case CMD_SET_SCAN_SPEED:
                DebugPrint(">Speed", param);
                if (param >= 1 and param <= 10) {   
                    ScanSpeed = param;
                    ScanSpeedDelay = BaseScanSpeedDelay + (10-param) * StepScanSpeedDelay;
                    scan_collect_timer = collect_timer = default_collect_timer + (10-param) * 100;
                    OriginalScanSpeedDelay = ScanSpeedDelay;
                    DecreaseSpeedFrameStepsBefore = max(3, 53 - 5*param);
                    DecreaseSpeedFrameSteps = MinFrameSteps - DecreaseSpeedFrameStepsBefore;
                }
                break;
            case CMD_REPORT_PLOTTER_INFO:
                DebugPrint(">PlotterInfo", param);
                IntegratedPlotter = param;
                break;
            case CMD_STOP_SCAN:
                DebugPrintStr(">Scan stop");
                FrameDetected = false;
                LastFrameSteps = 0;
                if (UVLedOn) {
                    analogWrite(11, 0); 
                    UVLedOn = false;
                }
                scan_process_ongoing = false;
                SetReelsAsNeutral(HIGH, HIGH, HIGH);
                ScanState = Sts_Idle;
                break;
            case CMD_SET_AUTO_STOP:
                DebugPrint(">Auto stop", param);
                AutoStopEnabled = param;
                break;
        }

        if (scan_process_ongoing) {
            if (AutoStopEnabled && NoFilmDetected && VFD_mode_active && !EndScanNotificationSent) {
                EndScanNotificationSent = true; 
                SendToRPi(RSP_SCAN_ENDED, 0, 0);
            }
            CollectOutgoingFilm();
        }

        switch (ScanState) {
            case Sts_Idle:
                switch (UI_Command) {
                    case CMD_VERSION_ID:
                        DebugPrintStr(">V_ID");
                        char *pt;
                        pt = strtok (__version__,".");
                        if (pt != NULL) {
                            cnt_ver_1 = atoi(pt);
                            pt = strtok (NULL, ".");
                            if (pt != NULL) {
                                cnt_ver_2 = atoi(pt);
                                pt = strtok (NULL, ".");
                                if (pt != NULL) {
                                    cnt_ver_3 = atoi(pt);
                                }
                                else
                                    cnt_ver_3 = 0;
                            }
                            else
                                cnt_ver_2 = 0;
                        }
                        else
                            cnt_ver_1 = 0;
                        SendToRPi(RSP_VERSION_ID, cnt_ver_1 * 256 + 1, cnt_ver_2 * 256 + cnt_ver_3); 
                        break;
                    case CMD_START_SCAN:
                        tone(A2, 2000, 50);
                        delay(100); 
                        SetReelsAsNeutral(HIGH, LOW, LOW);
                        DebugPrintStr(">Scan start");
                        digitalWrite(MotorB_Direction, HIGH);    
                        VFD_mode_active = param;
                        if (!VFD_mode_active) {   
                            ScanState = Sts_Scan;
                            StartFrameTime = micros();
                            ScanSpeedDelay = OriginalScanSpeedDelay;
                        }
                        analogWrite(11, UVLedBrightness);
                        UVLedOn = true;
                        FilmDetectedTime = millis() + MaxFilmStallTime;
                        NoFilmDetected = false;
                        EndScanNotificationSent = false;
                        switchTimeoutSent = false;  // <--- AGGIUNGI QUESTA LINEA
                        scan_process_ongoing = true;
                        delay(50);
                        collect_timer = scan_collect_timer;
                        break;
                    case CMD_GET_NEXT_FRAME:  
                        ScanState = Sts_Scan;
                        StartFrameTime = micros();
                        ScanSpeedDelay = OriginalScanSpeedDelay;
                        DebugPrint("Save t.",StartFrameTime-StartPictureSaveTime);
                        DebugPrintStr(">Next fr.");
                        
                        
                        if (PT_Level_Auto || Frame_Steps_Auto)
                            
                            SendToRPi(RSP_REPORT_AUTO_LEVELS, PerforationThresholdLevel, MinFrameSteps+FrameDeductSteps);
                        break;
                    case CMD_SET_REGULAR_8:  
                        DebugPrintStr(">R8");
                        IsS8 = false;
                        MinFrameSteps = MinFrameStepsR8;
                        DecreaseSpeedFrameSteps = MinFrameSteps - DecreaseSpeedFrameStepsBefore;
                        OriginalMinFrameSteps = MinFrameSteps;
                        if (!PT_Level_Auto)
                            PerforationThresholdLevel = PerforationThresholdLevelR8;
                        OriginalPerforationThresholdLevel = PerforationThresholdLevelR8;
                        break;
                    case CMD_SET_SUPER_8:  
                        DebugPrintStr(">S8");
                        IsS8 = true;
                        MinFrameSteps = MinFrameStepsS8;
                        DecreaseSpeedFrameSteps = MinFrameSteps - DecreaseSpeedFrameStepsBefore;
                        OriginalMinFrameSteps = MinFrameSteps;
                        if (!PT_Level_Auto)
                            PerforationThresholdLevel = PerforationThresholdLevelS8;
                        OriginalPerforationThresholdLevel = PerforationThresholdLevelS8;
                        break;
                    case CMD_SWITCH_REEL_LOCK_STATUS:
                        ScanState = Sts_UnlockReels;
                        delay(50);
                        break;
                    case CMD_MANUAL_UV_LED:
                        analogWrite(11, UVLedBrightness); 
                        UVLedOn = true;
                        ScanState = Sts_ManualUvLed;
                        break;
                    case CMD_FILM_FORWARD:
                        SetReelsAsNeutral(HIGH, LOW, LOW);
                        FilmDetectedTime = millis() + MaxFilmStallTime;
                        NoFilmDetected = false;
                        collect_timer = 500;
                        analogWrite(11, UVLedBrightness); 
                        UVLedOn = true;
                        ScanState = Sts_SlowForward;
                        digitalWrite(MotorB_Direction, HIGH);    
                        delay(50);
                        break;
                    case CMD_FILM_BACKWARD:
                        SetReelsAsNeutral(HIGH, LOW, HIGH);
                        digitalWrite(MotorB_Direction, LOW);    
                        collect_timer = 10;
                        ScanState = Sts_SlowBackward;
                        delay(50);
                        break;
                    case CMD_SINGLE_STEP:
                        SetReelsAsNeutral(HIGH, LOW, LOW);
                        DebugPrintStr(">SStep");
                        ScanState = Sts_SingleStep;
                        digitalWrite(MotorB_Direction, HIGH);    
                        delay(50);
                        break;
                    case CMD_REWIND: 
                    case CMD_UNCONDITIONAL_REWIND: 
                        if (FilmInFilmgate() and UI_Command == CMD_REWIND) { 
                            DebugPrintStr("Rwnd err");
                            SendToRPi(RSP_REWIND_ERROR, 0, 0);
                            tone(A2, 2000, 100);
                            delay (150);
                            tone(A2, 1000, 100);
                        }
                        else {
                            DebugPrintStr("Rwnd");
                            ScanState = Sts_Rewind;
                            delay (100);
                            SetReelsAsNeutral(LOW, HIGH, HIGH);
                            tone(A2, 2200, 100);
                            delay (150);
                            tone(A2, 2200, 100);
                            delay (150);
                            tone(A2, 2000, 200);
                            RewindSpeed = 4000;
                        }
                        delay(50);
                        break;
                    case CMD_FAST_FORWARD:  
                    case CMD_UNCONDITIONAL_FAST_FORWARD:  
                        if (FilmInFilmgate() and UI_Command == CMD_FAST_FORWARD) { 
                          
                            DebugPrintStr("FF err");
                            SendToRPi(RSP_FAST_FORWARD_ERROR, 0, 0);
                            tone(A2, 2000, 100);
                            delay (150);
                            tone(A2, 1000, 100);
                        }
                        else {
                            DebugPrintStr(">FF");
                            ScanState = Sts_FastForward;
                            delay (100);
                            SetReelsAsNeutral(HIGH, HIGH, LOW);
                            tone(A2, 2000, 100);
                            delay (150);
                            tone(A2, 2000, 100);
                            delay (150);
                            tone(A2, 2200, 200);
                            RewindSpeed = 4000;
                        }
                        break;
                    case CMD_INCREASE_WIND_SPEED:  
                        if (TargetRewindSpeedLoop < 4000)
                            TargetRewindSpeedLoop += 20;
                        break;
                    case CMD_DECREASE_WIND_SPEED:  
                        if (TargetRewindSpeedLoop > 200)
                          TargetRewindSpeedLoop -= 20;
                        break;
                    case CMD_ADVANCE_FRAME:
                        SetReelsAsNeutral(HIGH, LOW, LOW);
                        DebugPrint(">Advance frame", IsS8 ? MinFrameStepsS8 : MinFrameStepsR8);
                        if (IsS8)
                            capstan_advance(MinFrameStepsS8);
                        else
                            capstan_advance(MinFrameStepsR8);
                        break;
                    case CMD_ADVANCE_FRAME_FRACTION:
                        SetReelsAsNeutral(HIGH, LOW, LOW);
                        DebugPrint(">Advance frame", param);
                        
                        if (param >=1 and param <= 400)
                            capstan_advance(param);
                        break;
                    case CMD_RUN_FILM_COLLECTION:   
                        SetReelsAsNeutral(HIGH, LOW, LOW);
                        DebugPrint(">Collect film", param);
                        CollectOutgoingFilmNow();
                        break;
                }
                break;
            case Sts_Scan:
                switch (scan(UI_Command)) {
                    case SCAN_NO_FRAME_DETECTED:
                        break;
                    case SCAN_FRAME_DETECTED:
                        ScanState = Sts_Idle;
                        
                        SendToRPi(RSP_FRAME_AVAILABLE, LastFrameSteps, LastPTLevel);
                        break;
                    case SCAN_TERMINATION_REQUESTED:
                    case SCAN_FRAME_DETECTION_ERROR:
                        ScanState = Sts_Idle;
                        
                        break;
                }
                break;
            case Sts_SingleStep:
                if (scan(UI_Command) != SCAN_NO_FRAME_DETECTED) {
                    ScanState = Sts_Idle;
                    SetReelsAsNeutral(HIGH, HIGH, HIGH);
                }
                break;
            case Sts_UnlockReels:
                if (UI_Command == CMD_SWITCH_REEL_LOCK_STATUS) { 
                    ReelsUnlocked = false;
                    digitalWrite(MotorB_Neutral, LOW);
                    digitalWrite(MotorC_Neutral, LOW);
                    ScanState = Sts_Idle;
                }
                else {
                    if (not ReelsUnlocked){
                        ReelsUnlocked = true;
                        digitalWrite(MotorB_Neutral, HIGH);
                        digitalWrite(MotorC_Neutral, HIGH);
                    }
                    GetLevelPT(); 
                }
                break;
            case Sts_ManualUvLed:
                if (UI_Command == CMD_MANUAL_UV_LED) { 
                    UVLedOn = false;
                    analogWrite(11, 0);
                    ScanState = Sts_Idle;
                }
                else {
                    GetLevelPT(); 
                }
                break;
            case Sts_Rewind:
                if (!RewindFilm(UI_Command)) {
                    DebugPrintStr("-rwnd");
                    ScanState = Sts_Idle;
                    SetReelsAsNeutral(HIGH, HIGH, HIGH);
                }
                break;
            case Sts_FastForward:
                if (!FastForwardFilm(UI_Command)) {
                    DebugPrintStr("-FF");
                    ScanState = Sts_Idle;
                    SetReelsAsNeutral(HIGH, HIGH, HIGH);
                }
                break;
            case Sts_SlowForward:
                if (UI_Command == CMD_FILM_FORWARD) { 
                    delay(50);
                    analogWrite(11, 0); 
                    UVLedOn = false;
                    ScanState = Sts_Idle;
                    SetReelsAsNeutral(HIGH, HIGH, HIGH);
                }
                else {
                    if (!SlowForward()) {
                        analogWrite(11, 0); 
                        UVLedOn = false;
                        ScanState = Sts_Idle;
                        SetReelsAsNeutral(HIGH, HIGH, HIGH);
                    }
                }
                break;
            case Sts_SlowBackward:
                if (UI_Command == CMD_FILM_BACKWARD) { 
                    digitalWrite(MotorB_Direction, HIGH); 
                    delay(50);
                    ScanState = Sts_Idle;
                    SetReelsAsNeutral(HIGH, HIGH, HIGH);
                }
                else {
                    SlowBackward();
                }
                break;
        }
    }
}

void AdjustMinFrameStepsFromCapstanDiameter(float diameter) {
    MinFrameStepsR8 = R8_HEIGHT/((PI*diameter)/(360/(NEMA_STEP_DEGREES/NEMA_MICROSTEPS_IN_STEP))); 
    MinFrameStepsS8 = S8_HEIGHT/((PI*diameter)/(360/(NEMA_STEP_DEGREES/NEMA_MICROSTEPS_IN_STEP))); 
}

void SetReelsAsNeutral(boolean ReelA, boolean ReelB, boolean ReelC) {
    digitalWrite(MotorA_Neutral, ReelA); 
    digitalWrite(MotorB_Neutral, ReelB);
    digitalWrite(MotorC_Neutral, ReelC);

}


boolean RewindFilm(int UI_Command) {
    boolean retvalue = true;
    static boolean stopping = false;

    if (UI_Command == CMD_REWIND) {
        stopping = true;
    }
    else if (stopping) {
        if (RewindSpeed < 4000) {
            digitalWrite(MotorA_Stepper, HIGH);
            delayMicroseconds(RewindSpeed);
            digitalWrite(MotorA_Stepper, LOW);
            RewindSpeed += round(max(1,RewindSpeed/400));
        }
        else {
            retvalue = false;
            stopping = false;
            
            delay (100);
            SendToRPi(RSP_REWIND_ENDED, 0, 0);
        }
    }
    else {
        digitalWrite(MotorA_Stepper, HIGH);
        delayMicroseconds(RewindSpeed);
        digitalWrite(MotorA_Stepper, LOW);
        if (RewindSpeed >= TargetRewindSpeedLoop) {
            RewindSpeed -= round(max(1,RewindSpeed/400));
        }
    }
    return(retvalue);
}


boolean FastForwardFilm(int UI_Command) {
    boolean retvalue = true;
    static boolean stopping = false;

    if (UI_Command == CMD_FAST_FORWARD) {
        stopping = true;
    }
    else if (stopping) {
        if (RewindSpeed < 4000) {
            digitalWrite(MotorC_Stepper, HIGH);
            delayMicroseconds(RewindSpeed);
            digitalWrite(MotorC_Stepper, LOW);
            RewindSpeed += round(max(1,RewindSpeed/400));
        }
        else {
            retvalue = false;
            stopping = false;
            delay (100);
            SendToRPi(RSP_FAST_FORWARD_ENDED, 0, 0);
        }
    }
    else {
        digitalWrite(MotorC_Stepper, HIGH);
        delayMicroseconds(RewindSpeed);
        digitalWrite(MotorC_Stepper, LOW);
        if (RewindSpeed >= TargetRewindSpeedLoop) {
            RewindSpeed -= round(max(1,RewindSpeed/400));
        }
    }
    return(retvalue);
}







void CollectOutgoingFilm(void) {
    static unsigned long TimeToCollect = 0;
    unsigned long CurrentTime = millis();
    static bool lastKnownState = true; 
    bool currentState = digitalRead(TractionStopPin);

    
    
    
    if (currentState != lastKnownState) {
        
        lastSwitchChange = millis();
        
        lastKnownState = currentState;
    }
    

    if (CurrentTime < TimeToCollect && TimeToCollect - CurrentTime < collect_timer) {
        return;
    }
    else {
        TractionSwitchActive = currentState; 
        if (!TractionSwitchActive) {  
            digitalWrite(MotorC_Stepper, LOW);
            digitalWrite(MotorC_Stepper, HIGH);
            
            
            GetLevelPT();
            
            if (AutoStopEnabled && NoFilmDetected) {
                scan_process_ongoing = false;
                SendToRPi(RSP_SCAN_ENDED, 0, 0);
                return;
            }
            
            TractionSwitchActive = digitalRead(TractionStopPin);
        }
        if (TractionSwitchActive)
            TimeToCollect = CurrentTime + collect_timer;
        else
            TimeToCollect = CurrentTime + 3;
    }
}




void CollectOutgoingFilmNow(void) {
    TractionSwitchActive = digitalRead(TractionStopPin);
    while (!TractionSwitchActive) {  
        digitalWrite(MotorC_Stepper, LOW);
        digitalWrite(MotorC_Stepper, HIGH);
        
    
    GetLevelPT();

    
    if (AutoStopEnabled && NoFilmDetected) {
        scan_process_ongoing = false;
        SendToRPi(RSP_SCAN_ENDED, 0, 0);
        return;
    }

        TractionSwitchActive = digitalRead(TractionStopPin);
        delay(10);
    }
}


boolean film_detected(int pt_value)
{
    static int max_value, min_value;
    int instant_variance;
    static unsigned long time_to_renew_minmax = 0;
    unsigned long CurrentTime = millis();
    int minmax_validity_time = 2000; 

    if (CurrentTime > time_to_renew_minmax || time_to_renew_minmax - CurrentTime > minmax_validity_time) {
      time_to_renew_minmax = CurrentTime + minmax_validity_time;
      max_value = MinPT;
      min_value = MaxPT;
    }
    max_value = max(max_value, pt_value);
    min_value = min(min_value, pt_value);
    instant_variance = max_value - min_value;
    if (instant_variance > 50)
        return(true);
    else
        return(false);
}


int GetLevelPT() {
    float ratio;
    int user_margin, fixed_margin, average_pt;
    unsigned long CurrentTime = millis();

    PT_SignalLevelRead = analogRead(PHOTODETECT);
    MaxPT = max(PT_SignalLevelRead, MaxPT);
    MinPT = min(PT_SignalLevelRead, MinPT);
    MaxPT_Dynamic = max(PT_SignalLevelRead*10, MaxPT_Dynamic);
    MinPT_Dynamic = min(PT_SignalLevelRead*10, MinPT_Dynamic);
    if (MaxPT_Dynamic > (MinPT_Dynamic+5)) MaxPT_Dynamic-=5;
    
    
    if (MinPT_Dynamic < (MaxPT_Dynamic-15)) MinPT_Dynamic+=15;
    
    if (PT_Level_Auto && FrameStepsDone >= int((MinFrameSteps+FrameDeductSteps)*0.9)) {
        ratio = (float)PerforationThresholdAutoLevelRatio/100;
        fixed_margin = int((MaxPT_Dynamic-MinPT_Dynamic) * 0.1);
        user_margin = int((MaxPT_Dynamic-MinPT_Dynamic) * 0.9 * ratio);
        PerforationThresholdLevel = int((MinPT_Dynamic + fixed_margin + user_margin)/10);
    }

    
    if (CurrentTime > FilmDetectedTime) 
        NoFilmDetected = true;
    else if (film_detected(PT_SignalLevelRead))
        FilmDetectedTime = millis() + MaxFilmStallTime;

    return(PT_SignalLevelRead);
}


void ReportPlotterInfo() {
    static unsigned long NextReport = 0;
    static int Previous_PT_Signal = 0, PreviousFrameSteps = 0;
    static char out[100];
    if (millis() > NextReport) {
        if (Previous_PT_Signal != PT_SignalLevelRead || PreviousFrameSteps != LastFrameSteps) {
            NextReport = millis() + 20;
            if (DebugState == PlotterInfo) {  
                sprintf(out,"PT:%i, Th:%i, FSD:%i, PTALR:%i, MinD:%i, MaxD:%i", PT_SignalLevelRead, PerforationThresholdLevel, FrameStepsDone, PerforationThresholdAutoLevelRatio, MinPT_Dynamic/10, MaxPT_Dynamic/10);
                SerialPrintStr(out);
            }
            Previous_PT_Signal = PT_SignalLevelRead;
            PreviousFrameSteps = LastFrameSteps;
            if (IntegratedPlotter)  
                SendToRPi(RSP_REPORT_PLOTTER_INFO, PT_SignalLevelRead, PerforationThresholdLevel);
        }
    }
}

boolean SlowForward(){
    static unsigned long LastMove = 0;
    unsigned long CurrentTime = micros();
    if (CurrentTime > LastMove || LastMove-CurrentTime > 400) { 
        GetLevelPT(); 
        CollectOutgoingFilm();
        digitalWrite(MotorB_Stepper, LOW);
        digitalWrite(MotorB_Stepper, HIGH);
        LastMove = CurrentTime + 400;
    }
    
    if (AutoStopEnabled && NoFilmDetected) {
        SendToRPi(RSP_FILM_FORWARD_ENDED, 0, 0);
        return(false);
    }
    else return(true);
}

void SlowBackward(){
    static unsigned long LastMove = 0;
    unsigned long CurrentTime = micros();
    if (CurrentTime > LastMove || LastMove-CurrentTime > 700) { 
        GetLevelPT(); 
        
        
        digitalWrite(MotorB_Stepper, LOW);
        digitalWrite(MotorB_Stepper, HIGH);
        LastMove = CurrentTime + 700;
    }
}


boolean FilmInFilmgate() {
    int SignalLevel;
    boolean retvalue = false;
    int mini=800, maxi=0;

    analogWrite(11, UVLedBrightness); 
    UVLedOn = true;
    delay(200); 

    SetReelsAsNeutral(HIGH, LOW, HIGH); 

    
    
    for (int x = 0; x <= 300; x++) {
        digitalWrite(MotorB_Stepper, LOW);
        digitalWrite(MotorB_Stepper, HIGH);
        SignalLevel = GetLevelPT();
        if (SignalLevel > maxi) maxi = SignalLevel;
        if (SignalLevel < mini) mini = SignalLevel;
    }
    digitalWrite(MotorB_Stepper, LOW);
    SetReelsAsNeutral(HIGH, HIGH, HIGH);   

    analogWrite(11, 0); 
    UVLedOn = false;
    if (abs(maxi-mini) > 0.33*(MaxPT-MinPT))   
        retvalue = true;

    return(retvalue);
}

void adjust_framesteps(int frame_steps) {
    static int steps_per_frame_list[32];
    static int idx = 0;
    static int items_in_list = 0;
    int total = 0;

    
    if (frame_steps > int(OriginalMinFrameSteps*1.05) || frame_steps < int(OriginalMinFrameSteps*0.95)) {   
        return; 
    }

    
    steps_per_frame_list[idx] = frame_steps;
    idx = (idx + 1) % 32;
    if (items_in_list < 32)
        items_in_list++;
    if (Frame_Steps_Auto) {  
        for (int i = 0; i < items_in_list; i++)
            total = total + steps_per_frame_list[i];
        MinFrameSteps = int(total / items_in_list) - 5;
        DecreaseSpeedFrameSteps = MinFrameSteps - DecreaseSpeedFrameStepsBefore;
    }
}




boolean IsHoleDetected() {
    boolean hole_detected = false;
    int PT_Level;
    PT_Level = GetLevelPT();

    
    
    
    
    
    
    
    
    if (PT_Level >= PerforationThresholdLevel && FrameStepsDone >= int(MinFrameSteps+FrameDeductSteps)) {
        LastPTLevel = PT_Level;
        hole_detected = true;
        GreenLedOn = true;
        analogWrite(A1, 255); 
    }

    return(hole_detected);
}



void capstan_advance(int steps) {
    int middle, delay_factor;
    if (steps > 20) 
        middle = int(steps/2);
    for (int x = 0; x < steps; x++) {    
        digitalWrite(MotorB_Stepper, LOW);
        digitalWrite(MotorB_Stepper, HIGH);
        if (steps > 20) {
            delay_factor = (x < middle) ?
            int(middle - x) : (steps - x);
            delayMicroseconds(50 + min(500, delay_factor*10));
            GetLevelPT(); 
        }
        else if (steps >= 1)
            delayMicroseconds(100);
    }
    digitalWrite(MotorB_Stepper, LOW);
    if (VFD_mode_active)
        SendToRPi(RSP_ADVANCE_FRAME_FRACTION, steps, 0);
}



ScanResult scan(int UI_Command) {
    ScanResult retvalue = SCAN_NO_FRAME_DETECTED;
    static unsigned long TimeToScan = 0;
    unsigned long CurrentTime = micros();
    int FrameStepsToDo = 1;
    if (CurrentTime < TimeToScan && TimeToScan - CurrentTime < ScanSpeedDelay) {
        return (retvalue);
    }
    else {
        TimeToScan = CurrentTime + ScanSpeedDelay;
        if (GreenLedOn) {  
            GreenLedOn = false;
            analogWrite(A1, 0); 
        }

        if (FrameStepsDone > DecreaseSpeedFrameSteps)   
            ScanSpeedDelay = OriginalScanSpeedDelay +
                min(20000, DecreaseScanSpeedDelayStep * (FrameStepsDone - DecreaseSpeedFrameSteps + 1));
        FrameDetected = false;

        
        
        if (AutoStopEnabled && NoFilmDetected) {
            SendToRPi(RSP_SCAN_ENDED, 0, 0);
            return(SCAN_TERMINATION_REQUESTED);
        }

        
        bool currentSwitch = digitalRead(TractionStopPin);
        unsigned long now = millis();
        if (currentSwitch != switchStateBackup) {
            switchStateBackup = currentSwitch;
            lastSwitchChange = now;
            switchTimeoutSent = false;

        } else if (!switchTimeoutSent && (now - lastSwitchChange >= (unsigned long)MaxFilmStallTime)) {
        switchTimeoutSent = true;
        SendToRPi(RSP_SCAN_ENDED, 0, 0);
        return (SCAN_TERMINATION_REQUESTED);
        }
     
        FrameDetected = IsHoleDetected();
        if (!FrameDetected) {
            
            FrameStepsToDo = 1;
            capstan_advance(FrameStepsToDo);
            FrameStepsDone+=FrameStepsToDo;
        }

        if (FrameDetected) {
            DebugPrintStr("Frame!");
            if (Frame_Steps_Auto and FrameExtraSteps > 0)  
                capstan_advance(FrameExtraSteps);
            LastFrameSteps = FrameStepsDone;
            adjust_framesteps(LastFrameSteps);
            FrameStepsDone = 0;
            TimeToScan = 0;
            StartPictureSaveTime = micros();
            
            if (ScanState == Sts_SingleStep) {  
                tone(A2, 2000, 35);
                delay(100);     
                analogWrite(11, UVLedBrightness); 
            }
            FrameDetected = false;
            retvalue = SCAN_FRAME_DETECTED;
            DebugPrint("FrmS",LastFrameSteps);
            DebugPrint("FrmT",CurrentTime-StartFrameTime);
            if (DebugState == FrameSteps)
                SerialPrintInt(LastFrameSteps);
        }
        else if (FrameStepsDone > 2*MinFrameSteps) {
            retvalue = SCAN_FRAME_DETECTION_ERROR;
            
            SendToRPi(RSP_SCAN_ERROR, FrameStepsDone, 2*MinFrameSteps);
            FrameStepsDone = 0;
        }
        return (retvalue);
    }
}




void receiveEvent(int byteCount) {
    int IncomingIc, param = 0;
    if (Wire.available())
        IncomingIc = Wire.read();
    if (Wire.available())
        param =  Wire.read();
    if (Wire.available())
        param +=  256*Wire.read();
    while (Wire.available())
        Wire.read();
    if (IncomingIc > 0) {
        push_cmd(IncomingIc, param);
        
    }
}


void sendEvent() {
    int cmd, p1, p2;
    cmd = pop_rsp(&p1, &p2);
    if (cmd != -1) {
        BufferForRPi[0] = cmd;
        BufferForRPi[1] = p1/256;
        BufferForRPi[2] = p1%256;
        BufferForRPi[3] = p2/256;
        BufferForRPi[4] = p2%256;
        Wire.write(BufferForRPi,5);
    }
    else {
        BufferForRPi[0] = 0;
        BufferForRPi[1] = 0;
        BufferForRPi[2] = 0;
        BufferForRPi[3] = 0;
        BufferForRPi[4] = 0;
        Wire.write(BufferForRPi,5);
    }
}

boolean push(Queue * queue, int IncomingIc, int param, int param2) {
    boolean retvalue = false;
    if ((queue -> in+1) % QUEUE_SIZE != queue -> out) {
        queue -> Data[queue -> in] = IncomingIc;
        queue -> Param[queue -> in] = param;
        queue -> Param2[queue -> in] = param2;
        queue -> in++;
        queue -> in %= QUEUE_SIZE;
        retvalue = true;
    }
    
    
    return(retvalue);
}

int pop(Queue * queue, int * param, int * param2) {
    int retvalue = -1;
    
    if (queue -> out != queue -> in) {
        retvalue = queue -> Data[queue -> out];
        if (param != NULL)
            *param =  queue -> Param[queue -> out];
        if (param2 != NULL)
            *param2 =  queue -> Param2[queue -> out];
        queue -> out++;
        queue -> out %= QUEUE_SIZE;
    }
    
    return(retvalue);
}

boolean push_cmd(int cmd, int param) {
    push(&CommandQueue, cmd, param, 0);
}
int pop_cmd(int * param) {
    return(pop(&CommandQueue, param, NULL));
}
boolean push_rsp(int rsp, int param, int param2) {
    push(&ResponseQueue, rsp, param, param2);
}
int pop_rsp(int * param, int * param2) {
    return(pop(&ResponseQueue, param, param2));
}

boolean dataInCmdQueue(void) {
    return (CommandQueue.out != CommandQueue.in);
}

boolean dataInRspQueue(void) {
    return (ResponseQueue.out != ResponseQueue.in);
}

void DebugPrintAux(const char * str, unsigned long i) {
    static char PreviousDebug[64];
    static char AuxLine[64];
    static char PrintLine[64];
    static int CurrentRepetitions = 0;
    boolean GoPrint = true;
    if (DebugState != DebugInfo && DebugState != DebugInfoSingle) return;

    if (strlen(str) >= 50) {
        Serial.println("Cannot print debug line, too long");
        return;
    }

    if (i != -1)
        sprintf(PrintLine,"%s=%u",str,i);
    else
        strcpy(PrintLine,str);
  
    if (strcmp(PrintLine,PreviousDebug) == 0) {
        if (CurrentRepetitions < MAX_DEBUG_REPETITIONS_COUNT)
            CurrentRepetitions++;
        if (CurrentRepetitions > MaxDebugRepetitions) GoPrint = false;
    }
    else {
        if (CurrentRepetitions > MaxDebugRepetitions) {
            if (CurrentRepetitions < MAX_DEBUG_REPETITIONS_COUNT)
                sprintf(AuxLine,"Previous line repeated %u times",CurrentRepetitions-MaxDebugRepetitions);
            else
                strcpy(AuxLine,"Previous line repeated more than 30,000 times");
            Serial.println(AuxLine);
        }
        CurrentRepetitions = 0;
    }
    strcpy(PreviousDebug, PrintLine);
    if (GoPrint) Serial.println(PrintLine);
}

void DebugPrintStr(const char * str) {
    if (DebugState != DebugInfo) return;
    DebugPrintAux(str,-1);
}


void DebugPrint(const char * str, unsigned long i) {
    if (DebugState != DebugInfo) return;
    DebugPrintAux(str,i);
}


void SerialPrintStr(const char * str) {
    if (DebugState != DebugInfo) Serial.println(str);
}

void SerialPrintInt(int i) {
    if (DebugState != DebugInfo) Serial.println(i);
}