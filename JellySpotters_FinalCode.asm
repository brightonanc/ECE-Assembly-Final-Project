; Final Project Code for ECE 2031
; The Jelly Spotters Team consists of Brighton Ancelin, Reid Barton, Brett Bodamer, and Michael Chan
; Intended to be copied into the provided "library" assembly file under the "Main:" label
; 
; Intruder Detection Program:
; Moves a robot in a predetermined path (with some degree of hysteresis) and continuously uses
; ultrasonic sensors to scan for intruding objects. Calibration of position is done through a
; combination of odometry and ultrasonic means. To distinguish between intruding objects and
; allowed objects (e.g. walls, the baffle, objects outside the guarded zone), a variety of methods
; are implemented. These methods include, but are not limited to, consistency-checking,
; state transitions, sensor noise reduction, and multi-sensor validation.

; ------------------------------------------------------------- BEGIN JELLY SPOTTERS MAIN CODE -----------------------------------------        

    ;AUTHOR: Brighton Ancelin
    ; STEP 1 - Move into the field on the left
    
JSStep1:
    LOAD   FMid
    STORE  DVel         ; Move into the arena
Pos1:
    IN     XPOS         ; move from the one foot boarder zone to the reset position
    SUB    ZeroOne    
    JNEG   Pos1       
    LOADI  0
    STORE  DVel         ; stop the robot
    LOADI  270
    STORE  DTheta 
    CALL TurnWell ; turn right
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 2 - Move forward and calibrate on the left
    
JSStep2:
    CALL PosZ
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 3 - Check for under-the-baffle intruders on the left
    
JSStep3:
    CALL    Wait0Point1
    CALL    JSUnderBaffleScanL
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 4 - Series of turn-and-go's on the left
    
JSStep4:
    CALL    SideCalL
    CALL Pos15
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 5 - Move along the top arc (really a line) and scan for intruders starting from the left
    
JSStep5:
    LOAD   FMid        
    STORE  DVel  
    CALL    JSPreTopArcScanInit
JSStep5Sub:
    CALL    SonarCheck
    CALL    JSTopArcScanLR
    CALL    Wait0Point1
    IN      XPOS
    SUB     TOP_ARC_WIDTH
    JNEG    JSStep5Sub
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 6 - Series of turn-and-go's on the right
    
JSStep6:
    CALL Pos3Mod
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 7 - Move forward and calibrate on the right
    
JSStep7:
    CALL PosZ
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 8 - Check for under-the-baffle intruders on the right
    
JSStep8:
    CALL    Wait0Point1
    CALL    JSUnderBaffleScanR
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 9 - Series of turn-and-go's on the right
    
JSStep9:
    CALL    SideCalR
    CALL Pos15OtherDirection
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 10 - Move along the top arc (really a line) and scan for intruders starting from the right
    
JSStep10:
    CALL    JSPreTopArcScanInit
JSStep10Sub:
    CALL    SonarCheck
    CALL    JSTopArcScanRL
    CALL    Wait0Point1
    IN      XPOS
    SUB     TOP_ARC_WIDTH
    JNEG    JSStep10Sub
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 11 - Series of turn-and-go's on the right
    
JSStep11:
    CALL FinalTurnings
    
    ;AUTHOR: Brighton Ancelin
    ; STEP 12 - Jump back to step 2 to repeat the process indefinitely

JSStep12:   
    JUMP    JSStep2
    ; That's all folks!

; ------------------------------------------------------------- END JELLY SPOTTERS MAIN CODE -----------------------------------------
    
; ------------------------------------------------------------- BEGIN JELLY SPOTTERS AUXILIARY CODE -----------------------------------------        



FinalTurnings:
    LOADI  0
    STORE  DVel         ; stop the robot

    
    LOADI  90           ; +10 for error
    STORE  DTheta       ; turn left
    CALL TurnWell
    
    LOAD   Zero
    STORE  DTheta
    OUT    RESETPOS 
    
    
    
    LOAD   FMid        
    STORE  DVel   
    
Pos7:
    CALL SonarCheck
    IN     XPOS         ; X position from odometry
    SUB   OneMeter
    JNEG   Pos7         ; Not there yet, keep checking
    
    
    LOADI  0
    STORE  DVel         ; stop the robot
    
    LOADI 90;
    STORE DTheta
    CALL  TurnWell
    
    LOAD   Zero
    STORE  DTheta
    OUT    RESETPOS
    
    RETURN


Pos15OtherDirection:
    LOADI  270
    STORE  DTheta       ; turn right
    CALL TurnWell
    
    LOAD   Zero
    STORE  DTheta
    OUT    RESETPOS 
    
    LOAD   FMid        
    STORE  DVel   
    
Pos5:
    CALL SonarCheck
    IN     XPOS         ; X position from odometry
    SUB    YDelta
    SUB    OneMeter 
    JNEG   Pos5         ; Not there yet, keep checking
    
     LOADI  0
     STORE  DVel        ; stop the robot
    
    LOADI  90           ; + 5 because left turn error
    STORE  DTheta       ; turn left
    CALL TurnWell
    
    LOAD   Zero
    STORE  DTheta
    OUT    RESETPOS 
    
    LOAD   FMid        
    STORE  DVel
    
    RETURN
    
Pos3Mod:
    LOADI  0
    STORE  DVel         ; stop the robot
          
    LOADI  270
    STORE  DTheta       ; turn right
    CALL TurnWell
    
    
    LOAD   Zero
    STORE  DTheta
    OUT    RESETPOS 
    
    
    LOAD   FMid        
    STORE  DVel   
    
Pos4:
    CALL SonarCheck
    IN     XPOS         ; X position from odometry
    SUB    OneMeter
    SUB    OneFoot
    JNEG   Pos4         ; Not there yet, keep checking
    
    LOADI  0
    STORE  DVel         ; stop the robot
      
    LOADI  270
    STORE  DTheta       ; turn right
    CALL TurnWell
    
    RETURN  


Pos15:
    LOADI  90
    STORE  DTheta       ; turn left so that we face the wall
    CALL TurnWell
    
    
    LOAD   Zero
    STORE  DTheta
    OUT    RESETPOS     ; then reset position
    
;   CALL   JSSonarInit
    
    
    LOAD   FMid         ; defined below as 100
    STORE  DVel         ; move forward
    
;   LOADI &B00001100    ; renable the front sonar so that we dont run into anything
;   OUT   SONAREN
    
    
Pos2:
    CALL SonarCheck
    IN     XPOS        
    SUB    YDelta     
    SUB    Dist4B
    JNEG   Pos2         ; Use the YDelta calibration distance
    
      
    LOADI  0
    STORE  DVel         ; stop the robot
    
    LOADI  270
    STORE  DTheta       ; turn right
    CALL TurnWell
    
    LOAD   Zero
    STORE  DTheta
    OUT    RESETPOS 
    
    RETURN

    
PosZ:   
    
    LOAD   FSlow
    STORE  DVel         ; move slowly towards the baffle
    
;   LOADI &B00001100
;   OUT   SONAREN       ; use the front to sensors to detect when to stop
        
WallYet: IN DIST2
         ADDI -308
         JNEG Calibrate
         IN DIST3
         ADDI -308
         JNEG Calibrate
         JUMP WallYet
    
Calibrate: LOADI  0
           STORE  DVel  ; stop the robot when it gets to close to the baffle
           
CheckAng:  IN DIST2     ; read the left front sonar in
           STORE Temp2 ; store it for later
           IN DIST3 ; read in the right front sonar
           SUB Temp2 ; Sonar R - Sonar L
           STORE Temp2 ; store the differnce
           CALL Abs
           ADDI -20
           JNEG PosZEnd ; see if it straight enough to move on
           CALL WaitShort
           LOAD Temp2 ; otherwise take back out the difference to tell which way to turn
           OUT  LCD
           JPOS TurnR ; L < R
           JUMP TurnL
           

TurnL:      LOAD DTheta ;
            ADDI -3
            STORE DTheta ; Turn the robot left 3 degrees 
            JUMP CheckAng ; go see if we are straight enough

TurnR:      LOAD DTheta
            ADDI 3
            STORE DTheta
            JUMP CheckAng
            
PosZEnd:    RETURN

SideCalL:   ;LOADI &H01 ; enable the side sonar to see how far we are from the wall
            ;OUT   SONAREN
            ;CALL  WaitShort
            IN    DIST0
            STORE YDelta ; store the distance from the T for later use
            OUT   LCD
            
            
            LOAD   Zero
            OUT    RESETPOS 
            STORE  DTheta ; reset the robots position now that we know where we are
          
            LOAD  RMid
            STORE  DVel ; back up from the wall
            
PosYL:  
        IN     XPOS        
        ADD    Dist4A 
        JPOS   PosYL      ; move backwards two feet back to the reset position (position 1)
            
        LOAD Zero           
        STORE DVel  ; stop 
        RETURN
        
SideCalR:   ;LOADI &H01 ; enable the side sonar to see how far we are from the wall
            ;OUT   SONAREN
            ;CALL  WaitShort
            IN    DIST5
            STORE YDelta ; store the distance from the T for later use
            OUT   LCD
            
            
            LOAD   Zero
            OUT    RESETPOS 
            STORE  DTheta ; reset the robots position now that we know where we are
          
            LOAD  RMid
            STORE  DVel ; back up from the wall
            
PosYR:  
        IN     XPOS        
        ADD    OneMeter   
        ADD    OneFoot  
        JPOS   PosYR      ; move backwards two feet back to the reset position (position 1)
            
        LOAD Zero           
        STORE DVel  ; stop 
        RETURN
    
    
TurnWell:
    CALL   GetThetaErr ; get the heading error
    CALL   Abs         ; absolute value subroutine
    OUT    LCD         ; Display |angle error| for debugging
    ADDI   -5          ; check if within 5 degrees of target angle
    JPOS   TurnWell     ; if not, keep testing
    RETURN             ; the robot is now within 5 degrees of desired
    
;AUTHOR: Brighton Ancelin
JSUnderBaffleScanL:
    
    LOAD    STR_ABC0    ;Revision: New2
    STORE   STR_DEBUG   ;Revision: New2
    
    IN      DIST0
    STORE   UBTop
    SUB     UNDER_BAFFLE_WIDTH
    SUB     MAX_SENSOR_NOISE
    JNEG    JS_UBSL_NoIntruder
    CALL    JSIntruderDetected
    
    
    JS_UBSL_NoIntruder:
    
    LOAD    STR_ABC1    ;Revision: New2
    STORE   STR_DEBUG   ;Revision: New2
    
    IN      DIST5
    STORE   UBBottom
    ADD     UBTop
    SUB     UNDER_BAFFLE_WIDTH
    ADD     MAX_SENSOR_NOISE
    JNEG    JSIntruderDetected ; OK to jump ONLY because RETURN is next statement anyway
    RETURN
    
;AUTHOR: Brighton Ancelin
JSUnderBaffleScanR:
    
    LOAD    STR_ABC0    ;Revision: New2
    STORE   STR_DEBUG   ;Revision: New2

    IN      DIST5
    STORE   UBTop
    SUB     UNDER_BAFFLE_WIDTH
    SUB     MAX_SENSOR_NOISE
    JNEG    JS_UBSR_NoIntruder
    CALL    JSIntruderDetected
    
    JS_UBSR_NoIntruder:
    
    LOAD    STR_ABC1    ;Revision: New2
    STORE   STR_DEBUG   ;Revision: New2
    
    IN      DIST0
    STORE   UBBottom
    ADD     UBTop
    SUB     UNDER_BAFFLE_WIDTH
    ADD     MAX_SENSOR_NOISE
    JNEG    JSIntruderDetected ; OK to jump ONLY because RETURN is next statement anyway
    RETURN
    
;AUTHOR: Brighton Ancelin
JSPreTopArcScanInit:
    ; MUST BE CALLED BEFORE A NEW SCAN
    ; Initializes values for a scan
    LOAD    INIT_SamplesToWaitTop
    STORE   SamplesToWaitTop
    LOAD    INIT_SamplesToWaitBottom
    STORE   SamplesToWaitBottom
    LOADI   0
    STORE   ArcTransState
    STORE   DRT0
    STORE   DRT1
    STORE   DRT2
    STORE   DRT3
    STORE   DRB0
    STORE   DRB1
    STORE   DRB2
    STORE   DRB3
    STORE   PrevAvgTop
    STORE   CurAvgTop
    STORE   PrevAvgBottom
    STORE   CurAvgBottom
    RETURN
    
;AUTHOR: Brighton Ancelin
JSTopArcScanLR:
    CALL    JSPushSonarLR           ; Get fresh sonar data
    LOAD    SamplesToWaitTop        ; Check if top can be checked
    JZERO   JS_TASLR_CheckTop
    ADDI    -1
    STORE   SamplesToWaitTop
    JUMP    JS_TASLR_NextWaitCheck
JS_TASLR_CheckTop:
    CALL    JSCheckSonarScanTop     ; Check top
JS_TASLR_NextWaitCheck:
    LOAD    SamplesToWaitBottom     ; Check if bottom can be checked
    JZERO   JS_TASLR_CheckBottom
    ADDI    -1
    STORE   SamplesToWaitBottom
    JUMP    JS_TASLR_AfterAll
JS_TASLR_CheckBottom:
    CALL    JSCheckSonarScanBottom  ; Check bottom
JS_TASLR_AfterAll:
    RETURN
    
;AUTHOR: Brighton Ancelin
JSTopArcScanRL:
    CALL    JSPushSonarRL           ; Get fresh sonar data
    LOAD    SamplesToWaitTop        ; Check if top can be checked
    JZERO   JS_TASRL_CheckTop
    ADDI    -1
    STORE   SamplesToWaitTop
    JUMP    JS_TASRL_NextWaitCheck
JS_TASRL_CheckTop:
    CALL    JSCheckSonarScanTop     ; Check top
JS_TASRL_NextWaitCheck:
    LOAD    SamplesToWaitBottom     ; Check if bottom can be checked
    JZERO   JS_TASRL_CheckBottom
    ADDI    -1
    STORE   SamplesToWaitBottom
    JUMP    JS_TASRL_AfterAll
JS_TASRL_CheckBottom:
    CALL    JSCheckSonarScanBottom  ; Check bottom
JS_TASRL_AfterAll:
    RETURN
    
;AUTHOR: Brighton Ancelin
JSPushSonarLR:
    LOAD    SamplesToWaitTop
    ADDI    1
    STORE   SamplesToWaitTop
    IN      DIST0
    STORE   Temp4
    SUB     BAD_DIST_TOP
    JPOS    JS_PSLR_DontPushTop
    ; Push back data
    LOAD    DRT2
    STORE   DRT3
    LOAD    DRT1
    STORE   DRT2
    LOAD    DRT0
    STORE   DRT1
    LOAD    Temp4
    STORE   DRT0
    
    LOAD    SamplesToWaitTop
    ADDI    -1
    STORE   SamplesToWaitTop
    
;AUTHOR: Brighton Ancelin
JS_PSLR_DontPushTop:    
    ; Push back data
    LOAD    DRB2
    STORE   DRB3
    LOAD    DRB1
    STORE   DRB2
    LOAD    DRB0
    STORE   DRB1
    
    IN      DIST5
    STORE   DRB0
    RETURN
    
;AUTHOR: Brighton Ancelin
JSPushSonarRL:
    LOAD    SamplesToWaitTop
    ADDI    1
    STORE   SamplesToWaitTop
    IN      DIST5
    STORE   Temp4
    SUB     BAD_DIST_TOP
    JPOS    JS_PSRL_DontPushTop
    ; Push back data
    LOAD    DRT2
    STORE   DRT3
    LOAD    DRT1
    STORE   DRT2
    LOAD    DRT0
    STORE   DRT1
    LOAD    Temp4
    STORE   DRT0
    
    LOAD    SamplesToWaitTop
    ADDI    -1
    STORE   SamplesToWaitTop
    
;AUTHOR: Brighton Ancelin
JS_PSRL_DontPushTop:
    ; Push back data
    LOAD    DRB2
    STORE   DRB3
    LOAD    DRB1
    STORE   DRB2
    LOAD    DRB0
    STORE   DRB1
    ; Insert new readings
    IN      DIST0
    STORE   DRB0
    RETURN
    
;AUTHOR: Brighton Ancelin
JSCheckSonarScanTop:
    ; Ensure the top readings are a smooth function
    ; Average oldest half
    LOAD    DRT3
    SHIFT   -1
    STORE   Temp
    LOAD    DRT2
    SHIFT   -1
    ADD     Temp
    ; Store to variable
    STORE   PrevAvgTop
    ; Average newest half
    LOAD    DRT1
    SHIFT   -1
    STORE   Temp
    
    LOAD    STR_DAB0    ;New2
    STORE   STR_DEBUG   ;New2
    
    LOAD    DRT0
    SHIFT   -1
    ADD     Temp
    ; Store to variable
    STORE   CurAvgTop
    
    SUB     PrevAvgTop
    CALL    Abs
    ; AC now contains Abs(Averaged Error)
    SUB     MAX_SENSOR_NOISE
    JPOS    JSIntruderDetected  ; OK to jump ONLY because RETURN is next statement anyway
    RETURN
    
;AUTHOR: Brighton Ancelin
JSCheckSonarScanBottom:
    ; Ensure that width around robot is not too small (intruder below)
    ; Choose appropriate method of evaluation based on state
    LOAD    ArcTransState
    JZERO   JSArcBeforeBaffle   ; Will call RETURN
    SHIFT   -1
    JZERO   JSArcOverBaffle     ; Will call RETURN
    JUMP    JSArcPostBaffle     ; Will call RETURN
    
;AUTHOR: Brighton Ancelin
JSArcBeforeBaffle:  
    
    LOAD    DRB1
    SHIFT   -1
    STORE   Temp
    LOAD    DRB0
    SHIFT   -1
    ADD     Temp
    ; AC now contains CurAvgBottom
    
    ADD     CurAvgTop
    ; AC now contains total observed width
    SUB     KNOWN_BAFFLELESS_WIDTH
    ; AC now contains how much larger observed width is from known baffleless width
    ADD     MAX_SENSOR_NOISE
    JPOS    JS_ABB_NoIntr
    ; Change ArcTransState to over baffle
    LOADI   &H001
    STORE   ArcTransState
    LOADI   4
    STORE   SamplesToWaitBottom
JS_ABB_NoIntr:
    RETURN
    
;AUTHOR: Brighton Ancelin
JSArcOverBaffle:
    
    ; Ensure the bottom readings are a smooth function

    LOAD    DRB3
    SHIFT   -1
    STORE   Temp
    LOAD    DRB2
    SHIFT   -1
    ADD     Temp
    
    STORE   PrevAvgBottom
    
    LOAD    DRB1
    SHIFT   -1
    STORE   Temp
    LOAD    DRB0
    SHIFT   -1
    ADD     Temp
    
    STORE   CurAvgBottom
    
    SUB     PrevAvgBottom
    CALL    Abs
    ; AC now contains Abs(Averaged Error)
    SUB     MAX_SENSOR_NOISE
    JNEG    JS_AOB_NoAnomaly
    LOAD    CurAvgBottom
    SUB     PrevAvgBottom
    JNEG    JS_AOB_Intruder     ;Revision: NEW
    ; Change ArcTransState to after baffle
    LOADI   &H002
    STORE   ArcTransState
    LOADI   2
    STORE   SamplesToWaitBottom
JS_AOB_NoAnomaly:
    RETURN
JS_AOB_Intruder:                ;Revision: NEW
    
    LOAD    STR_BAD1            ;Revision: New2
    STORE   STR_DEBUG           ;Revision: New2

    CALL    JSIntruderDetected  ;Revision: NEW
    LOADI   2                   ;Revision: NEW
    STORE   SamplesToWaitBottom ;Revision: NEW
    RETURN                      ;Revision: NEW
    
;AUTHOR: Brighton Ancelin
JSArcPostBaffle:
    
    LOAD    STR_BAD2    ;Revision: New2
    STORE   STR_DEBUG   ;Revision: New2

    LOAD    DRB1
    SHIFT   -1
    STORE   Temp
    LOAD    DRB0
    SHIFT   -1
    ADD     Temp
    ; AC now contains CurAvgBottom
    
    ADD     CurAvgTop
    ; AC now contains total observed width
    SUB     KNOWN_BAFFLELESS_WIDTH
    ; AC now contains how much larger observed width is from known baffleless width
    ADD     MAX_SENSOR_NOISE
    JNEG    JS_APB_Intruder
    RETURN
    
    JS_APB_Intruder:
    
    CALL    JSIntruderDetected
    ; Change ArcTransState to on REAL baffle    ;Revision: NEW
    LOADI   &H001           ;Revision: NEW
    STORE   ArcTransState   ;Revision: NEW
    LOADI   4               ;Revision: NEW
    STORE   SamplesToWaitBottom ;Revision: NEW
    RETURN
    
;AUTHOR: Brighton Ancelin
JSIntruderDetected:
    LOAD    STR_DEBUG
    OUT     LCD
    LOADI   &H130
    OUT     BEEP
    
    LOAD    DVel
    STORE   Temp3
    LOADI   0
    STORE   DVel
    CALL    Wait1
    CALL    Wait1

    LOAD    DRT3    ;Revision: New3
    OUT     LCD     ;Revision: New3
    CALL    Wait1   ;Revision: New3
    
    LOAD    DRT2    ;vNew3
    OUT     LCD     ;Revision: New3
    CALL    Wait1   ;Revision: New3
    
    LOAD    DRT1    ;Revision: New3
    OUT     LCD     ;Revision: New3
    CALL    Wait1   ;Revision: New3
    
    LOAD    DRT0    ;Revision: New3
    OUT     LCD     ;Revision: New3
    CALL    Wait1   ;Revision: New3
    
    LOAD    Temp3
    STORE   DVel
    
    RETURN
    
    
    
    
SonarCheck:     ;LOADI &B00011110
                ;OUT SONAREN
                IN SONALARM ; avoid stuff
                JZero Skip
                CALL AvoidCol    
Skip:           RETURN  
    
    
         
AvoidCol:   
    
    LOAD    STR_AAA0    ;Revision: New2
    STORE   STR_DEBUG   ;Revision: New2

    LOAD   DVel
    STORE  Temp
    LOADI  0
    STORE  DVel ; stop the robot
    CALL   JSIntruderDetected
NotClear:
    CALL   Wait1
    IN SONALARM ; avoid object
    JZero Clear
    JUMP NotClear
Clear:      
    LOAD Temp
    STORE DVel
    RETURN
            
            
; ------------------------------------------------------------- END JELLY SPOTTERS AUXILIARY CODE -----------------------------------------

; ------------------------------------------------------------- BEGIN JELLY SPOTTERS VARIABLE DECLARATIONS -----------------------------------------        

Temp2:      DW 0
Temp3:      DW 0
Temp4:      DW 0

ZeroOne:    DW 500
OneTwo:     DW 1254
TwoThree:   DW 2115
SixSeven:   DW 2090

TOP_ARC_WIDTH:  DW 3500

YDelta:     DW 0



Dist4A:     DW 1547
Dist4B:     DW 961
Dist9A:     DW 0
Dist9B:     DW 0

BAD_DIST_TOP:   DW &H0800

DRT0:       DW  0
DRT1:       DW  0
DRT2:       DW  0
DRT3:       DW  0
DRB0:       DW  0
DRB1:       DW  0
DRB2:       DW  0
DRB3:       DW  0

PrevAvgTop:     DW  0
CurAvgTop:      DW  0
PrevAvgBottom:  DW  0
CurAvgBottom:   DW  0

MAX_SENSOR_NOISE:       DW  &H005C  ; Smallest known dimension of DE2 robot is &H0113. This is roughly 1/3rd of that
KNOWN_BAFFLELESS_WIDTH: DW  &H0B3A  ; Known width of field where there is no baffle

ArcTransState:  DW  &H0000  ; 0 means before baffle, 1 means over baffle, 2 means post-baffle

INIT_SamplesToWaitTop:      DW  4 ; Initialize to number of top samples.
INIT_SamplesToWaitBottom:   DW  4 ; Initialize to number of bottom samples.
SamplesToWaitTop:       DW  0 ; Number of samples to be skipping analysis, for fresh data purposes.
SamplesToWaitBottom:    DW  0 ; Number of samples to be skipping analysis, for fresh data purposes.

STR_DEBUG:  DW  &HFFFF
STR_ABC0:   DW  &HABC0
STR_ABC1:   DW  &HABC1
STR_DAB0:   DW  &HDAB0
STR_BAD1:   DW  &HBAD1
STR_BAD2:   DW  &HBAD2
STR_AAA0:   DW  &HAAA0




OA2Prev:    DW 0;
OA2:        DW 0;
DP1:        DW 0;
DP2:        DW 0;
CurErr:     DW 0;
MaxGoodSonarReading:    DW 16250 ; Half the max good sonar reading of 32500


UBTop:      DW  0
UBBottom:   DW  0

UNDER_BAFFLE_WIDTH:     DW  &H039D

; ------------------------------------------------------------- END JELLY SPOTTERS VARIABLE DECLARATIONS -----------------------------------------