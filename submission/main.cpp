#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"
//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key = (uint64_t*)((int)sequence + 48);
uint64_t* nonce = (uint64_t*)((int)sequence + 56);
uint8_t hash[32];


//message meanings
#define found_nonce         1 
#define show_nonce          2
#define show_key            3
#define required_positions  4
#define show_speed          5
#define required_positions  6
#define motor_position      7
#define er                  8
#define or_state            9
#define motor_power          10
#define new_key             11
#define show_actspeed       55


//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//unsigned long int count = 0;
uint32_t motorPower = 500;
uint32_t speed = 240;

int32_t requiredPositions = 1;


//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);
    

//Set a given drive state
void motorOut(int8_t driveState){
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(motorPower);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(motorPower);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(motorPower);
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorPower = 1000;
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}
int8_t orState = 0;

RawSerial pc(SERIAL_TX, SERIAL_RX);  


int32_t motorPosition;
void motorISR() {
    
    static int8_t oldRotorState;
    int8_t rotorState = readRotorState();
    motorOut((rotorState-orState+lead+6)%6);
    if (rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++;
    else motorPosition += (rotorState - oldRotorState);
    oldRotorState = rotorState;
} 

typedef struct{
    uint8_t code;
    uint32_t data;
    } message_t ;
    
Mail<message_t,16> outMessages;
Queue<void, 8> inCharQ;
    
//function to add messages to queue
void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
}     

//create Thread
Thread commOutT (osPriorityNormal,1024);
Thread commIn (osPriorityNormal,1024);   
Thread motorCtrlT (osPriorityNormal,1024);

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}

int32_t lastPosition;
uint8_t motorCount;
Timer timer;
void motorCtrlFn(){
    int32_t ys;
    int32_t yr;
    int32_t y;
    int32_t Er = 0;
    int32_t lastEr = 0;
    uint8_t kp1=20;
    uint8_t kp2=15;
    uint8_t kd=47;
    
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    timer.start();
    while(1){
        motorCtrlT.signal_wait(0x1);
        timer.stop();
        int32_t timeDiff = timer.read_us();
        int32_t actSpeed = motorPosition - lastPosition;
        lastPosition = motorPosition;
        timer.reset();
        timer.start(); 
        actSpeed = actSpeed * (1000000/timeDiff);
        //actSpeed = actSpeed/6;
        motorCount++;
        if(motorCount == 5) {
            //putMessage(show_actspeed, actSpeed);
            //putMessage(show_speed, speed);
            motorCount = 0;
        }
        
        if(requiredPositions != 0) Er = requiredPositions - motorPosition;
        else Er=1;
        
        if(speed==0) ys=1000;
        else ys = kp1*(speed - abs(actSpeed));
        
        if(Er<0) ys = -ys;
        
   //     pc.printf("%d\n\r", requiredPositions);  
        putMessage(4, requiredPositions);// change to COMMOUT
   //     pc.printf("%d\n\r", motorPosition); // change to COMMOUT
        putMessage(7, motorPosition);
    //    pc.printf("%d\n\r\n\r\n\r", Er);    // change to COMMOUT
        putMessage(8, Er);
        
        yr = kp2*Er + kd*(Er-lastEr);   
    
        if(requiredPositions == 0) y=ys;
        else if(abs(yr)<abs(ys)) y = yr;
        else y = ys;
        
        if(y<0) {
            lead = -2;
            y=-y;
        }
        else lead = +2;
        
        if(y>1000) y=1000;
        
        motorPower = y;
        lastEr = Er;
        if(actSpeed==0) motorISR();
    }
}

volatile uint64_t newKey;
Mutex newKey_mutex;

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}

//take messages from the queue and print them on the serial port
void commInFn(){
    char command[20];   //CHECK THIS VALUE
    
    int index = 0;
    pc.attach(&serialISR);
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        
        if(index<20) {
            if(newChar == '\r') {
                command[index] = '\0';
                index = 0;
                if(command[0] == 'K') {
                    newKey_mutex.lock();
                    sscanf(command, "K%x", &newKey); //Decode the command
               //     pc.printf("%016X\n\r", newKey);
                    putMessage(11, newKey);
                    newKey_mutex.unlock();
                }
                else if(command[0] == 'T') {
                    //tmp
                    sscanf(command, "T%d", &motorPower); //Decode the command
                }
                else if(command[0] == 'V') {
                    //tmp
                    sscanf(command, "V%d", &speed); //Decode the command
                    speed = speed*6;
                }
                else if(command[0] == 'R') {
                    //tmp
                    sscanf(command, "R%d", &requiredPositions); //Decode the command
                    if(requiredPositions!=0) requiredPositions = motorPosition + (requiredPositions*6);
                    
                    motorISR();
                }
            }
            else {
                command[index] = newChar;
                index ++;
            }
        }
    }
}

//take messages from the queue and print them on the serial port
void commOutFn(){
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        if(pMessage->code == found_nonce) {
            pc.printf("FOUND ONE\n\r");
        }
        else if(pMessage->code == show_speed) {
            pc.printf("Desired speed: %d\n\r", pMessage->data / 6);
        }
        else if(pMessage->code == show_actspeed) {
            pc.printf("Actual speed: %d\n\r", pMessage->data / 6);
        }
        else if(pMessage->code == show_nonce){
            pc.printf("Message %d with data 0x%016x\n\r",
                pMessage->code,pMessage->data);
        }
        else {
            pc.printf("Actual speed: %d\n\r", pMessage->data / 6);
        }
        outMessages.free(pMessage);
    }
}        

Ticker interval;
void print_rate() {
    //pc.printf("%i\n\r", count);
    //count = 0;      
//    pc.printf("%d\n\r", motorPower); // change to COMMOUT        
    putMessage(11, motorPower);
}
    
//Main
int main() {
    int us = 2000;
    L1L.period_us(us);
    L2L.period_us(us);
    L3L.period_us(us);
    
    orState = motorHome();
    
    I1.rise(&motorISR);
    I2.rise(&motorISR);
    I3.rise(&motorISR);
    I1.fall(&motorISR);
    I2.fall(&motorISR);
    I3.fall(&motorISR);
    //interval.attach(&print_rate, 2);
    *nonce = 0;
    *key = 0;
    
    commOutT.start(commOutFn);
    commIn.start(commInFn);
    motorCtrlT.start(motorCtrlFn);
   
    //Run the motor synchronisation
    putMessage(9, orState);
    motorISR();
    unsigned long int i = 1;
    while(1){
        //putMessage(2, *key);
        *key = newKey;
        SHA256::computeHash(hash, (uint8_t*)sequence, 64);
        if(hash[0]==0 && hash[1]==0){
            putMessage(found_nonce, 0);
            putMessage(show_nonce, *nonce);
            putMessage(show_key, *key);
        }
        *nonce = i;
        i++;
        //count++;
    }
}
