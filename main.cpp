#include "mbed.h"
#include "SHA256.h"
#include "Crypto.h"
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
const int8_t lead = 2;  //2 for forwards, -2 for backwards

unsigned long int count = 0;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);
    
//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}
const int8_t orState = motorHome();
const int8_t foundNonce=111;

//new func
void ISR(){
    motorOut((readRotorState()-orState+lead+6)%6);
}

RawSerial pc(SERIAL_TX, SERIAL_RX);   

typedef struct{
    uint8_t code;
    uint32_t data;
    } message_t ;
    
Mail<message_t,16> outMessages;
Queue<void, 8> inCharQ;
    
//create Thread
Thread commOutT;
Thread commIn;   
volatile uint64_t newKey;
Mutex newKey_mutex;

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}

//take messages from the queue and print them on the serial port
void commInFn(){
    char command[20];
    //char *pcommand = command;
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
                    pc.printf("%016X\n\r", newKey);
                    //*key = newKey;
                    newKey_mutex.unlock();
                }
                /*else {
                    pc.printf(command);
                }*/
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
        if(pMessage->code == 1) {
            pc.printf("FOUND ONE\n\r");
        }
        else {
            pc.printf("Message %d with data 0x%016x\n\r",
                pMessage->code,pMessage->data);
        }
        outMessages.free(pMessage);
    }
}       
  
    
//function to add messages to queue
void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
}    

Ticker interval;

void print_rate() {
    pc.printf("%i\n\r", count);
    count = 0;      
}
    
//Main
int main() {
    I1.rise(&ISR);
    I2.rise(&ISR);
    I3.rise(&ISR);
    I1.fall(&ISR);
    I2.fall(&ISR);
    I3.fall(&ISR);
    //interval.attach(&print_rate, 2);
    
    *nonce = 0;
    *key = 0;
    
    commOutT.start(commOutFn);
    commIn.start(commInFn);
    
    //Run the motor synchronisation
    pc.printf("Rotor origin: %x\n\r",orState);
    ISR();
    unsigned long int i = 1;
    while(1){
        //putMessage(2, *key);
        *key = newKey;
        SHA256::computeHash(hash, (uint8_t*)sequence, 64);
        if(hash[0]==0 && hash[1]==0){
            putMessage(1, 0);
            putMessage(foundNonce, *nonce);
            putMessage(foundNonce, *key);
        }
        *nonce = i;
        i++;
        count++;
    }
}
