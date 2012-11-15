/*  Nikolay Vladimirov - Main Robockey Code for Robot 1
    Mechatronics 410/510
    Update: Dec 1
    PINS USED:

*/
// INCLUDES
#include <avr/io.h>
#include <math.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_rf.h"
#include "m_bus.h"
#include "m_wii.h"
// PACKETS
#define CHANNEL 1
#define RXADDRESS 0x08
#define TXADDRESS 0x09
#define PACKET_LENGTH 12
// INDICES FOR STARS
#define X 0
#define Y 1
// INTERSTAR DISTANCES IN PX
#define MID_DIST 97 // 29 cm
#define LEFT_TO_BOT 67 // 20 cm
#define RIGHT_TO_BOT 87 // 26 cm
#define LEFT_TO_TOP 54 // 16 cm
#define RIGHT_TO_TOP 44 // 13.00 cm
#define LEFT_TO_RIGHT 77 // 23.08 cm
#define LBOUND 3 // margin of 2
#define UBOUND 3 // margin of 5
#define EPSILON 0.2816 // angle(rads) from left to right star
#define ZETA 0.3624 // angle(rads) from lower left corner to botMidStar
// MOVING AROUND THRESHOLDS
#define ANG_MOE 0.3 // moe for rotating angle
#define POS_MOE (unsigned int)20 // moe for moving to pos
#define WALL_UPPER 110
#define WALL_LOWER 10
// ABSOLUTE RINK DISTANCE
#define RINK_TO_LEFT_STAR 109  // 109 cm abs x to left star
#define RINK_HALF_HEIGHT 60  // 60cm
#define BOT_MID_Y 45.5
#define BOT_MID_X 120.0
// RELATIVE POSITION OF ROBOT TO VIEW IS CONSTANT
#define X_REL 512
#define Y_REL 384
// CONVERSION FACTORS
#define PX_TO_CM_DIV 3.35      // divide by this
#define RADS_TO_DEG_FAC 57.0   // multiply by this
// PI
#define PI 3.142
// STAR ARRAYS
unsigned int blobs[12];
unsigned int stars[4][2];
unsigned int topMidStar[2] = {0,0};
unsigned int botMidStar[2] = {0,0};
unsigned int leftStar[2] = {0,0};
unsigned int rightStar[2] = {0,0};
unsigned int myPos[2] = {0,0};
unsigned int myLastPos[2] = {0,0};
unsigned int strikerPos[2];
double desiredX;
double desiredY;
double angleNow;
bool direction;
// BUFFERS AND SHIT
char buffer[PACKET_LENGTH] = {0,0,0,0,0,0,0,0,0,0,0,0};
bool directionPacket;
volatile unsigned char packet_received;
// FUNCTION INITS
// gameplay
void init(void);
void playStriker(void);
void playHelper(void);
void playGoalie(void);
void detangle(void);
void pause(void);
void gameOver(void);
bool facing(int x, int y);
bool at(int x, int y);
// movement
void forward(void);
void backward(void);
void turnRight(void);
void turnLeft(void);
void spinRight(void);
void spinLeft(void);
void tryLeft(void);
void tryRight(void);
void stop(void);
void fire(void);
// position
unsigned int getDist(unsigned int a[], unsigned int b[]);
void getCurrentAngle(int x,int y);
double getDesiredAngle(int x,int y);
void getPos(void);
bool hasPuck(void);
bool hasSwerved = false;
bool posWithin(unsigned int x, unsigned int y);
bool stuckChecker(unsigned int x, unsigned int y);
void debug1(int i, int j, unsigned int distHoriz);
void debug2(int j, int k, unsigned int distDiag);
void debug3(int top, int bot, int lt, int rt);
// timer and adc
void timer1_init(void);
void timer3_init(void);
void ADC_init(void);
int ADC0_read(void);
int ADC1_read(void);
int ADC4_read(void);
int ADC5_read(void);
int ADC6_read(void);
long timer=0;
int flag =0; 

// FUNCTIONS
#include "playFxns.c"

int main(void){
    init();
    while(1){
    switch(packet_received){
        case 0xA0:
            if(packet_received==0xA0){
            stop();
            m_red(TOGGLE);
            packet_received = 0;
            //m_usb_tx_string("COM Test");
            //m_usb_tx_string("\n");

            }
            break;
        case 0xA1:
           while(packet_received==0xA1){
            if(RXADDRESS == 0x08) playStriker();
            else if(RXADDRESS == 0x09) playHelper();
            else if(RXADDRESS == 0x0A) playGoalie();
			   if(flag ==1){ 
				   flag =0;
				   break;
			   }
			   

            //m_usb_tx_string("Play");
            //m_usb_tx_string("\n");
            }
            break;
        case 0xA2:
            while(packet_received==0xA2){
            stop();
            //m_usb_tx_string("Goal A");
            //m_usb_tx_string("\n");
            }
            break;
        case 0xA3:
            while(packet_received==0xA3){
            stop();
            //m_usb_tx_string("Goal B");
            //m_usb_tx_string("\n");
            }
            break;
        case 0xA4:
            while(packet_received==0xA4){
            stop();
            //m_usb_tx_string("Pause");
            //m_usb_tx_string("\n");
            }
            break;
        case 0xA5:
            while(packet_received==0xA5){
            backward();
            m_wait(500);
            stop();
            //m_usb_tx_string("Detangle");
            //m_usb_tx_string("\n");
            }
            break;
        case 0xA6:
            while(packet_received==0xA6){
            stop();
            //m_usb_tx_string("Halftime");
            //m_usb_tx_string("\n");
            }
            break;
        case 0xA7:
            while(packet_received==0xA7){
            stop();
            //m_usb_tx_string("Game Over");
            //m_usb_tx_string("\n");
            }
            break;
        default:
			stop();
			            // if(RXADDRESS == 0x08) playStriker();
            // else if(RXADDRESS == 0x09) playHelper();
            // else if(RXADDRESS == 0x0A) playGoalie();
            // m_usb_tx_string("Going to center");
            // m_usb_tx_string("\n");
            break;
        }
    }
}

// INTERRUPT VECTOR FOR PACKETS FROM FIENESTER
ISR(INT2_vect){
    // m_usb_tx_string("INTERRUPT TRIGGERED\n");
    m_rf_read(buffer,PACKET_LENGTH);
    packet_received = (unsigned char)buffer[0];
    strikerPos[X] = (unsigned int)buffer[1];
    strikerPos[Y] = (unsigned int)buffer[2];
    directionPacket = (bool)buffer[3];
	flag=1; 
    // m_usb_tx_string("striker pos x,y,dir = (");
    // m_usb_tx_int(strikerPos[X]);
    // m_usb_tx_string(", ");
    // m_usb_tx_int(strikerPos[Y]);
    // m_usb_tx_string(", ");
    // m_usb_tx_int(directionPacket);
    // m_usb_tx_string(")\n");

}
ISR(TIMER3_OVF_vect){
    if(stuckChecker(myLastPos[X],myLastPos[Y])){
		timer++;
		}
   
	
	//if((buffer[1] == 0xA1) && (stuckChecker(myLastPos[X],myLastPos[Y])){ //clears timer everytime play is called
		else {
			timer = 0;
   }
    // Save position now as last pos for the next second
	
    myLastPos[X]=myPos[X];
	myLastPos[Y]=myPos[Y];
	
}
    // USB Debug
    // m_usb_tx_string("TIMER INTERRUPT TRIGGERED:");
    // m_usb_tx_int(timer);
    // m_usb_tx_string("\n");

void forward(){
    OCR1B = 250;            //100% duty cycle
    OCR1C = 255;
    clear(PORTB, 4);
    clear(PORTC, 6);
}
void backward(){
    OCR1B = 255;            //Both motors back up
    OCR1C = 255;
    set(PORTB, 4);
    set(PORTC, 6);
}
void turnRight(){
    OCR1B = 140;            //50% duty cycle
    OCR1C = 80;
    clear(PORTB, 4);        //otherwise, spin wheels at varying speeds
    clear(PORTC, 6);
}
void turnLeft(){
    OCR1B = 90;             //50% duty cycle
    OCR1C = 140;
    clear(PORTB, 4);        //otherwise, spin wheels at varying speeds
    clear(PORTC, 6);
}
void spinRight(){
    OCR1B = 128;            //50% duty cycle
    OCR1C = 128;
    clear(PORTB, 4);        //otherwise, spin wheels at varying speeds
    set(PORTC, 6);
}
void spinLeft(){
    OCR1B = 128;            //50% duty cycle
    OCR1C = 128;
    set(PORTB, 4);          //otherwise, spin wheels at varying speeds
    clear(PORTC, 6);
}
void tryRight(){
    if(myPos[Y]>=WALL_UPPER || myPos[Y]<=WALL_LOWER)
		turnLeft();
    else turnRight();
}
void tryLeft(){
    if(myPos[Y]>=WALL_UPPER || myPos[Y]<=WALL_LOWER)
		turnRight();
	else turnLeft();
}
void stop(){
    OCR1B = 0;              //Both motors stop
    OCR1C = 0;
}
void fire(){
    set(PORTF,4);
    m_wait(85);
    clear(PORTF,4);
}
bool facing(int x, int y){
    getPos();
    getCurrentAngle(x,y);
    double angleDesired = getDesiredAngle(x,y);
    m_usb_tx_string("Angle Now->");
    m_usb_tx_int(angleNow*RADS_TO_DEG_FAC);
    m_usb_tx_string("Angle Desired->");
    m_usb_tx_int(angleDesired*RADS_TO_DEG_FAC);
    // MOVE TOWARD DESIRED ANGLE
    if((angleDesired>=0 && angleNow>=0)||(angleDesired<0 && angleNow<0)){ // either both + or both - angles
        if(angleNow>=angleDesired-ANG_MOE && angleNow<=angleDesired+ANG_MOE) return true; // facing ang w/in moe
        if(angleDesired>angleNow) tryLeft();
        else tryRight();
        return false;
    } else {                                    // one + one -
        // GET ABSOLUTE ANGLE
        double absoluteAngle;
        if(angleNow>=0) absoluteAngle = angleNow;
        else absoluteAngle = 2*PI + angleNow;
        // SEE IF W/IN MOE
        if(angleDesired>=0&&angleNow<0){        // desired is +, current is -
            if(angleDesired<ANG_MOE && angleNow>(angleDesired-ANG_MOE)) return true;
            if((angleDesired-angleNow)<=PI) tryLeft();
            else tryRight();
            return false;
        } else {                                // desired is -, current is +
            if(absoluteAngle>=angleDesired-ANG_MOE && absoluteAngle<=angleDesired+ANG_MOE) return true;
            if(angleNow-angleDesired<=PI) tryRight();
            else tryLeft();
            return false;
        }
    }
}
bool at(int x, int y){
    getPos();
    if(posWithin(x,y)){
        stop();
        return true;
    }
    if(!facing(x,y)){
        return false;
    }
    forward();
    return false;
}
bool posWithin(unsigned int x, unsigned int y){
    if(myPos[X]>=x-POS_MOE&&myPos[X]<=x+POS_MOE&&myPos[Y]>=y-POS_MOE&&myPos[Y]<=y+POS_MOE) return true;
    else return false;
}
bool stuckChecker(unsigned int x, unsigned int y){
    unsigned int moe = 5;
    if(myPos[X]>=x-moe && myPos[X]<=x+moe && myPos[Y]>=y-moe && myPos[Y]<=y+moe) return true;
    else return false;
}
bool hasPuck(){ //FIX THIS
    int adc5 = ADC5_read();
    // m_usb_tx_string("ADC5 says:");
    // m_usb_tx_int(adc5);
    if(adc5>10){
        m_red(ON);
        return true;
    }
    else{
        m_red(OFF);
        hasSwerved = false;
        return false;
    }
}
// RETURN DISTANCE BETWEEN TO STARS
unsigned int getDist(unsigned int a[], unsigned int b[]){
    unsigned int dx = abs(a[X]-b[X]);
    unsigned int dy = abs(a[Y]-b[Y]);
    return sqrt(pow(dx,2)+pow(dy,2));
}
// RETURN DIRECTIONAL ANGLE IN RADS (0,PI/2)
void getCurrentAngle(int x, int y){
    double dx = (double)topMidStar[X]-(double)botMidStar[X];
    double dy = (double)botMidStar[Y]-(double)topMidStar[Y];
    angleNow = atan2(dx,dy);
}
double getDesiredAngle(int x, int y){
    double dy = (double)y-(double)myPos[Y];
    double dx = (double)x-(double)myPos[X];
    return atan2(dy,dx);
}
// FUNCTION THAT DETERMINES ROBOT'S POSITION IN ABSOLUTE CENTIMETERS
void getPos(){
    m_wii_read(blobs);
    // FILL STARS WITH BLOB VALUES
    int i, j, k, l;
    bool isDone = false;
    bool foundFour = false;
    for(i=0; i<4; i++){
        stars[i][X] = blobs[3*i];    //get xpos
        stars[i][Y] = blobs[3*i+1];  //get ypos
    }
    // FIND AN ARBITRARY NON-EMPTY STAR
    for(i=0; i<4; i++){                                                             // find a visible star
        if(stars[i][X]==1023 && stars[i][Y]==1023){
            continue;                                                               // continue if the bin sees nothing
        }
        //m_usb_tx_string("loop i \n");
        for(j=0; j<4; j++){                                                         // comp to another visible star w/ dist 25cm away
            if(stars[j][X]==1023 && stars[j][Y]==1023) continue;                    // continue if bin sees nothing
            if(j==i) continue;                                                      // continue if same bins
            unsigned int distHoriz = getDist(stars[i],stars[j]);                    // get the distance between the two
          //  debug1(i,j,distHoriz);
            if(distHoriz>=MID_DIST-LBOUND && distHoriz<=MID_DIST+UBOUND){           // if distance w/in 25cm, mids found
                //m_usb_tx_string("loop j \n");
                isDone=true;
                for(k=0; k<4; k++){                                                 // find a third point
                    if(stars[k][X]==1023 && stars[k][Y]==1023) continue;            // continue if the bin sees nothing
                    if(k==i||k==j) continue;                                        // continue if same bins
                    unsigned int distDiag = getDist(stars[j],stars[k]);             // get dist bet a midstar and l/r star
                  //  debug2(j,k,distDiag);
                    if(distDiag>=LEFT_TO_BOT-LBOUND && distDiag<=LEFT_TO_BOT+UBOUND){     // first mid star is top, last found is left
                       // m_usb_tx_string("loop k \n");
                        for(l=0; l<4; l++){
                            if(stars[l][X]==1023 && stars[l][Y]==1023) continue;    // continue if the bin sees nothing
                            if(l==i||l==j||l==k) continue;                          // continue if same bins
                            //m_usb_tx_string("loop l \n");
                            topMidStar[X]=stars[i][X]; topMidStar[Y]=stars[i][Y];
                            botMidStar[X]=stars[j][X]; botMidStar[Y]=stars[j][Y];
                            leftStar[X]=stars[k][X]; leftStar[Y]=stars[k][Y];
                            rightStar[X]=stars[l][X]; rightStar[Y]=stars[l][Y];
                            //debug3(i,j,k,l);
                            foundFour = true;
                            break;
                        }
                        break;
                    }
                    else if(distDiag>=RIGHT_TO_BOT-LBOUND && distDiag<=RIGHT_TO_BOT+UBOUND){  // first mid star is top, last found is right
                        //m_usb_tx_string("loop k \n");
                        for(l=0; l<4; l++){
                            if(stars[l][X]==1023 && stars[l][Y]==1023) continue;    // continue if the bin sees nothing
                            if(l==i||l==j||l==k) continue;                          // continue if same bins
                            //m_usb_tx_string("loop l \n");
                            topMidStar[X]=stars[i][X]; topMidStar[Y]=stars[i][Y];
                            botMidStar[X]=stars[j][X]; botMidStar[Y]=stars[j][Y];
                            rightStar[X]=stars[k][X]; rightStar[Y]=stars[k][Y];
                            leftStar[X]=stars[l][X]; leftStar[Y]=stars[l][Y];
                         //   debug3(i,j,l,k);
                            foundFour = true;
                            break;
                        }
                        break;
                    }
                    else if(distDiag>=LEFT_TO_TOP-LBOUND && distDiag<=LEFT_TO_TOP+UBOUND){    // first mid star is bot, last found is left
                        //m_usb_tx_string("loop k \n");
                        for(l=0; l<4; l++){
                            if(stars[l][X]==1023 && stars[l][Y]==1023) continue;    // continue if the bin sees nothing
                            if(l==i||l==j||l==k) continue;                          // continue if same bins
                            //m_usb_tx_string("loop l \n");
                            botMidStar[X]=stars[i][X]; botMidStar[Y]=stars[i][Y];
                            topMidStar[X]=stars[j][X]; topMidStar[Y]=stars[j][Y];
                            leftStar[X]=stars[k][X]; leftStar[Y]=stars[k][Y];
                            rightStar[X]=stars[l][X]; rightStar[Y]=stars[l][Y];
                           // debug3(j,i,k,l);
                            foundFour = true;
                            break;
                        }
                        break;
                    }
                    else if(distDiag>=RIGHT_TO_TOP-LBOUND && distDiag<=RIGHT_TO_TOP+UBOUND){  // first mid star is bot, last found is righ
                        //m_usb_tx_string("loop k \n");
                        for(l=0; l<4; l++){
                            if(stars[l][X]==1023 && stars[l][Y]==1023) continue;    // continue if the bin sees nothing
                            if(l==i||l==j||l==k) continue;                          // continue if same bins
                            //m_usb_tx_string("loop l \n");
                            botMidStar[X]=stars[i][X]; botMidStar[Y]=stars[i][Y];
                            topMidStar[X]=stars[j][X]; topMidStar[Y]=stars[j][Y];
                            rightStar[X]=stars[k][X]; rightStar[Y]=stars[k][Y];
                            leftStar[X]=stars[l][X]; leftStar[Y]=stars[l][Y];
                           // debug3(j,i,l,k);
                            foundFour = true;
                            break;
                        }
                        break;
                    }
                    else break; //if neither leftStar or rightStar matched some MidStar, something's wrong
                }
                break;  // if match for mids was found, stop comparing
            }
        }
        if(isDone)break; // if match for mids was found, break out of main loop
    }
    if(!isDone || !foundFour) return;
    // FILL MYPOS WITH ABSOLUTE COORDINATES IN CM
    myPos[X] = X_REL;
    myPos[Y] = Y_REL;
    // GET ROBOT'S DISTANCE TO EACH STAR IN PX
    unsigned int distTopMidStar = getDist(myPos,topMidStar);
    unsigned int distBotMidStar = getDist(myPos,botMidStar);
    unsigned int distLeftStar = getDist(myPos,leftStar);
    unsigned int distRightStar = getDist(myPos,rightStar);
    // GET ANGLES TO MIDSTARS IN RADIANS - ALPHA, BETA
    double distTopMidStarSq = pow((double)distTopMidStar,2.0);
    double distBotMidStarSq = pow((double)distBotMidStar,2.0);
    double midDistSq = pow((double)MID_DIST,2.0);
    double aDivisor = 2.0 * (double)MID_DIST * (double)distTopMidStar;
    double bDivisor = 2.0 * (double)MID_DIST * (double)distBotMidStar;
    double a = (distTopMidStarSq + midDistSq - distBotMidStarSq)/aDivisor;
    double b = (distBotMidStarSq + midDistSq - distTopMidStarSq)/bDivisor;
    double alpha = acos(a);
    double beta = acos(b);
    // FIND VERTICAL DISTANCE FROM HORIZONTAL MIDLINE IN PX
    double dyTop = cos(alpha)*(double)distTopMidStar-(double)MID_DIST/2.0;
    double dyBot = -1.0*cos(beta)*(double)distBotMidStar-(double)MID_DIST/2.0;
    double dyAve = (dyTop + dyBot)/2.0/PX_TO_CM_DIV; //dy from horizontal midline in cm
    // FIND ABSOLUTE DISTANCE FROM LEFTSTAR IN PX
    double gDivisor = 2.0 * (double)LEFT_TO_RIGHT * (double)distLeftStar;
    double g = (pow((double)LEFT_TO_RIGHT,2.0)+pow((double)distLeftStar,2.0)-pow((double)distRightStar,2.0))/gDivisor;
    double gamma = acos(g);
    double theta = gamma - EPSILON - PI/2.0;
    double dx = sin(theta)*(double)distLeftStar/PX_TO_CM_DIV; //dx from left star in cm
    // GET ABSOLUTE POSITION IN RINK IN CM, LOWER LEFT IS (0,0)
    myPos[X] = (double)RINK_TO_LEFT_STAR - dx;
    myPos[Y] = RINK_HALF_HEIGHT - dyAve;
    // DEBUG
   // m_usb_tx_string("myPos[X]->");
    //m_usb_tx_int((int)myPos[X]);
    //m_usb_tx_string("myPos[Y]->");
    //m_usb_tx_int((int)myPos[Y]);
    //m_usb_tx_string("\n");
}
// INITIALIZAION FUNCTION
void init(void){
    m_clockdivide(0);
    m_rf_open(CHANNEL, RXADDRESS, PACKET_LENGTH);
    m_usb_init();
    m_bus_init();
    while(m_wii_open()==0){}

    timer1_init();
    timer3_init();
    ADC_init();
    sei();                  // ENABLE GLOBAL INTERRUPTS

    // MOTOR DIRECTIONS
    set(DDRC,6);            //PIN C6 AS DIR PIN 1
    set(DDRB,4);            //PIN B4 AS DIR PIN 2
    // SOLENOID
    set(DDRF,4);            //PIN F4 AS SOLENOID OUTPUT
    // PUCK DETECT
    // clear(DDRF,5);          //PIN F5 AS PUCK-DETECT INPUT
    // clear(PORTF,5);         //DISABLE F5 PULL-UP RESISTOR
    // GOAL DIRECTION SWITCH
    clear(DDRB,5);          //PIN B5 AS DIRECTION SWITCH
    clear(PORTB,5);         //DISABLE B5 PULL-UP RESISTOR
}
//TIMER INITS
 void timer1_init(){
    clear(TCCR1B, CS12);    // Timer 1 divide by 64
    set(TCCR1B, CS11);      // ^
    set(TCCR1B, CS10);      // ^

    set(TCCR1B,WGM13);      //MODE 15: UP to OCR1A, PWM Mode
    set(TCCR1B,WGM12);      //^
    set(TCCR1A,WGM11);      //^
    set(TCCR1A,WGM10);      //^
    OCR1A = 255;            //^

    set(TCCR1A, COM1B1);    // Clear at OCR1B, set at rollover
    clear(TCCR1A, COM1B0);  // ^
    set(DDRB, 6);           //enable output to B6

    set(TCCR1A, COM1C1);    // Clear at OCR1C, set at rollover
    clear(TCCR1A, COM1C0);  // ^
    set(DDRB, 7);           //enable output to B7

    OCR1B = 0;              //one wheel at 0% duty cycle at init
    OCR1C = 0;              //another wheel at 0% duty cycle at init
 }
 void timer3_init(){
    clear(TCCR3B, CS32);    // Timer 3 divide by 64
    set(TCCR3B, CS31);      // ^
    set(TCCR3B, CS30);      // ^

    clear(TCCR3B,WGM33);    //MODE 0: UP to OxFFFF
    clear(TCCR3B,WGM32);    //^
    clear(TCCR3A,WGM31);    //^
    clear(TCCR3A,WGM30);    //^

    set(TIMSK3,TOIE3);      // Allow interrupt
}
//ADC INIT
void ADC_init(void){
    clear(ADMUX,REFS1);     //set voltage Vcc(5.0V)
    set(ADMUX,REFS0);       //^

    set(ADCSRA,ADPS2);      //set ADC prescaler to /128 -> 125KHz ADC clock
    set(ADCSRA,ADPS1);      //^
    set(ADCSRA,ADPS0);      //^

    m_disableJTAG();        //DISABLE JTAG COZ F PINS ARE BITCHES
    set(DIDR0,ADC0D);       //disable digital input for pin F0
    set(DIDR0,ADC1D);       //disable digital input for pin F1
    set(DIDR0,ADC4D);       //disable digital input for pin F4
    set(DIDR0,ADC5D);       //disable digital input for pin F5
    set(DIDR0,ADC6D);       //disable digital input for pin F6
}
int ADC0_read(void){
    clear(ADCSRA,ADEN);     //disable ADC subsystem
    clear(ADCSRB,MUX5);     //connect single-ended channel F0
    clear(ADMUX,MUX2);      //^
    clear(ADMUX,MUX1);      //^
    clear(ADMUX,MUX0);      //^
    set(ADCSRA,ADEN);       //enable ADC subsystem**/

    set(ADCSRA,ADSC);              //begin conversion
    while(!check(ADCSRA,ADIF)){}   //wait for flag to be set
    set(ADCSRA,ADIF);              //reset the flag

    return ADC;
}
int ADC1_read(void){
    clear(ADCSRA,ADEN);     //disable ADC subsystem
    clear(ADCSRB,MUX5);     //connect single-ended channel F1
    clear(ADMUX,MUX2);      //^
    clear(ADMUX,MUX1);      //^
    set(ADMUX,MUX0);        //^
    set(ADCSRA,ADEN);       //enable ADC subsystem**/

    set(ADCSRA,ADSC);              //begin conversion
    while(!check(ADCSRA,ADIF)){}   //wait for flag to be set
    set(ADCSRA,ADIF);              //reset the flag

    return ADC;
}
int ADC4_read(void){
    clear(ADCSRA,ADEN);     //disable ADC subsystem
    clear(ADCSRB,MUX5);     //connect single-ended channel F4
    set(ADMUX,MUX2);        //^
    clear(ADMUX,MUX1);      //^
    clear(ADMUX,MUX0);      //^
    set(ADCSRA,ADEN);       //enable ADC subsystem**/

    set(ADCSRA,ADSC);              //begin conversion
    while(!check(ADCSRA,ADIF)){}   //wait for flag to be set
    set(ADCSRA,ADIF);              //reset the flag

    return ADC;
}
int ADC5_read(void){
    clear(ADCSRA,ADEN);     //disable ADC subsystem
    clear(ADCSRB,MUX5);     //connect single-ended channel F5
    set(ADMUX,MUX2);        //^
    clear(ADMUX,MUX1);      //^
    set(ADMUX,MUX0);        //^
    set(ADCSRA,ADEN);       //enable ADC subsystem**/

    set(ADCSRA,ADSC);              //begin conversion
    while(!check(ADCSRA,ADIF)){}   //wait for flag to be set
    set(ADCSRA,ADIF);              //reset the flag

    return ADC;
}
int ADC6_read(void){
    clear(ADCSRA,ADEN);     //disable ADC subsystem
    clear(ADCSRB,MUX5);     //connect single-ended channel F6
    set(ADMUX,MUX2);        //^
    set(ADMUX,MUX1);        //^
    clear(ADMUX,MUX0);      //^
    set(ADCSRA,ADEN);       //enable ADC subsystem**/

    set(ADCSRA,ADSC);              //begin conversion
    while(!check(ADCSRA,ADIF)){}   //wait for flag to be set
    set(ADCSRA,ADIF);              //reset the flag

    return ADC;
}
// DEBUG FUNCTIONS
void debug1(int i, int j, unsigned int distHoriz){
    m_usb_tx_string("distHoriz bet Star");
    m_usb_tx_int(i+1);
    m_usb_tx_string("(");
    m_usb_tx_uint(stars[i][X]);
    m_usb_tx_string(",");
    m_usb_tx_uint(stars[i][Y]);
    m_usb_tx_string(")");
    m_usb_tx_string(" and Star");
    m_usb_tx_int(j+1);
    m_usb_tx_string("(");
    m_usb_tx_uint(stars[j][X]);
    m_usb_tx_string(",");
    m_usb_tx_uint(stars[j][Y]);
    m_usb_tx_string(") is ");
    m_usb_tx_uint(distHoriz);
    m_usb_tx_string("\n");
}
void debug2(int j, int k, unsigned int distDiag){
    m_usb_tx_string("distDiag bet Star");
    m_usb_tx_int(j+1);
    m_usb_tx_string("(");
    m_usb_tx_uint(stars[j][X]);
    m_usb_tx_string(",");
    m_usb_tx_uint(stars[j][Y]);
    m_usb_tx_string(")");
    m_usb_tx_string(" and Star");
    m_usb_tx_int(k+1);
    m_usb_tx_string("(");
    m_usb_tx_uint(stars[k][X]);
    m_usb_tx_string(",");
    m_usb_tx_uint(stars[k][Y]);
    m_usb_tx_string(") is ");
    m_usb_tx_uint(distDiag);
    m_usb_tx_string("\n");
}
void debug3(int top, int bot, int lt, int rt){
    m_usb_tx_string("topMidStar at (");
    m_usb_tx_uint(stars[top][X]);m_usb_tx_string(",");m_usb_tx_uint(stars[top][Y]);m_usb_tx_string(").");
    m_usb_tx_string("botMidStar at (");
    m_usb_tx_uint(stars[bot][X]);m_usb_tx_string(",");m_usb_tx_uint(stars[bot][Y]);m_usb_tx_string(").");
    m_usb_tx_string("leftStar at (");
    m_usb_tx_uint(stars[lt][X]);m_usb_tx_string(",");m_usb_tx_uint(stars[lt][Y]);m_usb_tx_string(").");
    m_usb_tx_string("rightStar at (");
    m_usb_tx_uint(stars[rt][X]);m_usb_tx_string(",");m_usb_tx_uint(stars[rt][Y]);m_usb_tx_string(").");
    m_usb_tx_string("\n");
}
