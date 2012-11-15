void playStriker(){
	
    // GET DIR
    if(check(PINB,5)){
        direction=true; //SET THE ROBOT DIRECTION TO SHOOT
    } else {
        direction=false;
    }
    // GET POS
    getPos();
    // SEND LOCATION TO HELPER
    //buffer[1]=myPos[X];
    //buffer[2]=myPos[Y];
    //buffer[3]=direction;
    //m_rf_send(TXADDRESS, buffer, PACKET_LENGTH);
    // READ ADC VALS
    int adc1 = ADC0_read();
    int adc2 = ADC1_read();
    // m_usb_tx_string("adc1, adc2 = (");
    // m_usb_tx_int(adc1);
    // m_usb_tx_string(", ");
    // m_usb_tx_int(adc2);
    // m_usb_tx_string(")\n");
    int adcDiff = adc2 - adc1;
    // TIMER CHECKS
    //m_usb_tx_string("myLastPos(x,y): (");
    //m_usb_tx_int(myLastPos[X]);
    //m_usb_tx_string(", ");
    //m_usb_tx_int(myLastPos[Y]);
    //m_usb_tx_string(") TIMER:");
    //m_usb_tx_int(timer);
    //m_usb_tx_string("\n");
    if(timer>5 && timer<10){
        backward();
        m_wait(250);
        turnLeft();
        m_wait(250);
        backward();
        m_wait(250);
        timer = 0;
    } else if(hasPuck()){// ROBOT SWITCH PUSHED.. PUCK IS SEEN
        m_green(OFF);
        m_wait(100);
        unsigned int xTarg;
        unsigned int yTarg = 60;
        if(direction){                  // Face goal A
            xTarg = 240;
        } else {                        // Face goal B
            xTarg = 0;
        } 
        // ROTATE TO FACE GOAL
        if(!facing(xTarg,yTarg)) return;
        if(direction){                  // Try to go to A
            if(myPos[X]>160){           // Close enough, fire!
                m_green(ON);
                fire();
            } else {
                m_green(OFF);           // Too far, keep going forward
                forward();           
            }
        } else {                        // Try to go to B
            if(myPos[X]<80){            // Close enough, fire!
                m_green(ON);
                fire();
            } else {                    // Too far, keep going forward
                m_green(OFF);
                forward();     
            }    
        }
    } else if(adc1>960||adc2>960){        // PUCK SEEN, hone in..
        if(adcDiff>10){
            spinLeft();
        } else if(adcDiff<-10){
            spinRight();
        } else {
            forward();
        }
    } else{                               //PUCK NOT SEEN, keep spinning..
        m_green(OFF);
        spinRight();
    }
}
void playHelper(){
    // GET DIR
    if(check(PINB,5)){
        direction=true; //SET THE ROBOT DIRECTION TO SHOOT
    } else {
        direction=false;
    }
    // direction = directionPacket;
    // READ ADC VALS
    int adc1 = ADC0_read();
    int adc2 = ADC1_read();
    //m_usb_tx_string("adc1, adc2 = (");
    //m_usb_tx_int(adc1);
    //m_usb_tx_string(", ");
    //m_usb_tx_int(adc2);
    //m_usb_tx_string(")\n");
    int adcDiff = adc2 - adc1;
    if(direction) m_usb_tx_string("direction true\n");
    else m_usb_tx_string("direction false\n");
    if(timer>=1){
        turnLeft();
        m_wait(1000);
        backward();
        m_wait(500);
        timer = 0;
    } else if(hasPuck()){// ROBOT SWITCH PUSHED.. PUCK IS SEEN
        m_green(OFF);
        m_wait(100);
        unsigned int xTarg;
        unsigned int yTarg = 60;
        if(direction){                  // Face goal A
            xTarg = 240;
        } else {                        // Face goal B
            xTarg = 0;
        } 
        // ROTATE TO FACE GOAL
        if(!facing(xTarg,yTarg)) return;
        if(direction){                  // Try to go to A
            if(myPos[X]>160){           // Close enough, fire!
                m_green(ON);
                fire();
            } else {
                m_green(OFF);           // Too far, keep going forward
                forward();           
            }
        } else {                        // Try to go to B
            if(myPos[X]<80){            // Close enough, fire!
                m_green(ON);
                fire();
            } else {                    // Too far, keep going forward
                m_green(OFF);
                forward();     
            }    
        }
    } else if(adc1>960||adc2>960){        // PUCK SEEN, hone in..
        if(adcDiff>10){
            spinLeft();
        } else if(adcDiff<-10){
            spinRight();
        } else {
            forward();
        }
    } else{                               // PUCK NOT SEEN, keep spinning..
        m_green(OFF);
        spinRight();
    }
}
void playGoalie(){
    // READ ADC VALS
    int adcBack = ADC0_read();
    int adcLowerRight = ADC1_read();
    int adcMid = ADC4_read();
    int adcTopRight = ADC5_read();
    int adcFront = ADC6_read();
    m_usb_tx_string("adcBack, adcLowerRight, adcMid, adcTopRight, adcFront = (");
    m_usb_tx_int(adcBack);
    m_usb_tx_string(", ");
    m_usb_tx_int(adcLowerRight);
    m_usb_tx_string(", ");
    m_usb_tx_int(adcMid);
    m_usb_tx_string(", ");
    m_usb_tx_int(adcTopRight);
    m_usb_tx_string(", ");
    m_usb_tx_int(adcFront);
    m_usb_tx_string(")\n");
    forward();
}