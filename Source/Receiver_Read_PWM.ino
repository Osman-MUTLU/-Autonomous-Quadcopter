unsigned long timer_1, timer_2, timer_3, timer_4,timer_5,timer_6, current_time;
int receiver_input[7];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4,last_channel_5, last_channel_6;

void setup() {
  Serial.begin(9600);
  PCICR |= (1 << PCIE2);                                                    //Set PCIE0 to enable PCMSK2 scan.
  PCMSK2 |= (1 << PCINT21);                                                  //Set PCINT0 (digital input 5) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT20);                                                  //Set PCINT1 (digital input 4)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT18);                                                  //Set PCINT2 (digital input 2)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT22);                                                  //Set PCINT3 (digital input 6)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT23);                                                  //Set PCINT0 (digital input 7) to trigger an interrupt on state change.
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                   //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
}

void loop() {
  read_pwm();
  printReceiver();
}
void printReceiver(){
  Serial.print(" PITCH = ");
  Serial.print(receiver_input[1]); // CH1
  Serial.print("    ROLL = ");
  Serial.print(receiver_input[2]); // CH2
  Serial.print("    Throttle = "); 
  Serial.print(receiver_input[3]); // CH3
  Serial.print("    YAW = ");
  Serial.print(receiver_input[4]); // CH4
  Serial.print("    AUX1 = ");
  Serial.print(receiver_input[5]); // CH5
  Serial.print("    AUX2 = ");
  Serial.print(receiver_input[6]); // CH6
  Serial.println();
}
void read_pwm(){
  if(receiver_input[1]<1000) receiver_input[1] = 1000;
  else if(receiver_input[1]>2000) receiver_input[1] = 2000;
  if(receiver_input[2]<1000) receiver_input[2] = 1000;
  else if(receiver_input[2]>2000) receiver_input[2] = 2000;
  if(receiver_input[3]<1000) receiver_input[3] = 1000;
  else if(receiver_input[3]>2000) receiver_input[3] = 2000;
  if(receiver_input[4]<1000) receiver_input[4] = 1000;
  else if(receiver_input[4]>2000) receiver_input[4] = 2000;
  if(receiver_input[5]<1000) receiver_input[5] = 1000;
  else if(receiver_input[5]>2000) receiver_input[5] = 2000;
  if(receiver_input[6]<1000) receiver_input[6] = 1000;
  else if(receiver_input[6]>2000) receiver_input[6] = 2000;
}
ISR(PCINT2_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PIND & B00100000){                                                     //Is input 5 high?
    if(last_channel_1 == 0){                                                //Input 5 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 5 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PIND & B00010000 ){                                                    //Is input 4 high?
    if(last_channel_2 == 0){                                                //Input 4 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 4 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PIND & B00000100 ){                                                    //Is input 2 high?
    if(last_channel_3 == 0){                                                //Input 2 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 2 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PIND & B01000000 ){                                                    //Is input 6 high?
    if(last_channel_4 == 0){                                                //Input 6 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 6 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  //Channel 5=========================================
  if(PIND & B10000000 ){                                                    //Is input 7 high?
    if(last_channel_5 == 0){                                                //Input 7 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_5 to current_time.
    }
  }
  else if(last_channel_5 == 1){                                             //Input 7 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer_5;                             //Channel 5 is current_time - timer_5.
  }
}
ISR(PCINT0_vect){
  //Channel 6=========================================
  if(PINB & B00000001 ){                                                    //Is input 8 high?
    if(last_channel_6 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_6 to current_time.
    }
  }
  else if(last_channel_6 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    receiver_input[6] = current_time - timer_6;                             //Channel 6 is current_time - timer_6.
  }
}
