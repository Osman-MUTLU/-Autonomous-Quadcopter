/* RECEIVER values*/
unsigned long timer_1, timer_2, timer_3, timer_4,timer_5,timer_6, current_time;
int receiver_input[7];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4,last_channel_5, last_channel_6;
int receiver_throttle,receiver_yaw,receiver_pitch,receiver_roll,receiver_aux1,receiver_aux2;
/* Motors*/
int motor_1_pow =0;
int motor_2_pow =0;
int motor_3_pow =0;
int motor_4_pow =0;
int throttle;
int min_throttle=1150;
/* Timers */ 
int timer;
unsigned long timer_motor_1, timer_motor_2, timer_motor_3, timer_motor_4, esc_timer, esc_loop_timer;
unsigned long loop_timer;

void setup() {
  Serial.begin(9600);
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18);                                                  //Set PCINT2 (digital input 2)to trigger an interrupt on state change.
  // put your setup code here, to run once:
  DDRD |= B00001000;//Configure digital poort 3 as output.
  DDRB |= B00001110;   //Configure digital poort 9 , 10 and 11 as output.
}

void loop() {
  throttle= receiver_input[3];
  if(throttle<=min_throttle+20) throttle=1000;
  else throttle = throttle-(min_throttle-1000);
  if(throttle>1600) throttle = 2000;
  Serial.print("    Throttle = "); 
  Serial.print(throttle); // CH3 Throttle
  Serial.println();
  motor_1_pow=throttle;
  motor_2_pow=throttle;
  motor_3_pow=throttle;
  motor_4_pow=throttle;
  while(micros() - loop_timer < 4000); 
  loop_timer = micros(); 
  PORTD |= B00001000;                                                     //Set digital poort 3 high.
  PORTB |= B00001110;                                                     //Set digital poort 9, 10 and 11 high.
  timer_motor_1 = motor_1_pow + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_motor_2 = motor_2_pow + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_motor_3 = motor_3_pow + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_motor_4 = motor_4_pow + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  delayMicroseconds(1000);
  boolean esc_1=false,esc_2=false,esc_3=false,esc_4=false;
  while(true){
    //Stay in this loop until output 3,9,10 and 11 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_motor_1 <= esc_loop_timer){
      PORTD &= B11110111;                //Set digital output 3 to low if the time is expired.
      esc_1=true;
    }
    if(timer_motor_2 <= esc_loop_timer){
      PORTB &= B11110111;                //Set digital output 9 to low if the time is expired.
      esc_2=true;
    }
    if(timer_motor_3 <= esc_loop_timer){
      PORTB &= B11111011;                //Set digital output 10 to low if the time is expired.
      esc_3=true;
    }
    if(timer_motor_4 <= esc_loop_timer){
      PORTB &= B11111101;                //Set digital output 11 to low if the time is expired.
      esc_4=true;
    }
    if(esc_1 && esc_2 && esc_3 && esc_4) break;
  }  
}

ISR(PCINT2_vect){
  current_time = micros();
  //Channel 3=========================================
  if(PIND & B00000100 ){                                                    //Is input 2 high?  Throttle channel 3
    if(last_channel_3 == 0){                                                //Input 2 changed from 0 to 1. 
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 2 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.
    if(receiver_input[3]<min_throttle)min_throttle=receiver_input[3];
  }
}
