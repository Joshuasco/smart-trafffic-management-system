
//BEGIN ALL PIN DECLARATION

//TRAFFIC LIGHT SIGNS PIN DECLARATION

//1st lane
#define green_led_go_1_pin 22
#define yellow_led_ready_1_pin 23
#define red_led_stop_1_pin 24

//2nd lane
#define green_led_go_2_pin 25
#define yellow_led_ready_2_pin 26
#define red_led_stop_2_pin 27

//3rd lane
#define green_led_go_3_pin 28
#define yellow_led_ready_3_pin 29
#define red_led_stop_3_pin 30

//4th lane
#define green_led_go_4_pin 31
#define yellow_led_ready_4_pin 32
#define red_led_stop_4_pin 33

//END TRAFFIC LIGHT SIGNS PIN DECLARATION


//VERIFIED CARD LED INDICATORS

//1st lane
#define green_led1 A0
#define red_led1  55 //A1

//2nd lane
#define green_led2 56  //A2
#define red_led2  A3

//3rd lane
#define green_led3 A4
#define red_led3  A5

//4th lane
#define green_led4  A6
#define red_led4  A7

#define buzzer_pin 47

//END VERIFIED CARD LED INDICATORS


//IR SENSOR PIN DECLARATION

//1st lane
#define IRsensor1_lane1_pin 34
#define IRsensor2_lane1_pin 35
#define IRsensor3_lane1_pin 36

//2nd lane
#define IRsensor1_lane2_pin 37
#define IRsensor2_lane2_pin 38
#define IRsensor3_lane2_pin 39

//3rd lane
#define IRsensor1_lane3_pin 40
#define IRsensor2_lane3_pin 41
#define IRsensor3_lane3_pin 42

//4th lane
#define IRsensor1_lane4_pin 43
#define IRsensor2_lane4_pin 44
#define IRsensor3_lane4_pin 45

//END IR SENSOR PIN DECLARATION


//RFID PIN DECLARATION
  
    /* Typical pin layout used:
   * ----------------------------------
   *             MFRC522       Arduino    
   *             Reader/PCD    Mega         
   * Signal      Pin           Pin                          
   * ----------------------------------
   * RST/Reset   RST           5                
   * SPI MOSI    MOSI          51                      
   * SPI MISO    MISO          50                      
   * SPI SCK     SCK           52                       
   */

  #include <SPI.h>
  #include <MFRC522.h>
  
  #define RST_PIN         5         // Configurable, see typical pin layout above
  #define SS_1_PIN        53         // Configurable, take an unused pin, only HIGH/LOW required, must be different to SS 2, SS 3 and SS4
  #define SS_2_PIN        49          // Configurable, take an unused pin, only HIGH/LOW required, must be different to SS 1, SS 3 and SS 4
  #define SS_3_PIN        48          // Configurable, take an unused pin, only HIGH/LOW required, must be different to SS 1, SS 2 and SS 4
  #define SS_4_PIN        46          // Configurable, take an unused pin, only HIGH/LOW required, must be different to SS 1, SS 2 and SS 3
  
  #define NR_OF_READERS   4 // declare num of RFID readers to be 4
  
  String MY_RFID_CARD = ""; //reads card and store for validation
  String CARDS[] = {"22747154172","21117788168","243559516","6755116165","195254150165","67209165166"}; // Array to hold all registered cards
  int CARD_NO = sizeof(CARDS)/sizeof(CARDS[0]);
  
  byte ssPins[] = {SS_1_PIN, SS_2_PIN, SS_3_PIN, SS_4_PIN};
  
  MFRC522 mfrc522[NR_OF_READERS];   // Create MFRC522 instance.
  
  uint8_t reader;
 
//END RFID PIN DECLARATION

//END ALL PIN DECLARATION
 

//ALL TRAFFIC SIGNAL LANE FUNCTIONS

//LANE1 TRAFFIC SIGNAL FUNCTION

void LANE1(bool stop_movement = false, int time_delay=20000 ){ //lane one traffic light signals
  if(stop_movement==true){
    //Turn on red light to stop lane1 movement  
    digitalWrite(red_led_stop_1_pin, HIGH);
    digitalWrite(yellow_led_ready_1_pin, LOW);
    digitalWrite(green_led_go_1_pin, LOW);
    }
  else{//activate lane1 for movement
    
    //Turn on red light for a 1sec
    digitalWrite(red_led_stop_1_pin, HIGH);
    digitalWrite(yellow_led_ready_1_pin, LOW);
    digitalWrite(green_led_go_1_pin, LOW);
    delay(1000);// delay for 1sec
    
    //turn on yellow light to get ready for 2sec
    digitalWrite(red_led_stop_1_pin, LOW);
    digitalWrite(yellow_led_ready_1_pin, HIGH);
    digitalWrite(green_led_go_1_pin, LOW);
    delay(2000);// delay for 2sec

    //turn on green light to go for 20sec
    digitalWrite(red_led_stop_1_pin, LOW);
    digitalWrite(yellow_led_ready_1_pin,LOW);
    digitalWrite(green_led_go_1_pin, HIGH);
    delay(time_delay);// delay for 20sec
    }
  }// End lane1 traffic signal


//LANE2 TRAFFIC SIGNAL FUNCTION

void LANE2(bool stop_movement = false, int time_delay=20000){ //lane one traffic light signals
  if(stop_movement==true){
    //Turn on red light to stop lane2 movement  
    digitalWrite(red_led_stop_2_pin, HIGH);
    digitalWrite(yellow_led_ready_2_pin, LOW);
    digitalWrite(green_led_go_2_pin, LOW);
    
    }
  else{//activate lane2 for movement
    
    //Turn on red light for a 1sec
    digitalWrite(red_led_stop_2_pin, HIGH);
    digitalWrite(yellow_led_ready_2_pin, LOW);
    digitalWrite(green_led_go_2_pin, LOW);
    delay(1000);// delay for 1sec
    
    //turn on yellow light to get ready for 2sec
    digitalWrite(red_led_stop_2_pin, LOW);
    digitalWrite(yellow_led_ready_2_pin, HIGH);
    digitalWrite(green_led_go_2_pin, LOW);
    delay(2000);// delay for 2sec

    //turn on green light to go for 20sec
    digitalWrite(red_led_stop_2_pin, LOW);
    digitalWrite(yellow_led_ready_2_pin,LOW);
    digitalWrite(green_led_go_2_pin, HIGH);
    delay(time_delay);// delay for 20sec
    }
  }// End lane2 traffic signal


//LANE3 TRAFFIC SIGNAL FUNCTION

void LANE3(bool stop_movement = false, int time_delay=20000){ //lane one traffic light signals
  if(stop_movement==true){
    //Turn on red light to stop lane3 movement  
    digitalWrite(red_led_stop_3_pin, HIGH);
    digitalWrite(yellow_led_ready_3_pin, LOW);
    digitalWrite(green_led_go_3_pin, LOW);
    
    }
  else{//activate lane3 for movement
    
    //Turn on red light for a 1sec
    digitalWrite(red_led_stop_3_pin, HIGH);
    digitalWrite(yellow_led_ready_3_pin, LOW);
    digitalWrite(green_led_go_3_pin, LOW);
    delay(1000);// delay for 1sec
    
    //turn on yellow light to get ready for 2sec
    digitalWrite(red_led_stop_3_pin, LOW);
    digitalWrite(yellow_led_ready_3_pin, HIGH);
    digitalWrite(green_led_go_3_pin, LOW);
    delay(2000);// delay for 2sec

    //turn on green light to go for 20sec
    digitalWrite(red_led_stop_3_pin, LOW);
    digitalWrite(yellow_led_ready_3_pin,LOW);
    digitalWrite(green_led_go_3_pin, HIGH);
    delay(time_delay);// delay for 20sec
    }
  }// End lane3 traffic signal


//LANE4 TRAFFIC SIGNAL FUNCTION

void LANE4(bool stop_movement = false, int time_delay=20000){ //lane one traffic light signals
  if(stop_movement==true){
    //Turn on red light to stop lane4 movement  
    digitalWrite(red_led_stop_4_pin, HIGH);
    digitalWrite(yellow_led_ready_4_pin, LOW);
    digitalWrite(green_led_go_4_pin, LOW);
    
    }
  else{//activate lane4 for movement
    
    //Turn on red light for a 1sec
    digitalWrite(red_led_stop_4_pin, HIGH);
    digitalWrite(yellow_led_ready_4_pin, LOW);
    digitalWrite(green_led_go_4_pin, LOW);
    delay(1000);// delay for 1sec
    
    //turn on yellow light to get ready for 2sec
    digitalWrite(red_led_stop_4_pin, LOW);
    digitalWrite(yellow_led_ready_4_pin, HIGH);
    digitalWrite(green_led_go_4_pin, LOW);
    delay(2000);// delay for 2sec

    //turn on green light to go for 20sec
    digitalWrite(red_led_stop_4_pin, LOW);
    digitalWrite(yellow_led_ready_4_pin,LOW);
    digitalWrite(green_led_go_4_pin, HIGH);
    delay(time_delay);// delay for 20sec
    }
  }// End lane4 traffic signal

//END ALL TRAFFIC SIGNAL LANE FUNCTION


//BEGIN CONGESTION COUNT BASE ON NUMBER OF MOTORS ON EACH LANE

//initialize count
int count1 =0;// count for mortors in lane1
int count2 =0;
int count3 =0;
int count4 =0;

//lane1 congestion count base on number on motors
int LANE1_CONGESTION(){
  
  bool motor_sense_1 = digitalRead(IRsensor1_lane1_pin); //read IR sensor 1 value on lane 1
  bool motor_sense_2 = digitalRead(IRsensor2_lane1_pin); //read IR sensor 2 value on lane 1
  bool motor_sense_3 = digitalRead(IRsensor3_lane1_pin); //read IR sensor 3 value on lane 1

    
  if (motor_sense_1==0){ // if motor sensed, increse countl by l
    count1++;
  }
   if (motor_sense_2==0){// if another motor sensed, increse countl by l
    count1++;
  }
   if (motor_sense_3==0){// if another motor sensed, increse countl by l again
    count1++;
  }
   Serial.print("total_count1 = ");
   Serial.println(count1);
  return count1; // return total number of motors in lane1
  
}//end lane1 congestion count base on number on motors


//lane2 congestion count base on number of motors
int LANE2_CONGESTION(){
  
  bool motor_sense_1 = digitalRead(IRsensor1_lane2_pin);//read IR sensor 1 value on lane 2
  bool motor_sense_2 = digitalRead(IRsensor2_lane2_pin);//read IR sensor 2 value on lane 2
  bool motor_sense_3 = digitalRead(IRsensor3_lane2_pin);//read IR sensor 3 value on lane 2

  if (motor_sense_1==0){ // if motor sensed, increse countl by l
    count2++;
  }
   if (motor_sense_2==0){// if another motor sensed, increse countl by l
    count2++;
  }
   if (motor_sense_3==0){// if another motor sensed, increse countl by l again
    count2++;
  }
  Serial.print("total_count2 = ");
    Serial.println(count2);
  return count2; // return total number of motors in lane1
  
}//end lane2 congestion count base on number on motors


//lane3 congestion count base on number on motors
int LANE3_CONGESTION(){
  
  bool motor_sense_1 = digitalRead(IRsensor1_lane3_pin);//read IR sensor 1 value on lane 3
  bool motor_sense_2 = digitalRead(IRsensor2_lane3_pin);//read IR sensor 2 value on lane 3
  bool motor_sense_3 = digitalRead(IRsensor3_lane3_pin);//read IR sensor 3 value on lane 3

  if (motor_sense_1==0){ // if motor sensed, increse countl by l
    count3++;
  }
   if (motor_sense_2==0){// if another motor sensed, increse countl by l
    count3++;
  }
   if (motor_sense_3==0){// if another motor sensed, increse countl by l again
    count3++;
  }
  Serial.print("total_count3 = ");
    Serial.println(count3);
  return count3; // return total number of motors in lane1
  
}//end lane3 congestion count base on number on motors


//lane4 congestion count base on number on motors
int LANE4_CONGESTION(){
  
  bool motor_sense_1 = digitalRead(IRsensor1_lane4_pin);//read IR sensor 1 value on lane 4
  bool motor_sense_2 = digitalRead(IRsensor2_lane4_pin);//read IR sensor 2 value on lane 4
  bool motor_sense_3 = digitalRead(IRsensor3_lane4_pin);//read IR sensor 3 value on lane 4

  if (motor_sense_1==0){ // if motor sensed, increse countl by l
    count4++;
  }
   if (motor_sense_2==0){// if another motor sensed, increse countl by l
    count4++;
  }
   if (motor_sense_3==0){// if another motor sensed, increse countl by l again
    count4++;
  }
  Serial.print("total_count4 = ");
    Serial.println(count4);
  return count4; // return total number of motors in lane1
  
}//end lane4 congestion count base on number on motors
//END CONGESTION COUNT BASE ON NUMBER OF MOTORS ON EACN LANE


//EMERGENCY PRIORITY USING RIFD CARDS

void UNVERIFIED_CARD(int led_pin){
  analogWrite(led_pin, 255);
  delay(1000);
  Serial.println();
  Serial.print("unverifide, should activate pin ");
  Serial.println(led_pin);
  analogWrite(led_pin, 0);
  }
  
void  VERIFIED_CARD(int led_pin){
  analogWrite(led_pin, 255);
  tone(buzzer_pin, 70, 500);
  delay(1000);
  noTone(buzzer_pin);
  analogWrite(led_pin, 0);
  }


void CHECK_EMERGENCY(){

  for (reader = 0; reader < NR_OF_READERS; reader++) {
    // Look for new cards

    if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial()) {
       Serial.println();
       
      Serial.print(F("Reader "));
      Serial.print(reader);
      // Show some details of the PICC (that is: the tag/card)
      Serial.print(F(": Card UID:"));
      dump_byte_array(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.println();
      Serial.print(F("PICC type: "));
      MFRC522::PICC_Type piccType = mfrc522[reader].PICC_GetType(mfrc522[reader].uid.sak);
      Serial.println(mfrc522[reader].PICC_GetTypeName(piccType));

    //VERIFIED Card Registration and  Validity
      
      for(int i=0; i<=CARD_NO; i++){
        if (MY_RFID_CARD == CARDS[i]){
          Serial.println("card registered and ACCEPTED");
          
          //check lane
             if(reader == 0){
              VERIFIED_CARD(green_led1);
              
              //stop all lanes, activate lane1
              Serial.print("Lane1 activated \n");
              LANE2(true);//stop movement
              LANE3(true);//stop movement
              LANE4(true);//stop movement
              LANE1(false, 30000);//activate lane1 for 30s
              LANE1(true);//stop lane1
              }
             else if(reader == 1){
                VERIFIED_CARD(green_led2);
                
              //stop all lanes, activate lane2
              Serial.print("Lane2 activated \n");
              LANE1(true);//stop movement
              LANE3(true);//stop movement
              LANE4(true);//stop movement
              LANE2(false, 30000);//activate lane2 for 30s
              LANE2(true);//stop lane2
              }
               else if(reader == 2){
                  VERIFIED_CARD(green_led3);
                  
              //stop all lanes, activate lane2
              Serial.print("Lane3 activated \n");
              LANE2(true);//stop movement
              LANE1(true);//stop movement
              LANE4(true);//stop movement
              LANE3(false, 30000);//activate lane3 for 30s
              LANE3(true); //stop lane3
              }
               else if(reader == 3){
                  VERIFIED_CARD(green_led4);
                  
              //stop all lanes, activate lane2
              Serial.print("Lane4 activated \n");
              LANE2(true);//stop movement
              LANE3(true);//stop movement
              LANE1(true);//stop movement
              LANE4(false, 30000);//activate lane4 for 30s
              LANE4(true);//stop lane4
              }
              else{
                //operate normally
                }  
          break;
          }
          else{
            if(i >= CARD_NO-1){
               Serial.println("card NOT registered and REJECTED");

         //check lane
             if(reader == 0){
              UNVERIFIED_CARD(red_led1);
              }
             if(reader == 1){
                UNVERIFIED_CARD(red_led2);
              }
              if(reader ==2 ){
                UNVERIFIED_CARD(red_led3);
              }
              if(reader == 3){
                UNVERIFIED_CARD(red_led4);
              }
                //operate normally
          break;
              }
            }
        }
        
      //End VERIFIED Card Registration and Validity

      // Halt PICC
      mfrc522[reader].PICC_HaltA();
      // Stop encryption on PCD
      mfrc522[reader].PCD_StopCrypto1();
    } //if (mfrc522[reader].PICC_IsNewC
  } //for(uint8_t reader
  MY_RFID_CARD = ""; //empty/reset MY_RFID_CARD
  
  }

  /* Helper routine to dump a byte array as hex values to Serial.*/
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
    MY_RFID_CARD += buffer[i];
    
  }// end for
} // end dump_byte_array
  
//END EMERGENCY PRIORITY USING RIFD CARDS


//SET ALL LED PINS TO OUTPUT
void LED_MODE_OUTPUT(){

  //1st lane
pinMode(green_led_go_1_pin, OUTPUT);
pinMode(yellow_led_ready_1_pin, OUTPUT);
pinMode(red_led_stop_1_pin, OUTPUT);

//2nd lane
pinMode(green_led_go_2_pin, OUTPUT);
pinMode(yellow_led_ready_2_pin, OUTPUT);
pinMode(red_led_stop_2_pin, OUTPUT);

//3rd lane
pinMode(green_led_go_3_pin, OUTPUT);
pinMode(yellow_led_ready_3_pin, OUTPUT);
pinMode(red_led_stop_3_pin, OUTPUT);

//4th lane
pinMode(green_led_go_4_pin, OUTPUT);
pinMode(yellow_led_ready_4_pin, OUTPUT);
pinMode(red_led_stop_4_pin, OUTPUT);

  }//END SET ALL LED TO OUTPUT


 //SET ALL IR SENSOR PIN MODE TO INPUT
 void IRSENSOR_MODE_INPUT(){
  
//1st lane
pinMode(IRsensor1_lane1_pin, INPUT);
pinMode(IRsensor2_lane1_pin, INPUT);
pinMode(IRsensor3_lane1_pin, INPUT);

//2nd lane
pinMode(IRsensor1_lane2_pin, INPUT);
pinMode(IRsensor2_lane2_pin, INPUT);
pinMode(IRsensor3_lane2_pin, INPUT);

//3rd lane
pinMode(IRsensor1_lane3_pin, INPUT);
pinMode(IRsensor2_lane3_pin, INPUT);
pinMode(IRsensor3_lane3_pin, INPUT);

//4th lane
pinMode(IRsensor1_lane4_pin, INPUT);
pinMode(IRsensor2_lane4_pin, INPUT);
pinMode(IRsensor3_lane4_pin, INPUT);

 } //END SET ALL IR SENSOR MODE TO INPUT


void setup() {
  // put your setup code here, to run once:
  
 LED_MODE_OUTPUT(); // run setup for LEDs as output
 IRSENSOR_MODE_INPUT(); // run setup for IRsensors as input
 
digitalWrite(red_led_stop_1_pin, HIGH);
digitalWrite(red_led_stop_2_pin, HIGH);
digitalWrite(red_led_stop_3_pin, HIGH);
digitalWrite(red_led_stop_4_pin, HIGH);

//rfid stuffs
  Serial.begin(115200);
  
  while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)

  SPI.begin();        // Init SPI bus

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN); // Init each MFRC522 card
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
    mfrc522[reader].PCD_DumpVersionToSerial();
  }
  //end rfid stuffs
}
  

//BEGIN PRIORITIZE LANE FUNC
void PRIORITIZE_LANE(int lane1_congestion, int lane2_congestion, int lane3_congestion, int lane4_congestion){

  CHECK_EMERGENCY();
  if(lane1_congestion > lane2_congestion && lane1_congestion > lane3_congestion && lane1_congestion > lane4_congestion){
  Serial.println("NORMAL OPERATION TIME ON LANE1 20s");
        LANE1(false);//begin movement
        LANE1(true);//stop movement
  }

   CHECK_EMERGENCY();
  if(lane2_congestion > lane1_congestion && lane2_congestion > lane3_congestion && lane2_congestion > lane4_congestion){
  Serial.println("NORMAL OPERATION TIME ON LANE2 20s");
        LANE2(false);//begin movement
        LANE2(true);//stop movement
  }
 CHECK_EMERGENCY();

 if(lane3_congestion > lane1_congestion && lane3_congestion > lane2_congestion && lane3_congestion > lane4_congestion){
  Serial.println("NORMAL OPERATION TIME ON LANE3 20s");
        LANE3(false);//begin movement
       LANE3(true);//stop movement
  }
  CHECK_EMERGENCY();

 if(lane4_congestion > lane1_congestion && lane4_congestion > lane3_congestion && lane4_congestion > lane2_congestion){
  Serial.println("NORMAL OPERATION TIME ON LANE3 20s");
        LANE4(false);//begin movement
        LANE4(true);//stop movement
  }
   CHECK_EMERGENCY();


 //check for equality and give priority to the first lane
 if(lane1_congestion == lane2_congestion || lane1_congestion == lane3_congestion || lane1_congestion == lane4_congestion){
  if(lane1_congestion >0){
  Serial.println("NORMAL OPERATION TIME ON LANE1 20s");
        LANE1(false);//begin movement
        LANE1(true);//stop movement
    }
  }
CHECK_EMERGENCY();

//check for equality and give priority to the second lane
  if( lane2_congestion == lane3_congestion || lane2_congestion == lane4_congestion){
  if(lane2_congestion >0){
  Serial.println("NORMAL OPERATION TIME ON LANE2 20s");
        LANE2(false);//begin movement
        LANE2(true);//stop movement
 }
  }
  CHECK_EMERGENCY();
  
 //check for equality and give prioriy to the thrid lane
 if( lane3_congestion == lane4_congestion){
  if(lane3_congestion >0){
  Serial.println("NORMAL OPERATION TIME ON LANE3 20s");
        LANE3(false);//begin movement
       LANE3(true);//stop movement
}
  }   CHECK_EMERGENCY();
  }
  //ENN PRIORITIZE LANE FUNC


void loop() {
  Serial.begin(115200);

int lane1_congestion = LANE1_CONGESTION();//get number of cars for lane1
int lane2_congestion = LANE2_CONGESTION();//get number of cars for lane2
int lane3_congestion = LANE3_CONGESTION();//get number of cars for lane3
int lane4_congestion = LANE4_CONGESTION();//get number of cars for lane4


  if(lane1_congestion == 0 && lane2_congestion == 0 && lane3_congestion == 0 && lane4_congestion == 0){
    //operate normally in 20sec interval
   
    LANE1(false);//begin movement
    LANE1(true);//stop movement
    
     lane1_congestion = LANE1_CONGESTION();//get number of cars for lane1
     lane2_congestion = LANE2_CONGESTION();//get number of cars for lane2
     lane3_congestion = LANE3_CONGESTION();//get number of cars for lane3
     lane4_congestion = LANE4_CONGESTION();//get number of cars for lane4
     PRIORITIZE_LANE(lane1_congestion, lane2_congestion, lane3_congestion, lane4_congestion);
     
    LANE2(false);//begin movement
    LANE2(true);//stop movement

     lane1_congestion = LANE1_CONGESTION();//get number of cars for lane1
     lane2_congestion = LANE2_CONGESTION();//get number of cars for lane2
     lane3_congestion = LANE3_CONGESTION();//get number of cars for lane3
     lane4_congestion = LANE4_CONGESTION();//get number of cars for lane4
     PRIORITIZE_LANE(lane1_congestion, lane2_congestion, lane3_congestion, lane4_congestion);
    
    LANE3(false);//begin movement
    LANE3(true);//stop movement
     lane1_congestion = LANE1_CONGESTION();//get number of cars for lane1
     lane2_congestion = LANE2_CONGESTION();//get number of cars for lane2
     lane3_congestion = LANE3_CONGESTION();//get number of cars for lane3
     lane4_congestion = LANE4_CONGESTION();//get number of cars for lane4
     PRIORITIZE_LANE(lane1_congestion, lane2_congestion, lane3_congestion, lane4_congestion);
     
    LANE4(false);//begin movement
    LANE4(true);//stop movement
     lane1_congestion = LANE1_CONGESTION();//get number of cars for lane1
     lane2_congestion = LANE2_CONGESTION();//get number of cars for lane2
     lane3_congestion = LANE3_CONGESTION();//get number of cars for lane3
     lane4_congestion = LANE4_CONGESTION();//get number of cars for lane4
     PRIORITIZE_LANE(lane1_congestion, lane2_congestion, lane3_congestion, lane4_congestion);
    }
    else{
      PRIORITIZE_LANE(lane1_congestion, lane2_congestion, lane3_congestion, lane4_congestion);
      }
      
  CHECK_EMERGENCY();

count1=0;count2=0;count3=0;count4=0; //reset sensors vehicle count to 0
  
}
