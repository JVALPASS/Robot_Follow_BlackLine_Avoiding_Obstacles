#include <Servo.h>
//#include <SoftwareSerial.h>
#define speedPinRight 5
#define speedPinLeft 6
#define RightBack 7
#define RightFront 8
#define LeftBack 9
#define LeftFront 10
#define SERVO_PIN 11
#define Echo_PIN 2
 
#define SensorIR1 A0
#define SensorIR2 A1
#define SensorIR3 A2
#define SensorIR4 A3
#define SensorIR5 A4

//#define BUZZ_PIN  13
#define Trig_PIN 3
Servo head;
int diff = 10;
int distances[3] = {0,0,0};
int FAST_SPEED = 130;
int SPEED = 100;
int turnSpeed = 90;
int stopDistFirstObj = 8;//cosi ho lo spazio per curvare//prima era 27 ricordarsi
int stopDist = 27;
const int numRow = 8;
const int numCol = 7;
int myMap[numRow][numCol];
int myHeadCol = 3;
int myHeadRow = 3;
int myTailCol = 3;
int myTailRow = 4;

int myHeadTempCol;
int myHeadTempRow;
int myTailTempCol;
int myTailTempRow;

int objectiveRow = myHeadRow - 2;
int objectiveCol = myHeadCol;
boolean finished = false;
boolean starting = true;
boolean no_obstacles = true;
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
String previousState = "000";
String previous_R_L_state;
int tentative = 0;
boolean foundBlackLine = false;
String actualOrientation ="";
void go_Advance(int speedMotorLeft,int speedMotorRight, int del){
  digitalWrite(RightBack,LOW);
  digitalWrite(RightFront,HIGH);
  digitalWrite(LeftBack,LOW);
  digitalWrite(LeftFront,HIGH);
  analogWrite(speedPinRight,speedMotorRight);
  analogWrite(speedPinLeft, speedMotorLeft);
  if(del > 0){
    delay(del);
    stop_Stop();
  }
}
void go_Left(int speedMotorLeft,int speedMotorRight, int del){
  digitalWrite(RightBack,LOW);
  digitalWrite(RightFront,HIGH);
  digitalWrite(LeftBack,HIGH);
  digitalWrite(LeftFront,LOW);
  analogWrite(speedPinRight,speedMotorRight);
  analogWrite(speedPinLeft, speedMotorLeft);
  if(del > 0){
    delay(del);
    stop_Stop();
  }
}
void go_Right(int speedMotorLeft,int speedMotorRight, int del){
  digitalWrite(RightBack,HIGH);
  digitalWrite(RightFront,LOW);
  digitalWrite(LeftBack,LOW);
  digitalWrite(LeftFront,HIGH);
  analogWrite(speedPinRight,speedMotorRight);
  analogWrite(speedPinLeft, speedMotorLeft);
  if(del > 0){
    delay(del);
    stop_Stop();
  }
}
void go_Large_Left(int speedMotorLeft,int speedMotorRight, int del){
  go_Advance(speedMotorLeft, speedMotorRight, del);
}
void go_Large_Right(int speedMotorLeft,int speedMotorRight, int del){
  go_Advance(speedMotorLeft, speedMotorRight, del);
}

void stop_Stop() //motor brake -->robot stop
{
  digitalWrite(RightBack, LOW);
  digitalWrite(RightFront, LOW);
  digitalWrite(LeftBack, LOW);
  digitalWrite(LeftFront,LOW); 
  set_Motorspeed(0,0);
}

void set_Motorspeed(int speedR,int speedL) //change motor speed
{
  analogWrite(speedPinRight,speedR);//lspeed:0-255
  analogWrite(speedPinLeft,speedL);//rspeed:0-255   
}

int getDistance(){
  long echo_distance;
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN,LOW);
  echo_distance=pulseIn(Echo_PIN,HIGH);
  echo_distance=echo_distance*0.01657; //how far away is the object in cm
  //Serial.println((int)echo_distance);
  return round(echo_distance);
  /*long echo_distance;
  long duration;
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_PIN, LOW);
  duration =pulseIn(Echo_PIN,HIGH);
  //echo_distance=echo_distance*0.01657; //how far away is the object in cm
  echo_distance = (duration/2)*0.0344; //codice del prof
  return round(echo_distance);*/
}
String checkOrientation(){
  String orientation;
  if((myHeadRow < myTailRow) && (myHeadCol == myTailCol))
    return "streight";
  else if((myHeadCol < myTailCol) && (myHeadRow == myTailRow))
    return "left";
  else if((myHeadCol > myTailCol) && (myHeadRow == myTailRow))
    return "right";
  if((myHeadRow > myTailRow) && (myHeadCol == myTailCol))
    return "down";
}
void addObstaclesToMap(String obs){
  String orientation = checkOrientation();
  if(obs == "010"){
    if(orientation == "streight"){
      myMap[myHeadRow - 1][myHeadCol] = 5;
    }
    else if(orientation == "left"){
      myMap[myHeadRow][myHeadCol - 1] = 5;
    }
    else if(orientation == "right"){
      myMap[myHeadRow][myHeadCol + 1] = 5;
    }
  }
  else if(obs == "001"){
    if(orientation == "streight"){
      myMap[myHeadRow][myHeadCol+1] = 5;
    }
    else if(orientation == "left"){
      myMap[myHeadRow - 1][myHeadCol] = 5;
    }
    else if(orientation == "right"){
      myMap[myHeadRow + 1][myHeadCol] = 5;
    }
  }
  else if(obs == "011"){
    if(orientation == "streight"){
      myMap[myHeadRow - 1][myHeadCol] = 5;
      myMap[myHeadRow][myHeadCol+1] = 5;
    }
    else if(orientation == "left"){
      myMap[myHeadRow][myHeadCol - 1] = 5;
      myMap[myHeadRow - 1][myHeadCol] = 5;
    }
    else if(orientation == "right"){
      myMap[myHeadRow][myHeadCol + 1] = 5;
      myMap[myHeadRow + 1][myHeadCol] = 5;
    }
  }
  else if(obs == "100"){
    if(orientation == "streight"){
      myMap[myHeadRow][myHeadCol - 1] = 5;
    }
    else if(orientation == "left"){
      myMap[myHeadRow + 1][myHeadCol] = 5;
    }
    else if(orientation == "right"){
      myMap[myHeadRow - 1][myHeadCol] = 5;
    }
  }
  else if(obs == "101"){
    if(orientation == "streight"){
      myMap[myHeadRow][myHeadCol - 1] = 5;
      myMap[myHeadRow][myHeadCol+1] = 5;
    }
    else if(orientation == "left"){
      myMap[myHeadRow + 1][myHeadCol] = 5;
      myMap[myHeadRow - 1][myHeadCol] = 5;
    }
    else if(orientation == "right"){
      myMap[myHeadRow - 1][myHeadCol] = 5;
      myMap[myHeadRow + 1][myHeadCol] = 5;
    }
  }
  else if(obs == "110"){
    if(orientation == "streight"){
      myMap[myHeadRow][myHeadCol - 1] = 5;
      myMap[myHeadRow - 1][myHeadCol] = 5;
    }
    else if(orientation == "left"){
      myMap[myHeadRow + 1][myHeadCol] = 5;
      myMap[myHeadRow][myHeadCol - 1] = 5;
    }
    else if(orientation == "right"){
      myMap[myHeadRow - 1][myHeadCol] = 5;
      myMap[myHeadRow][myHeadCol + 1] = 5;
    }
  }
}
void changeTempOrientation(String newOrientation, boolean predict){
  String oldOrientation = checkOrientation();
  //Serial.println(oldOrientation);
  //Serial.println(newOrientation);
  if(oldOrientation == "streight"){
    if(newOrientation == "streight"){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol;
      myHeadTempRow = myHeadRow -1;
      myHeadTempCol = myHeadCol;
    }
    else if(newOrientation == "left" && (!predict)){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol + 1;
      myHeadTempCol = myHeadCol;
      myHeadTempRow = myHeadRow;
    }
    else if(newOrientation == "large_left" || (newOrientation == "left" && (predict))){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol;
      myHeadTempCol = myHeadCol -1;
      myHeadTempRow = myHeadRow;
    }
    else if(newOrientation == "right" && (!predict)){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol - 1;
      myHeadTempCol = myHeadCol;
      myHeadTempRow = myHeadRow;
    }
    else if(newOrientation == "large_right" || (newOrientation == "right" && (predict))){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol;
      myHeadTempCol = myHeadCol +1;
      myHeadTempRow = myHeadRow;
    }
  }
  if(oldOrientation == "left"){
    if(newOrientation == "streight"){
      myTailTempRow = myTailRow;
      myTailTempCol = myHeadCol;
      myHeadTempCol = myHeadCol -1;
      myHeadTempRow = myHeadRow;
    }
    else if(newOrientation == "left" && (!predict)){
      myTailTempRow = myTailRow - 1;
      myTailTempCol = myHeadCol;
      myHeadTempRow = myHeadRow;
      myHeadTempCol = myHeadCol;
    }
    else if(newOrientation == "large_left" || (newOrientation == "left" && (predict))){
      myTailTempRow = myTailRow;
      myTailTempCol = myHeadCol;
      myHeadTempRow = myHeadRow +1;
      myHeadTempCol = myHeadCol;
    }
    else if(newOrientation == "right" && (!predict)){
      myTailTempRow = myTailRow + 1;
      myTailTempCol = myHeadCol;
      myHeadTempRow = myHeadRow;
      myHeadTempCol = myHeadCol;
    }
    else if(newOrientation == "large_right" || (newOrientation == "right" && (predict))){
      myTailTempRow = myTailRow;
      myTailTempCol = myHeadCol;
      myHeadTempRow = myHeadRow -1;
      myHeadTempCol = myHeadCol;
    }
  }
  if(oldOrientation == "right"){
    if(newOrientation == "streight"){
      //Serial.println("streight_Right");
      myTailTempRow = myTailRow;
      myTailTempCol = myHeadCol;
      myHeadTempCol = myHeadCol +1;
      myHeadTempRow = myHeadRow;
    }
    else if(newOrientation == "left" && (!predict)){
      /*Serial.println("left_Right");
      Serial.println("inizio stampa");
      Serial.println(myHeadRow);
      Serial.println(myHeadCol);
      Serial.println(myTailRow);
      Serial.println(myTailCol);
      Serial.println("fine stampa");*/
      myTailTempRow = myTailRow+1;
      myTailTempCol = myHeadCol;
      myHeadTempRow = myHeadRow;
      myHeadTempCol = myHeadCol;
    }
    else if(((newOrientation == "large_left") || (newOrientation == "left" && (predict)))){
      //Serial.println("LargeLeft_Right");
      myTailTempRow = myTailRow;
      myTailTempCol = myHeadCol;
      myHeadTempRow = myHeadRow -1;
      myHeadTempCol = myHeadCol;
    }
    else if(newOrientation == "right" && (!predict)){
      //Serial.println("right_Right");
      myTailTempRow = myTailRow - 1;
      myTailTempCol = myHeadCol;
      myHeadTempRow = myHeadRow;
      myHeadTempCol = myHeadCol;
    }
    else if(newOrientation == "large_right" || (newOrientation == "right" && (predict))){
      //Serial.println("LargeRight_Right");
      myTailTempRow = myTailRow;
      myTailTempCol = myHeadCol;
      myHeadTempRow = myHeadRow +1;
      myHeadTempCol = myHeadCol;
    }
  }
  if(oldOrientation == "down"){
    if(newOrientation == "streight"){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol;
      myHeadTempRow = myHeadRow +1;
      myHeadTempCol = myHeadCol;
    }
    else if(newOrientation == "left" && (!predict)){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol - 1;
      myHeadTempCol = myHeadCol;
      myHeadTempRow = myHeadRow;
    }
    else if(newOrientation == "large_left" || (newOrientation == "left" && (predict))){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol;
      myHeadTempRow = myHeadRow;
      myHeadTempCol = myHeadCol +1;
    }
    else if(newOrientation == "right" && (!predict)){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol + 1;
      myHeadTempCol = myHeadCol;
      myHeadTempRow = myHeadRow;
    }
    else if(newOrientation == "large_right" || (newOrientation == "right" && (predict))){
      myTailTempRow = myHeadRow;
      myTailTempCol = myTailCol;
      myHeadTempCol = myHeadCol -1;
      myHeadTempRow = myHeadRow;
    }
  }
}
boolean largeTurn(String turn){
  /*Serial.print("largeTurn:");
  Serial.println(previous_R_L_state);*/
  if(turn == "left"){
    if(previous_R_L_state.charAt(0) == '1'){
      return true;
    }
    else{
      return false;
    }
  }
  else if(turn == "right"){
    if(previous_R_L_state.charAt(2) == '1'){
      return true;
    }
    else{
      return false;
    }
  }
}
void changeOrientationFinal(){
  myHeadRow = myHeadTempRow;
  myHeadCol = myHeadTempCol;
  myTailRow = myTailTempRow;
  myTailCol = myTailTempCol;
}
void printActualCoordinate(){
  delay(1000);
  Serial.print("myHeadRow:");
  Serial.println(myHeadRow);
      
  delay(1000);
  Serial.print("myHeadCol:");
  Serial.println(myHeadCol);
      
  delay(1000);
  Serial.print("myTailRow:");
  Serial.println(myTailRow);
      
  delay(1000);
  Serial.print("myTailCol:");
  Serial.println(myTailCol);
}
void printAfterChangedCoordinate(String rotation){
  delay(1000);
  Serial.print("myHeadRow aft");
  Serial.print(rotation);
  Serial.print(": ");
  Serial.println(myHeadRow);
      
  delay(1000);
  Serial.print("myHeadCol aft");
  Serial.print(rotation);
  Serial.print(": ");
  Serial.println(myHeadCol);
      
  delay(1000);
  Serial.print("myTailRow aft");
  Serial.print(rotation);
  Serial.print(": ");
  Serial.println(myTailRow);
      
  delay(1000);
  Serial.print("myTailCol aft");
  Serial.print(rotation);
  Serial.print(": ");
  Serial.println(myTailCol);
}
void printTempCoordinate(String orientation){
    delay(1000);
    Serial.print("myHeadTempRow ");
    Serial.print(orientation);
    Serial.print(": ");
    Serial.println(myHeadTempRow);
      
    delay(1000);
    Serial.print("myHeadTempCol ");
    Serial.print(orientation);
    Serial.print(": ");
    Serial.println(myHeadTempCol);
}
void printTempSubraction(String orientation, int tempRow, int tempCol){
    Serial.print("tempRow ");
    Serial.print(orientation);
    Serial.print(": ");
    Serial.println(tempRow);      
    Serial.print("tempCol ");
    Serial.print(orientation);
    Serial.print(": ");
    Serial.println(tempCol);
    Serial.println("");
}
void trackPath(){
  myMap[myHeadRow][myHeadCol] = 1;
  myMap[myTailRow][myTailCol] = 1;
}
void decision(){
  delay(1000);
  String obstacle_sign = checkDirectionFree();
  //printActualCoordinate();
  if( obstacle_sign=="010"){
    Serial.println("obs = 010");
    addObstaclesToMap(obstacle_sign);
    
    changeTempOrientation("right", true);
    
    //printTempCoordinate("right");
    
    int rowRight = abs(myHeadTempRow-objectiveRow);
    int colRight = abs(myHeadTempCol-objectiveCol);
    printTempSubraction("right",rowRight,colRight);
    changeTempOrientation("left", true);
    
    //printTempCoordinate("left");

    int rowLeft = abs(myHeadTempRow-objectiveRow);
    int colLeft = abs(myHeadTempCol-objectiveCol);
    printTempSubraction("left",rowLeft,colLeft);
    if(rowLeft == 0 && colLeft == 0){
      //Serial.println("1");
      if(largeTurn("left")){
        go_Large_Left(0, 100-11, 570);
        changeTempOrientation("large_left", false);
        //printAfterChangedCoordinate("large_left");
      }else{
        go_Left(100,100-10,380);//sinistra
        changeTempOrientation("left", false);
        //printAfterChangedCoordinate("left");
      }
      trackPath();
      changeOrientationFinal();
      
      //vai a sinistra  sei arrivato all'obiettivo
    }
    else if(rowRight == 0 && colRight == 0){
      //Serial.println("2");
      if(largeTurn("right")){
        go_Large_Right(100,0,570);
        changeTempOrientation("large_right", false);
        //printAfterChangedCoordinate("large_right");
      }else{
        go_Right(100,100-10, 420);//destra
        changeTempOrientation("right", false);
        //printAfterChangedCoordinate("right");
      }
      trackPath();
      changeOrientationFinal();
      
      //vai a destra sei arrivato all'obiettivo
    }
    if((rowRight == rowLeft) && (colRight == colLeft)){
      //Serial.println("3");
      
      if(largeTurn("right")){
        go_Large_Right(100,0,570);
        changeTempOrientation("large_right", false);
        //printAfterChangedCoordinate("large_right");
      }else{
        go_Right(100,100-10, 420);//destra
        changeTempOrientation("right", false);
        //printAfterChangedCoordinate("right");
      }
      trackPath();
      changeOrientationFinal();
      
      //gira destra
    }
    else if((rowRight < rowLeft) && (colRight == colLeft)){
      //Serial.println("4");
      
      if(largeTurn("right")){
        go_Large_Right(100,0,570);
        changeTempOrientation("large_right", false);
        //printAfterChangedCoordinate("large_right");
      }else{
        go_Right(100,100-10, 420);//destra
        changeTempOrientation("right", false);
        //printAfterChangedCoordinate("right");
      }
      trackPath();
      changeOrientationFinal();
      
      //gira destra
    }
    else if((rowLeft < rowRight) && (colRight == colLeft)){
      //Serial.println("5");
      
      if(largeTurn("left")){
        go_Large_Left(0, 100-11, 570);
        changeTempOrientation("large_left", false);
        //printAfterChangedCoordinate("large_left");
      }else{
        go_Left(100,100-10,380);//sinistra
        changeTempOrientation("left", false);
        //printAfterChangedCoordinate("left");
      }
      
      trackPath();
      changeOrientationFinal();
      
      //gira sinistra
    }

    else if((rowRight == rowLeft) && (colRight < colLeft)){
      //Serial.println("6");
      
      if(largeTurn("right")){
        go_Large_Right(100,0,570);
        changeTempOrientation("large_right", false);
        //printAfterChangedCoordinate("large_right");
      }else{
        go_Right(100,100-10, 420);//destra
        changeTempOrientation("right", false);
        //printAfterChangedCoordinate("right");
      }
      trackPath();
      changeOrientationFinal();
      
      //gira destra
    }
    else if((rowRight == rowLeft) && (colLeft < colRight)){
      //Serial.println("7");
      
      if(largeTurn("left")){
        go_Large_Left(0, 100-11, 570);
        changeTempOrientation("large_left", false);
        //printAfterChangedCoordinate("large_left");
      }else{
        go_Left(100,100-10,380);//sinistra
        changeTempOrientation("left", false);
        //printAfterChangedCoordinate("left");
      }
      trackPath();
      changeOrientationFinal();


      //gira sinistra
    }
  }

  else if( obstacle_sign=="100"){
    Serial.println("obs= 100");
    addObstaclesToMap(obstacle_sign);
      
    changeTempOrientation("streight", true);
    int rowStreight = abs(myHeadTempRow-objectiveRow);
    int colStreight = abs(myHeadTempCol-objectiveCol);
    printTempSubraction("streight",rowStreight,colStreight);
    
    //printTempCoordinate("streight");
 
    changeTempOrientation("right", true);
    int rowRight = abs(myHeadTempRow-objectiveRow);
    int colRight = abs(myHeadTempCol-objectiveCol);
    printTempSubraction("right",rowRight,colRight);
    //printTempCoordinate("right");
    
    if(rowStreight == 0 && colStreight == 0){
      //Serial.println("8");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();
      
      //vai dritto sei arrivato all'obiettivo
    }
    else if(rowRight == 0 && colRight == 0){
      //Serial.println("9");
      
      go_Large_Right(100,0,570);
      changeTempOrientation("large_right", false);
      //printAfterChangedCoordinate("large_right");
      trackPath();
      changeOrientationFinal();
      
      //vai a destra sei arrivato all'obiettivo
    }
    else if((rowStreight < rowRight) && (colRight < colStreight)){
      //Serial.println("10");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();
      
      
      //vai dritto
    }
    else if((rowStreight < rowRight) && (colStreight < colRight)){
      //Serial.println("11");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();
      
      
      //vai dritto
    }
    else if((rowRight < rowStreight) && (colRight < colStreight)){
      //Serial.println("12");
      
      go_Large_Right(100,0,570);
      changeTempOrientation("large_right", false);
      //printAfterChangedCoordinate("large_right");
      trackPath();
      changeOrientationFinal();

      
      //vai a destra
    }

    else if((rowRight < rowStreight) && (colStreight < colRight)){// meglio destra secondo me perhce le righe devoo avere la priorita
      //Serial.println("13");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();
      
      //vai dritto
    }
  }

  else if( obstacle_sign=="001"){
    
    Serial.println("obs= 001");
    addObstaclesToMap(obstacle_sign);
    
    changeTempOrientation("streight", true);

    //printTempCoordinate("streight");
    
    int rowStreight = abs(myHeadTempRow-objectiveRow);
    int colStreight = abs(myHeadTempCol-objectiveCol);
    changeTempOrientation("left", true);
    
    //printTempCoordinate("left");
    
    int rowLeft = abs(myHeadTempRow-objectiveRow);
    int colLeft = abs(myHeadTempCol-objectiveCol);
    
    if(rowStreight == 0 && colStreight == 0){
      //Serial.println("14");

      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto sei arrivato all'obiettivo
    }
    else if(rowLeft == 0 && colLeft == 0){
      //Serial.println("15");
      
      go_Large_Left(0, 100-11, 570);
      changeTempOrientation("large_left", false);
      //printAfterChangedCoordinate("large_left");
      trackPath();
      changeOrientationFinal();
      
      //vai a sinistra sei arrivato all'obiettivo
    }
    else if((rowStreight < rowLeft) && (colLeft < colStreight)){
      //Serial.println("16");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
    else if((rowStreight < rowLeft) && (colStreight < colLeft)){
      //Serial.println("17");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
    else if((rowLeft < rowStreight) && (colLeft < colStreight)){
      //Serial.println("18");
       
      go_Large_Left(0, 100-11, 570);
      changeTempOrientation("large_left", false);
      //printAfterChangedCoordinate("large_left");
      trackPath();
      changeOrientationFinal();

      //vai a sinistra
    }

    else if((rowLeft < rowStreight) && (colStreight < colLeft)){
      //Serial.println("19");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
  }
  
  else if( obstacle_sign=="000"){
    Serial.println("obs= 000");
    
    changeTempOrientation("streight", true);
    //printTempCoordinate("streight");
    int rowStreight = abs(myHeadTempRow-objectiveRow);
    int colStreight = abs(myHeadTempCol-objectiveCol);
    printTempSubraction("streight",rowStreight,colStreight);
    
    changeTempOrientation("right", true);
    //printTempCoordinate("right");
    int rowRight = abs(myHeadTempRow-objectiveRow);
    int colRight = abs(myHeadTempCol-objectiveCol);
    printTempSubraction("right",rowRight,colRight);
    
    changeTempOrientation("left", true);
    //printTempCoordinate("left");
    int rowLeft = abs(myHeadTempRow-objectiveRow);
    int colLeft = abs(myHeadTempCol-objectiveCol);
    printTempSubraction("left",rowLeft,colLeft);
    
    if(rowStreight == 0 && colStreight == 0){
      //Serial.println("20");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto sei arrivato all'obiettivo
    }
    else if(rowRight == 0 && colRight == 0){
      //Serial.println("21");
      
      if(largeTurn("right")){
        go_Large_Right(100,0,570);
        changeTempOrientation("large_right", false);
        //printAfterChangedCoordinate("large_right");
      }else{
        go_Right(100,100-10, 420);//destra
        changeTempOrientation("right", false);
        //printAfterChangedCoordinate("right");
      }
      trackPath();
      changeOrientationFinal();

      //vai a destra sei arrivato all'obiettivo
    }
    else if(rowLeft == 0 && colLeft == 0){
      //Serial.println("22");
      
      if(largeTurn("left")){
        go_Large_Left(0, 100-11, 570);
        changeTempOrientation("large_left", false);
        //printAfterChangedCoordinate("large_left");
      }else{
        go_Left(100,100-10,380);//sinistra
        changeTempOrientation("left", false);      
        //printAfterChangedCoordinate("left");
      }
      trackPath();
      changeOrientationFinal();

      //vai a sinistra sei arrivato all'obiettivo
    }
    else if(((rowRight == rowLeft) && (colRight < colLeft)) && ((rowStreight < rowRight) && (colRight < colStreight))){
      //Serial.println("23");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
    else if(((rowRight == rowLeft) && (colRight < colLeft)) && ((rowStreight < rowRight) && (colStreight < colRight))){
      //Serial.println("24");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
    else if(((rowRight == rowLeft) && (colRight < colLeft)) && ((rowRight < rowStreight) && (colRight < colStreight))){
      //Serial.println("25");

      if(largeTurn("right")){
        go_Large_Right(100,0,570);
        changeTempOrientation("large_right", false);
        //printAfterChangedCoordinate("large_right");
      }else{
        go_Right(100,100-10, 420);//destra
        changeTempOrientation("right", false);
        //printAfterChangedCoordinate("right");
      }
      trackPath();
      changeOrientationFinal();

      //vai a destra
    }

    else if(((rowRight == rowLeft) && (colRight < colLeft)) && ((rowRight < rowStreight) && (colStreight < colRight))){
      //Serial.println("26");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto// secondo me è meglio che vada a destra perchè se sta all'incontrario si avvicina di riga e la riga deve essere sempre prioritaria rispetto alla colonna
    }
    
    else if(((colRight == colLeft) && (rowRight < rowLeft)) && ((rowStreight < rowRight) && (colRight < colStreight))){
      //Serial.println("27");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
    else if(((colRight == colLeft) && (rowRight < rowLeft)) && ((rowStreight < rowRight) && (colStreight < colRight))){
      //Serial.println("28");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();
      
      //vai dritto
    }
    else if(((colRight == colLeft) && (rowRight < rowLeft)) && ((rowRight < rowStreight) && (colRight < colStreight))){
      //Serial.println("29");

      if(largeTurn("right")){
        go_Large_Right(100,0,570);
        changeTempOrientation("large_right", false);
        //printAfterChangedCoordinate("large_right");
      }else{
        go_Right(100,100-10, 420);//destra
        changeTempOrientation("right", false);
        //printAfterChangedCoordinate("right");
      }
      //printAfterChangedCoordinate("right");
      trackPath();
      changeOrientationFinal();

      //vai a destra
    }

    else if(((colRight == colLeft) && (rowRight < rowLeft)) && ((rowRight < rowStreight) && (colStreight < colRight))){
      //Serial.println("30");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto// secondo me è meglio che vada a destra perchè se sta all'incontrario si avvicina di riga e la riga deve essere sempre prioritaria rispetto alla colonna
    }
    
    else if(((rowRight == rowLeft) && (colLeft < colRight)) && ((rowStreight < rowLeft) && (colLeft < colStreight))){
      //Serial.println("31");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
    
    else if(((rowRight == rowLeft) && (colLeft < colRight)) && ((rowStreight < rowLeft) && (colStreight < colLeft))){
      //Serial.println("32");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
    else if(((rowRight == rowLeft) && (colLeft < colRight)) && ((rowLeft < rowStreight) && (colLeft < colStreight))){
      //Serial.println("33");
      
      if(largeTurn("left")){
        go_Large_Left(0, 100-11, 570);
        changeTempOrientation("large_left", false);
        //printAfterChangedCoordinate("large_left");
      }else{
        go_Left(100,100-10,380);//sinistra
        changeTempOrientation("left", false);
        //printAfterChangedCoordinate("left");
      }
      trackPath();
      changeOrientationFinal();

      //vai a sinistra
    }

    else if(((rowRight == rowLeft) && (colLeft < colRight)) && ((rowLeft < rowStreight) && (colStreight < colLeft))){
      //Serial.println("34");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto // secondo me è meglio che vada a destra perchè se sta all'incontrario si avvicina di riga e la riga deve essere sempre prioritaria rispetto alla colonna
    }

    else if(((colLeft == colRight) && (rowLeft < rowRight)) && ((rowStreight < rowLeft) && (colLeft < colStreight))){
      //Serial.println("35");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();
      //vai dritto
    }
    
    else if(((colLeft == colRight) && (rowLeft < rowRight)) && ((rowStreight < rowLeft) && (colStreight < colLeft))){
      //Serial.println("36");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto
    }
    else if(((colLeft == colRight) && (rowLeft < rowRight)) && ((rowLeft < rowStreight) && (colLeft < colStreight))){
      //Serial.println("37");
      if(largeTurn("left")){
        go_Large_Left(0, 100-11, 570);
        //Serial.println("Large left");
        changeTempOrientation("large_left", false);
        trackPath();
        changeOrientationFinal();
        //printAfterChangedCoordinate("large_left");
      }else{
        go_Left(100,100-10,380);//sinistra
        //Serial.println("left");
        changeTempOrientation("left", false);
        trackPath();
        changeOrientationFinal();
        //printAfterChangedCoordinate("left");
      }

      //vai a sinistra
    }

    else if(((colLeft == colRight) && (rowLeft < rowRight)) && ((rowLeft < rowStreight) && (colStreight < colLeft))){
      //Serial.println("38");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto // secondo me è meglio che vada a destra perchè se sta all'incontrario si avvicina di riga e la riga deve essere sempre prioritaria rispetto alla colonna
    }
    else if(((colLeft == colRight) && (rowLeft == rowRight)) && ((rowStreight < rowLeft) && (colStreight < colLeft))){
      /*
       * situazione tipo
       * 3
       * 2
       * 23 obj
       * 2
       * 3
      */
      //Serial.println("39");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto // secondo me è meglio che vada a destra perchè se sta all'incontrario si avvicina di riga e la riga deve essere sempre prioritaria rispetto alla colonna
    }
    else if(((colLeft == colRight) && (rowLeft == rowRight)) && ((rowLeft < rowStreight) && (colStreight < colLeft))){//non avverrà mai
      //Serial.println("40");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto // secondo me è meglio che vada a destra perchè se sta all'incontrario si avvicina di riga e la riga deve essere sempre prioritaria rispetto alla colonna
    }
    else if(((colLeft == colRight) && (rowLeft == rowRight)) && ((rowStreight < rowLeft) && (colLeft < colStreight))){//non avverrà mai
      //Serial.println("41");
      
      go_Advance(100,100-10, 400);//dritto
      changeTempOrientation("streight", false);
      //printAfterChangedCoordinate("streight");
      trackPath();
      changeOrientationFinal();

      //vai dritto // secondo me è meglio che vada a destra perchè se sta all'incontrario si avvicina di riga e la riga deve essere sempre prioritaria rispetto alla colonna
    }
  }
  else if( obstacle_sign=="101"){
    Serial.println("obs= 101");
    //Serial.println("42");
    addObstaclesToMap(obstacle_sign);
    printActualCoordinate();
    
    go_Advance(100,100-10, 400);//dritto
    changeTempOrientation("streight", false);
    //printAfterChangedCoordinate("streight");
    trackPath();
    changeOrientationFinal();

    //vai dritto
  }
  else if( obstacle_sign=="110"){// puo andare in down quindi risolvere questo problema
    Serial.println("obs= 110");
    //Serial.println("43");
    addObstaclesToMap(obstacle_sign);
    go_Large_Right(100,0,570);
    changeTempOrientation("large_right", false);
    //printAfterChangedCoordinate("large_right");
    trackPath();
    changeOrientationFinal();

    //vai a destra
  }
  else if( obstacle_sign=="011"){// puo andare in down quindi risolvere questo problema
    Serial.println("obs= 011");
    //Serial.println("44");
    addObstaclesToMap(obstacle_sign);
    go_Large_Left(0, 100-11, 570);
    changeTempOrientation("large_left", false);
    //printAfterChangedCoordinate("large_left");
    trackPath();
    changeOrientationFinal();
    //vai a sinistra
  }
  else if( obstacle_sign=="111"){
    Serial.println("obs= 111");
    //Serial.println("45");
    stop_Stop();
    
    //printAfterChangedCoordinate("stop");
    //stop poi modificare con un giro di retromarcia fino allinietro qausi 
  }
  myMap[myHeadRow][myHeadCol] = 3;
  myMap[myTailRow][myTailCol] = 2;
  printMap();
}

void checkOvercomeObjective(){// if i goes beyond the objective because for istance the objective is on right but on right i find an obstacle so i have to change the position of the objective on the below row
  if(myHeadRow < objectiveRow){
    myMap[objectiveRow][objectiveCol] = 0;
    objectiveRow = objectiveRow - 1;
    myMap[objectiveRow][objectiveCol] = 4;
    Serial.println("over obj");
  }
}
boolean checkDetectObj(){
  if((myHeadRow == objectiveRow) && (myHeadCol == objectiveCol)){
    Serial.println("end");
    no_obstacles = true;
  }
}
void checkDetectBlackLine(){//check if i'm reached objective i colud not found the line if i not found change the position of the objective to the below row if i'm in the streight position in the following colum if i'm in th right position in the previous colum if i'm in the left postion e di questo ne faccio 3 tentativi se non riesco stoppo, return true if i found a black line false if is not found
  /*if(tentative < 3){
   * if(detect balck line){
       avvia percorso su liena nera
      }
     else
       tentativo++;
  }else{
    stop;
  }*/
}

String checkDirectionFree(){
/*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */
 
  int obstacle_status = B100000;
  
  head.write(90); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = getDistance();
  if(centerscanval < stopDist){
    stop_Stop();
    obstacle_status  = obstacle_status | B100;
  }
  
  head.write(120);
  delay(100);
  ldiagonalscanval = getDistance();
  if(ldiagonalscanval < stopDist){
    stop_Stop();
    obstacle_status  = obstacle_status | B1000;
  }
  
  head.write(170); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = getDistance();
  if(leftscanval < stopDist){
    stop_Stop();
    obstacle_status  = obstacle_status | B10000;
  }
  
  head.write(40);
  delay(300);
  rdiagonalscanval = getDistance();
  if(rdiagonalscanval < stopDist){
    stop_Stop();
    obstacle_status  = obstacle_status | B10;
  }
  head.write(0);
  delay(100);
  rightscanval = getDistance();
  if(rightscanval < stopDist){
    stop_Stop();
    obstacle_status  = obstacle_status | 1;
  }
  
  head.write(90); //Finish looking around (look forward again)
  delay(300);
  String obstacle_str = String(obstacle_status,BIN);
  obstacle_str = obstacle_str.substring(1,6);
  //Serial.println(obstacle_str);
  obstacle_str = reduceBits(obstacle_str);
  //Serial.print("checkdirectionfree_pr:");
  //Serial.println(obstacle_str);
  obstacle_str = comparePriviousActualState(obstacle_str);
  //Serial.print("checkdirectionfree:");
  //Serial.println(obstacle_str);
  //Serial.print("checkdirectionfree:");
  //Serial.println(previous_R_L_state);
  return obstacle_str; //return 5-character string standing for 5 direction obstacle status
}
String comparePriviousActualState(String actual_state){
  String temp = "";
  //Serial.print("comparePriviousActualState:");
  //Serial.println(actual_state);
  //Serial.print("comparePriviousActualState:");
  //Serial.println(previousState);
  if(actual_state != previousState){
    temp = actual_state;
    if(previousState.charAt(0) == '1' && actual_state.charAt(0) == '0'){
      actual_state.setCharAt(0, '1');
      previous_R_L_state = actual_state;
    }
    if(previousState.charAt(2) == '1' && actual_state.charAt(2) == '0'){
      actual_state.setCharAt(2, '1');
      previous_R_L_state = actual_state;
    }
    previousState = temp;
  }
  return actual_state;
}
String reduceBits(String bits){
  int obstacle_status = B1000;
  if(bits.charAt(0) == '1' || bits.charAt(4) == '1'){
    if(bits.charAt(0) == '1'){
      obstacle_status = obstacle_status | B1100;
      if(bits.charAt(1) == '1'){
        if(bits.charAt(2) == '0' && bits.charAt(3) == '0'){
          obstacle_status = obstacle_status | B1000;
        }else{
          obstacle_status = obstacle_status | B1010;
        }
      }else{
        if(bits.charAt(2) == '1'){
          obstacle_status = obstacle_status | B1010;
        }
      }
    }else{
      if(bits.charAt(1) == '1' || bits.charAt(2) == '1'){
          obstacle_status = obstacle_status | B1010;
      }
    }
    if(bits.charAt(4) == '1'){
      obstacle_status = obstacle_status | B1001;
      if(bits.charAt(3) == '1'){
        if(bits.charAt(2) == '0' && bits.charAt(1) == '0'){
          obstacle_status = obstacle_status | B1000;
        }else{
          obstacle_status = obstacle_status | B1010;
        }
      }else{
        if(bits.charAt(2) == '1'){
          obstacle_status = obstacle_status | B1010;
        }
      }
    }else{
      if(bits.charAt(3) == '1' || bits.charAt(2) == '1'){
          obstacle_status = obstacle_status | B1010;
       }
    }
  }
  else{
    if(bits.charAt(1) == '1' || bits.charAt(2) == '1' || bits.charAt(3) == '1'){
          obstacle_status = obstacle_status | B1010;
     }
  }
  String obstacle_str = String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,4);
  return obstacle_str;
}
void printMap(){
  Serial.print("[ ");
  for(int i = 0; i < numRow; i++){
    for(int j = 0; j < numCol; j++){
      if(j == 0 && i != 0){
        Serial.print("  ");
      }
      Serial.print(myMap[i][j]);
      Serial.print(" ");
    }
    if(i <7)
      Serial.println("");
  }
  Serial.println("]");
}
void setup(){
  pinMode(SensorIR1, INPUT);
  pinMode(SensorIR2, INPUT);
  pinMode(SensorIR3, INPUT);
  pinMode(SensorIR4, INPUT);
  pinMode(SensorIR5, INPUT);
  pinMode(RightBack, OUTPUT);
  pinMode(RightFront, OUTPUT);
  pinMode(speedPinRight, OUTPUT);
  pinMode(LeftBack, OUTPUT);
  pinMode(LeftFront, OUTPUT);
  pinMode(speedPinLeft, OUTPUT);
  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN,INPUT); 
  stop_Stop();
  head.attach(SERVO_PIN); 
  head.write(90);
  myMap[3][3] = 2;//head
  myMap[4][3] = 1; //tail
  myMap[objectiveRow][objectiveCol] = 4;//head
  Serial.begin(9600);
  //BTSerial.begin(9600);
  //pinMode(BUZZ_PIN, OUTPUT);
  //digitalWrite(BUZZ_PIN, HIGH);
  delay(10000);
}
String read_sensor_values(){
  String value="";
  value = value + !digitalRead(SensorIR1);
  value = value + !digitalRead(SensorIR2);
  value = value + !digitalRead(SensorIR3);
  value = value + !digitalRead(SensorIR4);
  value = value + !digitalRead(SensorIR5);
  return value;
}
void auto_tracking(String  startingOrientation){
  detectObstacles();
  String sensorval= read_sensor_values();
  Serial.println(sensorval);
  if (sensorval=="10000" ){ 
    //Serial.println("1");
    tentative = 0;
    foundBlackLine = true;
    //The black line is in the left of the car, need  left turn 
    go_Left(100, 100, 80);  //Turn left // fino a 90 di delay mi sono spinto
    delay(70);//70 buon valore
  }
  else if (sensorval == "11000" || sensorval=="10100"  || sensorval=="01000" || sensorval=="01100" 
    || sensorval=="11100"  || sensorval=="10010" || sensorval == "11001" || sensorval == "11101" || sensorval=="11010" || sensorval=="11110"){
    //Serial.println("2");
    tentative = 0;
    foundBlackLine = true;
    go_Large_Left(0,100,80);  //Turn slight left
    delay(70);
  }
  else if (sensorval=="00001"  ){
    //Serial.println("3");//The black line is  on the right of the car, need  right turn 
    tentative = 0;
    foundBlackLine = true;
    go_Right(100,100,80);  //Turn right
    delay(70);
  }
  else if (sensorval=="00011" || sensorval=="00010"  || sensorval=="00101" || sensorval=="00110" 
    || sensorval=="00111" || sensorval=="01101" || sensorval=="01111" ||
    sensorval=="01011"  || sensorval=="01001" || sensorval=="01110"){
    //Serial.println("4");
    tentative = 0;
    foundBlackLine = true;
    go_Large_Right(100,0,80);  //Turn slight right
    delay(70);
  }
 else if (sensorval=="00100"){
    //Serial.println("5");//The black line is  on the right of the car, need  right turn 
    tentative = 0;
    foundBlackLine = true;
    go_Advance(100,100,80);  //go streight
    delay(70);
  }
  else if (sensorval=="11111"){
    //Serial.println("6");
    Serial.println(actualOrientation);
    tentative = 0;
    foundBlackLine = true;
    if(actualOrientation == "right"){
      go_Large_Left(0,80,100);
    }
    if(actualOrientation == "left"){
        go_Large_Right(80,0,100);
      }
    if(actualOrientation == "streight"){
      go_Large_Left(0,80,80);
    }
  }
  else if (sensorval=="00000"){
    //Serial.println("7");
    foundBlackLine = false;
    if(startingOrientation == "right"){
      if(tentative < 15){
        go_Advance(100, 100-10, 100);
        actualOrientation = "right";
        delay(100);
        //The car front touch stop line, need stop
      }
      else if(tentative >= 15 && tentative < 16){
        go_Left(100,100-10,410);
        actualOrientation = "streight";
        delay(500);
      }
      else if(tentative >= 16 && tentative < 17){
        go_Left(100,100-10,410);
        actualOrientation = "left";
        delay(500);
      }
      else if(tentative >= 17 && tentative < 40){
        go_Advance(100, 100-10, 100);
        actualOrientation = "left";
        delay(100);
      }
      else{
        stop_Stop();
      }
    }
    else if (startingOrientation == "left"){
      if(tentative < 15){
        go_Advance(100, 100-10, 100);
        actualOrientation = "left";
        delay(100);
        //The car front touch stop line, need stop
      }
      else if(tentative >= 15 && tentative < 16){
        go_Right(100,100-10,420);
        actualOrientation = "streight";
        delay(500);
      }
      else if(tentative >= 16 && tentative < 17){
        go_Right(100,100-10,420);
        actualOrientation = "right";
        delay(500);
      }
      else if(tentative >= 17 && tentative < 40){
        go_Advance(100, 100-10, 100);
        actualOrientation = "right";
        delay(100);
      }
      else{
        stop_Stop();
      }
    }
    //The car front touch stop line, need stop
    else if (startingOrientation == "streight"){
      if(tentative < 1){
        go_Left(100,100-10,410);
        actualOrientation = "left";
        delay(500);
      }
      if(tentative >= 1 && tentative < 16){
        go_Advance(100, 100-10, 100);
        actualOrientation = "left";
        delay(100);
        //The car front touch stop line, need stop
      }
      else if(tentative >= 16 && tentative < 17){
        go_Right(100,100-10,420);
        actualOrientation = "streight";
        delay(500);
      }
      else if(tentative >= 17 && tentative < 18){
        go_Right(100,100-10,460);
        actualOrientation = "right";
        delay(100);
      }
      else if(tentative >= 18 && tentative < 40){
        go_Advance(100, 100-10, 100);
        actualOrientation = "right";
        delay(100);
      }
      else{
        stop_Stop();
      }
    }
    else{
      stop_Stop();
    }
  }  
}
void detectingLine(){
  String startingOrientation = checkOrientation();
  if(tentative == 0 && actualOrientation ==""){
    actualOrientation = startingOrientation;
  }
  /*else if(tentative == 0 && actualOrientation != ""){// togliere non credo sia utile
    startingOrientation = "streight";
  }*/
  auto_tracking(startingOrientation);
  if(!foundBlackLine){
    tentative = tentative + 1;
  }else{
    reInitalizePosition();
  }
}
void reInitalizePosition(){
  myHeadCol = 3;
  myHeadRow = 3;
  myTailCol = 3;
  myTailRow = 4;
  for(int i = 0; i < numRow; i++){
    for(int j = 0; j < numCol; j++){
      myMap[i][j] = 0;
    }
  }
}
void detectObstacles(){
  int distance = getDistance();
    if(distance < stopDistFirstObj){
      no_obstacles = false;
      Serial.println("Revelead object");
    }
    stop_Stop();
}
void avoidObstaclesRecoverLine(){
    decision();
    if(checkDetectObj()){
      stop_Stop();
    }
    checkOvercomeObjective();
    delay(1000);
}
void loop() {
  //Serial.println("go");//inzio
  if(no_obstacles){
    detectingLine();
  }
  else if(!no_obstacles){
    avoidObstaclesRecoverLine();
  }
}
