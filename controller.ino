/*
///////////////////////////////////////////////////////

//ボードの設定を"Arduino Esplora"に変更する必要あり

//////////////////////////////////////////////////////
*/
#include <XBee.h>
#include <Esplora.h>
#include <SoftwareSerial.h>
#include <TFT.h>
#include <SPI.h>
#include "xbeeID.h"

#define NOR 15
#define W 160
#define H 128

//for control robot
uint8_t controller[17] = {'A','G','S', 'M', 'F','1', 'T','2', 'L',0,0, 'R',0,0, 'A','G','E'};

//for change mode
uint8_t mode[12] = {'A','G','S', 'C', 'F','1', 'T','2', 0, 'A','G','E'};

XBee xbee = XBee();


SoftwareSerial mySerial(3,11);

//初期送信先(PAN1 R1)
XBeeAddress64 remoteAddress = XBeeAddress64(PAN1_R1_SH, PAN1_R1_SL);


//ボタンの値用
int val[5] = {0, 0, 0, 0, 0};
int old_val[5] = {0, 0, 0, 0, 0};

//ロボット切り替え用
int robot_id = 1, robot_flag = 0;
char robot_id_c;
String robot_id_s;

//ロボット移動速度切り替え用
unsigned long speed, old_speed, turn_speed = 16;
unsigned long speed_array[8] = {16,32,48,64,80,96,112,128};
int speed_state = 0,speed_state1 = 0;
String speed_s;
char speed_c[3];



//コマンドの要素番号
int Lmode = 9, Lspeed = 10, Rmode = 12, Rspeed = 13;
int mode_select = 8;

//ループ
int i;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//セットアップ
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup () {
  delay(1000);
  Serial.begin(57600);
  mySerial.begin(57600);
  xbee.setSerial(mySerial);
  
  EsploraTFT.begin();
  EsploraTFT.background(255,255,255);
  EsploraTFT.stroke(0,0,0);
  EsploraTFT.setTextSize(5);
  EsploraTFT.text("1",70,30);
  
  EsploraTFT.setTextSize(2);
  
  /*
  speed = Esplora.readSlider();
  speed *= 172;
  speed /= 1023;
  */
  
  speed = speed_array[0];
  
  speed_s = String(speed);
  speed_s.toCharArray(speed_c, 4);
  //EsploraTFT.stroke(0,0,0);
  EsploraTFT.text(speed_c, 70,100);
  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//各ボタンの状態の読み込み
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_readButton(){
 val[0] = Esplora.readJoystickButton();
 val[1] = Esplora.readButton(SWITCH_DOWN);
 val[2] = Esplora.readButton(SWITCH_LEFT);
 val[3] = Esplora.readButton(SWITCH_UP);
 val[4] = Esplora.readButton(SWITCH_RIGHT);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//液晶画面への表示
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_tftDraw(){
  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//モードの切り替え
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_modeChange(){
  //マニュアルモード//////////
  if((val[4] == LOW) && (old_val[4] == HIGH)){
    mode[mode_select] = 1; 
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, mode, sizeof(mode));
    xbee.send(zbTx_con);
  }
  
  //スタンバイモード//////////
  if((val[3] == LOW) && (old_val[3] == HIGH)){
    mode[mode_select] = 0; 
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, mode, sizeof(mode));
    xbee.send(zbTx_con);
  }
  
  //オートモード//////////
  if((val[2] == LOW) && (old_val[2] == HIGH)){
    mode[mode_select] = 2; 
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, mode, sizeof(mode));
    xbee.send(zbTx_con);
   }
   
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//操作対象ロボットの切り替え
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_robotChange(){
  if((val[3] == LOW) && (old_val[3] == HIGH)){
    if(robot_id == NOR)robot_id = 1;
    else robot_id++;
    
    robot_flag = 1;
  }
  if((val[1] == LOW) && (old_val[1] == HIGH)){
    if(robot_id == 1)robot_id = NOR;
    else robot_id--;
    
    robot_flag = 1;
  }
  
  if(robot_flag == 1){
    switch(robot_id){
      case 1: remoteAddress = XBeeAddress64(PAN1_R1_SH, PAN1_R1_SL);     //PAN1 R1
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("1",70,30);
              break;
      case 2: remoteAddress = XBeeAddress64(PAN1_R2_SH, PAN1_R2_SL);     //PAN1 R2
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("2",70,30);
              break;
      case 3: remoteAddress = XBeeAddress64(PAN1_R3_SH, PAN1_R3_SL);     //PAN1 R3
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("3",70,30);
              break;
      case 4: remoteAddress = XBeeAddress64(PAN1_R4_SH, PAN1_R4_SL);     //PAN1 R4
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("4",70,30);
              break;
      case 5: remoteAddress = XBeeAddress64(PAN1_R5_SH, PAN1_R5_SL);     //PAN1 R5
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("5",70,30);
              break;      
      case 6: remoteAddress = XBeeAddress64(PAN1_R6_SH, PAN1_R6_SL);     //PAN1 R6
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("6",70,30);
              break;
      case 7: remoteAddress = XBeeAddress64(PAN1_R7_SH, PAN1_R7_SL);     //PAN1 R7
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("7",70,30);
              break;
      case 8: remoteAddress = XBeeAddress64(PAN1_R8_SH, PAN1_R8_SL);     //PAN1 R8
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("8",70,30);
              break;           
      case 9: remoteAddress = XBeeAddress64(PAN1_R9_SH, PAN1_R9_SL);     //PAN1 R9
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("9",70,30);
              break;
      case 10: remoteAddress = XBeeAddress64(PAN1_R10_SH, PAN1_R10_SL);     //PAN1 R10
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("10",70,30);
              break;
      case 11: remoteAddress = XBeeAddress64(PAN1_R11_SH, PAN1_R11_SL);     //PAN1 R11
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("11",70,30);
              break;
      case 12: remoteAddress = XBeeAddress64(PAN1_R12_SH, PAN1_R12_SL);     //PAN1 R12
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("12",70,30);
              break;
      case 13: remoteAddress = XBeeAddress64(PAN1_R13_SH, PAN1_R13_SL);     //PAN1 R13
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("13",70,30);
              break;
      case 14: remoteAddress = XBeeAddress64(PAN1_R14_SH, PAN1_R14_SL);     //PAN1 R14
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("14",70,30);
              break;
      case 15: remoteAddress = XBeeAddress64(PAN1_R15_SH, PAN1_R15_SL);     //PAN1 R15
              EsploraTFT.setTextSize(5);
              EsploraTFT.stroke(255,255,255);
              EsploraTFT.fill(255,255,255);
              EsploraTFT.rect(0,0,W,70);
              EsploraTFT.stroke(0,0,0);
              EsploraTFT.text("15",70,30);
              break;
      default: EsploraTFT.setTextSize(5);
               EsploraTFT.stroke(255,255,255);
               EsploraTFT.fill(255,255,255);
               EsploraTFT.rect(0,0,W,70);
               EsploraTFT.stroke(0,0,0);
               EsploraTFT.text("Error",70,30);
               break;
    }
    robot_flag = 0;
  }
  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ロボットの移動・速さの切り替え
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_controlRobot(){
  //スピード切り替え//////////
  if((val[0] == LOW) && (old_val[0] == HIGH)){
    //speed_state1 = 0;
    if(speed_state == 2)speed_state=0;
    else speed_state++;
    
    if(speed_state == 0)speed = 16;
    else if(speed_state == 1)speed = 32;
    else if(speed_state == 2)speed = 64;
   // else if(speed_state == 3)speed = 64;
    //else if(speed_state == 4)speed = 80;
    //else if(speed_state == 5)speed = 96;
    //else if(speed_state == 6)speed = 112;
    //else if(speed_state == 7)speed = 128;
    speed_s = String(speed);
    speed_s.toCharArray(speed_c, 4);
  
    EsploraTFT.setTextSize(2);
    EsploraTFT.stroke(255,255,255);
    EsploraTFT.fill(255,255,255);
    EsploraTFT.rect(0, 70, W, H);
    EsploraTFT.stroke(0,0,0);
    EsploraTFT.text(speed_c,70,100);
    
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));    
    xbee.send(zbTx_con);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //add 2016/05/17
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  /*
  speed = Esplora.readSlider();
  
  Serial.println(speed);
  
  speed *= 172;
  speed /= 1023;
  
  if(speed != old_speed){
    speed_s = String(speed);
    speed_s.toCharArray(speed_c, 4);
    //EsploraTFT.stroke(0,0,0);
    
    EsploraTFT.setTextSize(2);
    EsploraTFT.stroke(255,255,255);
    EsploraTFT.fill(255,255,255);
    EsploraTFT.rect(0, 70, W, H);
    EsploraTFT.stroke(0,0,0);
    EsploraTFT.text(speed_c,70,100);
  }
  */

  else{
  //右//////////
  if((val[4] == LOW) && (old_val[4] == HIGH)){
    controller[Lmode] = 1;
    controller[Rmode] = 2;
    controller[Lspeed] = turn_speed;
    controller[Rspeed] = turn_speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  }
  
  else if((val[4] == HIGH) && (old_val[4] ==LOW)){
    controller[Lmode] = 0;
    controller[Rmode] = 0;
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  }
  
  
  //前//////////
  if((val[3] == LOW) && (old_val[3] == HIGH)){
    controller[Lmode] = 1;
    controller[Rmode] = 1;
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
     
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  }
  else if((val[3] == HIGH) && (old_val[3] ==LOW)){
    controller[Lmode] = 0;
    controller[Rmode] = 0;
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
     
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  }
  
  
  //後ろ//////////
  if((val[1] == LOW) && (old_val[1] == HIGH)){
    controller[Lmode] = 2;
    controller[Rmode] = 2;
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
     
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  }
  else if((val[1] == HIGH) && (old_val[1] ==LOW)){
    controller[Lmode] = 0;
    controller[Rmode] = 0;
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
     
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  }
  
  
  //左//////////
  if((val[2] == LOW) && (old_val[2] == HIGH)){
    controller[Lmode] = 2;
    controller[Rmode] = 1;
    controller[Lspeed] = turn_speed;
    controller[Rspeed] = turn_speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  }
  else if((val[2] == HIGH) && (old_val[2] ==LOW)){
    controller[Lmode] = 0;
    controller[Rmode] = 0;
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  }
  
  old_speed = speed;
  } 
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ロボットの速さ切り替え２
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_speedChenge(){
  
    if((val[3] == LOW) && (old_val[3] == HIGH)){
    if(speed_state1 == 7)speed_state1=0;
    else speed_state1++;
    
    if(speed_state1 == 0)speed = 16;
    else if(speed_state1 == 1)speed = 32;
    else if(speed_state1 == 2)speed = 48;
    else if(speed_state1 == 3)speed = 64;
    else if(speed_state1 == 4)speed = 80;
    else if(speed_state1 == 5)speed = 96;
    else if(speed_state1 == 6)speed = 112;
    else if(speed_state1 == 7)speed = 128;
    speed_s = String(speed);
    speed_s.toCharArray(speed_c, 4);
  
    EsploraTFT.setTextSize(2);
    EsploraTFT.stroke(255,255,255);
    EsploraTFT.fill(255,255,255);
    EsploraTFT.rect(0, 70, W, H);
    EsploraTFT.stroke(0,0,0);
    EsploraTFT.text(speed_c,70,100);
    
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));    
    xbee.send(zbTx_con);
    }
  else if((val[1] == LOW) && (old_val[1] == HIGH)){
    if(speed_state1 == 0)speed_state1=7;
    else speed_state1--;
    
    if(speed_state1 == 0)speed = 16;
    else if(speed_state1 == 1)speed = 32;
    else if(speed_state1 == 2)speed = 48;
    else if(speed_state1 == 3)speed = 64;
    else if(speed_state1 == 4)speed = 80;
    else if(speed_state1 == 5)speed = 96;
    else if(speed_state1 == 6)speed = 112;
    else if(speed_state1 == 7)speed = 128;
    speed_s = String(speed);
    speed_s.toCharArray(speed_c, 4);
  
    EsploraTFT.setTextSize(2);
    EsploraTFT.stroke(255,255,255);
    EsploraTFT.fill(255,255,255);
    EsploraTFT.rect(0, 70, W, H);
    EsploraTFT.stroke(0,0,0);
    EsploraTFT.text(speed_c,70,100);
    
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));    
    xbee.send(zbTx_con);
    }
  }
  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//old_valの更新
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_updateOldVal(){
  for(i=0;i<5;i++){
    old_val[i] = val[i];
  }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main Loop
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop () {
  
  agz_readButton();
  
  //アナログスティックを倒し、各機能へ
  if(Esplora.readJoystickY() < -400  && Esplora.readJoystickX() <300 && Esplora.readJoystickX() > -300) 
    {
      agz_modeChange();
    }
  else if(Esplora.readJoystickY() > 400 && Esplora.readJoystickX() <300 && Esplora.readJoystickX() > -300)
    {
    agz_robotChange();
    }
    
    //add  2016/05/17
    //右スティックを倒してボタンを押す
    else if(Esplora.readJoystickX() < -511 && Esplora.readJoystickY() <200 && Esplora.readJoystickY() > -200)
    {
    agz_speedChenge();  
    }
    
  //else if(analogRead(0) > 900)agz_emergencyStop();
  //else if(analogRead(0) < 100)agz_autoMode();
  else 
  {
  agz_controlRobot();
  }
  agz_updateOldVal();
  
  delay(20);
}
