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

//for reset karlman filter
uint8_t reset_karman[11] = {'A','G','S','a','F','1','T','2','A','G','E'};


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
unsigned long speed, old_speed,turn_speed = 0,turn_dfspeed = 32;
unsigned long speed_array[7] = {32,48,64,80,96,112,128};
int speed_state = 0,speed_state1 = 0,turn_speed_state = 0;
String speed_s,speed_t;
char speed_c[3];
char count_c[2];



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

  //Esplora表示初期化
  EsploraTFT.begin();
  EsploraTFT.background(255,255,255);
  //ロボットの速度を32に設定
  speed = speed_array[0];
  speed_s = String(speed);
  speed_t = String(turn_speed);
  //表示用の関数を２つ作成
  showEsplora("1",70,25,5);
  showEsplora2(speed_t,40,99,2);
  showEsplora3(speed_s,120,99,2);
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

   //カルマンフィルタリセットモード
   /*
   if((val[1] == LOW) && (old_val[1] == HIGH)){
    mode[mode_select] = 3;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress,reset_karman,sizeof(reset_karman));
    xbee.send(zbTx_con);
    agz_resetKarman();
    }*/    
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
              showEsplora("1",70,25,5);              
              break;
      case 2: remoteAddress = XBeeAddress64(PAN1_R2_SH, PAN1_R2_SL);     //PAN1 R2
              showEsplora("2",70,25,5);              
              break;
      case 3: remoteAddress = XBeeAddress64(PAN1_R3_SH, PAN1_R3_SL);     //PAN1 R3
              showEsplora("3",70,25,5);   
              break;
      case 4: remoteAddress = XBeeAddress64(PAN1_R4_SH, PAN1_R4_SL);     //PAN1 R4
              showEsplora("4",70,25,5);   
              break;
      case 5: remoteAddress = XBeeAddress64(PAN1_R5_SH, PAN1_R5_SL);     //PAN1 R5
              showEsplora("5",70,25,5);            
              break;      
      case 6: remoteAddress = XBeeAddress64(PAN1_R6_SH, PAN1_R6_SL);     //PAN1 R6
              showEsplora("6",70,25,5);            
              break;
      case 7: remoteAddress = XBeeAddress64(PAN1_R7_SH, PAN1_R7_SL);     //PAN1 R7
              showEsplora("7",70,25,5);
              break;
      case 8: remoteAddress = XBeeAddress64(PAN1_R8_SH, PAN1_R8_SL);     //PAN1 R8
              showEsplora("8",70,25,5);
              break;           
      case 9: remoteAddress = XBeeAddress64(PAN1_R9_SH, PAN1_R9_SL);     //PAN1 R9
              showEsplora("9",70,25,5);             
              break;
      case 10: remoteAddress = XBeeAddress64(PAN1_R10_SH, PAN1_R10_SL);  //PAN1 R10
               showEsplora("10",70,25,5);              
               break;
      case 11: remoteAddress = XBeeAddress64(PAN1_R11_SH, PAN1_R11_SL);  //PAN1 R11
               showEsplora("11",70,25,5);              
               break;
      case 12: remoteAddress = XBeeAddress64(PAN1_R12_SH, PAN1_R12_SL);  //PAN1 R12
               showEsplora("12",70,25,5);  
               break;
      case 13: remoteAddress = XBeeAddress64(PAN1_R13_SH, PAN1_R13_SL);  //PAN1 R13
               showEsplora("13",70,25,5);           
               break;
      case 14: remoteAddress = XBeeAddress64(PAN1_R14_SH, PAN1_R14_SL);  //PAN1 R14
               showEsplora("14",70,25,5);            
               break;
      case 15: remoteAddress = XBeeAddress64(PAN1_R15_SH, PAN1_R15_SL);  //PAN1 R15
               showEsplora("15",70,25,5);            
               break;
      default: showEsplora("Error",70,25,5);
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
    if(turn_speed_state == 2)turn_speed_state=0;
    else turn_speed_state++;
    
    if(turn_speed_state == 0)turn_speed = 0;
    else if(turn_speed_state == 1)turn_speed = 16;
    else if(turn_speed_state == 2)turn_speed = 32;
   
    //else if(speed_state == 3)turn_speed = 64;
    //else if(speed_state == 4)turn_speed = 80;
    //else if(speed_state == 5)turn_speed = 96;
    //else if(speed_state == 6)turn_speed = 112;
    //else if(speed_state == 7)turn_speed = 128;
    speed_s = String(turn_speed);
    showEsplora2(speed_s,40,99,2);
    
  /*
    controller[Lspeed] = turn_speed;
    controller[Rspeed] = turn_speed;
    
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));    
    xbee.send(zbTx_con);
    */
  }

  
  else{
    //右//////////
  if((val[4] == LOW) && (old_val[4] == HIGH)){
   LRmotor(1,1,turn_speed,speed);
  }  
  else if((val[4] == HIGH) && (old_val[4] ==LOW)){
    LRmotor(0,0,speed,speed);
  }
  
  //前//////////
  if((val[3] == LOW) && (old_val[3] == HIGH)){
    LRmotor(1,1,speed,speed);
  }
  else if((val[3] == HIGH) && (old_val[3] ==LOW)){
    LRmotor(0,0,speed,speed);
  }  
  
  //後ろ//////////
  if((val[1] == LOW) && (old_val[1] == HIGH)){
    LRmotor(2,2,speed,speed);
  }
  else if((val[1] == HIGH) && (old_val[1] ==LOW)){
    LRmotor(0,0,speed,speed);
  }
  
  //左//////////
  if((val[2] == LOW) && (old_val[2] == HIGH)){
      LRmotor(1,1,speed,turn_speed);
  }
  else if((val[2] == HIGH) && (old_val[2] ==LOW)){
  LRmotor(0,0,speed,speed);
  }
  
  old_speed = speed;
  
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ロボットの速さ切り替え２
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_speedChange(){
  
    if((val[3] == LOW) && (old_val[3] == HIGH)){
    if(speed_state1 == 6)speed_state1=0;
    else speed_state1++;
    
    switch(speed_state1){
      case 0:
            speed = 32;
            break;
      case 1:
            speed = 48;
            break;
      case 2:
            speed = 64;
            break;
      case 3:
            speed = 80;
            break;
      case 4:
            speed = 96;
            break;
      case 5:
            speed = 112;
            break;
      case 6:
            speed = 128;
            break;
    }
    speed_s = String(speed);
    showEsplora3(speed_s,120,99,2);
    
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));    
    xbee.send(zbTx_con);
    }
  else if((val[1] == LOW) && (old_val[1] == HIGH)){
    if(speed_state1  == 0)speed_state1=6;
    else speed_state1--;
    switch(speed_state1){
      case 0:
            speed = 32;
            break;
      case 1:
            speed = 48;
            break;
      case 2:
            speed = 64;
            break;
      case 3:
            speed = 80;
            break;
      case 4:
            speed = 96;
            break;
      case 5:
            speed = 112;
            break;
      case 6:
            speed = 128;
            break;
    }
    speed_s = String(speed);
    showEsplora3(speed_s,120,99,2);
    
    controller[Lspeed] = speed;
    controller[Rspeed] = speed;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));    
    xbee.send(zbTx_con);
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//カルマンフィルタのリセット
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_resetKarman(){
  showEsplora("RESET",22,35,4);
  delay(1000);
  String str;
  
  for (int i = 10; i > 0; i--){
  str = String(i);
  showEsplora(str,70,30,5);
  delay(500);
  }
  
  str = String(robot_id);
  showEsplora("OK",60,40,4);
  delay(1000);
  
  showEsplora(str,70,30,5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//バック旋回
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_Back(){
  
  //右//////////
  if((val[4] == LOW) && (old_val[4] == HIGH)){
   LRmotor(2,2,turn_speed,speed);
  }  
  else if((val[4] == HIGH) && (old_val[4] ==LOW)){
    LRmotor(0,0,speed,speed);
  }
  
  //前//////////
  if((val[3] == LOW) && (old_val[3] == HIGH)){
    LRmotor(1,1,speed,speed);
  }
  else if((val[3] == HIGH) && (old_val[3] ==LOW)){
    LRmotor(0,0,speed,speed);
  }  
  
  //後ろ//////////
  if((val[1] == LOW) && (old_val[1] == HIGH)){
    LRmotor(2,2,speed,speed);
  }
  else if((val[1] == HIGH) && (old_val[1] ==LOW)){
    LRmotor(0,0,speed,speed);
  }
  
  //左//////////
  if((val[2] == LOW) && (old_val[2] == HIGH)){
      LRmotor(2,2,speed,turn_speed);
  }
  else if((val[2] == HIGH) && (old_val[2] ==LOW)){
  LRmotor(0,0,speed,speed);
  }
  
  old_speed = speed;
  
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//左右のモーターの設定
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LRmotor(int lnum,int rnum, int lspeed, int rspeed){
  
      controller[Lmode] = lnum;
    controller[Rmode] = rnum;
    controller[Lspeed] = lspeed;
    controller[Rspeed] = rspeed;
    if(lspeed <= 0) controller[Lspeed] = 0;
    if(rspeed <= 0) controller[Rspeed] = 0;
    
    ZBTxRequest zbTx_con = ZBTxRequest(remoteAddress, controller, sizeof(controller));
    xbee.send(zbTx_con);
  
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//old_valの更新
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void agz_updateOldVal(){
  for(i=0;i<5;i++){
    old_val[i] = val[i];
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Esplora上に表示するための関数
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void showEsplora( String s,int w,int h,int strS){
  s.toCharArray(count_c,6);
  
  EsploraTFT.stroke(255,255,255);
  EsploraTFT.fill(255,255,255);
  EsploraTFT.rect(0,0,W,70);
  EsploraTFT.stroke(0,0,0);
  EsploraTFT.setTextSize(2);
  EsploraTFT.text("Number",w-20,h-10);
  EsploraTFT.setTextSize(strS);
  EsploraTFT.text(count_c,w,h+10);
  }
  
void showEsplora2(String s,int w,int h,int strS){
  String str;
  s.toCharArray(speed_c,6) ;
  EsploraTFT.stroke(255,255,255);
  EsploraTFT.fill(255,255,255);
  EsploraTFT.rect(0,70,W/2,H);
  EsploraTFT.stroke(0,0,0);
  EsploraTFT.setTextSize(1);
  EsploraTFT.text("turn speed",w-20,h - 10);
  EsploraTFT.setTextSize(strS);
  EsploraTFT.text(speed_c,w,h + 10);
 
  }
  void showEsplora3(String s,int w,int h,int strS){
  String str;
  s.toCharArray(speed_c,6);

  EsploraTFT.stroke(255,255,255);
  EsploraTFT.fill(255,255,255);
  EsploraTFT.rect(W/2,70,W,H);
  EsploraTFT.stroke(0,0,0);
  EsploraTFT.setTextSize(1);
  EsploraTFT.text("speed",w-5,h - 10);
  EsploraTFT.setTextSize(strS);
  EsploraTFT.text(speed_c,w,h + 10);
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main Loop
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop () {
  
  agz_readButton();
  
  //アナログスティックを倒し、各機能へ
  if(Esplora.readJoystickY() < -300  && Esplora.readJoystickX() < 200 && Esplora.readJoystickX() > -200) 
    {
      agz_modeChange();
    }
  else if(Esplora.readJoystickY() > 400 && Esplora.readJoystickX() <300 && Esplora.readJoystickX() > -300)
    {
      agz_robotChange();
    }
    
  //add  2016/05/17
  //左スティックを右に倒してボタンを押し、速度を変える
  else if(Esplora.readJoystickX() < -400 && Esplora.readJoystickY() <300 && Esplora.readJoystickY() > -300)
    {
      agz_speedChange();  
    }
    //左スティック左に倒してボタンを押し、バック旋回を行う(バック旋回中も前進、後退可能)
  else if(Esplora.readJoystickX() > 400 && Esplora.readJoystickY() <300 && Esplora.readJoystickY() > -300){
      agz_Back();
      }
  else 
    {
      agz_controlRobot();
    }
  agz_updateOldVal();
  
  delay(30);
}


