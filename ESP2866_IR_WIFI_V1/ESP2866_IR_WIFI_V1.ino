/***********************************************************
1. Wifi and IR with Persistance  -- 02-09-2019
2. Serail Monitor baud=74880
    calls : Terturn Html pasge with button contrils
    1.  /SW<n>ON    -> Switch <n> ON
    2. /SW<n>OFF   -> Switch <n> OFF
    3. /SW<n>TGL   ->  Toggle switch <n>

    API Calls return OK on Sucess ( NO HTML Page generate)
    1.  /API/SW<n>ON    -> Switch <n> ON
    2. /API/SW<n>OFF   -> Switch <n> OFF
    3. /API/SW<n>TGL    ->  Toggle seitych <n>

    SCEEN
      1. /SCN/<PATERN>   -> patern with 0/1  0r H/L for each switch   
          ie ?SCN/10101  -> Switch on swith nos 1,3,5 an off 2,3

    CONFIG
    1. /CFG/INV/<0|1>    -> Inverse HI/Lo . (EPROM 63 rd byte)
      Updated: 30-03-2020 -> V1, Adding Config for Device Name,ssid,pwd,IP  using EPROM.
    2. /CFG/NAM/<8-32 char key>  (eprom 31-> 62)
    3. /CFG/SID/<8-32 chars ssid>  (Eprom 64-> 97)
    4. /CFG/PWD/<8-16 char key>  (eprom 98-> 111)
    5. /CFG/IPL/<Local IP for ESP> - max 16 bytes  (eprom 128->159)
   
    FACTORY / RESET:
     1. /FACRST   -> Facroty Reset.
     2. /RESET    -> Restart.

*************************************************************/
#include <IRrecv.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>

#define ST_DIGITAL 5
#define BT_PINS 2
#define MAX_TRY_SEC  8   // Max seconds to waite and try to Re/connect Wifi.

uint16_t RECV_PIN = 14; //D5

int EROM_FAC_RESET=30;
int EROM_DEV_NAME=31;
int EROM_INV_VAL=63;
int EROM_SSD_VAL=64;
int EROM_PWD_VAL=98;
int EROM_IPL_VAL=128;

IRrecv irrecv(RECV_PIN);
decode_results results;

// IR Codes                    
long RCODE[10]={0x1FEE01F,0x1FE50AF,0x1FED827,0x1FEF807,0x1FE30CF,0x1FEB04F,0x1FE708F,0x1FE00FF,0x1FEF00F,0x1FE9867};  
String OnSw[ST_DIGITAL]={"SW1","SW2","SW3","SW4","SW5"};

int st_Digital[ST_DIGITAL]={16,5,4,12,13}; //D0,D1,D2,D6,D7
bool va_Digital[ST_DIGITAL]={0,0,0,0,0};
bool swval[ST_DIGITAL] = {0,0,0,0,0};
String sON="On";
String sOFF="Off";
int EEPROM_SIZE=168;
int STORE_DIGITAL=0;   // eprom 0->4 : Pin value digtal
int  devstate=0;
int HI=1;
int LO=0;
int k=0;
int inv_op=1;

//use your wifi ssid and password
 String ssid = "nagesh";
String password = "90000109031";
String deviceName="NAG_ESP66_063";
byte ip1=192;
byte ip2=168;
byte ip3=0;
byte ip4=63;


IPAddress subnet(255, 255, 255, 0);
WiFiServer server(80);

void setup() { 
 irrecv.enableIRIn();  // Start the receiver
 Serial.begin(74880);
 delay(10); 
 EEPROM.begin(EEPROM_SIZE);  
 init_pins();
  if(check_reset()==0) //Default get config details from eprom
  {  // set_reset(1);  
     get_ROM_config(); 
     //  Serial.println("use Eprom config");         
  }
  else { // Set Factory settings to Eprom and release reset flag..
     set_reset(0);  
     set_ROM_config();  
     // Serial.println("set Eprom config,use factory.");        
     }
  IPAddress local_IP(ip1, ip2,ip3, ip4);
  IPAddress gateway(ip1, ip2,ip3, 1);
  WiFi.hostname(deviceName); 
  // Connect to WiFi network
  if (!WiFi.config(local_IP, gateway, subnet)) {
     Serial.println("STA Failed to configure");
  }
  WiFi.mode(WIFI_STA); //you can use different modes, also make sure to change the mode in the rewifi() function
  delay(2000); //the delay. Incase of electricy cut-off, when power returns, the router takes about 15sec to make the wifi functional.

  WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED && k<=MAX_TRY_SEC) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++k); Serial.print('.');
 }
  IPAddress IP = WiFi.localIP();
  Serial.print("IP:");Serial.print(IP);Serial.print(",SSID:");
  Serial.println(ssid); Serial.print("Device:");Serial.println(deviceName);
  server.begin();
}
 
void toggle(int i){  //toggle
  int op=digitalRead(st_Digital[i]);  
  setpin(i,!op);   
}
void allset(int si){ //0: All low ,9/1: all High
  for(int i=0;i<ST_DIGITAL;i++){
    setpin(i,si); 
   }
 }
void setpin(int idx,int val)
{  digitalWrite(st_Digital[idx], val);
   EEPROM.write(STORE_DIGITAL+idx,byte(val));  
   EEPROM.commit();
   //Serial.print(idx); Serial.print(":"); Serial.println(val);
   swval[idx]=val;
}
void rewifi(){
  int count=0;
  if(WiFi.status() != WL_CONNECTED) {
     Serial.println("Re-Try:");
     WiFi.disconnect();
     delay(500);     
     WiFi.mode(WIFI_STA); //you can use different modes, also make sure to change the mode in the rewifi() function
      delay(1000); //you can change the delay. Incase of electricy cut-off, when power returns, the router takes about 15sec to make the wifi functional.
     WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED   && count<= MAX_TRY_SEC) { // Wait for the Wi-Fi to connect
    delay(1000);    
    Serial.print(++count); Serial.print('.');
  }
   IPAddress IP = WiFi.localIP();
  Serial.print("IP:");Serial.println(IP);
 }
}

void loop() {
  int op=0;
 //ir receiveing code, change with your ir codes
  if (irrecv.decode(&results)) {
   unsigned int ircode = results.value;    
     for(int i=0;i<10;i++)
     {
       if(ircode==RCODE[i]){//Serial.print(i);Serial.print("->");
       Serial.println(ircode,HEX);     
        if(i==0)allset(LO); 
        else if(i==9)  allset(HI);     
        else toggle(((i-1)%(ST_DIGITAL+1)));
      }  
    }
     irrecv.resume();  // Receive the next value
  }

if(WiFi.status() != WL_CONNECTED)rewifi(); // Try reconnect if not connected

  // Check if a client has connected
  WiFiClient client = server.available();
  if (client) {  
  // Read the first line of the request
  String request = client.readStringUntil('\r');
  Serial.print(request);

  // Display html page
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(); //  do not forget this one          

 // Return the response j th Switch
 for(int j=0;j<ST_DIGITAL;j++){  
  if (request.indexOf(OnSw[j]+"ON") > 0)  {  
    Serial.println("On "+OnSw[j]);
     setpin(j, HI);   
    }
  else if (request.indexOf(OnSw[j]+"OFF")>0)  {   
    Serial.println("Off "+OnSw[j]); 
    setpin(j, LO);   
   }
  else if (request.indexOf(OnSw[j]+"TGL")>0)  {   
    op=digitalRead(st_Digital[j]); 
    op=!op;   
    if(op==HI)Serial.print("On ");
    else Serial.print("Off ");
    Serial.println(OnSw[j]);
     setpin(j, op);   
    }   
 }
 
  if (request.indexOf("ALLON") >0){
     allset(HI); 
    Serial.println("All On");  
    } 
 else if (request.indexOf("ALLOFF")>0){
     allset(LO); 
    Serial.println("All Off");
    }

  //GET /CFG/<CMD>/<VAL> 
 if (request.indexOf("/CFG/")>0)
 {  
  int Sidx=request.indexOf("/",8)+1;
  int Eidx=request.indexOf(" ",8);
  String cmd=request.substring(Sidx,Sidx+3);
  String pattern="";
  int addr=0;
  int  val=0;
 // Serial.println("cmd="+cmd);
  if(Sidx+4<Eidx){
  pattern=request.substring(Sidx+4,Eidx);
 // Serial.println("val="+pattern);
  }
    if (cmd=="INV")
     {
       if(pattern=="1" || pattern=="H"){ val=1; HI=0;LO=1;sON="On";sOFF="Off";  }
       else if(pattern=="0" || pattern=="L"){ val=0; HI=1;LO=0;sON="Off";sOFF="On"; }
      addr=-1;
       setEPRByte(EROM_INV_VAL,val);
       client.println(val);
     }
    else  if (cmd=="SID")
     {
      addr=EROM_SSD_VAL;      
     }
    else  if (cmd=="PWD")
     {
      addr=EROM_PWD_VAL;      
     }
    else  if (cmd=="IPL")
     {
      addr=EROM_IPL_VAL;      
     } 
    else  if (cmd=="DEV")
     {
      addr=EROM_DEV_NAME;     
     }
      if(addr>0){
       if(pattern!="")
        {writeEPRSrt(addr,pattern);
      //   client.print("EPROM Updated:"); 
        }
       else
        {pattern=readEPRSrt(addr);
       //  client.print("EPROM Read:"); 
        }
        client.println(pattern); 
      }
      
 } //end of /CFG/ 
  else if (request.indexOf("/FACRST")>0)
  {
    Serial.println("Factory Reset !.. Rebooting..");
    client.println("Factory Reset");    
    set_reset(1);
    delay(10);
    ESP.restart();
   delay(10);   
   }
   else if (request.indexOf("/RESET")>0)
  {    
    ESP.restart();
  
   }
 else  if (request.indexOf("/GETINV")>0)
 {
    client.println(getEPRByte(EROM_INV_VAL));
 }
 else  if (request.indexOf("/GETPINS")>0)
 {
  String pins=getPinStatus(0);
    client.println(pins);
 }
 else  if (request.indexOf("/API")>0) // already the SW<n> is processed above.
 {
    client.println("OK");
 }
 else if (request.indexOf("/SCN")>0)
 {
  int lidx=request.indexOf("/",7)+1;
  String pattern=request.substring(lidx,lidx+ST_DIGITAL);  
  //Serial.println(pattern);
  for(int i=0;i<ST_DIGITAL || pattern[i]==' ';i++ ){
      if(pattern[i]=='1' || pattern[i]=='H'){ setpin(i, HI);  }
      else { setpin(i,LO); }
    }
  client.println("OK");
 }
 else {  
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
  client.println("<title>"+deviceName+"</title>");
  client.println("<Body>");
  
  for(int j=0;j<ST_DIGITAL;j++){     
   client.println("<br>");
   client.println(OnSw[j]+":<a href=\"/" + OnSw[j] + "ON\"><button>"+sON+"</button></a>");  
   client.println("<a href=\"/"+ OnSw[j] + "OFF\"><button>"+sOFF+"</button></a>");
   client.println("<a href=\"/"+ OnSw[j] + "TGL\"><button>Tgl</button></a>");    
   if(swval[j]==HI){client.print(sON);}
   else if(swval[j]==LO){client.print(sOFF);}
  } 
 client.println("<br>");
 client.println("ALL:<a href=\"/ALLON\"><button>" + sON + "</button></a>");
 client.println("<a href=\"/ALLOFF\"><button>" + sOFF + "</button></a>"); 
  client.println("</Body>"); 
 client.println("</html>");
}
 client.println();
 //client.close();
  }
  //delay(10);
}

void setEPRByte(int addr,int val)
{  
   EEPROM.write(addr,byte(val)); 
   delay(10); 
   EEPROM.commit();  
   delay(10);
}

int getEPRByte(int addr)
{
  return EEPROM.read(addr);    
}

void writeEPRSrt(int add,String data)
{
  int _size = data.length();
  int i;
  for(i=0;i<_size && i<32;i++)  //Max 32 Bytes
  {
    EEPROM.write(add+i,data[i]);
  }
  EEPROM.write(add+_size,'\0');   //Add termination null character for String Data
  delay(10);
  EEPROM.commit();
  delay(10);
}

String readEPRSrt(int add)
{
  int i;
  char data[32]; //Max 32 Bytes
  int len=0;
  unsigned char k;
  k=EEPROM.read(add);
  while(k != '\0' && len<32)   //Read until null character
  {    
    k=EEPROM.read(add+len);
    data[len]=k;
    len++;
  }
  data[len]='\0';
  return String(data);
 //return data;
}

String getPinStatus(int id)
{
  String pins="000000000";
 if(id<0 || id>ST_DIGITAL-1)return "NA";
  if(id==0){
   for(int i=0;i<ST_DIGITAL;i++)pins[i]=(swval[i]==1)?'1':'0';  
   return pins.substring(0,ST_DIGITAL);  
  }
  return (swval[id-1]==0)?"0":"1";
}

void init_pins(){  
  int val=0;
  int inv_val=0;
  for(int i=0;i<ST_DIGITAL;i++)pinMode(st_Digital[i],OUTPUT);

   inv_val=EEPROM.read(EROM_INV_VAL);     
   if(inv_val==1){HI=0;LO=1;sON="On";sOFF="Off";}
   else {HI=1;LO=0;sON="Off";sOFF="On";}
   Serial.print(",Hi=");Serial.print(HI);Serial.print(",LO=");Serial.print(LO);Serial.print(",INV=");Serial.println(inv_val);

   for(int j=0;j<ST_DIGITAL;j++){         
    val=byte(EEPROM.read(STORE_DIGITAL+j));//EEPROM.readByte(STORE_DIGITAL+j); //byte(EEPROM.read(STORE_DIGITAL+j)))
    swval[j]=val;
   digitalWrite(st_Digital[j], val);
   }      
}

void  get_ROM_config()
{
 String ipstr=readEPRSrt(EROM_IPL_VAL);
 ssid = readEPRSrt(EROM_SSD_VAL);
 password = readEPRSrt(EROM_PWD_VAL);
 deviceName=readEPRSrt(EROM_DEV_NAME);
 //char ip[5];
 byte ip[5]={0,0,0,0};
 strToIpSeg(ipstr,ip);
// retstr.toCharArray(ip,4);
ip1=(byte)ip[0];
ip2=(byte)ip[1];
ip3=(byte)ip[2];
ip4=(byte)ip[3];
}

void  set_ROM_config()
{
 writeEPRSrt(EROM_SSD_VAL,ssid);
 writeEPRSrt(EROM_PWD_VAL,password);
 writeEPRSrt(EROM_DEV_NAME,deviceName);
 String ipstr=String(ip1)+"."+String(ip2)+"."+String(ip3)+"."+String(ip4);
 writeEPRSrt(EROM_IPL_VAL,ipstr);
}

void strToIpSeg(String tip4,byte* ip)
{
  int st=0;
  int en=0;
  int len=tip4.length(); 
  for(int i=0;i<4 ;i++){
 en=tip4.indexOf(".",st+1);
 if(en>len)break;
 ip[i]=(byte)tip4.substring(st,en).toInt();
 // Serial.println((byte)ip[i]);
 st=en+1;
  }
 }

int check_reset()
{
  byte ret=getEPRByte(EROM_FAC_RESET);
 // Serial.println("RESET:");Serial.print(ret);
  if(ret==0)return 0;
return 1; 
}

void  set_reset(byte val)
{
  setEPRByte(EROM_FAC_RESET,val); 
}
