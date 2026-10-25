// LoRa Field Test na Heltec Vision Master T190 (ESP32-S3 + SX1262 + TFT 1,9" 170x320, dwa przyciski, bez dotyku).
// Samodzielny tester z wlasnym radiem (NIE jest to firmware Meshtastic):
//   - SZUM: prawdziwy pomiar chwilowego RSSI radia (10x/s), podloga, szczyty, zajetosc kanalu, okna 1 min / 10 min / 1 h
//   - SKAN: przemiatanie pasma 863-870 MHz z zapamietaniem szczytow
//   - MESH: pasywny podsluch pakietow Meshtastic (naglowki jawne; nazwy wezlow tylko dla domyslnego klucza kanalu)
#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <TFT_eSPI.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "config.h"
#include "presets.h"
#include "noiselog.h"
#include "quality.h"
#include "meshhdr.h"

// ================= Stan globalny =================
TFT_eSPI tft;
TFT_eSprite spr=TFT_eSprite(&tft);
bool sprOk=false;
SPIClass loraSpi(FSPI);
SX1262 radio=new Module(PIN_LORA_CS,PIN_LORA_DIO1,PIN_LORA_RST,PIN_LORA_BUSY,loraSpi);
Preferences prefs;

struct Cfg{uint8_t power=2,meshPreset=3,invert=1,flip=0,pwrHigh=0,beacon=2,autoPing=1,ble=0;} cfg;   // beacon: indeks odstepu (0 = wylaczony), autoPing: ping wybranego wezla co 10 s
// Bluetooth (konfigurator przez Web Bluetooth): definicje ponizej, przed obsluga przyciskow
uint32_t blePin=0;volatile bool bleCfgDirty=true;void bleSet(bool on);
enum Screen{S_HOME,S_NOISE,S_SCAN,S_MESH,S_NODE,S_SET,S_COUNT};
enum Mode{M_MESH,M_SCAN};
const char* SCR_NAME[S_COUNT]={"HOME","NOISE","SCAN","MESH","NODE","SETUP"};
int screen=S_HOME;Mode mode=M_MESH;bool dirty=true;uint32_t lastDraw=0;

struct Prof{float mhz,bw;uint8_t sf,cr,sync;uint16_t pre;int8_t pwr;};
Prof prof={869.525f,250,11,5,MESH_SYNC,MESH_PREAMBLE,14};
bool radioOk=false;int16_t rlErr=0;

pre::Duty duty;bool dutyBlocked=false;
nz::Log nzMesh;
mh::Table mtab;mh::Air meshAir;uint32_t meshRx=0,crcErrors=0;
bool txBusy=false;uint32_t txDeadline=0;
uint32_t lastNoiseAt=0,lastBatAt=0;float batV=0;int batPct=-1;bool batChg=false;float batPrev=0;
constexpr int BAT_OFF_PCT=5;constexpr uint8_t BAT_OFF_READS=3;   // wylaczenie: ponizej 5% przez 3 kolejne odczyty (ok. 30 s), bez ladowania
uint8_t batLowCnt=0;RTC_DATA_ATTR bool rtcLowOff=false;RTC_DATA_ATTR float rtcMinV=9.f;
unsigned noiseWin=0;                       // 0 = 1 min, 1 = 10 min, 2 = 1 h
char nodeName[13]="";   // nazwa tego wezla (wpisywana przez BLE), max 12 znakow ASCII
struct Pos{int32_t lat=0,lon=0,alt=0;bool has=false;} gpos;   // wpisana recznie pozycja GPS (1e-7 stopnia)
uint32_t nextBeaconAt=0;unsigned beaconsSent=0;bool beaconPosDue=false;uint32_t beaconPosAt=0;
bool meshPicked=false;uint32_t lastPingTry=0;constexpr uint32_t AUTO_PING_MS=10000;
unsigned meshOff=0,setSel=0;uint32_t meshSelId=0,myId=0;bool scanPaused=false;
int8_t scanCur[SCAN_N],scanPeak[SCAN_N];unsigned scanIdx=0;

// Ping wezla Meshtastic (pakiet z want_ack, odpowiedz ROUTING_APP)
enum PingState{PG_IDLE,PG_WAIT,PG_ACK,PG_NAK,PG_TIMEOUT,PG_BLOCKED};
struct Ping{uint32_t target=0,id=0,sentAt=0;PingState st=PG_IDLE;unsigned sent=0,acks=0;int rssi=0,hops=-1;float snr=0;uint32_t rtt=0;uint8_t err=0;} pingSt;
constexpr uint32_t PING_TIMEOUT_MS=12000;
nz::Log& nzCur(){return nzMesh;}
int lastRssi=0;float lastSnr=0;uint32_t lastRxAt=0;bool hasRx=false;   // ostatni odebrany pakiet

// ================= Kolory (paleta jak w CYD) =================
constexpr uint16_t C565(uint8_t r,uint8_t g,uint8_t b){return uint16_t(((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3));}
const uint16_t cBG=C565(0,34,22),cHDR=C565(0,18,12),cPANEL=C565(6,56,40),cLINE=C565(30,170,120),cTRACK=C565(14,84,60),cBAND=C565(10,110,92),
 cTURQ=C565(0,255,224),cGREEN=C565(30,255,120),cAMBER=C565(255,190,0),cRED=C565(255,56,44),cDIM=C565(140,230,200),cWHITE=C565(240,255,250),
 cGREY=C565(170,190,182),cOKBG=C565(4,78,46),cWARNBG=C565(80,54,0),cBADBG=C565(86,14,10);

// ================= Radio =================
#define RL(x) do{int16_t e_=(x);if(e_!=RADIOLIB_ERR_NONE&&!rlErr)rlErr=e_;}while(0)
Prof profFor(Mode m){
 if(m==M_SCAN)return Prof{SCAN_START,125,7,5,SCAN_SYNC,8,2};
 const pre::Preset& p=pre::get(cfg.meshPreset);float f=pre::meshFreq(cfg.meshPreset);int pw=pre::linkPower(cfg.power);if(pw>pre::powerLimit(f))pw=pre::powerLimit(f);return Prof{f,p.bw,p.sf,p.cr,MESH_SYNC,MESH_PREAMBLE,int8_t(pw)};}
void radioApply(const Prof& p){
 if(!radioOk)return;prof=p;txBusy=false;rlErr=0;
 RL(radio.standby());RL(radio.setBandwidth(p.bw));RL(radio.setSpreadingFactor(p.sf));RL(radio.setCodingRate(p.cr));RL(radio.setSyncWord(p.sync));RL(radio.setPreambleLength(p.pre));
 RL(radio.setFrequency(p.mhz));RL(radio.setOutputPower(p.pwr));RL(radio.startReceive());}
void switchMode(Mode m){mode=m;radioApply(profFor(m));lastNoiseAt=millis();}
Mode modeFor(int s){return s==S_SCAN?M_SCAN:M_MESH;}
void radioInit(){
 loraSpi.begin(PIN_LORA_SCK,PIN_LORA_MISO,PIN_LORA_MOSI,PIN_LORA_CS);
 Prof p=profFor(M_MESH);prof=p;
 int16_t e=radio.begin(p.mhz,p.bw,p.sf,p.cr,p.sync,p.pwr,p.pre,LORA_TCXO_V,false);
 if(e!=RADIOLIB_ERR_NONE){rlErr=e;radioOk=false;Serial.printf("RADIO begin error %d\n",int(e));return;}
 radioOk=true;RL(radio.setDio2AsRfSwitch(true));RL(radio.setCurrentLimit(140.0f));RL(radio.setRxBoostedGainMode(true));RL(radio.startReceive());
 Serial.printf("RADIO OK %.3f MHz SF%u BW%.1f\n",p.mhz,p.sf,p.bw);}
uint32_t toaMs(size_t len){return uint32_t(radio.getTimeOnAir(len)/1000)+1;}
bool txStart(const uint8_t* b,size_t n,uint32_t now){
 if(!radioOk||txBusy||mode==M_SCAN)return false;uint32_t air=toaMs(n);
 if(!duty.allow(now,air,pre::dutyLimit(prof.mhz))){dutyBlocked=true;return false;}
 int16_t e=radio.startTransmit(b,n);if(e!=RADIOLIB_ERR_NONE){rlErr=e;radio.startReceive();return false;}
 dutyBlocked=false;duty.add(now,air);txBusy=true;txDeadline=now+air+250;return true;}
void txPoll(uint32_t now){if(!txBusy)return;if(digitalRead(PIN_LORA_DIO1)||int32_t(now-txDeadline)>0){radio.finishTransmit();txBusy=false;radio.startReceive();}}

// ================= Bateria =================
void batLowTrack(){bool low=batV>=2.5f&&(batPct<0||batPct<BAT_OFF_PCT)&&!batChg;if(low){if(batLowCnt<255)batLowCnt++;}else batLowCnt=0;}
void batRead(){analogSetPinAttenuation(PIN_BAT_ADC,ADC_2_5db);digitalWrite(PIN_BAT_CTRL,HIGH);delay(3);uint32_t mv=0;for(int i=0;i<8;i++)mv+=analogReadMilliVolts(PIN_BAT_ADC);digitalWrite(PIN_BAT_CTRL,LOW);
 batV=(mv/8)/1000.f*BAT_MULT;
 // no charge-status pin known: heuristic. ON: voltage rose >=20 mV since last read or >=4.18 V. OFF: fell >=20 mV, or 6 reads (60 s) without a rise while below 4.15 V
 static uint8_t flat=0;
 if(batPrev>0){float d=batV-batPrev;if(d>=0.02f||batV>=4.18f){batChg=true;flat=0;}else if(d<=-0.02f)batChg=false;else if(batChg&&batV<4.15f){if(++flat>=6)batChg=false;}else flat=0;}else batChg=batV>=4.18f;
 batPrev=batV;
 static const float V[]={4.20f,4.10f,4.00f,3.90f,3.80f,3.70f,3.60f,3.50f,3.30f};static const int P[]={100,90,80,65,50,30,15,5,0};
 if(batV<3.0f){batPct=-1;batLowTrack();return;}int p=0;if(batV>=V[0])p=100;else if(batV<=V[8])p=0;else for(int i=0;i<8;i++)if(batV<=V[i]&&batV>=V[i+1]){p=P[i+1]+int((batV-V[i+1])/(V[i]-V[i+1])*(P[i]-P[i+1]));break;}batPct=p;batLowTrack();}

// ================= Odbior pakietow =================
void onMeshFrame(const uint8_t* b,size_t n,float rssi,float snr,uint32_t now){
 mh::Hdr h;if(!mh::parse(b,n,h))return;if(h.from==myId)return;
 if(pingSt.st==PG_WAIT&&h.from==pingSt.target&&h.to==myId){uint8_t er=0;if(mh::ackFor(b,n,h,pingSt.id,&er)){pingSt.rssi=int(lroundf(rssi));pingSt.snr=snr;pingSt.rtt=now-pingSt.sentAt;pingSt.hops=mh::hops(h);pingSt.err=er;if(er==0){pingSt.st=PG_ACK;++pingSt.acks;}else pingSt.st=PG_NAK;dirty=true;
   Serial.printf("PING ack from=%08lx rssi=%.0f snr=%.1f rtt=%lu err=%u\n",(unsigned long)h.from,rssi,snr,(unsigned long)pingSt.rtt,unsigned(er));}}
 ++meshRx;hasRx=true;lastRssi=int(lroundf(rssi));lastSnr=snr;lastRxAt=now;mtab.add(h,b,n,int(lroundf(rssi)),snr,now);meshAir.add(now,uint32_t(radio.getTimeOnAir(n)));dirty=true;
 Serial.printf("MESH from=%08lx rssi=%.0f snr=%.1f hops=%d len=%u\n",(unsigned long)h.from,rssi,snr,mh::hops(h),unsigned(n));}
void rxPoll(uint32_t now){
 if(!radioOk||txBusy||mode==M_SCAN)return;if(!digitalRead(PIN_LORA_DIO1))return;
 uint8_t buf[256];size_t len=radio.getPacketLength();int16_t e=radio.readData(buf,len>sizeof buf?sizeof buf:len);float rssi=radio.getRSSI(),snr=radio.getSNR();radio.startReceive();
 if(e==RADIOLIB_ERR_CRC_MISMATCH){++crcErrors;return;}if(e!=RADIOLIB_ERR_NONE||!len||len>sizeof buf)return;
 onMeshFrame(buf,len,rssi,snr,now);}

// ================= Skan pasma =================
void scanReset(){memset(scanCur,nz::NONE,sizeof scanCur);memset(scanPeak,nz::NONE,sizeof scanPeak);scanIdx=0;}
void scanTick(){
 if(!radioOk||scanPaused)return;
 for(int k=0;k<4;k++){float f=SCAN_START+scanIdx*SCAN_STEP;RL(radio.setFrequency(f));RL(radio.startReceive());delay(2);float r=radio.getRSSI(false);int v=int(lroundf(r));if(v<-127)v=-127;if(v>0)v=0;
  scanCur[scanIdx]=int8_t(v);if(scanPeak[scanIdx]==nz::NONE||v>scanPeak[scanIdx])scanPeak[scanIdx]=int8_t(v);scanIdx=(scanIdx+1)%SCAN_N;}
 dirty=true;}

// ================= Ping i szum =================
void pingSend(uint32_t now,bool force=false){
 if(!radioOk||mode!=M_MESH||(pingSt.st==PG_WAIT&&!force))return;lastPingTry=now;
 unsigned idx[mh::Table::MAXN];unsigned n=mtab.order(now,idx);uint32_t target=0;for(unsigned i=0;i<n;i++)if(mtab.n[idx[i]].id==meshSelId)target=meshSelId;if(!target&&n)target=mtab.n[idx[0]].id;if(!target)return;
 if(target!=pingSt.target){pingSt.sent=pingSt.acks=0;}
 pingSt.target=target;uint32_t id=esp_random();if(!id)id=1;uint8_t pkt[40];size_t len=mh::buildPing(pkt,sizeof pkt,target,myId,id,mtab.defHash,0);
 pingSt.id=id;if(txStart(pkt,len,now)){pingSt.st=PG_WAIT;pingSt.sentAt=now;++pingSt.sent;Serial.printf("PING to=%08lx id=%08lx\n",(unsigned long)target,(unsigned long)id);}else pingSt.st=PG_BLOCKED;dirty=true;}
bool selNodeKnown(){for(unsigned i=0;i<mtab.used;i++)if(mtab.n[i].id==meshSelId)return true;return false;}
bool autoPingOn(){return cfg.autoPing&&meshPicked&&meshSelId&&selNodeKnown();}
void pingTick(uint32_t now){
 const uint32_t to=autoPingOn()?AUTO_PING_MS-1000:PING_TIMEOUT_MS;   // przy pingu automatycznym brak ACK stwierdzamy przed kolejnym pingiem
 if(pingSt.st==PG_WAIT&&!txBusy&&now-pingSt.sentAt>to){pingSt.st=PG_TIMEOUT;dirty=true;}
 if(autoPingOn()&&mode==M_MESH&&!txBusy&&now-lastPingTry>=AUTO_PING_MS)pingSend(now,true);}
// ---- Beacon: rozglaszanie NODEINFO (nazwa) i POSITION (wpisana pozycja) na domyslnym kluczu kanalu, zeby inne wezly Meshtastic nas widzialy ----
bool beaconForce=false;
void beaconNames(char* lng,size_t lc,char* sh,size_t sc){char id4[8];snprintf(id4,sizeof id4,"%04lx",(unsigned long)(myId&0xFFFF));
 if(nodeName[0]){snprintf(lng,lc,"%s",nodeName);size_t k=0;for(const char* c=nodeName;*c&&k<4&&k<sc-1;c++)if(*c!=' ')sh[k++]=*c;sh[k]=0;if(!k)snprintf(sh,sc,"%s",id4);}
 else{snprintf(lng,lc,"T190 %s",id4);snprintf(sh,sc,"%s",id4);}}
bool beaconSendInfo(uint32_t now){
 char lng[44],sh[8];beaconNames(lng,sizeof lng,sh,sizeof sh);uint8_t user[80];size_t un=mh::buildUser(user,sizeof user,myId,lng,sh);if(!un)return false;
 uint32_t id=esp_random();if(!id)id=1;uint8_t pkt[120];size_t n=mh::buildBroadcast(pkt,sizeof pkt,myId,id,mtab.defHash,4,user,un);if(!n)return false;
 if(!txStart(pkt,n,now))return false;++beaconsSent;Serial.printf("BEACON nodeinfo '%s' (%s) id=%08lx\n",lng,sh,(unsigned long)id);dirty=true;return true;}
bool beaconSendPos(uint32_t now){
 if(!gpos.has)return true;uint8_t pl[32];size_t pn=mh::buildPosition(pl,sizeof pl,gpos.lat,gpos.lon,gpos.alt);uint32_t id=esp_random();if(!id)id=1;
 uint8_t pkt[80];size_t n=mh::buildBroadcast(pkt,sizeof pkt,myId,id,mtab.defHash,3,pl,pn);if(!n)return false;if(!txStart(pkt,n,now))return false;Serial.printf("BEACON position %d,%d alt %d\n",int(gpos.lat),int(gpos.lon),int(gpos.alt));return true;}
void beaconSoon(uint32_t now){if(cfg.beacon){nextBeaconAt=now+3000;beaconForce=false;}}
void beaconNow(uint32_t now){beaconForce=true;nextBeaconAt=now;dirty=true;}
void beaconTick(uint32_t now){
 if(!radioOk||mode!=M_MESH||txBusy)return;
 if(beaconPosDue&&int32_t(now-beaconPosAt)>=0){if(beaconSendPos(now)||int32_t(now-beaconPosAt)>20000)beaconPosDue=false;else beaconPosAt=now+2000;return;}
 if((cfg.beacon||beaconForce)&&int32_t(now-nextBeaconAt)>=0){
  if(beaconSendInfo(now)){beaconForce=false;nextBeaconAt=now+pre::beaconMin(cfg.beacon)*60000UL;if(!cfg.beacon)nextBeaconAt=now+3600000UL;if(gpos.has){beaconPosDue=true;beaconPosAt=now+4000;}}
  else nextBeaconAt=now+30000;}}
void noiseTick(uint32_t now){
 if(!radioOk||mode==M_SCAN||txBusy||now-lastNoiseAt<NOISE_PERIOD_MS)return;lastNoiseAt=now;
 nzCur().add(now,radio.getRSSI(false));}

// ================= Rysowanie (wszystko do sprite'a 320x170, potem jeden push) =================
void txt(const String& s,int x,int y,int font,uint16_t fg,uint16_t bg,uint8_t datum=TL_DATUM){spr.setTextDatum(datum);spr.setTextColor(fg,bg);spr.drawString(s,x,y,font);spr.setTextDatum(TL_DATUM);}
void panel(int x,int y,int w,int h,uint16_t bg=cPANEL,uint16_t line=cLINE){spr.fillRoundRect(x,y,w,h,5,bg);spr.drawRoundRect(x,y,w,h,5,line);}
uint16_t qColor(qual::Q q){return q==qual::Q_GOOD?cGREEN:q==qual::Q_FAIR?cAMBER:q==qual::Q_BAD?cRED:cGREY;}
uint16_t qBg(qual::Q q){return q==qual::Q_GOOD?cOKBG:q==qual::Q_FAIR?cWARNBG:q==qual::Q_BAD?cBADBG:cPANEL;}
void chip(int x,int y,int w,const char* s,uint16_t fg,uint16_t bg){spr.fillRoundRect(x,y,w,13,6,bg);spr.drawRoundRect(x,y,w,13,6,fg);txt(s,x+w/2,y+3,1,fg,bg,TC_DATUM);}
// Poziomy pasek wartosci v w zakresie lo..hi
void bar(int x,int y,int w,int h,float v,float lo,float hi,uint16_t c){spr.fillRect(x,y,w,h,cTRACK);float t=(v-lo)/(hi-lo);if(t<0)t=0;if(t>1)t=1;int k=int(t*w);if(k>0)spr.fillRect(x,y,k,h,c);}
void tile(int x,int y,const char* title,uint16_t accent){panel(x,y,100,62);spr.fillCircle(x+8,y+8,2,accent);txt(title,x+15,y+4,1,cDIM,cPANEL);}
unsigned linkSf(){return pre::get(cfg.meshPreset).sf;}
unsigned winSec(){static const unsigned W[]={60,600,3600};return W[noiseWin%3];}
const char* winName(){static const char* N[]={"1 min","10 min","1 h"};return N[noiseWin%3];}

void drawHeader(){
 spr.fillRect(0,0,SCR_W,18,cHDR);spr.drawFastHLine(0,18,SCR_W,cLINE);
 spr.drawLine(5,11,9,6,cTURQ);spr.drawLine(9,6,14,12,cTURQ);const int nx[]={5,9,14},ny[]={11,6,12};for(int i=0;i<3;i++){spr.fillCircle(nx[i],ny[i],2,cTURQ);spr.fillCircle(nx[i],ny[i],1,cHDR);}
 txt(nodeName[0]?nodeName:"LoRa Field Test",22,1,2,cWHITE,cHDR);
 txt(SCR_NAME[screen],196,1,2,cTURQ,cHDR);
 if(!radioOk)chip(250,3,66,"RADIO ERR",cRED,cBADBG);
 else if(mode==M_MESH)chip(250,3,66,"LISTEN",cAMBER,cWARNBG);
 else chip(250,3,66,"SCAN",cAMBER,cWARNBG);}
void drawFooter(const char* l,const char* r){
 spr.fillRect(0,151,SCR_W,19,cHDR);spr.drawFastHLine(0,151,SCR_W,cLINE);
 txt(l,6,157,1,cDIM,cHDR);txt(r,314,157,1,cTURQ,cHDR,TR_DATUM);}

void drawHome(){
 const int X[]={4,110,216},Y[]={20,86};char b[40];uint32_t now=millis();const qual::Q q=qual::quality(hasRx,lastSnr,linkSf());
 // NODES
 {tile(X[0],Y[0],"NODES HEARD",cGREEN);snprintf(b,sizeof b,"%u",mtab.used);txt(b,X[0]+8,Y[0]+14,4,mtab.used?cGREEN:cGREY,cPANEL);
  snprintf(b,sizeof b,"pkts %lu  crc %lu",(unsigned long)mtab.packets,(unsigned long)crcErrors);txt(b,X[0]+8,Y[0]+42,1,cWHITE,cPANEL);
  if(hasRx){snprintf(b,sizeof b,"last %lus ago",(unsigned long)((now-lastRxAt)/1000));txt(b,X[0]+8,Y[0]+52,1,cDIM,cPANEL);}else txt("listening",X[0]+8,Y[0]+52,1,cDIM,cPANEL);}
 // SIGNAL (ostatni odebrany pakiet)
 {tile(X[1],Y[0],"LAST PACKET",cTURQ);if(hasRx){snprintf(b,sizeof b,"%d",lastRssi);txt(b,X[1]+8,Y[0]+14,4,qColor(q),cPANEL);txt("dBm",X[1]+8+spr.textWidth(b,4)+3,Y[0]+26,1,cDIM,cPANEL);
   snprintf(b,sizeof b,"SNR %.1f dB",lastSnr);txt(b,X[1]+8,Y[0]+42,1,cWHITE,cPANEL);txt(qual::qText(q),X[1]+8,Y[0]+52,1,qColor(q),cPANEL);}
  else{txt("--",X[1]+8,Y[0]+14,4,cGREY,cPANEL);txt("no packets yet",X[1]+8,Y[0]+42,1,cAMBER,cPANEL);}}
 // NOISE
 {tile(X[2],Y[0],"NOISE",cAMBER);nz::Stats s=nzMesh.stats(60);if(s.has){snprintf(b,sizeof b,"%d",s.now);txt(b,X[2]+8,Y[0]+14,4,cWHITE,cPANEL);txt("dBm",X[2]+8+spr.textWidth(b,4)+3,Y[0]+26,1,cDIM,cPANEL);
   snprintf(b,sizeof b,"floor %d dBm",s.floor);txt(b,X[2]+8,Y[0]+42,1,cWHITE,cPANEL);snprintf(b,sizeof b,"peak %d dBm",s.peak);txt(b,X[2]+8,Y[0]+52,1,cDIM,cPANEL);}
  else{txt("--",X[2]+8,Y[0]+14,4,cGREY,cPANEL);txt("measuring...",X[2]+8,Y[0]+42,1,cAMBER,cPANEL);}}
 // CHANNEL BUSY
 {tile(X[0],Y[1],"CHANNEL BUSY",cGREEN);nz::Stats s=nzMesh.stats(60);if(s.has){snprintf(b,sizeof b,"%.0f%%",s.busy);txt(b,X[0]+8,Y[1]+14,4,s.busy>50?cRED:s.busy>25?cAMBER:cGREEN,cPANEL);}else txt("--",X[0]+8,Y[1]+14,4,cGREY,cPANEL);
  txt("> floor +8 dB",X[0]+8,Y[1]+42,1,cDIM,cPANEL);snprintf(b,sizeof b,"my TX %.1f%%/h",duty.pct(now));txt(b,X[0]+8,Y[1]+52,1,cDIM,cPANEL);}
 // RADIO
 {tile(X[1],Y[1],"RADIO",cAMBER);const Prof p=profFor(M_MESH);txt(pre::get(cfg.meshPreset).name,X[1]+8,Y[1]+13,2,cWHITE,cPANEL);snprintf(b,sizeof b,"%.3f MHz",(double)p.mhz);txt(b,X[1]+8,Y[1]+31,1,cTURQ,cPANEL);
  snprintf(b,sizeof b,"SF%u BW%g kHz",p.sf,(double)p.bw);txt(b,X[1]+8,Y[1]+41,1,cDIM,cPANEL);snprintf(b,sizeof b,"TX %d dBm",int(p.pwr));txt(b,X[1]+8,Y[1]+51,1,cDIM,cPANEL);}
 // BATTERY
 {tile(X[2],Y[1],"BATTERY",cGREEN);if(batPct>=0){const bool low=batPct<=10&&!batChg;snprintf(b,sizeof b,"%.2f",batV);txt(b,X[2]+8,Y[1]+14,4,low?cRED:batPct<40?cAMBER:cGREEN,cPANEL);txt("V",X[2]+8+spr.textWidth(b,4)+3,Y[1]+26,1,low?cRED:cDIM,cPANEL);
   if(batChg)txt("CHARGING",X[2]+8,Y[1]+42,1,cGREEN,cPANEL);else{snprintf(b,sizeof b,"~%d%%",batPct);txt(b,X[2]+8,Y[1]+42,1,low?cRED:cWHITE,cPANEL);}}else{txt("--",X[2]+8,Y[1]+14,4,cGREY,cPANEL);txt("no reading",X[2]+8,Y[1]+42,1,cAMBER,cPANEL);}}}

void drawNoise(){
 const nz::Log& L=nzMesh;nz::Stats s=L.stats(winSec());char b[32];const char* LB[]={"NOW dBm","FLOOR dBm","PEAK dBm","BUSY %"};
 for(int i=0;i<4;i++){int x=4+i*79;panel(x,20,75,30);txt(LB[i],x+37,23,1,cDIM,cPANEL,TC_DATUM);
  if(s.has){if(i==0)snprintf(b,sizeof b,"%d",s.now);else if(i==1)snprintf(b,sizeof b,"%d",s.floor);else if(i==2)snprintf(b,sizeof b,"%d",s.peak);else snprintf(b,sizeof b,"%.0f",s.busy);}else snprintf(b,sizeof b,"--");
  txt(b,x+37,32,2,i==3&&s.has?(s.busy>50?cRED:s.busy>25?cAMBER:cGREEN):cWHITE,cPANEL,TC_DATUM);}
 panel(4,53,312,96);const int x0=34,x1=310,yt=58,yb=116,w=x1-x0;
 int lo=-130,hi=-60;if(s.has){lo=((s.floor-10)/10)*10-(s.floor-10<0?10:0);if(lo<-130)lo=-130;hi=((s.peak+5)/10)*10+10;if(hi>-30)hi=-30;if(hi-lo<30)hi=lo+30;}
 auto Y=[&](int v){return yb-(constrain(v,lo,hi)-lo)*(yb-yt)/(hi-lo);};
 for(int v=lo;v<=hi;v+=10){int y=Y(v);if(((v-lo)/10)%2==0||hi-lo<=40){for(int x=x0;x<=x1;x+=6)spr.drawPixel(x,y,cTRACK);snprintf(b,sizeof b,"%d",v);txt(b,30,y,1,cDIM,cPANEL,MR_DATUM);}}
 unsigned win=winSec();
 for(int cx=0;cx<w;cx++){unsigned a0=unsigned(uint64_t(w-1-cx)*win/w),a1=unsigned(uint64_t(w-cx)*win/w);if(a1<=a0)a1=a0+1;if(a0>=L.used)continue;if(a1>L.used)a1=L.used;
  int mn=0,mx=-200;long sum=0;unsigned n=0;bool any=false;int bs=0;for(unsigned a=a0;a<a1;a++){int i=L.idx(a);if(L.avg[i]==nz::NONE)continue;if(!any){mn=L.lo[i];any=true;}mn=std::min<int>(mn,L.lo[i]);mx=std::max<int>(mx,L.hi[i]);sum+=L.avg[i];bs+=L.busy[i];++n;}
  if(!any)continue;int x=x0+cx;spr.drawFastVLine(x,Y(mx),std::max(1,Y(mn)-Y(mx)),cBAND);spr.drawPixel(x,Y(int(sum/n)),cTURQ);
  int p=bs/int(n);int h=p<=0?1:std::max(1,p*14/100);spr.drawFastVLine(x,148-h,h,p>50?cRED:p>25?cAMBER:cGREEN);}
 if(!s.has)txt("collecting noise samples...",(x0+x1)/2,(yt+yb)/2-4,2,cDIM,cPANEL,MC_DATUM);
 spr.fillCircle(40,121,2,cTURQ);txt("avg",46,117,1,cDIM,cPANEL);spr.fillRect(76,119,6,4,cBAND);txt("min-max",86,117,1,cDIM,cPANEL);txt(String("time ")+winName(),310,117,1,cWHITE,cPANEL,TR_DATUM);
 txt("BUSY",8,136,1,cDIM,cPANEL);}

void drawScan(){
 panel(4,20,312,129);const int x0=34,x1=308,yt=26,yb=112;char b[40];
 auto Y=[&](int v){return yb-(constrain(v,NOISE_MIN,-40)-NOISE_MIN)*(yb-yt)/(-40-NOISE_MIN);};
 for(int v=-120;v<=-50;v+=20){int y=Y(v);for(int x=x0;x<=x1;x+=6)spr.drawPixel(x,y,cTRACK);snprintf(b,sizeof b,"%d",v);txt(b,30,y,1,cDIM,cPANEL,MR_DATUM);}
 auto X=[&](float f){return x0+int((f-SCAN_START)/(SCAN_STEP*(SCAN_N-1))*(x1-x0-3));};
 const float fm=pre::meshFreq(cfg.meshPreset);
 spr.drawFastVLine(X(fm)+1,yt,yb-yt,cAMBER);
 for(unsigned i=0;i<SCAN_N;i++){int x=X(SCAN_START+i*SCAN_STEP);if(scanCur[i]!=nz::NONE){int y=Y(scanCur[i]);spr.fillRect(x,y,3,yb-y+1,cBAND);spr.fillRect(x,y,3,2,cTURQ);}
  if(scanPeak[i]!=nz::NONE)spr.fillRect(x,Y(scanPeak[i])-1,3,2,cAMBER);}
 for(int f=863;f<=870;f++){int x=X(float(f));spr.drawFastVLine(x+1,yb+1,3,cDIM);snprintf(b,sizeof b,"%d",f);txt(b,x+1,yb+5,1,cDIM,cPANEL,TC_DATUM);}
 int cur=(scanIdx+SCAN_N-1)%SCAN_N;snprintf(b,sizeof b,"%.1f MHz %d dBm",SCAN_START+cur*SCAN_STEP,scanCur[cur]==nz::NONE?0:scanCur[cur]);
 txt(b,8,129,1,cWHITE,cPANEL);spr.fillRect(150,131,6,4,cBAND);txt("now",158,129,1,cDIM,cPANEL);spr.fillRect(184,131,6,2,cAMBER);txt("peak",194,129,1,cDIM,cPANEL);
 if(scanPaused)txt("PAUSED",312,129,1,cAMBER,cPANEL,TR_DATUM);
 spr.fillRect(8,140,6,2,cAMBER);snprintf(b,sizeof b,"Meshtastic channel %.3f MHz",fm);txt(b,17,137,1,cDIM,cPANEL);}

// Skraca tekst tak, by miescil sie w podanej szerokosci (w pikselach) danym fontem.
void fitText(char* s,int maxW,int font){size_t n=strlen(s);while(n>1&&spr.textWidth(s,font)>maxW){s[--n]=0;}}
const char* nodeLabel(const mh::Node& n,char* b,size_t cap){if(n.name[0])snprintf(b,cap,"%s",n.name);else snprintf(b,cap,"!%08lx",(unsigned long)n.id);return b;}
// Panel pingu wybranego wezla (na dole ekranow MESH i NODE)
void drawPingPanel(uint32_t now){
 char b[64];panel(4,130,312,20);uint16_t pc=cDIM;
 if(pingSt.target&&pingSt.target==meshSelId){
  snprintf(b,sizeof b,"PING !%08lx  sent %u  ack %u%s",(unsigned long)pingSt.target,pingSt.sent,pingSt.acks,autoPingOn()?"  AUTO 10s":"");txt(b,10,132,1,cWHITE,cPANEL);
  switch(pingSt.st){
   case PG_WAIT:snprintf(b,sizeof b,"waiting for ACK... %lu s",(unsigned long)((now-pingSt.sentAt)/1000));pc=cAMBER;break;
   case PG_ACK:snprintf(b,sizeof b,"ACK  %d dBm  SNR %.1f  %.1f s  hops %d",pingSt.rssi,pingSt.snr,pingSt.rtt/1000.0,pingSt.hops);pc=cGREEN;break;
   case PG_NAK:snprintf(b,sizeof b,"NAK (error %u)  %d dBm  SNR %.1f",unsigned(pingSt.err),pingSt.rssi,pingSt.snr);pc=cRED;break;
   case PG_TIMEOUT:snprintf(b,sizeof b,"no ACK (out of range or other key)");pc=cRED;break;
   case PG_BLOCKED:snprintf(b,sizeof b,"not sent: duty limit or radio busy");pc=cAMBER;break;
   default:snprintf(b,sizeof b,autoPingOn()?"auto ping in a moment":"hold KEY to ping");}
  txt(b,10,141,1,pc,cPANEL);}
 else txt(meshPicked?(cfg.autoPing?"pinging the selected node every 10 s":"hold KEY: ping the selected node"):(cfg.autoPing?"KEY: select a node (it will be pinged every 10 s)":"KEY: select a node, hold KEY: ping"),10,137,1,cDIM,cPANEL);}

void drawMesh(){
 uint32_t now=millis();char b[64];const pre::Preset& pr=pre::get(cfg.meshPreset);
 panel(4,20,312,26);snprintf(b,sizeof b,"%s  %.3f MHz  SF%u BW%g",pr.name,(double)pre::meshFreq(cfg.meshPreset),pr.sf,(double)pr.bw);txt(b,10,23,1,cWHITE,cPANEL);
 snprintf(b,sizeof b,"pkts %lu  nodes %u  air %.1f%%  crc %lu",(unsigned long)mtab.packets,mtab.used,meshAir.pct(now),(unsigned long)crcErrors);txt(b,10,34,1,cDIM,cPANEL);
 unsigned idx[mh::Table::MAXN];unsigned n=mtab.order(now,idx);
 if(!n){panel(4,50,312,99);txt("listening for Meshtastic packets...",160,86,2,cDIM,cPANEL,MC_DATUM);txt("(headers only, content is not decoded)",160,108,1,cDIM,cPANEL,MC_DATUM);return;}
 unsigned sel=0;bool found=false;for(unsigned i=0;i<n;i++)if(mtab.n[idx[i]].id==meshSelId){sel=i;found=true;}if(!found)meshSelId=mtab.n[idx[0]].id;
 if(sel<meshOff)meshOff=sel;if(sel>=meshOff+4)meshOff=sel-3;if(meshOff+4>n)meshOff=n>4?n-4:0;
 for(unsigned r=0;r<4&&meshOff+r<n;r++){const mh::Node& x=mtab.n[idx[meshOff+r]];int y=49+int(r)*20;bool isSel=meshOff+r==sel;uint16_t bg=isSel?cOKBG:cPANEL;panel(4,y,312,19,bg,isSel?cGREEN:cLINE);
  char nm[28];nodeLabel(x,nm,sizeof nm);fitText(nm,148,2);txt(nm,10,y+3,2,cWHITE,bg);
  uint16_t c=x.rssi>-90?cGREEN:x.rssi>-110?cAMBER:cRED;snprintf(b,sizeof b,"%d",x.rssi);txt(b,196,y+3,2,c,bg,TR_DATUM);
  snprintf(b,sizeof b,"%d",int(lroundf(x.snr)));txt(b,226,y+3,2,cDIM,bg,TR_DATUM);
  if(x.hops>=0)snprintf(b,sizeof b,"h%d",x.hops);else snprintf(b,sizeof b,"h?");txt(b,246,y+3,2,cDIM,bg,TR_DATUM);
  uint32_t age=(now-x.last)/1000;if(age<100)snprintf(b,sizeof b,"%lus",(unsigned long)age);else snprintf(b,sizeof b,"%lum",(unsigned long)(age/60));txt(b,310,y+3,2,cDIM,bg,TR_DATUM);}
 snprintf(b,sizeof b,"%u/%u",sel+1,n);txt(b,314,23,1,cDIM,cPANEL,TR_DATUM);
 drawPingPanel(now);}

// Ekran szczegolow wybranego wezla Meshtastic: dane z podsluchu + wynik pingu.
void drawNode(){
 uint32_t now=millis();char b[64];const mh::Node* x=nullptr;for(unsigned i=0;i<mtab.used;i++)if(mtab.n[i].id==meshSelId)x=&mtab.n[i];
 if(!x){panel(4,20,312,129);txt("no node selected",160,70,2,cDIM,cPANEL,MC_DATUM);txt("pick one on MESH (KEY: next node)",160,94,1,cDIM,cPANEL,MC_DATUM);return;}
 // naglowek: nazwa, identyfikator, liczba pakietow, wiek
 panel(4,20,312,34);char nm[28];nodeLabel(*x,nm,sizeof nm);fitText(nm,200,2);txt(nm,10,23,2,cWHITE,cPANEL);
 snprintf(b,sizeof b,"!%08lx  ch %02X%s",(unsigned long)x->id,unsigned(x->chan),x->mqtt?"  MQTT":"");txt(b,10,42,1,cDIM,cPANEL);
 uint32_t age=(now-x->last)/1000;if(age<100)snprintf(b,sizeof b,"%lu s ago",(unsigned long)age);else snprintf(b,sizeof b,"%lu min ago",(unsigned long)(age/60));txt(b,310,23,2,age>120?cAMBER:cGREEN,cPANEL,TR_DATUM);
 snprintf(b,sizeof b,"%lu packets",(unsigned long)x->count);txt(b,310,42,1,cDIM,cPANEL,TR_DATUM);
 // cztery pola: RSSI, SNR, hopy, srednie RSSI
 const char* LB[4]={"RSSI dBm","SNR dB","HOPS","AVG RSSI"};
 for(int i=0;i<4;i++){int bx=4+i*79;panel(bx,56,75,30);txt(LB[i],bx+37,59,1,cDIM,cPANEL,TC_DATUM);
  uint16_t c=cWHITE;if(i==0)snprintf(b,sizeof b,"%d",x->rssi),c=x->rssi>-90?cGREEN:x->rssi>-110?cAMBER:cRED;
  else if(i==1)snprintf(b,sizeof b,"%.1f",x->snr);
  else if(i==2){if(x->hops>=0)snprintf(b,sizeof b,"%d",x->hops);else snprintf(b,sizeof b,"?");}
  else snprintf(b,sizeof b,"%d",x->ravg());
  txt(b,bx+37,68,2,c,cPANEL,TC_DATUM);}
 // wykres ostatnich odczytow RSSI + min / max
 panel(4,88,312,40);const int cx0=30,cx1=240,cy0=92,cy1=122;txt("-30",26,cy0,1,cDIM,cPANEL,TR_DATUM);txt("-130",26,cy1-7,1,cDIM,cPANEL,TR_DATUM);
 for(int v=-50;v>=-110;v-=20){int y=cy1-(v+130)*(cy1-cy0)/100;for(int px=cx0;px<=cx1;px+=6)spr.drawPixel(px,y,cTRACK);}
 auto Y=[&](int v){return cy1-(constrain(v,-130,-30)+130)*(cy1-cy0)/100;};int px0=0,py0=0;
 for(unsigned i=0;i<x->hn;i++){int px=cx0+int(uint32_t(i)*(cx1-cx0)/(mh::Node::HIST-1));int py=Y(x->at(i));if(i)spr.drawLine(px0,py0,px,py,cTRACK);px0=px;py0=py;}
 for(unsigned i=0;i<x->hn;i++){int px=cx0+int(uint32_t(i)*(cx1-cx0)/(mh::Node::HIST-1));spr.fillRect(px-1,Y(x->at(i))-1,2,2,cAMBER);}
 if(x->hn<2)txt("collecting samples...",(cx0+cx1)/2,(cy0+cy1)/2-4,1,cDIM,cPANEL,MC_DATUM);
 snprintf(b,sizeof b,"min %d",x->rmin);txt(b,250,93,1,cDIM,cPANEL);snprintf(b,sizeof b,"max %d",x->rmax);txt(b,250,103,1,cDIM,cPANEL);snprintf(b,sizeof b,"snr %.1f",x->savg());txt(b,250,113,1,cDIM,cPANEL);
 drawPingPanel(now);}

// ---- Ustawienia ----
constexpr unsigned SET_N=8;
void setLine(unsigned i,char* name,size_t nc,char* val,size_t vc){
 static const char* N[SET_N]={"TX power (ping)","Mesh preset","Invert colors","Flip screen","TFT power pin","Node beacon","Auto ping","Bluetooth"};
 snprintf(name,nc,"%s",N[i]);const Prof p=profFor(M_MESH);
 switch(i){
  case 0:snprintf(val,vc,"%d dBm%s",int(p.pwr),pre::linkPower(cfg.power)>pre::powerLimit(p.mhz)?" (limit)":"");break;
  case 1:snprintf(val,vc,"%s",pre::get(cfg.meshPreset).name);break;
  case 2:snprintf(val,vc,"%s",cfg.invert?"ON":"OFF");break;
  case 3:snprintf(val,vc,"%s",cfg.flip?"180 deg":"normal");break;
  case 4:snprintf(val,vc,"%s",cfg.pwrHigh?"HIGH = on":"LOW = on");break;
  case 5:if(cfg.beacon)snprintf(val,vc,"every %u min",pre::beaconMin(cfg.beacon));else snprintf(val,vc,"OFF");break;
  case 6:snprintf(val,vc,"%s",cfg.autoPing?"ON  10 s":"OFF");break;
  default:if(cfg.ble)snprintf(val,vc,"ON  PIN %06lu",(unsigned long)blePin);else snprintf(val,vc,"OFF");}}
void drawSet(){
 unsigned first=setSel<3?0:setSel-3;if(first+6>SET_N)first=SET_N-6;
 for(unsigned r=0;r<6;r++){unsigned i=first+r;int y=21+int(r)*21;bool sel=i==setSel;char nm[24],v[32];setLine(i,nm,sizeof nm,v,sizeof v);
  panel(4,y,312,20,sel?cOKBG:cPANEL,sel?cGREEN:cLINE);txt(nm,12,y+2,2,sel?cWHITE:cDIM,sel?cOKBG:cPANEL);txt(v,308,y+2,2,sel?cGREEN:cWHITE,sel?cOKBG:cPANEL,TR_DATUM);}}

// ---------- ekran powitalny: deszcz znakow + sieć mesh + pulsujace pierscienie ----------
void drawSplash(int f,int nf){
 spr.fillSprite(cHDR);
 for(int c=0;c<SCR_W/12;c++){uint32_t h=(c+1)*2654435761u;int sp=2+(h>>8)%4,y0=(h>>16)%SCR_H;int head=(y0+f*sp*3)%(SCR_H+80)-40;
  for(int k=0;k<7;k++){int y=head-k*11;if(y<0||y>SCR_H-8)continue;char ch[2]={char('0'+((h>>(k+3))+f/3+k)%10),0};
   uint16_t col=k==0?cGREEN:k<3?cLINE:k<5?cTRACK:cPANEL;txt(ch,c*12+2,y,1,col,cHDR);}}
 static const int8_t NX[6]={-96,-52,60,98,-20,36},NY[6]={-30,30,-34,28,-8,40};const int cx=SCR_W/2,cy=62;
 int shown=1+f*6/nf;
 for(int i=0;i<6&&i<shown;i++){int x=cx+NX[i],y=cy+NY[i];spr.drawLine(cx,cy,x,y,cLINE);if(i>0&&i<shown-1){spr.drawLine(x,y,cx+NX[i+1],cy+NY[i+1],cTRACK);}}
 for(int i=0;i<6&&i<shown;i++)spr.fillCircle(cx+NX[i],cy+NY[i],4,i%2?cAMBER:cTURQ);
 int r=(f*5)%48+6;spr.drawCircle(cx,cy,r,cTURQ);if(r>16)spr.drawCircle(cx,cy,r-12,cLINE);
 spr.fillCircle(cx,cy,7,cGREEN);spr.fillCircle(cx,cy,3,cHDR);
 spr.fillRect(20,100,SCR_W-40,52,cBG);spr.drawRect(20,100,SCR_W-40,52,cLINE);
 txt("MESH FIELD",(SCR_W-spr.textWidth("MESH FIELD",4))/2,106,4,cTURQ,cBG);
 {const char* sub="Heltec Vision Master T190  -  LoRa 868";txt(sub,(SCR_W-spr.textWidth(sub,1))/2,138,1,cDIM,cBG);}
 spr.fillRect(0,SCR_H-4,SCR_W*(f+1)/nf,4,cGREEN);
}
void splash(){if(!sprOk)return;const int N=86;for(int f=0;f<N;f++){drawSplash(f,N);spr.pushSprite(0,0);delay(70);}}   // 86 x 70 ms = 6 s

void render(){
 spr.fillSprite(cBG);
 switch(screen){case S_HOME:drawHome();break;case S_NOISE:drawNoise();break;case S_SCAN:drawScan();break;case S_MESH:drawMesh();break;case S_NODE:drawNode();break;default:drawSet();}
 drawHeader();
 switch(screen){
  case S_HOME:{char fb[40];snprintf(fb,sizeof fb,"hold KEY: beacon now (%u sent)",beaconsSent);drawFooter("PRG: next screen",fb);break;}
  case S_NOISE:drawFooter("PRG: next screen","KEY: window  hold: clear");break;
  case S_SCAN:drawFooter("PRG: next screen","KEY: pause  hold: clear");break;
  case S_MESH:drawFooter("KEY: select node  PRG: observe it","hold KEY: ping now");break;
  case S_NODE:drawFooter("KEY: next node  PRG: next screen","hold KEY: ping now");break;
  default:drawFooter("PRG: item  hold: exit","KEY: change  hold: back");}
 spr.pushSprite(0,0);}

// ================= NVS =================
void cfgLoad(){prefs.begin("t190",true);size_t n=prefs.getBytesLength("cfg4");if(n>0&&n<=sizeof cfg)prefs.getBytes("cfg4",&cfg,n);prefs.end();if(cfg.ble>1)cfg.ble=0;
 if(cfg.meshPreset>=pre::N)cfg.meshPreset=0;if(cfg.power>=pre::NPWR)cfg.power=2;if(cfg.invert>1)cfg.invert=1;if(cfg.flip>1)cfg.flip=0;if(cfg.pwrHigh>1)cfg.pwrHigh=0;if(cfg.beacon>=pre::NBCN)cfg.beacon=2;if(cfg.autoPing>1)cfg.autoPing=1;}
void cfgSave(){prefs.begin("t190",false);prefs.putBytes("cfg4",&cfg,sizeof cfg);prefs.end();}

// ================= Ekran / zasilanie =================
// Wlasny identyfikator wezla (losowy, trwaly) uzywany jako nadawca pingow Meshtastic.
void nodeIdLoad(){prefs.begin("t190",true);uint32_t v=0;if(prefs.getBytesLength("nid")==4)prefs.getBytes("nid",&v,4);prefs.end();
 if(!v||v==0xFFFFFFFFu||(v&0xFF)==0||(v&0xFF)==0xFF){do{v=esp_random();}while(!v||v==0xFFFFFFFFu||(v&0xFF)==0||(v&0xFF)==0xFF);prefs.begin("t190",false);prefs.putBytes("nid",&v,4);prefs.end();}
 myId=v;Serial.printf("node id !%08lx\n",(unsigned long)myId);}
// Nazwa wezla: tylko drukowalny ASCII, bez spacji na brzegach, max 12 znakow.
void nameClean(char* o,const char* in,size_t n){size_t k=0;for(size_t i=0;i<n&&k<12;i++){unsigned char c=(unsigned char)in[i];if(c>=32&&c<127)o[k++]=char(c);}o[k]=0;
 while(k&&o[k-1]==' ')o[--k]=0;size_t a=0;while(o[a]==' ')a++;if(a)memmove(o,o+a,strlen(o+a)+1);}
void nameLoad(){char t[16]={0};prefs.begin("t190",true);size_t n=prefs.getBytesLength("nname");if(n>0&&n<sizeof t)prefs.getBytes("nname",t,n);prefs.end();nameClean(nodeName,t,n<sizeof t?n:0);}
void nameSave(){prefs.begin("t190",false);prefs.putBytes("nname",nodeName,strlen(nodeName));prefs.end();}
void posLoad(){uint8_t b[13]={0};prefs.begin("t190",true);size_t n=prefs.getBytesLength("pos");if(n==13)prefs.getBytes("pos",b,13);prefs.end();
 if(n==13&&b[12]==1){memcpy(&gpos.lat,b,4);memcpy(&gpos.lon,b+4,4);memcpy(&gpos.alt,b+8,4);gpos.has=gpos.lat>=-900000000&&gpos.lat<=900000000&&gpos.lon>=-1800000000&&gpos.lon<=1800000000;}if(!gpos.has)gpos=Pos();}
void posSave(){uint8_t b[13];memcpy(b,&gpos.lat,4);memcpy(b+4,&gpos.lon,4);memcpy(b+8,&gpos.alt,4);b[12]=gpos.has?1:0;prefs.begin("t190",false);prefs.putBytes("pos",b,13);prefs.end();}
void posText(char* o,size_t cap){if(gpos.has)snprintf(o,cap,"%.6f,%.6f,%ld",gpos.lat/1e7,gpos.lon/1e7,long(gpos.alt));else o[0]=0;}
// "szerokosc,dlugosc[,wysokosc]" w stopniach dziesietnych; pusty tekst kasuje pozycje. Zwraca false przy bledzie (stan bez zmian).
bool posParse(const char* in,size_t n){char t[48];if(n>=sizeof t)return false;memcpy(t,in,n);t[n]=0;char* p=t;while(*p==' ')++p;
 if(!*p){gpos=Pos();return true;}
 char* e;double la=strtod(p,&e);if(e==p)return false;while(*e==' ')++e;if(*e!=',')return false;p=e+1;double lo=strtod(p,&e);if(e==p)return false;while(*e==' ')++e;long al=0;
 if(*e==','){p=e+1;al=strtol(p,&e,10);while(*e==' ')++e;}if(*e)return false;
 if(!(la>=-90&&la<=90&&lo>=-180&&lo<=180&&al>=-500&&al<=9000))return false;
 gpos.lat=int32_t(lround(la*1e7));gpos.lon=int32_t(lround(lo*1e7));gpos.alt=int32_t(al);gpos.has=true;return true;}
void tftPower(){pinMode(PIN_TFT_PWR,OUTPUT);digitalWrite(PIN_TFT_PWR,cfg.pwrHigh?HIGH:LOW);}
void tftApply(){tft.setRotation(cfg.flip?3:1);tft.invertDisplay(cfg.invert!=0);}
void applyChange(unsigned item){
 cfgSave();
 if(item==0){radioApply(profFor(mode));}
 if(item==1){mtab.clear();pingSt=Ping();meshSelId=0;meshPicked=false;mtab.defHash=mh::chanHash(pre::get(cfg.meshPreset).name,mh::defaultKey(),16);radioApply(profFor(mode));if(mode==M_SCAN)scanReset();}
 if(item==2||item==3)tftApply();
 if(item==4)tftPower();
 if(item==5)beaconSoon(millis());
 if(item==6)lastPingTry=millis()-AUTO_PING_MS+1500;
 if(item==7)bleSet(cfg.ble!=0);
 bleCfgDirty=true;}
void changeSetting(unsigned i){
 switch(i){
  case 0:cfg.power=(cfg.power+1)%pre::NPWR;break;
  case 1:cfg.meshPreset=(cfg.meshPreset+1)%pre::N;break;
  case 2:cfg.invert^=1;break;
  case 3:cfg.flip^=1;break;
  case 4:cfg.pwrHigh^=1;break;
  case 5:cfg.beacon=(cfg.beacon+1)%pre::NBCN;break;
  case 6:cfg.autoPing^=1;break;
  default:cfg.ble^=1;}
 applyChange(i);}

void gotoScreen(int s){screen=(s+S_COUNT)%S_COUNT;Mode m=modeFor(screen);if(m!=mode)switchMode(m);if(screen==S_SCAN)scanReset();dirty=true;}


// ================= Bluetooth LE: konfigurator =================
// Wlasna usluga GATT (nie Meshtastic): strona tools/configurator.html laczy sie przez Web Bluetooth, podaje PIN pokazany w SETUP
// i ustawia te same parametry co menu SETUP. Zapis konfiguracji bez poprawnego PIN-u jest ignorowany.
#define BLE_UUID_SERVICE "7a1d0001-5e4b-4c8e-9c5a-543139307031"
#define BLE_UUID_CONFIG  "7a1d0002-5e4b-4c8e-9c5a-543139307031"   // odczyt / zapis / powiadomienia: 7 bajtow ustawien
#define BLE_UUID_AUTH    "7a1d0003-5e4b-4c8e-9c5a-543139307031"   // zapis: PIN (ASCII), odczyt: 1 = autoryzowano
#define BLE_UUID_STATUS  "7a1d0004-5e4b-4c8e-9c5a-543139307031"   // odczyt: stan urzadzenia (JSON)
#define BLE_UUID_NAME    "7a1d0006-5e4b-4c8e-9c5a-543139307031"   // odczyt / zapis (po PIN): nazwa wezla, ASCII, max 12 znakow
#define BLE_UUID_POS     "7a1d0007-5e4b-4c8e-9c5a-543139307031"   // odczyt / zapis (po PIN): pozycja GPS jako tekst \"szerokosc,dlugosc,wysokosc\"; pusty = brak
#define BLE_UUID_OPTIONS "7a1d0005-5e4b-4c8e-9c5a-543139307031"   // odczyt: dostepne opcje (JSON)
constexpr unsigned BLE_CFG_LEN=7;
BLEServer* bleSrv=nullptr;BLECharacteristic *bleChCfg=nullptr,*bleChAuth=nullptr,*bleChStat=nullptr,*bleChOpt=nullptr,*bleChName=nullptr,*bleChPos=nullptr;
bool bleInit=false,bleAdv=false;volatile bool bleConn=false,bleAuthed=false,bleKick=false,bleReAdv=false,bleCfgNew=false;volatile int bleBad=0;
char blePosBuf[48];volatile size_t blePosLen=0;volatile bool blePosNew=false;
char bleNameBuf[16];volatile size_t bleNameLen=0;volatile bool bleNameNew=false;
uint8_t bleCfgBuf[16];volatile size_t bleCfgLen=0;uint32_t bleStatAt=0;

uint8_t* cfgField(Cfg& c,unsigned i){switch(i){case 0:return &c.power;case 1:return &c.meshPreset;case 2:return &c.invert;case 3:return &c.flip;case 4:return &c.pwrHigh;case 5:return &c.beacon;default:return &c.autoPing;}}
bool cfgValid(const uint8_t* d){return d[0]<pre::NPWR&&d[1]<pre::N&&d[2]<2&&d[3]<2&&d[4]<2&&d[5]<pre::NBCN&&d[6]<2;}
// Zastosowanie zapisu z BLE: tylko zmienione pozycje, kazda przez to samo applyChange co w menu SETUP.
bool bleApplyCfg(const uint8_t* d,size_t n){if(n<BLE_CFG_LEN||!cfgValid(d))return false;for(unsigned i=0;i<BLE_CFG_LEN;i++){uint8_t* f=cfgField(cfg,i);if(*f!=d[i]){*f=d[i];applyChange(i);}}dirty=true;return true;}

struct BleSrvCb:BLEServerCallbacks{
 void onConnect(BLEServer*)override{bleConn=true;bleAuthed=false;bleBad=0;dirty=true;}
 void onDisconnect(BLEServer*)override{bleConn=false;bleAuthed=false;bleReAdv=true;dirty=true;}};
struct BleCfgCb:BLECharacteristicCallbacks{
 void onWrite(BLECharacteristic* c)override{if(!bleAuthed)return;auto v=c->getValue();size_t n=v.length();if(n>sizeof bleCfgBuf)n=sizeof bleCfgBuf;memcpy(bleCfgBuf,(const uint8_t*)v.c_str(),n);bleCfgLen=n;bleCfgNew=true;}};
struct BleNameCb:BLECharacteristicCallbacks{
 void onWrite(BLECharacteristic* c)override{if(!bleAuthed)return;auto v=c->getValue();size_t n=v.length();if(n>sizeof bleNameBuf)n=sizeof bleNameBuf;memcpy(bleNameBuf,v.c_str(),n);bleNameLen=n;bleNameNew=true;}};
struct BlePosCb:BLECharacteristicCallbacks{
 void onWrite(BLECharacteristic* c)override{if(!bleAuthed)return;auto v=c->getValue();size_t n=v.length();if(n>=sizeof blePosBuf)n=sizeof blePosBuf-1;memcpy(blePosBuf,v.c_str(),n);blePosLen=n;blePosNew=true;}};
struct BleAuthCb:BLECharacteristicCallbacks{
 void onWrite(BLECharacteristic* c)override{auto v=c->getValue();char want[12];snprintf(want,sizeof want,"%06lu",(unsigned long)blePin);
  if(blePin&&v.length()==6&&memcmp(v.c_str(),want,6)==0){bleAuthed=true;bleBad=0;}else{bleAuthed=false;if(++bleBad>=3)bleKick=true;}
  uint8_t a=bleAuthed?1:0;c->setValue(&a,1);}};
BleSrvCb bleSrvCb;BleCfgCb bleCfgCb;BleNameCb bleNameCb;BlePosCb blePosCb;BleAuthCb bleAuthCb;

void bleOptionsJson(char* o,size_t cap){size_t k=0;
 k+=snprintf(o+k,cap-k,"{\"presets\":[");for(unsigned i=0;i<pre::N&&k<cap;i++)k+=snprintf(o+k,cap-k,"%s[\"%s\",%u,%g,%.3f]",i?",":"",pre::get(i).name,pre::get(i).sf,(double)pre::get(i).bw,(double)pre::meshFreq(i));   // [nazwa, SF, BW kHz, MHz kanalu]
 k+=snprintf(o+k,cap-k,"],\"powers\":[");for(unsigned i=0;i<pre::NPWR&&k<cap;i++)k+=snprintf(o+k,cap-k,"%s%d",i?",":"",pre::linkPower(i));
 k+=snprintf(o+k,cap-k,"],\"beacons\":[");for(unsigned i=0;i<pre::NBCN&&k<cap;i++)k+=snprintf(o+k,cap-k,"%s%u",i?",":"",pre::beaconMin(i));
 snprintf(o+k,cap-k,"]}");}
void bleStatusJson(char* o,size_t cap,uint32_t now){nz::Stats s=nzMesh.stats(60);char pt[40];posText(pt,sizeof pt);const Prof p=profFor(mode);
 snprintf(o,cap,"{\"node\":\"%08lx\",\"name\":\"%s\",\"mode\":%d,\"mhz\":%.3f,\"sf\":%u,\"bw\":%g,\"pwr\":%d,\"pkts\":%lu,\"crc\":%lu,\"hasRx\":%s,\"rssi\":%d,\"snr\":%.1f,\"noise\":%d,\"floor\":%d,\"peak\":%d,\"busy\":%.0f,\"chg\":%s,\"bat\":%.2f,\"batPct\":%d,\"duty\":%.2f,\"nodes\":%u,\"radio\":%s,\"beacons\":%u,\"pos\":\"%s\"}",
  (unsigned long)myId,nodeName,int(mode),(double)p.mhz,p.sf,(double)p.bw,int(p.pwr),(unsigned long)mtab.packets,(unsigned long)crcErrors,hasRx?"true":"false",lastRssi,(double)lastSnr,
  s.has?s.now:0,s.has?s.floor:0,s.has?s.peak:0,s.has?(double)s.busy:0.0,batChg?"true":"false",(double)batV,batPct,(double)duty.pct(now),mtab.used,radioOk?"true":"false",beaconsSent,pt);}
void bleStart(){
 if(!bleInit){char dn[24];snprintf(dn,sizeof dn,"T190-%s",nodeName[0]?nodeName:"FieldTest");BLEDevice::init(dn);bleSrv=BLEDevice::createServer();bleSrv->setCallbacks(&bleSrvCb);BLEService* sv=bleSrv->createService(BLE_UUID_SERVICE);
  bleChCfg=sv->createCharacteristic(BLE_UUID_CONFIG,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_NOTIFY);bleChCfg->addDescriptor(new BLE2902());bleChCfg->setCallbacks(&bleCfgCb);
  bleChAuth=sv->createCharacteristic(BLE_UUID_AUTH,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE);bleChAuth->setCallbacks(&bleAuthCb);
  bleChStat=sv->createCharacteristic(BLE_UUID_STATUS,BLECharacteristic::PROPERTY_READ);
  bleChName=sv->createCharacteristic(BLE_UUID_NAME,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE);bleChName->setCallbacks(&bleNameCb);bleChName->setValue((uint8_t*)nodeName,strlen(nodeName));
  {char pt[40];posText(pt,sizeof pt);bleChPos=sv->createCharacteristic(BLE_UUID_POS,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE);bleChPos->setCallbacks(&blePosCb);bleChPos->setValue((uint8_t*)pt,strlen(pt));}
  bleChOpt=sv->createCharacteristic(BLE_UUID_OPTIONS,BLECharacteristic::PROPERTY_READ);
  static char opt[560];bleOptionsJson(opt,sizeof opt);bleChOpt->setValue((uint8_t*)opt,strlen(opt));uint8_t z=0;bleChAuth->setValue(&z,1);sv->start();bleInit=true;}
 blePin=123456;bleAuthed=false;bleBad=0;bleCfgDirty=true;
 BLEAdvertising* adv=BLEDevice::getAdvertising();adv->addServiceUUID(BLE_UUID_SERVICE);adv->setScanResponse(true);BLEDevice::startAdvertising();bleAdv=true;
 Serial.printf("BLE on, PIN %06lu\n",(unsigned long)blePin);}
void bleStop(){if(!bleInit)return;BLEDevice::getAdvertising()->stop();bleAdv=false;if(bleConn&&bleSrv)bleSrv->disconnect(bleSrv->getConnId());blePin=0;bleAuthed=false;Serial.println("BLE off");}
void bleSet(bool on){if(on)bleStart();else bleStop();}
void bleTick(uint32_t now){
 if(!bleInit)return;
 if(bleKick){bleKick=false;if(bleSrv)bleSrv->disconnect(bleSrv->getConnId());}
 static uint32_t reAdvAt=0;static bool reAdvPend=false;if(bleReAdv){bleReAdv=false;reAdvAt=now+300;reAdvPend=true;}
 if(reAdvPend&&int32_t(now-reAdvAt)>=0){reAdvPend=false;if(cfg.ble&&bleAdv)BLEDevice::startAdvertising();}
 if(bleCfgNew){bleCfgNew=false;uint8_t tmp[16];size_t n=bleCfgLen;if(n>sizeof tmp)n=sizeof tmp;memcpy(tmp,bleCfgBuf,n);if(!bleApplyCfg(tmp,n))bleCfgDirty=true;}   // niepoprawny zapis: odeslij biezace ustawienia
 if(bleNameNew){bleNameNew=false;char t[16];size_t n=bleNameLen;if(n>sizeof t)n=sizeof t;memcpy(t,bleNameBuf,n);nameClean(nodeName,t,n);nameSave();bleChName->setValue((uint8_t*)nodeName,strlen(nodeName));beaconSoon(now);dirty=true;Serial.printf("node name: %s\n",nodeName);}
 if(blePosNew){blePosNew=false;size_t n=blePosLen;if(n>=sizeof blePosBuf)n=sizeof blePosBuf-1;char t[48];memcpy(t,blePosBuf,n);if(posParse(t,n)){posSave();beaconSoon(now);Serial.println(gpos.has?"position set":"position cleared");}
  char pt[40];posText(pt,sizeof pt);bleChPos->setValue((uint8_t*)pt,strlen(pt));dirty=true;}
 if(bleCfgDirty){bleCfgDirty=false;uint8_t v[BLE_CFG_LEN];for(unsigned i=0;i<BLE_CFG_LEN;i++)v[i]=*cfgField(cfg,i);bleChCfg->setValue(v,BLE_CFG_LEN);if(bleConn)bleChCfg->notify();}
 if(now-bleStatAt>=1000){bleStatAt=now;static char js[512];bleStatusJson(js,sizeof js,now);bleChStat->setValue((uint8_t*)js,strlen(js));}}

// ================= Przyciski =================
struct Btn{int pin;bool down,lng;uint32_t t;Btn(int p):pin(p),down(false),lng(false),t(0){}};
Btn btnA(PIN_BTN_A),btnB(PIN_BTN_B);
int btnPoll(Btn& b,uint32_t now){   // 0 nic, 1 krotkie (po puszczeniu), 2 dlugie (po 700 ms trzymania)
 bool d=digitalRead(b.pin)==LOW;
 if(d&&!b.down){b.down=true;b.lng=false;b.t=now;return 0;}
 if(d&&b.down&&!b.lng&&now-b.t>=700){b.lng=true;return 2;}
 if(!d&&b.down){b.down=false;if(!b.lng&&now-b.t>=30)return 1;}
 return 0;}
void onButtons(uint32_t now){
 int a=btnPoll(btnA,now),k=btnPoll(btnB,now);
 if(a){dirty=true;
  if(screen==S_SET){if(a==1)setSel=(setSel+1)%SET_N;else gotoScreen(S_HOME);}
  else gotoScreen(screen+(a==1?1:-1));}
 if(k){dirty=true;
  if(screen==S_SET){if(k==1)changeSetting(setSel);else setSel=(setSel+SET_N-1)%SET_N;}
  else if(k==1){switch(screen){
    case S_NOISE:noiseWin=(noiseWin+1)%3;break;
    case S_SCAN:scanPaused=!scanPaused;break;
    case S_MESH:case S_NODE:{unsigned idx[mh::Table::MAXN];unsigned n=mtab.order(now,idx);if(n){unsigned cur=0;for(unsigned i=0;i<n;i++)if(mtab.n[idx[i]].id==meshSelId)cur=i;if(!meshPicked)meshPicked=true;else cur=(cur+1)%n;meshSelId=mtab.n[idx[cur]].id;pingSt.st=PG_IDLE;lastPingTry=now-AUTO_PING_MS+1500;}}break;}}
  else{switch(screen){
    case S_NOISE:nzCur().reset();break;
    case S_SCAN:scanReset();break;
    case S_HOME:beaconNow(now);break;
    case S_MESH:case S_NODE:pingSend(now);break;}}}}

// ================= setup / loop =================
// ---------- krytycznie niska bateria: wylacz (deep sleep), wznow po podlaczeniu zasilania ----------
// Brak znanego pinu statusu ladowania, wiec co 30 s budzimy sie, mierzymy napiecie i startujemy dopiero, gdy wyraznie wzroslo (ladowarka) lub bateria jest naladowana.
void batShutdown(){
 Serial.println("battery critical: shutting down");
 if(sprOk){spr.fillSprite(cBG);spr.fillRect(0,0,SCR_W,22,cRED);
  txt("BATTERY CRITICAL",SCR_W/2,3,2,cWHITE,cRED,TC_DATUM);
  char b[40];snprintf(b,sizeof b,"%.2f V",(double)batV);txt(b,SCR_W/2,48,4,cRED,cBG,TC_DATUM);
  txt("Shutting down",SCR_W/2,92,2,cWHITE,cBG,TC_DATUM);
  txt("Connect power to restart",SCR_W/2,118,2,cDIM,cBG,TC_DATUM);
  spr.pushSprite(0,0);delay(4000);}
 radio.sleep();
 tft.writecommand(0x28);tft.writecommand(0x10);   // display off, sleep in
 digitalWrite(PIN_TFT_BL,LOW);gpio_hold_en((gpio_num_t)PIN_TFT_BL);gpio_deep_sleep_hold_en();
 rtcLowOff=true;rtcMinV=batV;batLowCnt=0;
 esp_sleep_enable_timer_wakeup(30ULL*1000000ULL);esp_deep_sleep_start();}
// Po obudzeniu timerem: true = kontynuuj start, false = wrocic do snu
bool batWakeCheck(){
 gpio_deep_sleep_hold_dis();gpio_hold_dis((gpio_num_t)PIN_TFT_BL);
 if(!rtcLowOff)return true;
 if(esp_sleep_get_wakeup_cause()!=ESP_SLEEP_WAKEUP_TIMER){rtcLowOff=false;return true;}   // reset / przycisk: normalny start
 pinMode(PIN_BAT_CTRL,OUTPUT);batRead();
 if(batV<rtcMinV)rtcMinV=batV;
 if(batV>=rtcMinV+0.06f||batV>=3.75f){rtcLowOff=false;return true;}
 digitalWrite(PIN_TFT_BL,LOW);gpio_hold_en((gpio_num_t)PIN_TFT_BL);gpio_deep_sleep_hold_en();
 esp_sleep_enable_timer_wakeup(30ULL*1000000ULL);esp_deep_sleep_start();return false;}

void setup(){
 if(!batWakeCheck())return;
 Serial.begin(115200);delay(200);
 pinMode(PIN_BTN_A,INPUT_PULLUP);pinMode(PIN_BTN_B,INPUT_PULLUP);pinMode(PIN_LORA_DIO1,INPUT);
 pinMode(PIN_VEXT,OUTPUT);digitalWrite(PIN_VEXT,HIGH);pinMode(PIN_BAT_CTRL,OUTPUT);digitalWrite(PIN_BAT_CTRL,LOW);
 pinMode(PIN_TFT_BL,OUTPUT);digitalWrite(PIN_TFT_BL,HIGH);
 cfgLoad();
 // Przytrzymany KEY przy starcie odwraca polaryzacje zasilania TFT (gdyby ekran zostal czarny).
 if(digitalRead(PIN_BTN_B)==LOW){cfg.pwrHigh^=1;cfgSave();Serial.println("TFT power polarity flipped");}
 tftPower();delay(60);
 tft.init();tftApply();tft.fillScreen(cBG);
 sprOk=spr.createSprite(SCR_W,SCR_H)!=nullptr;
 if(!sprOk){tft.setTextColor(cRED,cBG);tft.drawString("No memory for frame buffer",8,70,2);Serial.println("sprite alloc failed");for(;;)delay(1000);}
 mtab.defHash=mh::chanHash(pre::get(cfg.meshPreset).name,mh::defaultKey(),16);
 nodeIdLoad();nameLoad();posLoad();nextBeaconAt=millis()+30000;if(cfg.ble)bleStart();radioInit();batRead();splash();
 Serial.println("t_ms,mode,noise_now,noise_floor,noise_peak,busy_pct,mesh_pkts,crc_err");
 render();}

void loop(){
 uint32_t now=millis();
 onButtons(now);bleTick(now);txPoll(now);rxPoll(now);pingTick(now);beaconTick(now);noiseTick(now);
 static uint32_t lastScan=0;if(mode==M_SCAN&&now-lastScan>=40){lastScan=now;scanTick();}
 if(now-lastBatAt>=10000||!lastBatAt){lastBatAt=now?now:1;batRead();dirty=true;if(batLowCnt>=BAT_OFF_READS)batShutdown();}
 static uint32_t lastCsv=0;if(now-lastCsv>=5000){lastCsv=now;nz::Stats s=nzCur().stats(60);
  Serial.printf("%lu,%d,%d,%d,%d,%.0f,%lu,%lu\n",(unsigned long)now,int(mode),s.has?s.now:0,s.has?s.floor:0,s.has?s.peak:0,s.has?s.busy:0.f,(unsigned long)mtab.packets,(unsigned long)crcErrors);}
 if((dirty&&now-lastDraw>=80)||now-lastDraw>=500){lastDraw=now;dirty=false;render();}
 delay(1);}
