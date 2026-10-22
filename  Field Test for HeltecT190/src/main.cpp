// LoRa Field Test na Heltec Vision Master T190 (ESP32-S3 + SX1262 + TFT 1,9" 170x320, dwa przyciski, bez dotyku).
// Samodzielny tester z wlasnym radiem (NIE jest to firmware Meshtastic):
//   - TEST LACZA: ping-pong miedzy dwoma T190 (INIT wysyla, ECHO odpowiada) z RSSI/SNR w obu kierunkach, strata pakietow, RTT
//   - SZUM: prawdziwy pomiar chwilowego RSSI radia (10x/s), podloga, szczyty, zajetosc kanalu, okna 1 min / 10 min / 1 h
//   - SKAN: przemiatanie pasma 863-870 MHz z zapamietaniem szczytow
//   - MESH: pasywny podsluch pakietow Meshtastic (naglowki jawne; nazwy wezlow tylko dla domyslnego klucza kanalu)
#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <TFT_eSPI.h>
#include "config.h"
#include "presets.h"
#include "noiselog.h"
#include "linkproto.h"
#include "meshhdr.h"

// ================= Stan globalny =================
TFT_eSPI tft;
TFT_eSprite spr=TFT_eSprite(&tft);
bool sprOk=false;
SPIClass loraSpi(FSPI);
SX1262 radio=new Module(PIN_LORA_CS,PIN_LORA_DIO1,PIN_LORA_RST,PIN_LORA_BUSY,loraSpi);
Preferences prefs;

struct Cfg{uint8_t role=0,linkPreset=4,linkFreq=0,power=2,interval=1,meshPreset=0,invert=1,flip=0,pwrHigh=0;} cfg;   // role: 0 = INIT, 1 = ECHO
enum Screen{S_HOME,S_LINK,S_NOISE,S_SCAN,S_MESH,S_SET,S_COUNT};
enum Mode{M_LINK,M_MESH,M_SCAN};
const char* SCR_NAME[S_COUNT]={"HOME","LINK","NOISE","SCAN","MESH","SETUP"};
int screen=S_HOME;Mode mode=M_LINK;bool dirty=true;uint32_t lastDraw=0;

struct Prof{float mhz,bw;uint8_t sf,cr,sync;uint16_t pre;int8_t pwr;};
Prof prof={869.525f,250,9,5,LINK_SYNC,LINK_PREAMBLE,14};
bool radioOk=false;int16_t rlErr=0;

lnk::Stats lk;pre::Duty duty;bool dutyBlocked=false;
nz::Log nzLink,nzMesh;
mh::Table mtab;mh::Air meshAir;uint32_t meshRx=0,crcErrors=0;
bool txBusy=false;uint32_t txDeadline=0;
bool replyDue=false;uint32_t replyAt=0;lnk::Frame replyFrame;
uint32_t probesHeard=0,repliesSent=0,lastProbeAt=0;
uint32_t lastNoiseAt=0,lastBatAt=0;float batV=0;int batPct=-1;
unsigned noiseWin=0;                       // 0 = 1 min, 1 = 10 min, 2 = 1 h
unsigned meshOff=0,setSel=0;uint32_t meshSelId=0,myId=0;bool scanPaused=false;
int8_t scanCur[SCAN_N],scanPeak[SCAN_N];unsigned scanIdx=0;

// Ping wezla Meshtastic (pakiet z want_ack, odpowiedz ROUTING_APP)
enum PingState{PG_IDLE,PG_WAIT,PG_ACK,PG_NAK,PG_TIMEOUT,PG_BLOCKED};
struct Ping{uint32_t target=0,id=0,sentAt=0;PingState st=PG_IDLE;unsigned sent=0,acks=0;int rssi=0,hops=-1;float snr=0;uint32_t rtt=0;uint8_t err=0;} pingSt;
constexpr uint32_t PING_TIMEOUT_MS=12000;
nz::Log& nzCur(){return mode==M_MESH?nzMesh:nzLink;}

// ================= Kolory (paleta jak w CYD) =================
constexpr uint16_t C565(uint8_t r,uint8_t g,uint8_t b){return uint16_t(((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3));}
const uint16_t cBG=C565(0,34,22),cHDR=C565(0,18,12),cPANEL=C565(6,56,40),cLINE=C565(30,170,120),cTRACK=C565(14,84,60),cBAND=C565(10,110,92),
 cTURQ=C565(0,255,224),cGREEN=C565(30,255,120),cAMBER=C565(255,190,0),cRED=C565(255,56,44),cDIM=C565(140,230,200),cWHITE=C565(240,255,250),
 cGREY=C565(170,190,182),cOKBG=C565(4,78,46),cWARNBG=C565(80,54,0),cBADBG=C565(86,14,10);

// ================= Radio =================
#define RL(x) do{int16_t e_=(x);if(e_!=RADIOLIB_ERR_NONE&&!rlErr)rlErr=e_;}while(0)
Prof profFor(Mode m){
 if(m==M_MESH){const pre::Preset& p=pre::get(cfg.meshPreset);float f=pre::meshFreq(cfg.meshPreset);int pw=pre::linkPower(cfg.power);if(pw>pre::powerLimit(f))pw=pre::powerLimit(f);return Prof{f,p.bw,p.sf,p.cr,MESH_SYNC,MESH_PREAMBLE,int8_t(pw)};}
 if(m==M_SCAN)return Prof{SCAN_START,125,7,5,LINK_SYNC,8,2};
 const pre::Preset& p=pre::get(cfg.linkPreset);float f=pre::linkFreq(cfg.linkFreq);int pw=pre::linkPower(cfg.power);if(pw>pre::powerLimit(f))pw=pre::powerLimit(f);
 return Prof{f,p.bw,p.sf,p.cr,LINK_SYNC,LINK_PREAMBLE,int8_t(pw)};}
void radioApply(const Prof& p){
 if(!radioOk)return;prof=p;txBusy=false;replyDue=false;rlErr=0;
 RL(radio.standby());RL(radio.setBandwidth(p.bw));RL(radio.setSpreadingFactor(p.sf));RL(radio.setCodingRate(p.cr));RL(radio.setSyncWord(p.sync));RL(radio.setPreambleLength(p.pre));
 RL(radio.setFrequency(p.mhz));RL(radio.setOutputPower(p.pwr));RL(radio.startReceive());}
void switchMode(Mode m){mode=m;radioApply(profFor(m));lastNoiseAt=millis();}
Mode modeFor(int s){return s==S_MESH?M_MESH:s==S_SCAN?M_SCAN:M_LINK;}
void radioInit(){
 loraSpi.begin(PIN_LORA_SCK,PIN_LORA_MISO,PIN_LORA_MOSI,PIN_LORA_CS);
 Prof p=profFor(M_LINK);prof=p;
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
void batRead(){analogSetPinAttenuation(PIN_BAT_ADC,ADC_2_5db);digitalWrite(PIN_BAT_CTRL,HIGH);delay(3);uint32_t mv=0;for(int i=0;i<8;i++)mv+=analogReadMilliVolts(PIN_BAT_ADC);digitalWrite(PIN_BAT_CTRL,LOW);
 batV=(mv/8)/1000.f*BAT_MULT;
 static const float V[]={4.20f,4.10f,4.00f,3.90f,3.80f,3.70f,3.60f,3.50f,3.30f};static const int P[]={100,90,80,65,50,30,15,5,0};
 if(batV<3.0f){batPct=-1;return;}int p=0;if(batV>=V[0])p=100;else if(batV<=V[8])p=0;else for(int i=0;i<8;i++)if(batV<=V[i]&&batV>=V[i+1]){p=P[i+1]+int((batV-V[i+1])/(V[i]-V[i+1])*(P[i]-P[i+1]));break;}batPct=p;}

// ================= Odbior pakietow =================
int8_t noiseNow(){nz::Stats s=nzLink.stats(30);return s.has?int8_t(s.now):int8_t(nz::NONE);}
void onLinkFrame(const uint8_t* b,size_t n,float rssi,float snr,uint32_t now){
 lnk::Frame f;if(!lnk::decode(b,n,f))return;
 if(f.type==lnk::T_PROBE&&cfg.role==1){
  ++probesHeard;lastProbeAt=now;lk.hasUp=true;lk.upRssi=int(lroundf(rssi));lk.upSnr=snr;lk.push(lnk::clampDb(rssi,-127,0),127);
  replyFrame=lnk::Frame();replyFrame.type=lnk::T_REPLY;replyFrame.session=f.session;replyFrame.seq=f.seq;replyFrame.rssi=lnk::clampDb(rssi,-127,0);replyFrame.snr4=lnk::clampDb(snr*4,-127,127);
  replyFrame.noise=noiseNow();replyFrame.batt=batPct<0?255:uint8_t(batPct);replyDue=true;replyAt=now+REPLY_DELAY_MS;dirty=true;
  Serial.printf("PROBE seq=%u rssi=%.0f snr=%.1f\n",f.seq,rssi,snr);}
 else if(f.type==lnk::T_REPLY&&cfg.role==0){
  if(lk.onReply(f,rssi,snr,now)){Serial.printf("REPLY seq=%u rtt=%lu up=%.0f/%.1f dn=%d/%.1f\n",f.seq,(unsigned long)lk.rttLast,rssi,snr,lk.dnRssi,lk.dnSnr);dirty=true;lk.nextAt=now+pre::linkInterval(cfg.interval);}}}
void onMeshFrame(const uint8_t* b,size_t n,float rssi,float snr,uint32_t now){
 mh::Hdr h;if(!mh::parse(b,n,h))return;if(h.from==myId)return;
 if(pingSt.st==PG_WAIT&&h.from==pingSt.target&&h.to==myId){uint8_t er=0;if(mh::ackFor(b,n,h,pingSt.id,&er)){pingSt.rssi=int(lroundf(rssi));pingSt.snr=snr;pingSt.rtt=now-pingSt.sentAt;pingSt.hops=mh::hops(h);pingSt.err=er;if(er==0){pingSt.st=PG_ACK;++pingSt.acks;}else pingSt.st=PG_NAK;dirty=true;
   Serial.printf("PING ack from=%08lx rssi=%.0f snr=%.1f rtt=%lu err=%u\n",(unsigned long)h.from,rssi,snr,(unsigned long)pingSt.rtt,unsigned(er));}}
 ++meshRx;mtab.add(h,b,n,int(lroundf(rssi)),snr,now);meshAir.add(now,uint32_t(radio.getTimeOnAir(n)));dirty=true;
 Serial.printf("MESH from=%08lx rssi=%.0f snr=%.1f hops=%d len=%u\n",(unsigned long)h.from,rssi,snr,mh::hops(h),unsigned(n));}
void rxPoll(uint32_t now){
 if(!radioOk||txBusy||mode==M_SCAN)return;if(!digitalRead(PIN_LORA_DIO1))return;
 uint8_t buf[256];size_t len=radio.getPacketLength();int16_t e=radio.readData(buf,len>sizeof buf?sizeof buf:len);float rssi=radio.getRSSI(),snr=radio.getSNR();radio.startReceive();
 if(e==RADIOLIB_ERR_CRC_MISMATCH){++crcErrors;return;}if(e!=RADIOLIB_ERR_NONE||!len||len>sizeof buf)return;
 if(mode==M_LINK)onLinkFrame(buf,len,rssi,snr,now);else onMeshFrame(buf,len,rssi,snr,now);}

// ================= Skan pasma =================
void scanReset(){memset(scanCur,nz::NONE,sizeof scanCur);memset(scanPeak,nz::NONE,sizeof scanPeak);scanIdx=0;}
void scanTick(){
 if(!radioOk||scanPaused)return;
 for(int k=0;k<4;k++){float f=SCAN_START+scanIdx*SCAN_STEP;RL(radio.setFrequency(f));RL(radio.startReceive());delay(2);float r=radio.getRSSI(false);int v=int(lroundf(r));if(v<-127)v=-127;if(v>0)v=0;
  scanCur[scanIdx]=int8_t(v);if(scanPeak[scanIdx]==nz::NONE||v>scanPeak[scanIdx])scanPeak[scanIdx]=int8_t(v);scanIdx=(scanIdx+1)%SCAN_N;}
 dirty=true;}

// ================= Logika testu lacza i szumu =================
void linkTick(uint32_t now){
 if(!radioOk||mode!=M_LINK||txBusy)return;
 if(cfg.role==0&&lk.on){
  uint32_t limit=2*toaMs(lnk::LEN)+REPLY_DELAY_MS+600;
  if(lk.timeout(now,limit)){Serial.printf("LOST seq=%u\n",lk.seq);lk.nextAt=now+pre::linkInterval(cfg.interval);dirty=true;}
  else if(!lk.pending&&int32_t(now-lk.nextAt)>=0){
   lnk::Frame f;f.type=lnk::T_PROBE;f.session=lk.session;f.seq=uint16_t(lk.seq+1);f.pwr=int8_t(prof.pwr);uint8_t buf[lnk::LEN];lnk::encode(f,buf);
   if(txStart(buf,sizeof buf,now)){lk.startProbe(now);lk.nextAt=now+pre::linkInterval(cfg.interval);}else lk.nextAt=now+1000;dirty=true;}}
 if(replyDue&&int32_t(now-replyAt)>=0){uint8_t buf[lnk::LEN];lnk::encode(replyFrame,buf);if(txStart(buf,sizeof buf,now)){replyDue=false;++repliesSent;dirty=true;}else if(int32_t(now-replyAt)>2000)replyDue=false;}}
void pingSend(uint32_t now){
 if(!radioOk||mode!=M_MESH||pingSt.st==PG_WAIT)return;
 unsigned idx[mh::Table::MAXN];unsigned n=mtab.order(now,idx);uint32_t target=0;for(unsigned i=0;i<n;i++)if(mtab.n[idx[i]].id==meshSelId)target=meshSelId;if(!target&&n)target=mtab.n[idx[0]].id;if(!target)return;
 if(target!=pingSt.target){pingSt.sent=pingSt.acks=0;}
 pingSt.target=target;uint32_t id=esp_random();if(!id)id=1;uint8_t pkt[40];size_t len=mh::buildPing(pkt,sizeof pkt,target,myId,id,mtab.defHash,0);
 pingSt.id=id;if(txStart(pkt,len,now)){pingSt.st=PG_WAIT;pingSt.sentAt=now;++pingSt.sent;Serial.printf("PING to=%08lx id=%08lx\n",(unsigned long)target,(unsigned long)id);}else pingSt.st=PG_BLOCKED;dirty=true;}
void pingTick(uint32_t now){if(pingSt.st==PG_WAIT&&!txBusy&&now-pingSt.sentAt>PING_TIMEOUT_MS){pingSt.st=PG_TIMEOUT;dirty=true;}}
void noiseTick(uint32_t now){
 if(!radioOk||mode==M_SCAN||txBusy||now-lastNoiseAt<NOISE_PERIOD_MS)return;lastNoiseAt=now;
 nzCur().add(now,radio.getRSSI(false));}
void linkRestart(){lk.reset();lk.session=uint8_t(esp_random()&0xFF);lk.nextAt=millis()+1000;replyDue=false;probesHeard=repliesSent=0;lastProbeAt=0;}

// ================= Rysowanie (wszystko do sprite'a 320x170, potem jeden push) =================
void txt(const String& s,int x,int y,int font,uint16_t fg,uint16_t bg,uint8_t datum=TL_DATUM){spr.setTextDatum(datum);spr.setTextColor(fg,bg);spr.drawString(s,x,y,font);spr.setTextDatum(TL_DATUM);}
void panel(int x,int y,int w,int h,uint16_t bg=cPANEL,uint16_t line=cLINE){spr.fillRoundRect(x,y,w,h,5,bg);spr.drawRoundRect(x,y,w,h,5,line);}
uint16_t qColor(lnk::Q q){return q==lnk::Q_GOOD?cGREEN:q==lnk::Q_FAIR?cAMBER:q==lnk::Q_BAD?cRED:cGREY;}
uint16_t qBg(lnk::Q q){return q==lnk::Q_GOOD?cOKBG:q==lnk::Q_FAIR?cWARNBG:q==lnk::Q_BAD?cBADBG:cPANEL;}
void chip(int x,int y,int w,const char* s,uint16_t fg,uint16_t bg){spr.fillRoundRect(x,y,w,13,6,bg);spr.drawRoundRect(x,y,w,13,6,fg);txt(s,x+w/2,y+3,1,fg,bg,TC_DATUM);}
// Poziomy pasek wartosci v w zakresie lo..hi
void bar(int x,int y,int w,int h,float v,float lo,float hi,uint16_t c){spr.fillRect(x,y,w,h,cTRACK);float t=(v-lo)/(hi-lo);if(t<0)t=0;if(t>1)t=1;int k=int(t*w);if(k>0)spr.fillRect(x,y,k,h,c);}
void tile(int x,int y,const char* title,uint16_t accent){panel(x,y,100,62);spr.fillCircle(x+8,y+8,2,accent);txt(title,x+15,y+4,1,cDIM,cPANEL);}
unsigned linkSf(){return pre::get(cfg.linkPreset).sf;}
unsigned winSec(){static const unsigned W[]={60,600,3600};return W[noiseWin%3];}
const char* winName(){static const char* N[]={"1 min","10 min","1 h"};return N[noiseWin%3];}

void drawHeader(){
 spr.fillRect(0,0,SCR_W,18,cHDR);spr.drawFastHLine(0,18,SCR_W,cLINE);
 spr.drawLine(5,11,9,6,cTURQ);spr.drawLine(9,6,14,12,cTURQ);const int nx[]={5,9,14},ny[]={11,6,12};for(int i=0;i<3;i++){spr.fillCircle(nx[i],ny[i],2,cTURQ);spr.fillCircle(nx[i],ny[i],1,cHDR);}
 txt("LoRa Field Test",22,1,2,cWHITE,cHDR);
 txt(SCR_NAME[screen],196,1,2,cTURQ,cHDR);
 if(!radioOk)chip(250,3,66,"RADIO ERR",cRED,cBADBG);
 else if(mode==M_MESH)chip(250,3,66,"LISTEN",cAMBER,cWARNBG);
 else if(mode==M_SCAN)chip(250,3,66,"SCAN",cAMBER,cWARNBG);
 else if(cfg.role==0)chip(250,3,66,lk.on?"INIT":"INIT PAUSE",lk.on?cGREEN:cAMBER,lk.on?cOKBG:cWARNBG);
 else chip(250,3,66,"ECHO",cGREEN,cOKBG);}
void drawFooter(const char* l,const char* r){
 spr.fillRect(0,151,SCR_W,19,cHDR);spr.drawFastHLine(0,151,SCR_W,cLINE);
 txt(l,6,157,1,cDIM,cHDR);txt(r,314,157,1,cTURQ,cHDR,TR_DATUM);}

void drawHome(){
 const int X[]={4,110,216},Y[]={20,86};char b[40];uint32_t now=millis();const lnk::Q q=lnk::quality(lk.hasUp,float(lk.upSnr),linkSf());
 // LINK
 {tile(X[0],Y[0],cfg.role==0?"LINK TEST":"ECHO",cGREEN);
  if(cfg.role==0){uint32_t done=lk.ok+lk.lost;if(done)snprintf(b,sizeof b,"%u%%",unsigned(100*lk.ok/done));else snprintf(b,sizeof b,"--");txt(b,X[0]+8,Y[0]+14,4,done?(lk.lossPct()>20?cRED:lk.lossPct()>5?cAMBER:cGREEN):cGREY,cPANEL);
   snprintf(b,sizeof b,"ok %lu  lost %lu",(unsigned long)lk.ok,(unsigned long)lk.lost);txt(b,X[0]+8,Y[0]+42,1,cWHITE,cPANEL);
   txt(!lk.on?"PAUSED":dutyBlocked?"DUTY LIMIT":lk.pending?"waiting reply":"sending",X[0]+8,Y[0]+52,1,!lk.on||dutyBlocked?cAMBER:cDIM,cPANEL);}
  else{snprintf(b,sizeof b,"%lu",(unsigned long)probesHeard);txt(b,X[0]+8,Y[0]+14,4,probesHeard?cGREEN:cGREY,cPANEL);txt("probes heard",X[0]+8,Y[0]+42,1,cWHITE,cPANEL);
   if(probesHeard){snprintf(b,sizeof b,"last %lus ago",(unsigned long)((now-lastProbeAt)/1000));txt(b,X[0]+8,Y[0]+52,1,cDIM,cPANEL);}else txt("waiting",X[0]+8,Y[0]+52,1,cDIM,cPANEL);}}
 // SIGNAL
 {tile(X[1],Y[0],"SIGNAL",cTURQ);if(lk.hasUp){snprintf(b,sizeof b,"%d",lk.upRssi);txt(b,X[1]+8,Y[0]+14,4,qColor(q),cPANEL);txt("dBm",X[1]+8+spr.textWidth(b,4)+3,Y[0]+26,1,cDIM,cPANEL);
   snprintf(b,sizeof b,"SNR %.1f dB",lk.upSnr);txt(b,X[1]+8,Y[0]+42,1,cWHITE,cPANEL);
   if(cfg.role==0&&lk.hasDn){snprintf(b,sizeof b,"peer %d / %.1f",lk.dnRssi,lk.dnSnr);txt(b,X[1]+8,Y[0]+52,1,cDIM,cPANEL);}else txt(lnk::qText(q),X[1]+8,Y[0]+52,1,qColor(q),cPANEL);}
  else{txt("--",X[1]+8,Y[0]+14,4,cGREY,cPANEL);txt("no packets yet",X[1]+8,Y[0]+42,1,cAMBER,cPANEL);}}
 // NOISE
 {tile(X[2],Y[0],"NOISE",cAMBER);nz::Stats s=nzLink.stats(60);if(s.has){snprintf(b,sizeof b,"%d",s.now);txt(b,X[2]+8,Y[0]+14,4,cWHITE,cPANEL);txt("dBm",X[2]+8+spr.textWidth(b,4)+3,Y[0]+26,1,cDIM,cPANEL);
   snprintf(b,sizeof b,"floor %d dBm",s.floor);txt(b,X[2]+8,Y[0]+42,1,cWHITE,cPANEL);snprintf(b,sizeof b,"peak %d dBm",s.peak);txt(b,X[2]+8,Y[0]+52,1,cDIM,cPANEL);}
  else{txt("--",X[2]+8,Y[0]+14,4,cGREY,cPANEL);txt("measuring...",X[2]+8,Y[0]+42,1,cAMBER,cPANEL);}}
 // CHANNEL BUSY
 {tile(X[0],Y[1],"CHANNEL BUSY",cGREEN);nz::Stats s=nzLink.stats(60);if(s.has){snprintf(b,sizeof b,"%.0f%%",s.busy);txt(b,X[0]+8,Y[1]+14,4,s.busy>50?cRED:s.busy>25?cAMBER:cGREEN,cPANEL);}else txt("--",X[0]+8,Y[1]+14,4,cGREY,cPANEL);
  txt("> floor +8 dB",X[0]+8,Y[1]+42,1,cDIM,cPANEL);snprintf(b,sizeof b,"my TX %.1f%%/h",duty.pct(now));txt(b,X[0]+8,Y[1]+52,1,cDIM,cPANEL);}
 // RADIO
 {tile(X[1],Y[1],"RADIO",cAMBER);const Prof p=profFor(M_LINK);txt(pre::get(cfg.linkPreset).name,X[1]+8,Y[1]+13,2,cWHITE,cPANEL);snprintf(b,sizeof b,"%.3f MHz",(double)p.mhz);txt(b,X[1]+8,Y[1]+31,1,cTURQ,cPANEL);
  snprintf(b,sizeof b,"SF%u BW%g kHz",p.sf,(double)p.bw);txt(b,X[1]+8,Y[1]+41,1,cDIM,cPANEL);snprintf(b,sizeof b,"TX %d dBm",int(p.pwr));txt(b,X[1]+8,Y[1]+51,1,cDIM,cPANEL);}
 // BATTERY
 {tile(X[2],Y[1],"BATTERY",cGREEN);if(batPct>=0){snprintf(b,sizeof b,"%.2f",batV);txt(b,X[2]+8,Y[1]+14,4,batPct<20?cRED:batPct<40?cAMBER:cGREEN,cPANEL);txt("V",X[2]+8+spr.textWidth(b,4)+3,Y[1]+26,1,cDIM,cPANEL);
   snprintf(b,sizeof b,"~%d%%",batPct);txt(b,X[2]+8,Y[1]+42,1,cWHITE,cPANEL);}else{txt("--",X[2]+8,Y[1]+14,4,cGREY,cPANEL);txt("no reading",X[2]+8,Y[1]+42,1,cAMBER,cPANEL);}}}

// Wiersz kierunku w oknie LINK: etykieta, RSSI duzym fontem, SNR, paski i ocena
void linkRow(int y,const char* label,bool has,int rssi,float snr,const char* extra){
 panel(4,y,312,38);txt(label,12,y+2,1,cDIM,cPANEL);lnk::Q q=lnk::quality(has,snr,linkSf());char b[32];
 if(has){snprintf(b,sizeof b,"%d",rssi);txt(b,12,y+11,4,qColor(q),cPANEL);txt("dBm",12+spr.textWidth(b,4)+3,y+21,1,cDIM,cPANEL);
  snprintf(b,sizeof b,"SNR %.1f dB",snr);txt(b,110,y+8,2,cWHITE,cPANEL);bar(110,y+28,92,5,rssi,-130,-30,qColor(q));bar(210,y+28,92,5,snr,-20,15,qColor(q));
  chip(240,y+6,68,lnk::qText(q),qColor(q),qBg(q));}
 else txt(extra,110,y+14,2,cAMBER,cPANEL);}
void drawLink(){
 char b[48];
 if(cfg.role==0){linkRow(20,"UPLINK reply",lk.hasUp,lk.upRssi,lk.upSnr,"waiting reply");linkRow(60,"DOWNLINK probe",lk.hasDn,lk.dnRssi,lk.dnSnr,"no data");}
 else{linkRow(20,"PROBE heard",lk.hasUp,lk.upRssi,lk.upSnr,"waiting for probe");panel(4,60,312,38);txt("REPLIES",12,64,1,cDIM,cPANEL);
  snprintf(b,sizeof b,"%lu probes heard, %lu replies sent",(unsigned long)probesHeard,(unsigned long)repliesSent);txt(b,12,78,2,cWHITE,cPANEL);}
 panel(4,100,312,49);
 // wykres RSSI ostatnich pomiarow: amber = uplink, turkus = downlink
 const int cx0=30,cx1=204,cy0=106,cy1=134;txt("-30",26,cy0,1,cDIM,cPANEL,TR_DATUM);txt("-130",26,cy1-7,1,cDIM,cPANEL,TR_DATUM);
 for(int v=-50;v>=-110;v-=20){int y=cy1-(v+130)*(cy1-cy0)/100;for(int x=cx0;x<=cx1;x+=6)spr.drawPixel(x,y,cTRACK);}
 for(unsigned i=0;i<lk.hn;i++){int x=cx0+int(uint32_t(i)*(cx1-cx0)/(LINK_HIST-1));auto Y=[&](int v){return cy1-(constrain(v,-130,-30)+130)*(cy1-cy0)/100;};
  int8_t u=lk.up(i),d=lk.dn(i);if(u==127&&d==127){spr.drawFastVLine(x,cy1-3,3,cRED);continue;}if(u!=127)spr.fillRect(x-1,Y(u)-1,2,2,cAMBER);if(d!=127)spr.fillRect(x-1,Y(d)-1,2,2,cTURQ);}
 if(!lk.hn)txt("waiting for first probe...",(cx0+cx1)/2,(cy0+cy1)/2-4,1,cDIM,cPANEL,MC_DATUM);
 spr.fillRect(cx0,cy1+8,4,3,cAMBER);txt(cfg.role==0?"uplink":"probe",cx0+8,cy1+5,1,cDIM,cPANEL);spr.fillRect(cx0+60,cy1+8,4,3,cTURQ);txt("downlink",cx0+68,cy1+5,1,cDIM,cPANEL);
 // statystyki po prawej
 int x=214,y=106;snprintf(b,sizeof b,"SENT %lu  OK %lu",(unsigned long)lk.sent,(unsigned long)lk.ok);txt(b,x,y,1,cWHITE,cPANEL);
 snprintf(b,sizeof b,"LOST %lu (%.0f%%)",(unsigned long)lk.lost,lk.lossPct());txt(b,x,y+11,1,lk.lossPct()>20?cRED:lk.lossPct()>5?cAMBER:cWHITE,cPANEL);
 snprintf(b,sizeof b,"RTT %lu ms",(unsigned long)lk.rttLast);txt(lk.ok?b:"RTT --",x,y+22,1,cWHITE,cPANEL);snprintf(b,sizeof b,"avg %lu ms",(unsigned long)lk.rttAvg());txt(lk.ok?b:"",x,y+33,1,cDIM,cPANEL);}

void drawNoise(){
 const nz::Log& L=nzLink;nz::Stats s=L.stats(winSec());char b[32];const char* LB[]={"NOW dBm","FLOOR dBm","PEAK dBm","BUSY %"};
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
 const float fl=pre::linkFreq(cfg.linkFreq),fm=pre::meshFreq(cfg.meshPreset);
 spr.drawFastVLine(X(fl)+1,yt,yb-yt,cGREEN);spr.drawFastVLine(X(fm)+1,yt,yb-yt,cAMBER);
 for(unsigned i=0;i<SCAN_N;i++){int x=X(SCAN_START+i*SCAN_STEP);if(scanCur[i]!=nz::NONE){int y=Y(scanCur[i]);spr.fillRect(x,y,3,yb-y+1,cBAND);spr.fillRect(x,y,3,2,cTURQ);}
  if(scanPeak[i]!=nz::NONE)spr.fillRect(x,Y(scanPeak[i])-1,3,2,cAMBER);}
 for(int f=863;f<=870;f++){int x=X(float(f));spr.drawFastVLine(x+1,yb+1,3,cDIM);snprintf(b,sizeof b,"%d",f);txt(b,x+1,yb+5,1,cDIM,cPANEL,TC_DATUM);}
 int cur=(scanIdx+SCAN_N-1)%SCAN_N;snprintf(b,sizeof b,"%.1f MHz %d dBm",SCAN_START+cur*SCAN_STEP,scanCur[cur]==nz::NONE?0:scanCur[cur]);
 txt(b,8,129,1,cWHITE,cPANEL);spr.fillRect(150,131,6,4,cBAND);txt("now",158,129,1,cDIM,cPANEL);spr.fillRect(184,131,6,2,cAMBER);txt("peak",194,129,1,cDIM,cPANEL);
 if(scanPaused)txt("PAUSED",312,129,1,cAMBER,cPANEL,TR_DATUM);
 spr.fillRect(8,140,6,2,cGREEN);snprintf(b,sizeof b,"link %.3f",fl);txt(b,17,137,1,cDIM,cPANEL);spr.fillRect(96,140,6,2,cAMBER);snprintf(b,sizeof b,"mesh %.3f",fm);txt(b,105,137,1,cDIM,cPANEL);}

// Skraca tekst tak, by miescil sie w podanej szerokosci (w pikselach) danym fontem.
void fitText(char* s,int maxW,int font){size_t n=strlen(s);while(n>1&&spr.textWidth(s,font)>maxW){s[--n]=0;}}
const char* nodeLabel(const mh::Node& n,char* b,size_t cap){if(n.name[0])snprintf(b,cap,"%s",n.name);else snprintf(b,cap,"!%08lx",(unsigned long)n.id);return b;}
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
 // panel pingu wybranego wezla
 panel(4,130,312,20);uint16_t pc=cDIM;
 if(pingSt.target&&pingSt.target==meshSelId){
  snprintf(b,sizeof b,"PING !%08lx   sent %u   ack %u",(unsigned long)pingSt.target,pingSt.sent,pingSt.acks);txt(b,10,132,1,cWHITE,cPANEL);
  switch(pingSt.st){
   case PG_WAIT:snprintf(b,sizeof b,"waiting for ACK... %lu s",(unsigned long)((now-pingSt.sentAt)/1000));pc=cAMBER;break;
   case PG_ACK:snprintf(b,sizeof b,"ACK  %d dBm  SNR %.1f  %.1f s  hops %d",pingSt.rssi,pingSt.snr,pingSt.rtt/1000.0,pingSt.hops);pc=cGREEN;break;
   case PG_NAK:snprintf(b,sizeof b,"NAK (error %u)  %d dBm  SNR %.1f",unsigned(pingSt.err),pingSt.rssi,pingSt.snr);pc=cRED;break;
   case PG_TIMEOUT:snprintf(b,sizeof b,"no ACK (out of range or other key)");pc=cRED;break;
   case PG_BLOCKED:snprintf(b,sizeof b,"not sent: duty limit or radio busy");pc=cAMBER;break;
   default:snprintf(b,sizeof b,"hold KEY to ping");}
  txt(b,10,141,1,pc,cPANEL);}
 else txt("hold KEY: ping the selected node",10,137,1,cDIM,cPANEL);}

// ---- Ustawienia ----
constexpr unsigned SET_N=9;
void setLine(unsigned i,char* name,size_t nc,char* val,size_t vc){
 static const char* N[SET_N]={"Role","Link preset","Link frequency","TX power","Probe interval","Mesh preset","Invert colors","Flip screen","TFT power pin"};
 snprintf(name,nc,"%s",N[i]);const Prof p=profFor(M_LINK);
 switch(i){
  case 0:snprintf(val,vc,"%s",cfg.role==0?"INIT":"ECHO");break;
  case 1:snprintf(val,vc,"%s SF%u",pre::get(cfg.linkPreset).name,pre::get(cfg.linkPreset).sf);break;
  case 2:snprintf(val,vc,"%.3f MHz",(double)pre::linkFreq(cfg.linkFreq));break;
  case 3:snprintf(val,vc,"%d dBm%s",int(p.pwr),pre::linkPower(cfg.power)>pre::powerLimit(pre::linkFreq(cfg.linkFreq))?" (limit)":"");break;
  case 4:snprintf(val,vc,"%lu s",(unsigned long)(pre::linkInterval(cfg.interval)/1000));break;
  case 5:snprintf(val,vc,"%s",pre::get(cfg.meshPreset).name);break;
  case 6:snprintf(val,vc,"%s",cfg.invert?"ON":"OFF");break;
  case 7:snprintf(val,vc,"%s",cfg.flip?"180 deg":"normal");break;
  default:snprintf(val,vc,"%s",cfg.pwrHigh?"HIGH = on":"LOW = on");}}
void drawSet(){
 unsigned first=setSel<3?0:setSel-3;if(first+6>SET_N)first=SET_N-6;
 for(unsigned r=0;r<6;r++){unsigned i=first+r;int y=21+int(r)*21;bool sel=i==setSel;char nm[24],v[32];setLine(i,nm,sizeof nm,v,sizeof v);
  panel(4,y,312,20,sel?cOKBG:cPANEL,sel?cGREEN:cLINE);txt(nm,12,y+2,2,sel?cWHITE:cDIM,sel?cOKBG:cPANEL);txt(v,308,y+2,2,sel?cGREEN:cWHITE,sel?cOKBG:cPANEL,TR_DATUM);}}

void render(){
 spr.fillSprite(cBG);
 switch(screen){case S_HOME:drawHome();break;case S_LINK:drawLink();break;case S_NOISE:drawNoise();break;case S_SCAN:drawScan();break;case S_MESH:drawMesh();break;default:drawSet();}
 drawHeader();
 switch(screen){
  case S_HOME:drawFooter("PRG: next screen",cfg.role==0?"KEY: pause/run":"");break;
  case S_LINK:drawFooter("PRG: next screen",cfg.role==0?"KEY: pause  hold: reset":"hold KEY: reset");break;
  case S_NOISE:drawFooter("PRG: next screen","KEY: window  hold: clear");break;
  case S_SCAN:drawFooter("PRG: next screen","KEY: pause  hold: clear");break;
  case S_MESH:drawFooter("PRG: next screen","KEY: next node  hold: ping");break;
  default:drawFooter("PRG: item  hold: exit","KEY: change  hold: back");}
 spr.pushSprite(0,0);}

// ================= NVS =================
void cfgLoad(){prefs.begin("t190",true);size_t n=prefs.getBytesLength("cfg");if(n==sizeof cfg)prefs.getBytes("cfg",&cfg,sizeof cfg);prefs.end();
 if(cfg.role>1)cfg.role=0;if(cfg.linkPreset>=pre::N)cfg.linkPreset=4;if(cfg.meshPreset>=pre::N)cfg.meshPreset=0;if(cfg.linkFreq>=pre::NFREQ)cfg.linkFreq=0;if(cfg.power>=pre::NPWR)cfg.power=2;if(cfg.interval>=pre::NIV)cfg.interval=1;}
void cfgSave(){prefs.begin("t190",false);prefs.putBytes("cfg",&cfg,sizeof cfg);prefs.end();}

// ================= Ekran / zasilanie =================
// Wlasny identyfikator wezla (losowy, trwaly) uzywany jako nadawca pingow Meshtastic.
void nodeIdLoad(){prefs.begin("t190",true);uint32_t v=0;if(prefs.getBytesLength("nid")==4)prefs.getBytes("nid",&v,4);prefs.end();
 if(!v||v==0xFFFFFFFFu||(v&0xFF)==0||(v&0xFF)==0xFF){do{v=esp_random();}while(!v||v==0xFFFFFFFFu||(v&0xFF)==0||(v&0xFF)==0xFF);prefs.begin("t190",false);prefs.putBytes("nid",&v,4);prefs.end();}
 myId=v;Serial.printf("node id !%08lx\n",(unsigned long)myId);}
void tftPower(){pinMode(PIN_TFT_PWR,OUTPUT);digitalWrite(PIN_TFT_PWR,cfg.pwrHigh?HIGH:LOW);}
void tftApply(){tft.setRotation(cfg.flip?3:1);tft.invertDisplay(cfg.invert!=0);}
void applyChange(unsigned item){
 cfgSave();
 if(item==0||item==1||item==2||item==3||item==4){linkRestart();if(mode==M_LINK)radioApply(profFor(M_LINK));if(item==3&&mode==M_MESH)radioApply(profFor(M_MESH));}
 if(item==5){mtab.clear();pingSt=Ping();meshSelId=0;mtab.defHash=mh::chanHash(pre::get(cfg.meshPreset).name,mh::defaultKey(),16);if(mode==M_MESH)radioApply(profFor(M_MESH));}
 if(item==6||item==7)tftApply();
 if(item==8)tftPower();}
void changeSetting(unsigned i){
 switch(i){
  case 0:cfg.role^=1;break;
  case 1:cfg.linkPreset=(cfg.linkPreset+1)%pre::N;break;
  case 2:cfg.linkFreq=(cfg.linkFreq+1)%pre::NFREQ;break;
  case 3:cfg.power=(cfg.power+1)%pre::NPWR;break;
  case 4:cfg.interval=(cfg.interval+1)%pre::NIV;break;
  case 5:cfg.meshPreset=(cfg.meshPreset+1)%pre::N;break;
  case 6:cfg.invert^=1;break;
  case 7:cfg.flip^=1;break;
  default:cfg.pwrHigh^=1;}
 applyChange(i);}

void gotoScreen(int s){screen=(s+S_COUNT)%S_COUNT;Mode m=modeFor(screen);if(m!=mode)switchMode(m);if(screen==S_SCAN)scanReset();dirty=true;}

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
    case S_HOME:case S_LINK:if(cfg.role==0){lk.on=!lk.on;if(lk.on)lk.nextAt=now+500;}break;
    case S_NOISE:noiseWin=(noiseWin+1)%3;break;
    case S_SCAN:scanPaused=!scanPaused;break;
    case S_MESH:{unsigned idx[mh::Table::MAXN];unsigned n=mtab.order(now,idx);if(n){unsigned cur=0;for(unsigned i=0;i<n;i++)if(mtab.n[idx[i]].id==meshSelId)cur=i;meshSelId=mtab.n[idx[(cur+1)%n]].id;}}break;}}
  else{switch(screen){
    case S_HOME:case S_LINK:linkRestart();break;
    case S_NOISE:nzCur().reset();break;
    case S_SCAN:scanReset();break;
    case S_MESH:pingSend(now);break;}}}}

// ================= setup / loop =================
void setup(){
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
 nodeIdLoad();radioInit();linkRestart();batRead();
 Serial.println("t_ms,mode,noise_now,noise_floor,noise_peak,busy_pct,link_ok,link_lost");
 render();}

void loop(){
 uint32_t now=millis();
 onButtons(now);txPoll(now);rxPoll(now);pingTick(now);noiseTick(now);linkTick(now);
 static uint32_t lastScan=0;if(mode==M_SCAN&&now-lastScan>=40){lastScan=now;scanTick();}
 if(now-lastBatAt>=10000||!lastBatAt){lastBatAt=now?now:1;batRead();dirty=true;}
 static uint32_t lastCsv=0;if(now-lastCsv>=5000){lastCsv=now;nz::Stats s=nzCur().stats(60);
  Serial.printf("%lu,%d,%d,%d,%d,%.0f,%lu,%lu\n",(unsigned long)now,int(mode),s.has?s.now:0,s.has?s.floor:0,s.has?s.peak:0,s.has?s.busy:0.f,(unsigned long)lk.ok,(unsigned long)lk.lost);}
 if((dirty&&now-lastDraw>=80)||now-lastDraw>=500){lastDraw=now;dirty=false;render();}
 delay(1);}
