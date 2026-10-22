// Symulacja calego firmware na hoscie: wirtualne radio SX1262, przyciski, zegar, ekran (rejestr operacji rysowania).
#include <cassert>
#include <map>
#include <cstdlib>
#include <cstdio>
#include "../stubs/Arduino.h"
#include "../stubs/RadioLib.h"
#include "../stubs/Preferences.h"
SimRadio gSim;
static unsigned long gNow=0;static uint32_t gRand=1;static bool gBtn[64];static int gPinOut[64];static std::map<std::string,std::vector<uint8_t>> gNvs;
unsigned long millis(){return gNow;}void delay(unsigned long){}void delayMicroseconds(unsigned){}uint32_t esp_random(){return gRand++*2654435761u;}
void pinMode(int,int){}void digitalWrite(int p,int v){gPinOut[p]=v;}
int digitalRead(int p){if(p==14)return gSim.irq?1:0;return gBtn[p]?0:1;}
size_t strlcpy(char*d,const char*s,size_t n){size_t l=strlen(s);if(n){size_t k=l<n-1?l:n-1;memcpy(d,s,k);d[k]=0;}return l;}
void analogSetPinAttenuation(int,int){}uint32_t analogReadMilliVolts(int){return 780;}
void SerialC::begin(unsigned){}int SerialC::printf(const char*,...){return 0;}void SerialC::println(const char*){}SerialC Serial;
static std::string gNs;
void Preferences::begin(const char* n,bool){gNs=n;}void Preferences::end(){}
size_t Preferences::getBytesLength(const char* k){auto i=gNvs.find(gNs+"/"+k);return i==gNvs.end()?0:i->second.size();}
size_t Preferences::getBytes(const char* k,void* b,size_t n){auto i=gNvs.find(gNs+"/"+k);if(i==gNvs.end())return 0;size_t m=std::min(n,i->second.size());memcpy(b,i->second.data(),m);return m;}
size_t Preferences::putBytes(const char* k,const void* b,size_t n){gNvs[gNs+"/"+k].assign((const uint8_t*)b,(const uint8_t*)b+n);return n;}
#include "../src/main.cpp"

static void run(unsigned ms,void(*hook)()=nullptr){for(unsigned i=0;i<ms;i++){++gNow;if(hook)hook();loop();}}
static void press(int pin,bool lng){gBtn[pin]=true;run(lng?760:80);gBtn[pin]=false;run(60);}
static void pressA(bool l=false){press(0,l);}static void pressB(bool l=false){press(21,l);}
static size_t shown=0;
// Zdalny uczestnik: odpowiada na kazdy PROBE w REPLY po 90 ms z zadanym RSSI/SNR.
static bool peerOn=true;static float peerRssi=-92,peerSnr=5;static size_t peerSeen=0;static unsigned long peerReplyAt=0;static lnk::Frame peerFr;static bool peerPend=false;
static void peerHook(){
 if(gSim.tx.size()>peerSeen){auto& p=gSim.tx[peerSeen++];lnk::Frame f;if(peerOn&&lnk::decode(p.data(),p.size(),f)&&f.type==lnk::T_PROBE){peerFr=lnk::Frame();peerFr.type=lnk::T_REPLY;peerFr.session=f.session;peerFr.seq=f.seq;peerFr.rssi=-95;peerFr.snr4=18;peerFr.noise=-110;peerFr.batt=88;peerPend=true;peerReplyAt=gNow+90;}}
 if(peerPend&&gNow>=peerReplyAt&&!gSim.txMode){uint8_t b[lnk::LEN];lnk::encode(peerFr,b);gSim.rx.push_back(SimPkt{std::vector<uint8_t>(b,b+lnk::LEN),peerRssi,peerSnr,0});peerPend=false;gSim.arm();}}
static void dump(const char* name){FILE* f=fopen((std::string("shots/")+name+".txt").c_str(),"w");
 fprintf(f,"%d %d\n",spr.W,spr.H);for(int y=0;y<spr.H;y++){for(int x=0;x<spr.W;x++)fprintf(f,"%04x ",spr.px[size_t(y)*spr.W+x]);fprintf(f,"\n");}
 fprintf(f,"%zu\n",spr.texts.size());for(auto& t:spr.texts){std::string s;for(char c:t.s)s+=(c=='\n'?' ':c);fprintf(f,"%d %d %d %d %u %u|%s\n",t.x0,t.y0,t.font,t.datum,t.fg,t.bg,s.c_str());}fclose(f);}
static int overlaps(const char* name){int n=0;auto& T=spr.texts;for(size_t i=0;i<T.size();i++)for(size_t j=i+1;j<T.size();j++){auto&a=T[i];auto&b=T[j];
  if(a.x0<b.x0+b.w&&b.x0<a.x0+a.w&&a.y0<b.y0+b.h&&b.y0<a.y0+a.h){++n;fprintf(stderr,"  [%s] OVERLAP '%s'(%d,%d %dx%d) x '%s'(%d,%d %dx%d)\n",name,a.s.c_str(),a.x0,a.y0,a.w,a.h,b.s.c_str(),b.x0,b.y0,b.w,b.h);}}return n;}
static int gOverlaps=0,gOob=0;
static void shot(const char* name){render();dump(name);gOverlaps+=overlaps(name);if(spr.textOob){fprintf(stderr,"  [%s] text outside screen: %d\n",name,spr.textOob);gOob+=spr.textOob;}if(spr.oob){fprintf(stderr,"  [%s] pixels outside: %d\n",name,spr.oob);}}

int main(){
 system("mkdir -p shots");
 setup();
 // --- start: parametry radia
 assert(radioOk);assert(fabsf(gSim.freq-869.525f)<1e-3&&gSim.bw==250&&gSim.sf==9&&gSim.cr==5&&gSim.sync==0x12&&gSim.pre==8&&gSim.pwr==14);   // SF9 = MediumFast
 assert(cfg.role==0&&mode==M_LINK&&screen==S_HOME);
 shot("home_start");
 // --- INIT z odpowiadajacym: 3 minuty
 peerSeen=0;run(180000,peerHook);
 fprintf(stderr,"INIT: sent=%lu ok=%lu lost=%lu rtt=%lu avg=%lu duty=%.2f%% up=%d/%.1f dn=%d/%.1f\n",(unsigned long)lk.sent,(unsigned long)lk.ok,(unsigned long)lk.lost,(unsigned long)lk.rttLast,(unsigned long)lk.rttAvg(),duty.pct(gNow),lk.upRssi,lk.upSnr,lk.dnRssi,lk.dnSnr);
 assert(lk.sent>=30&&lk.ok==lk.sent-(lk.pending?1:0)&&lk.lost==0);assert(lk.upRssi==-92&&lk.dnRssi==-95&&fabsf(lk.dnSnr-4.5f)<0.01f);
 assert(lk.rttLast<600);
 shot("home_link");gotoScreen(S_LINK);shot("link_init");
 // --- nic z drugiej strony -> straty
 peerOn=false;run(60000,peerHook);assert(lk.lost>=8);fprintf(stderr,"after silence: lost=%lu loss=%.0f%%\n",(unsigned long)lk.lost,lk.lossPct());shot("link_lost");peerOn=true;
 // --- zmiana ustawien
 gotoScreen(S_SET);for(unsigned i=0;i<SET_N;i++){setSel=i;shot(("set"+std::to_string(i)).c_str());}
 // --- INIT: pauza / wznowienie / reset
 gotoScreen(S_HOME);pressB();assert(!lk.on);size_t n0=gSim.tx.size();run(20000,peerHook);assert(gSim.tx.size()==n0);pressB();assert(lk.on);run(10000,peerHook);assert(gSim.tx.size()>n0);
 pressB(true);assert(lk.sent<=2);
 // --- limit wypelnienia na 868.1 MHz (1 %) z interwalem 3 s
 cfg.linkFreq=1;cfg.interval=0;cfg.linkPreset=0;cfg.power=4;applyChange(2);assert(fabsf(gSim.freq-868.1f)<1e-3&&gSim.pwr==14);   // moc przycieta do 14 dBm
 peerSeen=gSim.tx.size();static bool everBlocked=false;static float maxPct=0;everBlocked=false;maxPct=0;run(3600000UL,[](){peerHook();if(dutyBlocked)everBlocked=true;maxPct=std::max(maxPct,duty.pct(gNow));});fprintf(stderr,"duty test: sent=%lu used=%.2f%% blocked=%d\n",(unsigned long)lk.sent,duty.pct(gNow),int(dutyBlocked));assert(maxPct<=1.0f+1e-3);assert(everBlocked);shot("home_duty");
 cfg.linkFreq=0;cfg.interval=1;cfg.linkPreset=4;cfg.power=2;applyChange(2);
 // --- NOISE: podloga -112, impuls -60 raz na 30 s
 gotoScreen(S_NOISE);nzLink.reset();gSim.noise=-112;
 auto burst=[](){static unsigned c=0;++c;gSim.noise=(c%30000<400)?-60.f:-112.f+float((c*7)%5)-2;};
 run(700000,burst);{nz::Stats s=nzLink.stats(600);fprintf(stderr,"noise10m: now=%d floor=%d peak=%d busy=%.1f\n",s.now,s.floor,s.peak,s.busy);assert(s.has&&s.floor<=-112&&s.floor>=-116&&s.peak==-60);}
 shot("noise_1m");pressB();shot("noise_10m");pressB();shot("noise_1h");pressB();pressB(true);shot("noise_empty");gSim.noise=-112;run(5000);shot("noise_few");
 // --- SCAN
 gotoScreen(S_SCAN);auto scanHook=[](){gSim.noise=-110;if(scanIdx==37||scanIdx==38)gSim.noise=-70;};run(4000,scanHook);shot("scan");
 pressB();assert(scanPaused);pressB();pressB(true);shot("scan_cleared");run(8000,scanHook);
 assert(fabsf(gSim.freq-(SCAN_START+((scanIdx+SCAN_N-1)%SCAN_N)*SCAN_STEP))<0.06f);
 int pk=0;for(unsigned i=0;i<SCAN_N;i++)if(scanPeak[i]>-100)++pk;fprintf(stderr,"scan peaks above -100: %d\n",pk);assert(pk>=1);
 // --- MESH: wstrzykniecie pakietow (NODEINFO z domyslnym kluczem + zaszyfrowany inny kanal)
 gotoScreen(S_MESH);assert(mode==M_MESH&&fabsf(gSim.freq-869.525f)<1e-3&&gSim.sync==0x2B&&gSim.pre==16&&gSim.sf==11);shot("mesh_empty");
 auto mkpkt=[](uint32_t from,uint32_t id,uint8_t chan,const char* lname,const char* sname,bool nodeinfo){std::vector<uint8_t> p(16,0);uint32_t to=0xffffffff;memcpy(&p[0],&to,4);memcpy(&p[4],&from,4);memcpy(&p[8],&id,4);p[12]=uint8_t(2|(3<<5));p[13]=chan;
  std::vector<uint8_t> pl;if(nodeinfo){std::vector<uint8_t> u;u.push_back(0x12);u.push_back(uint8_t(strlen(lname)));u.insert(u.end(),lname,lname+strlen(lname));u.push_back(0x1a);u.push_back(uint8_t(strlen(sname)));u.insert(u.end(),sname,sname+strlen(sname));
   pl={0x08,0x04,0x12,uint8_t(u.size())};pl.insert(pl.end(),u.begin(),u.end());}else pl={0x08,0x01,0x12,0x05,'h','e','l','l','o'};
  uint8_t nonce[16]={};for(int i=0;i<4;i++)nonce[i]=uint8_t(id>>(8*i));for(int i=0;i<4;i++)nonce[8+i]=uint8_t(from>>(8*i));mh::Aes128 a;a.key(mh::defaultKey());std::vector<uint8_t> c(pl.size());a.ctr(nonce,pl.data(),c.data(),pl.size());p.insert(p.end(),c.begin(),c.end());return p;};
 for(int i=0;i<12;i++){char nm[40];snprintf(nm,sizeof nm,"Wezel testowy numer %d dlugi",i);auto p=mkpkt(0x1000+i,500+i,8,nm,"WT",true);gSim.rx.push_back(SimPkt{p,-80.f-i*3,6.f-i*0.7f,0});gSim.arm();run(200);}
 auto p2=mkpkt(0x2222,900,77,"x","y",false);gSim.rx.push_back(SimPkt{p2,-118,-12,0});gSim.arm();run(200);
 gSim.rx.push_back(SimPkt{std::vector<uint8_t>(40,0x55),-100,0,RADIOLIB_ERR_CRC_MISMATCH});gSim.arm();run(200);
 gSim.rx.push_back(SimPkt{std::vector<uint8_t>(5,0x55),-100,0,0});gSim.arm();run(200);   // za krotki
 fprintf(stderr,"mesh: pkts=%lu nodes=%u crc=%lu\n",(unsigned long)mtab.packets,mtab.used,(unsigned long)crcErrors);assert(mtab.used==13&&crcErrors==1&&mtab.packets==13);
 {unsigned idx[40];unsigned n=mtab.order(gNow,idx);int named=0;for(unsigned i=0;i<n;i++)if(mtab.n[idx[i]].name[0])++named;assert(named==12);}
 shot("mesh_list");pressB();pressB();pressB();shot("mesh_scrolled");for(int i=0;i<10;i++)pressB();shot("mesh_wrap");
 // --- ping wezla Meshtastic
 {unsigned idx[40];unsigned n=mtab.order(gNow,idx);assert(n==13);uint32_t tg=meshSelId;assert(tg);size_t t0=gSim.tx.size();
  auto mkack=[&](uint32_t from,uint32_t pid,uint32_t req,int err){std::vector<uint8_t> d={0x08,0x05};if(err>=0){d.push_back(0x12);d.push_back(2);d.push_back(0x18);d.push_back(uint8_t(err));}d.push_back(0x35);for(int i=0;i<4;i++)d.push_back(uint8_t(req>>(8*i)));
   std::vector<uint8_t> p(16,0);for(int i=0;i<4;i++){p[i]=uint8_t(myId>>(8*i));p[4+i]=uint8_t(from>>(8*i));p[8+i]=uint8_t(pid>>(8*i));}p[12]=0x63;p[13]=8;std::vector<uint8_t> c(d.size());mh::ctrCrypt(pid,from,d.data(),c.data(),d.size());p.insert(p.end(),c.begin(),c.end());return p;};
  shot("mesh_before_ping");pressB(true);assert(gSim.tx.size()==t0+1&&pingSt.st==PG_WAIT&&pingSt.target==tg);
  {auto& pk=gSim.tx.back();mh::Hdr h;assert(pk.size()==23&&mh::parse(pk.data(),pk.size(),h));assert(h.to==tg&&h.from==myId&&h.wantAck&&h.hopLimit==0&&h.chan==8&&h.id==pingSt.id);assert(gSim.sync==0x2B&&fabsf(gSim.freq-869.525f)<1e-3);}
  assert(gSim.rxMode&&!gSim.txMode);shot("mesh_ping_wait");pressB(true);assert(gSim.tx.size()==t0+1);   // drugi ping w trakcie oczekiwania nie wychodzi
  // obca odpowiedz, zly request_id i odpowiedz od innego wezla sa ignorowane
  {auto a=mkack(tg,0x5001,pingSt.id+1,0);gSim.rx.push_back(SimPkt{a,-90,4,0});gSim.arm();run(100);assert(pingSt.st==PG_WAIT);
   unsigned idx2[40];mtab.order(gNow,idx2);uint32_t other=meshSelId==0x1000?0x1001:0x1000;auto b2=mkack(other,0x5002,pingSt.id,0);gSim.rx.push_back(SimPkt{b2,-90,4,0});gSim.arm();run(100);assert(pingSt.st==PG_WAIT);}
  run(1500);{auto a=mkack(tg,0x5003,pingSt.id,0);gSim.rx.push_back(SimPkt{a,-97,3.25f,0});gSim.arm();}run(100);
  assert(pingSt.st==PG_ACK&&pingSt.acks==1&&pingSt.sent==1&&pingSt.rssi==-97&&pingSt.hops==0&&pingSt.rtt>=1500&&pingSt.rtt<4000);fprintf(stderr,"ping: ack rssi=%d snr=%.2f rtt=%lu hops=%d\n",pingSt.rssi,pingSt.snr,(unsigned long)pingSt.rtt,pingSt.hops);shot("mesh_ping_ack");
  // drugi ping bez odpowiedzi -> timeout
  pressB(true);assert(pingSt.st==PG_WAIT&&pingSt.sent==2);run(13000);assert(pingSt.st==PG_TIMEOUT&&pingSt.acks==1);shot("mesh_ping_timeout");
  // NAK
  pressB(true);{auto a=mkack(tg,0x5004,pingSt.id,5);gSim.rx.push_back(SimPkt{a,-100,-3,0});gSim.arm();}run(100);assert(pingSt.st==PG_NAK&&pingSt.err==5);shot("mesh_ping_nak");
  // limit wypelnienia blokuje wyslanie
  duty.add(gNow,3600000);pressB(true);assert(pingSt.st==PG_BLOCKED);shot("mesh_ping_blocked");duty=pre::Duty();
  // zmiana wybranego wezla zeruje liczniki
  pressB();pressB(true);assert(pingSt.sent==1&&pingSt.acks==0);run(13000);
  // zmiana presetu Mesh czysci stan
  gotoScreen(S_SET);setSel=5;pressB();assert(pingSt.target==0&&mtab.used==0&&meshSelId==0);cfg.meshPreset=0;applyChange(5);gotoScreen(S_MESH);run(300);}
 shot("mesh_cleared");
 // --- ECHO
 gotoScreen(S_HOME);cfg.role=1;applyChange(0);assert(mode==M_LINK);size_t tx0=gSim.tx.size();
 for(unsigned s=1;s<=5;s++){lnk::Frame f;f.type=lnk::T_PROBE;f.session=42;f.seq=uint16_t(s);f.pwr=14;uint8_t b[lnk::LEN];lnk::encode(f,b);gSim.rx.push_back(SimPkt{std::vector<uint8_t>(b,b+lnk::LEN),-100.f+s,3.f+s*0.5f,0});gSim.arm();
  unsigned long t0=gNow;run(400);assert(gSim.tx.size()==tx0+s);lnk::Frame r;assert(lnk::decode(gSim.tx.back().data(),gSim.tx.back().size(),r));assert(r.type==lnk::T_REPLY&&r.session==42&&r.seq==s&&r.rssi==-100+int(s)&&r.snr4==int(lroundf((3.f+s*0.5f)*4)));
  assert(gSim.txAt.back()-t0>=REPLY_DELAY_MS&&gSim.txAt.back()-t0<REPLY_DELAY_MS+50);}
 assert(probesHeard==5&&repliesSent==5);shot("home_echo");gotoScreen(S_LINK);shot("link_echo");
 // ECHO nie odpowiada na REPLY ani na obce ramki
 {size_t t=gSim.tx.size();lnk::Frame f;f.type=lnk::T_REPLY;f.session=1;f.seq=1;uint8_t b[lnk::LEN];lnk::encode(f,b);gSim.rx.push_back(SimPkt{std::vector<uint8_t>(b,b+lnk::LEN),-90,5,0});gSim.arm();gSim.rx.push_back(SimPkt{std::vector<uint8_t>(12,0xAA),-90,5,0});gSim.arm();run(600);assert(gSim.tx.size()==t);}
 // --- ustawienia, NVS, wszystkie pozycje po kilka razy
 gotoScreen(S_SET);for(unsigned i=0;i<SET_N;i++)for(int k=0;k<10;k++){changeSetting(i);}
 cfg.role=0;cfg.linkPreset=2;cfg.meshPreset=7;cfg.invert=0;cfg.flip=1;cfg.pwrHigh=1;cfgSave();Cfg saved=cfg;cfg=Cfg();cfgLoad();assert(memcmp(&saved,&cfg,sizeof cfg)==0);
 cfg=Cfg();applyChange(0);
 // --- przyciski: caly obieg ekranow
 gotoScreen(S_HOME);for(int i=0;i<2*S_COUNT;i++){pressA();run(300,peerHook);shot(("cycle"+std::to_string(i)).c_str());}
 pressA(true);pressA(true);
 // --- bledy radia
 gSim.beginOk=false;radioOk=false;gotoScreen(S_HOME);shot("home_radio_err");gSim.beginOk=true;
 // --- bateria
 batPct=-1;shot("home_nobat");
 fprintf(stderr,"overlaps=%d textOob=%d\n",gOverlaps,gOob);
 puts("sim OK");return 0;}
