#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "config.h"
// Wlasny protokol testu lacza ("ping-pong"): inicjator wysyla PROBE, odpowiadajacy odsyla REPLY z RSSI/SNR, z jakimi uslyszal probe.
// Dzieki temu widac jakosc w obu kierunkach. Ramka ma stala dlugosc, wiec czas antenowy jest przewidywalny.
namespace lnk {
constexpr uint8_t MAGIC=0xF7,T_PROBE=1,T_REPLY=2;
constexpr size_t LEN=12;
struct Frame{uint8_t type=0,session=0;uint16_t seq=0;int8_t pwr=0,rssi=0,snr4=0,noise=0;uint8_t batt=255;};   // snr4 = SNR*4, batt 255 = brak
// 0 magic | 1 typ | 2 sesja | 3-4 numer (BE) | 5 moc nadawania dBm | 6 RSSI | 7 SNR*4 | 8 szum | 9 bateria % | 10 zarezerwowane | 11 XOR 0..10
inline size_t encode(const Frame& f,uint8_t* o){o[0]=MAGIC;o[1]=f.type;o[2]=f.session;o[3]=uint8_t(f.seq>>8);o[4]=uint8_t(f.seq);o[5]=uint8_t(f.pwr);o[6]=uint8_t(f.rssi);o[7]=uint8_t(f.snr4);o[8]=uint8_t(f.noise);o[9]=f.batt;o[10]=0;
 uint8_t x=0;for(unsigned i=0;i<11;i++)x^=o[i];o[11]=x;return LEN;}
inline bool decode(const uint8_t* b,size_t n,Frame& f){if(n!=LEN||b[0]!=MAGIC)return false;uint8_t x=0;for(unsigned i=0;i<11;i++)x^=b[i];if(x!=b[11])return false;if(b[1]!=T_PROBE&&b[1]!=T_REPLY)return false;
 Frame o;o.type=b[1];o.session=b[2];o.seq=uint16_t(b[3]<<8|b[4]);o.pwr=int8_t(b[5]);o.rssi=int8_t(b[6]);o.snr4=int8_t(b[7]);o.noise=int8_t(b[8]);o.batt=b[9];f=o;return true;}
inline int8_t clampDb(float v,int lo,int hi){int r=int(lroundf(v));return int8_t(r<lo?lo:r>hi?hi:r);}

// Ocena odbioru wzgledem granicy demodulacji dla danego SF (SNR graniczny: SF7 -7,5 dB, kazdy kolejny SF o 2,5 dB mniej).
enum Q{Q_NONE,Q_GOOD,Q_FAIR,Q_BAD};
inline float snrLimit(unsigned sf){return -7.5f-2.5f*float(int(sf)-7);}
inline float margin(float snr,unsigned sf){return snr-snrLimit(sf);}
inline Q quality(bool has,float snr,unsigned sf){if(!has)return Q_NONE;float m=margin(snr,sf);return m>=10?Q_GOOD:m>=3?Q_FAIR:Q_BAD;}
inline const char* qText(Q q){return q==Q_GOOD?"GOOD":q==Q_FAIR?"MARGINAL":q==Q_BAD?"WEAK":"NO DATA";}

struct Stats{
 bool on=true,pending=false;uint8_t session=0;uint16_t seq=0;uint32_t sent=0,ok=0,lost=0,sentAt=0,nextAt=0,lastOkAt=0,rttLast=0;uint64_t rttSum=0;
 // uplink = ostatni REPLY uslyszany tutaj, downlink = RSSI/SNR PROBE zgloszone przez drugi koniec
 bool hasUp=false,hasDn=false;int upRssi=0,dnRssi=0,dnNoise=0;float upSnr=0,dnSnr=0;uint8_t peerBatt=255;
 int8_t histUp[LINK_HIST],histDn[LINK_HIST];unsigned hn=0,hhead=0;   // historia RSSI (127 = brak)
 Stats(){memset(histUp,127,sizeof histUp);memset(histDn,127,sizeof histDn);}
 void reset(){bool o=on;*this=Stats();on=o;}
 void push(int8_t up,int8_t dn){histUp[hhead]=up;histDn[hhead]=dn;hhead=(hhead+1)%LINK_HIST;if(hn<LINK_HIST)++hn;}
 // i-ty wpis od najstarszego
 int8_t up(unsigned i)const{return histUp[(hhead+LINK_HIST-hn+i)%LINK_HIST];}
 int8_t dn(unsigned i)const{return histDn[(hhead+LINK_HIST-hn+i)%LINK_HIST];}
 void startProbe(uint32_t now){++seq;++sent;pending=true;sentAt=now;}
 // REPLY pasuje do wyslanego PROBE (ta sama sesja i numer) - rtt liczone od poczatku nadawania, bez opoznienia odpowiedzi
 bool onReply(const Frame& f,float rxRssi,float rxSnr,uint32_t now){if(!pending||f.type!=T_REPLY||f.session!=session||f.seq!=seq)return false;
  pending=false;++ok;lastOkAt=now;uint32_t rtt=now-sentAt;rtt=rtt>REPLY_DELAY_MS?rtt-REPLY_DELAY_MS:0;rttLast=rtt;rttSum+=rtt;
  hasUp=true;upRssi=int(lroundf(rxRssi));upSnr=rxSnr;hasDn=true;dnRssi=f.rssi;dnSnr=f.snr4/4.f;dnNoise=f.noise;peerBatt=f.batt;
  push(clampDb(rxRssi,-127,0),f.rssi);return true;}
 bool timeout(uint32_t now,uint32_t limitMs){if(!pending||now-sentAt<limitMs)return false;pending=false;++lost;push(127,127);return true;}
 float lossPct()const{uint32_t done=ok+lost;return done?100.f*lost/done:0.f;}
 uint32_t rttAvg()const{return ok?uint32_t(rttSum/ok):0;}
};
}
