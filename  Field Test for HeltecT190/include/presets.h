#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
// Presety modemu Meshtastic oraz ustawienia testu lacza (region EU868).
namespace pre {
struct Preset{const char* name;uint8_t sf;float bw;uint8_t cr;};   // cr 5..8 = kodowanie 4/5..4/8
constexpr unsigned N=9;
inline const Preset& get(unsigned i){static const Preset p[N]={{"LongFast",11,250,5},{"LongSlow",12,125,8},{"VLongSlow",12,62.5f,8},{"MediumSlow",10,250,5},{"MediumFast",9,250,5},
 {"ShortSlow",8,250,5},{"ShortFast",7,250,5},{"LongMod",11,125,8},{"ShortTurbo",7,500,5}};return p[i<N?i:0];}
// Czestotliwosc srodkowa kanalu Meshtastic w EU868 (podpasmo 869,4-869,65 MHz): numer slotu = hash(nazwa presetu) % liczba kanalow.
// Dla BW 250 kHz miesci sie jeden kanal, stad 869,525 MHz.
inline float meshFreq(unsigned i){const Preset& p=get(i);const float start=869.4f,end=869.65f;unsigned n=unsigned((end-start)/(p.bw/1000.f)+0.0001f);if(!n)return(start+end)/2;   // szerszy niz podpasmo: srodek podpasma
 uint32_t h=5381;for(const char* c=p.name;*c;c++)h=((h<<5)+h)+uint8_t(*c);unsigned slot=h%n;return start+p.bw/2000.f+slot*p.bw/1000.f;}
// Opcje ustawien testu lacza.
constexpr unsigned NFREQ=4,NPWR=5,NIV=4;
inline float linkFreq(unsigned i){static const float f[NFREQ]={869.525f,868.1f,868.3f,868.5f};return f[i<NFREQ?i:0];}
inline int linkPower(unsigned i){static const int p[NPWR]={2,10,14,17,22};return p[i<NPWR?i:0];}
inline uint32_t linkInterval(unsigned i){static const uint32_t v[NIV]={3000,5000,10000,30000};return v[i<NIV?i:0];}
// Limit wypelnienia (ETSI EN 300 220): 869,4-869,65 MHz 10 %, 868,0-868,6 MHz 1 %, reszta 0,1-1 %. Moc: 868,0-868,6 MHz max 14 dBm.
inline float dutyLimit(float mhz){if(mhz>=869.4f&&mhz<=869.65f)return 0.10f;if(mhz>=868.0f&&mhz<=868.6f)return 0.01f;return 0.001f;}
inline int powerLimit(float mhz){return(mhz>=868.0f&&mhz<=868.6f)?14:22;}
// Czas nadawania w ms z ostatnich 60 minut (kubelki minutowe), sluzy do pilnowania limitu wypelnienia.
struct Duty{uint32_t ms[60]={};uint32_t minute=0;bool started=false;
 void roll(uint32_t nowMs){uint32_t m=nowMs/60000;if(!started||m<minute){started=true;minute=m;memset(ms,0,sizeof ms);return;}while(minute<m){++minute;ms[minute%60]=0;if(m-minute>60){minute=m;memset(ms,0,sizeof ms);}}}
 void add(uint32_t nowMs,uint32_t airMs){roll(nowMs);ms[minute%60]+=airMs;}
 uint32_t used(uint32_t nowMs){roll(nowMs);uint32_t s=0;for(unsigned i=0;i<60;i++)s+=ms[i];return s;}
 float pct(uint32_t nowMs){return used(nowMs)/36000.f;}                              // % z godziny (3 600 000 ms)
 bool allow(uint32_t nowMs,uint32_t airMs,float limit){return used(nowMs)+airMs<=uint32_t(limit*3600000.f);}};
}
