#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "proto.h"
// Pasywny podsluch Meshtastic: 16-bajtowy naglowek pakietu LoRa jest jawny, wiec bez klucza widac nadawce, hopy, RSSI i SNR.
// Jesli siec uzywa domyslnego klucza kanalu (AQ==), z pakietow NODEINFO odczytujemy tez nazwy wezlow. Tresc wiadomosci nie jest wyswietlana.
namespace mh {
// ---- AES-128 (tylko szyfrowanie bloku + tryb CTR), bez zaleznosci od mbedtls ----
inline const uint8_t* sbox(){static uint8_t s[256];static bool init=false;if(!init){uint8_t p=1,q=1;
 do{p=uint8_t(p^(p<<1)^((p&0x80)?0x1B:0));q^=uint8_t(q<<1);q^=uint8_t(q<<2);q^=uint8_t(q<<4);if(q&0x80)q^=0x09;
  uint8_t x=uint8_t(q^((q<<1)|(q>>7))^((q<<2)|(q>>6))^((q<<3)|(q>>5))^((q<<4)|(q>>4)));s[p]=uint8_t(x^0x63);}while(p!=1);s[0]=0x63;init=true;}return s;}
inline uint8_t xt(uint8_t x){return uint8_t((x<<1)^((x>>7)*0x1B));}
struct Aes128{uint8_t rk[176];
 void key(const uint8_t* k){const uint8_t* S=sbox();memcpy(rk,k,16);uint8_t rc=1;for(unsigned i=16;i<176;i+=4){uint8_t t[4]={rk[i-4],rk[i-3],rk[i-2],rk[i-1]};
  if(i%16==0){uint8_t u=t[0];t[0]=uint8_t(S[t[1]]^rc);t[1]=S[t[2]];t[2]=S[t[3]];t[3]=S[u];rc=xt(rc);}for(unsigned j=0;j<4;j++)rk[i+j]=uint8_t(rk[i-16+j]^t[j]);}}
 void enc(const uint8_t* in,uint8_t* out)const{const uint8_t* S=sbox();uint8_t s[16];for(unsigned i=0;i<16;i++)s[i]=uint8_t(in[i]^rk[i]);
  for(unsigned r=1;r<=10;r++){uint8_t t[16];for(unsigned i=0;i<16;i++)t[i]=S[s[i]];
   // ShiftRows (stan w kolejnosci kolumnami)
   uint8_t u[16];for(unsigned c=0;c<4;c++)for(unsigned row=0;row<4;row++)u[c*4+row]=t[((c+row)%4)*4+row];
   if(r<10){for(unsigned c=0;c<4;c++){uint8_t* a=u+c*4;uint8_t a0=a[0],a1=a[1],a2=a[2],a3=a[3];uint8_t all=uint8_t(a0^a1^a2^a3);
     t[c*4+0]=uint8_t(a0^all^xt(uint8_t(a0^a1)));t[c*4+1]=uint8_t(a1^all^xt(uint8_t(a1^a2)));t[c*4+2]=uint8_t(a2^all^xt(uint8_t(a2^a3)));t[c*4+3]=uint8_t(a3^all^xt(uint8_t(a3^a0)));}
    memcpy(u,t,16);}
   for(unsigned i=0;i<16;i++)s[i]=uint8_t(u[i]^rk[r*16+i]);}
  memcpy(out,s,16);}
 // CTR jak w mbedtls: licznik 128-bitowy big-endian rosnacy od ostatniego bajtu.
 void ctr(const uint8_t* nonce,const uint8_t* in,uint8_t* out,size_t n)const{uint8_t c[16],ks[16];memcpy(c,nonce,16);
  for(size_t i=0;i<n;i+=16){enc(c,ks);size_t k=n-i<16?n-i:16;for(size_t j=0;j<k;j++)out[i+j]=uint8_t(in[i+j]^ks[j]);for(int b=15;b>=0;b--)if(++c[b])break;}}
};
// Domyslny klucz kanalu Meshtastic (PSK "AQ==", indeks 1).
inline const uint8_t* defaultKey(){static const uint8_t k[16]={0xd4,0xf1,0xbb,0x3a,0x20,0x29,0x07,0x59,0xf0,0xbc,0xff,0xab,0xcf,0x4e,0x69,0x01};return k;}
// Bajt skrotu kanalu w naglowku = XOR bajtow nazwy ^ XOR bajtow klucza (domyslny kanal "LongFast" daje 8).
inline uint8_t chanHash(const char* name,const uint8_t* key,size_t kl){uint8_t h=0;for(const char* c=name;*c;c++)h^=uint8_t(*c);for(size_t i=0;i<kl;i++)h^=key[i];return h;}

struct Hdr{uint32_t to=0,from=0,id=0;uint8_t hopLimit=0,hopStart=0,chan=0;bool wantAck=false,viaMqtt=false;};
// Naglowek: to, from, id (u32 LE), flagi (hop_limit 0-2, want_ack 3, via_mqtt 4, hop_start 5-7), skrot kanalu, next_hop, relay_node.
inline bool parse(const uint8_t* b,size_t len,Hdr& h){if(len<16||len>255)return false;Hdr o;
 auto u32=[&](size_t i){return uint32_t(b[i])|uint32_t(b[i+1])<<8|uint32_t(b[i+2])<<16|uint32_t(b[i+3])<<24;};
 o.to=u32(0);o.from=u32(4);o.id=u32(8);if(!o.from||!o.id)return false;uint8_t f=b[12];o.hopLimit=f&7;o.wantAck=f&8;o.viaMqtt=f&16;o.hopStart=uint8_t(f>>5);o.chan=b[13];h=o;return true;}
inline int hops(const Hdr& h){return(h.hopStart>0&&h.hopStart>=h.hopLimit)?int(h.hopStart-h.hopLimit):-1;}
// Rozszyfrowanie ladunku domyslnym kluczem i wyciagniecie nazwy z NODEINFO (portnum 4: User{long_name=2, short_name=3}).
// Zwraca true tylko dla poprawnie zdekodowanego NODEINFO; w innych przypadkach (inny klucz, inny port) false.
inline bool nodeName(const uint8_t* pkt,size_t len,const Hdr& h,char* out,size_t cap){if(len<=16+2||cap<2)return false;static Aes128 a;static bool init=false;if(!init){a.key(defaultKey());init=true;}
 uint8_t nonce[16]={};uint64_t id=h.id;for(unsigned i=0;i<8;i++)nonce[i]=uint8_t(id>>(8*i));for(unsigned i=0;i<4;i++)nonce[8+i]=uint8_t(h.from>>(8*i));
 uint8_t plain[240];size_t n=len-16;if(n>sizeof plain)return false;a.ctr(nonce,pkt+16,plain,n);
 mesh::Reader r(plain,n);mesh::Field f;unsigned port=0;const uint8_t* pl=nullptr;size_t pn=0;while(r.next(f)){if(f.tag==1&&f.wire==0)port=unsigned(f.value);if(f.tag==2&&f.wire==2){pl=f.bytes;pn=f.size;}}
 if(!r.ok()||port!=4||!pl)return false;mesh::Reader u(pl,pn);char lng[40]={},sh[12]={};while(u.next(f)){if(f.tag==2&&f.wire==2)mesh::copyText(lng,sizeof lng,f);if(f.tag==3&&f.wire==2)mesh::copyText(sh,sizeof sh,f);}
 if(!u.ok())return false;char t[40];size_t k=mesh::displayName(lng[0]?lng:sh,t,sizeof t);if(!k)return false;strlcpy(out,t,cap);return true;}

// ---- Ping wezla Meshtastic: pakiet z zadaniem potwierdzenia (want_ack) i odpowiedz ROUTING_APP ----
// Szyfrowanie/odszyfrowanie ladunku domyslnym kluczem kanalu (nonce: id pakietu LE 8 B + nadawca LE 4 B + 4 B zer).
inline void ctrCrypt(uint32_t id,uint32_t from,const uint8_t* in,uint8_t* out,size_t n){static Aes128 a;static bool init=false;if(!init){a.key(defaultKey());init=true;}
 uint8_t nonce[16]={};uint64_t i64=id;for(unsigned i=0;i<8;i++)nonce[i]=uint8_t(i64>>(8*i));for(unsigned i=0;i<4;i++)nonce[8+i]=uint8_t(from>>(8*i));a.ctr(nonce,in,out,n);}
// Pakiet do wezla 'to': Data{portnum=PRIVATE_APP(256), payload "PG"}. Wezel docelowy odsyla ACK niezaleznie od obslugi portu.
// hopLimit 0 = pakiet nie jest przekazywany dalej (test bezposredni); hopStart ustawiany tak samo.
inline size_t buildPing(uint8_t* out,size_t cap,uint32_t to,uint32_t from,uint32_t id,uint8_t chan,uint8_t hopLimit){
 const uint8_t data[]={0x08,0x80,0x02,0x12,0x02,'P','G'};const size_t n=16+sizeof data;if(cap<n)return 0;
 for(unsigned i=0;i<4;i++){out[i]=uint8_t(to>>(8*i));out[4+i]=uint8_t(from>>(8*i));out[8+i]=uint8_t(id>>(8*i));}
 out[12]=uint8_t((hopLimit&7)|8|((hopLimit&7)<<5));out[13]=chan;out[14]=0;out[15]=uint8_t(from&0xFF);ctrCrypt(id,from,data,out+16,sizeof data);return n;}
// Czy pakiet jest odpowiedzia ROUTING_APP na nasz pakiet 'requestId'? err = kod bledu (0 = ACK).
inline bool ackFor(const uint8_t* pkt,size_t len,const Hdr& h,uint32_t requestId,uint8_t* err){if(len<=16+2||len>16+240)return false;uint8_t plain[240];size_t n=len-16;ctrCrypt(h.id,h.from,pkt+16,plain,n);
 mesh::Reader r(plain,n);mesh::Field f;unsigned port=0;uint32_t req=0;bool hasReq=false;const uint8_t* pl=nullptr;size_t pn=0;
 while(r.next(f)){if(f.tag==1&&f.wire==0)port=unsigned(f.value);else if(f.tag==6&&f.wire==5){req=uint32_t(f.value);hasReq=true;}else if(f.tag==2&&f.wire==2){pl=f.bytes;pn=f.size;}}
 if(!r.ok()||port!=5||!hasReq||req!=requestId)return false;uint8_t e=0;if(pl){mesh::Reader q(pl,pn);while(q.next(f))if(f.tag==3&&f.wire==0)e=uint8_t(f.value);}if(err)*err=e;return true;}

struct Node{uint32_t id=0,last=0,count=0;int rssi=0;float snr=0;int hops=-1;uint8_t chan=0;bool mqtt=false;char name[24]={};};
struct Table{static constexpr unsigned MAXN=40;Node n[MAXN];unsigned used=0;uint32_t packets=0;uint8_t defHash=8;
 void clear(){for(auto& x:n)x=Node{};used=0;packets=0;}
 Node* touch(uint32_t id,uint32_t now){for(unsigned i=0;i<used;i++)if(n[i].id==id)return &n[i];if(used<MAXN){n[used]=Node{};n[used].id=id;return &n[used++];}
  unsigned o=0;for(unsigned i=1;i<used;i++)if(uint32_t(now-n[i].last)>uint32_t(now-n[o].last))o=i;n[o]=Node{};n[o].id=id;return &n[o];}
 // defHash = skrot domyslnego kanalu biezacego presetu; nazwy probujemy odczytac tylko z pakietow tego kanalu.
 void add(const Hdr& h,const uint8_t* pkt,size_t len,int rssi,float snr,uint32_t now){Node* x=touch(h.from,now);x->last=now;x->rssi=rssi;x->snr=snr;x->hops=hops(h);x->chan=h.chan;x->mqtt=h.viaMqtt;++x->count;++packets;
  if(!x->name[0]&&h.chan==defHash)nodeName(pkt,len,h,x->name,sizeof x->name);}
 // Indeksy wezlow od najswiezszego (sortowanie przez wstawianie, max 40 pozycji).
 unsigned order(uint32_t now,unsigned* idx)const{for(unsigned i=0;i<used;i++){unsigned j=i;while(j>0&&uint32_t(now-n[idx[j-1]].last)>uint32_t(now-n[i].last)){idx[j]=idx[j-1];--j;}idx[j]=i;}return used;}
};
// Suma czasu antenowego odebranych pakietow (us) z ostatnich 60 s, w kubelkach sekundowych: obciazenie kanalu przez siec.
struct Air{uint32_t us[60]={};uint32_t sec=0;bool started=false;
 void roll(uint32_t nowMs){uint32_t s=nowMs/1000;if(!started||s<sec){started=true;sec=s;memset(us,0,sizeof us);return;}while(sec<s){++sec;us[sec%60]=0;if(s-sec>60){sec=s;memset(us,0,sizeof us);}}}
 void add(uint32_t nowMs,uint32_t airUs){roll(nowMs);us[sec%60]+=airUs;}
 float pct(uint32_t nowMs){roll(nowMs);uint64_t t=0;for(unsigned i=0;i<60;i++)t+=us[i];return float(t)/600000.f;}   // % z 60 s (60 000 000 us)
};
}
