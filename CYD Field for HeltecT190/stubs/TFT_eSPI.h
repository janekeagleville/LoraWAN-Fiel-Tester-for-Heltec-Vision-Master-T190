#pragma once
#include <cmath>
#include <Arduino.h>
#include <vector>
#define TL_DATUM 0
#define TC_DATUM 1
#define TR_DATUM 2
#define ML_DATUM 3
#define MC_DATUM 4
#define MR_DATUM 5
// Zapis operacji rysowania: potem tools/render.py sklada z nich obraz PNG, a test sprawdza granice i zachodzenie tekstow.
struct TextOp{int x,y,font,datum,w,h;uint32_t fg,bg;std::string s;int x0,y0;};
struct Gfx{
 static int fw(int f){return f==1?6:f==2?8:14;}static int fh(int f){return f==1?8:f==2?16:26;}
 int W=320,H=170,dat=0;uint32_t fg=0xFFFF,bg=0;
 std::vector<uint16_t> px;std::vector<TextOp> texts;int oob=0,textOob=0;
 void clear(uint32_t c){px.assign(size_t(W)*H,uint16_t(c));texts.clear();oob=textOob=0;}
 void dp(int x,int y,uint32_t c){if(x<0||y<0||x>=W||y>=H){++oob;return;}px[size_t(y)*W+x]=uint16_t(c);}
 void drawPixel(int x,int y,uint32_t c){dp(x,y,c);}
 void fillRect(int x,int y,int w,int h,uint32_t c){for(int j=0;j<h;j++)for(int i=0;i<w;i++)dp(x+i,y+j,c);}
 void drawRect(int x,int y,int w,int h,uint32_t c){drawFastHLine(x,y,w,c);drawFastHLine(x,y+h-1,w,c);drawFastVLine(x,y,h,c);drawFastVLine(x+w-1,y,h,c);}
 void drawFastHLine(int x,int y,int w,uint32_t c){for(int i=0;i<w;i++)dp(x+i,y,c);}
 void drawFastVLine(int x,int y,int h,uint32_t c){for(int j=0;j<h;j++)dp(x,y+j,c);}
 void drawLine(int x0,int y0,int x1,int y1,uint32_t c){int dx=abs(x1-x0),sx=x0<x1?1:-1,dy=-abs(y1-y0),sy=y0<y1?1:-1,e=dx+dy;for(;;){dp(x0,y0,c);if(x0==x1&&y0==y1)break;int e2=2*e;if(e2>=dy){e+=dy;x0+=sx;}if(e2<=dx){e+=dx;y0+=sy;}}}
 void fillRoundRect(int x,int y,int w,int h,int,uint32_t c){fillRect(x,y,w,h,c);}
 void drawRoundRect(int x,int y,int w,int h,int,uint32_t c){drawRect(x,y,w,h,c);}
 void drawCircle(int x,int y,int r,uint32_t c){int px=x+r,py=y;for(int a=1;a<=48;a++){float t=a*6.2831853f/48;int nx=x+int(r*cosf(t)+(cosf(t)>=0?.5f:-.5f)),ny=y+int(r*sinf(t)+(sinf(t)>=0?.5f:-.5f));drawLine(px,py,nx,ny,c);px=nx;py=ny;}}
 void fillCircle(int x,int y,int r,uint32_t c){for(int j=-r;j<=r;j++)for(int i=-r;i<=r;i++)if(i*i+j*j<=r*r)dp(x+i,y+j,c);}
 void setTextDatum(uint8_t d){dat=d;}void setTextColor(uint32_t f,uint32_t b){fg=f;bg=b;}
 int textWidth(const char* s,int f){return int(strlen(s))*fw(f);}int textWidth(const String& s,int f){return textWidth(s.c_str(),f);}
 int drawString(const char* s,int x,int y,int f){TextOp t;t.s=s;t.x=x;t.y=y;t.font=f;t.datum=dat;t.fg=fg;t.bg=bg;t.w=textWidth(s,f);t.h=fh(f);
  int col=dat%3,row=dat/3;t.x0=col==0?x:col==1?x-t.w/2:x-t.w;t.y0=row==0?y:row==1?y-t.h/2:y-t.h;
  if(t.x0<0||t.y0<0||t.x0+t.w>W||t.y0+t.h>H)++textOob;texts.push_back(t);return t.w;}
 int drawString(const String& s,int x,int y,int f){return drawString(s.c_str(),x,y,f);}
};
struct TFT_eSPI:Gfx{int pushes=0;bool inv=false;int rot=0;
 void init(){}void writecommand(uint8_t){}void invertDisplay(bool i){inv=i;}void setRotation(int r){rot=r;}void fillScreen(uint32_t c){(void)c;}};
struct TFT_eSprite:Gfx{TFT_eSprite(TFT_eSPI*){}int pushes=0;
 void* createSprite(int w,int h){W=w;H=h;clear(0);return this;}void fillSprite(uint32_t c){clear(c);}void pushSprite(int,int){++pushes;}};
