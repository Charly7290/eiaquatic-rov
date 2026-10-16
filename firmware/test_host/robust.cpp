#include <initializer_list>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "control.h"
// Planta: el pistón real tiene carrera trueStroke; el firmware estima con strokeEst (desajuste real).
float run(float gain, float noise, float trueStroke, float strokeEst, float b0, float* settle, float* over, int* switches){
  ctl::DepthHoldParams p = {0.05f, 0.10f, 0.15f, 0.5f, 0.15f, 0.10f};
  ctl::DepthHold h; ctl::Ballast est; float bTrue=0.5f, depth=0.3f, vz=0; est.pos=0.5f; h.reset(1.0f, est.pos);
  float dEst=0.3f, vEst=0, prev=0.3f, worst=0; *over=0; *switches=0; int8_t last=0; srand(1);
  const float dt=0.02f; int k=0;
  for(int i=0;i<(int)(300/dt);i++){
    if(i%2==0){ float meas=depth+noise*((rand()/(float)RAND_MAX)*2-1);
      prev=dEst; dEst+=0.3f*(meas-dEst); vEst+=0.2f*((dEst-prev)/0.04f-vEst); }
    int8_t req=h.update(dEst,vEst,est.pos,0,dt,p);
    int8_t out=est.update(req,bTrue>=1,bTrue<=0,dt,strokeEst,90);
    if(out!=last && out!=0) (*switches)++; last=out;
    bTrue+=out*dt/trueStroke; if(bTrue<0)bTrue=0; if(bTrue>1)bTrue=1;
    float acc=gain*(bTrue-b0)-2.0f*vz*fabsf(vz)-0.3f*vz; vz+=acc*dt; depth+=vz*dt; if(depth<0){depth=0;vz=0;}
    if(depth>1) *over=fmaxf(*over,depth-1);
    if(fabsf(depth-1)>0.15f) k=i;
    if(i*dt>200) worst=fmaxf(worst,fabsf(depth-1));
  }
  *settle=k*dt; return worst;
}
int main(){
  float g[]={0.2f,0.6f,1.5f}, n[]={0.0f,0.02f}, s[]={10,20,40}; int bad=0;
  for(float gg:g) for(float nn:n) for(float ss:s) for(float b0: {0.5f, 0.65f}){
    float st,ov; int sw; float w=run(gg,nn,ss,20,b0,&st,&ov,&sw);
    printf("G=%.1f ruido=±%.0fcm carrera real=%2.0fs neutro=%.2f -> sobrepaso %.2f m, asentado (±15cm) %5.1f s, error t>200s %.2f m, arranques motor %d\n",gg,nn*100,ss,b0,ov,st,w,sw);
    if(w>0.15f) bad++;
  }
  printf("\ncasos fuera de ±15 cm después de 200 s: %d\n", bad);
}
