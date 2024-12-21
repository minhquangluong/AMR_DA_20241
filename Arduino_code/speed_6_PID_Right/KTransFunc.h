/*
 This file is a part of KTransFunc.
 
 Author: Ryo Kikuuwe
 
 Copyright (c) 2000-2015 Ryo Kikuuwe
 KTransFunc is a set of functions that deals with transfer functions
 in the Z-transform domain.
 
 KTransFunc is a free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 KTransFunc is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with SiconosPlugin; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 
 Contact: Ryo Kikuuwe, kikuuwe@ieee.org

------------------------------------------------------------------
 Example of lowpass filter:

 Sampling frequency : 	Freq(Hz) --> DeltaT = 1/Freq (s)
 Filter order : 		Order (value = 0,1,2,3 or 4)
 Cut-off Frequency:     CutFreq(Hz)

#include "KTransFunc.h"

double input; // raw data
double output; // filtered data
KTransFunc LowPassFilter; // declare low pass filter

KBWLPass::Make(&LowPassFilter,DeltaT,Order,CutFreq); // create filter using the above parameter

output = LowPassFilter.Work(input); // real-time filtering raw data 
---------------------------------------------------------------------------------
 */
#ifndef Headder_KTransFunc
#define Headder_KTransFunc
#include "math.h"

///////////////////////////////////////////////////////////////

class KTransFunc
{
 public:
    int N;
    int M;
  public:
    double * in ;
    double * out ;
    double * mB ; // relate to in
    double * mA ; // relate to out
  public:
    KTransFunc() :N(-1),M(-1),in(NULL),out(NULL),mB(NULL),mA(NULL) {}
    void Construct(int aN, int aM)
    {
      Destruct();
      N = aN;
      M = aM ;
      in  = new double[N+1];
      out = new double[M+1];
      mB  = new double[N+1];
      mA  = new double[M+1];
      mA[0] = 1;
      Reset();
    }
    void Destruct()
    {
      if(in !=NULL) delete [] in  ; in  = NULL ;
      if(out!=NULL) delete [] out ; out = NULL ;
      if(mB !=NULL) delete [] mB  ; mB  = NULL ;
      if(mA !=NULL) delete [] mA  ; mA  = NULL ;
      N=-1;
      M=-1;
    }
    virtual ~KTransFunc()
    {
      Destruct();
    }
    void Reset()
    {
      if((M==-1)||(N==-1)) return;
      {for(int p=0;p<=N;p++) in [p]=0;}
      {for(int p=0;p<=M;p++) out[p]=0;}
    };
    double Work(const double inValue)
      {
	//if(in == NULL) perror("Transfunc Not Allocated \n");
        {for(int p=N-1 ;p>=0 ;p--) in [p+1]=in[p];}
        {for(int p=M-1 ;p>=0 ;p--) out[p+1]=out[p];}
        in [0] = inValue;
        out[0] = 0  ;
        {for(int p=N ;p>=0 ;p--) out[0] += mB[p] * in [p] ;}
        {for(int p=M ;p>=1 ;p--) out[0] -= mA[p] * out[p] ;}
	return out[0];
      }
};
///////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
class KBWLPass
{
 public:
  static void Make(KTransFunc* const pTF, double aDeltaT, int Order ,double aHerz)   //  1/(s+w)
    {
      if(Order==0 || aHerz<= 0)
	{
	  pTF->Construct(0,0);
	  pTF->mB[0] = 1.;
	}
      else if(0.5 < aHerz*aDeltaT)
	{
	  pTF->Construct(0,0);
	  pTF->mB[0] = 1.;
	 // perror("[Wrn] Cutoff Freq Too Large.\n");
	}
      else if(Order==1)
	{
	  pTF->Construct(1,1);
	  double tW=2./ aDeltaT * tan( aDeltaT * PI * aHerz );
	  pTF->mB[0]= aDeltaT*tW/(2.+aDeltaT*tW);
	  pTF->mB[1]= aDeltaT*tW/(2.+aDeltaT*tW);
	  pTF->mA[0]= 1;
	  pTF->mA[1]= (-2.+aDeltaT*tW)/(2.+aDeltaT*tW);
	}
      else if(Order==2)
	{
	  double tW=2./ aDeltaT * tan( aDeltaT * PI * aHerz );
	  double tG=tW * aDeltaT /2;
	  pTF->Construct(2,2);
	  double tDenomi = 1./(1. + sqrt(2.)*tG + tG*tG );
	  pTF->mB[0] = tDenomi * tG*tG;
	  pTF->mB[1] = 2.* pTF->mB[0];
	  pTF->mB[2] =     pTF->mB[0];
	  pTF->mA[1] = tDenomi * 2.*(tG*tG-1.);
	  pTF->mA[2] = tDenomi * (tG*tG - sqrt(2.)*tG +1. );
	}
      else if(Order==3)
	{
	  double tW=2./ aDeltaT * tan( aDeltaT * PI * aHerz );
	  double tG=tW * aDeltaT /2;
	  pTF->Construct(3,3);
	  double tDenomi = 1./(1. + tG)/(1. + tG + tG * tG ) ;
	  pTF->mB[0]=tDenomi * ( tG*tG*tG )  ;
	  pTF->mB[1]= 3.* pTF->mB[0] ;
	  pTF->mB[2]= 3.* pTF->mB[0] ;
	  pTF->mB[3]=     pTF->mB[0] ;
	  pTF->mA[1]=tDenomi * ( -3. - 2.*tG + 2.*tG*tG+ 3.*tG*tG*tG ) ;
	  pTF->mA[2]=tDenomi * ( 1. + tG )*(3. - 5.*tG + 3.*tG*tG )  ;
	  pTF->mA[3]=tDenomi * ( -1. + 2.*tG - 2.*tG*tG + tG*tG*tG ) ;
	}
      else if(Order==4)
	{
	  double tW=2./ aDeltaT * tan( aDeltaT * PI * aHerz );
	  double tG=tW * aDeltaT /2;
	  pTF->Construct(4,4);
	  double tDenomi = 1./(1. + 2.*cos(PI/8)*tG + tG*tG )/(1. + 2.*cos(3.*PI/8)*tG + tG*tG) ;
	  pTF->mB[0]= tDenomi * tG*tG*tG*tG ;
	  pTF->mB[1]= pTF->mB[0]*4. ;
	  pTF->mB[2]= pTF->mB[0]*6. ;
	  pTF->mB[3]= pTF->mB[0]*4. ;
	  pTF->mB[4]= pTF->mB[0]    ;
	  pTF->mA[1]= tDenomi* 4.*(-1. + tG*tG)*(1. + (cos(PI/8) +cos(3.*PI/8)) *tG + tG*tG) ;
	  pTF->mA[2]= tDenomi* 2.*( 3. - (2.+sqrt(2.))*tG*tG + 3. *tG*tG*tG*tG) ;
	  pTF->mA[3]= tDenomi* 4.*(-1. + tG*tG)*(1. - (cos(PI/8)+cos(3*PI/8)) * tG  + tG*tG)   ;
	  pTF->mA[4]= tDenomi* (1. - 2.*cos(PI/8)*tG + tG*tG) * (1. - 2.*tG*cos(3.*PI/8) + tG*tG)  ;
	}
      else
	{
	  pTF->Construct(0,0);
	  pTF->mB[0] = 1.;
	  printf("not implemented yet");
	}
      pTF->Reset();
    }
};
///////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
class KBWHPass
{
 public:
  static void Make(KTransFunc* const pTF, double aDeltaT, int Order ,double aHerz)   //  s/(s+w)
    {
      if(Order==0 || aHerz<= 0)
	{
	  pTF->Construct(0,0);
	  pTF->mB[0] = 1.;
	}
      else if(0.5 < aHerz*aDeltaT)
	{
	  pTF->Construct(0,0);
	  pTF->mB[0] = 1.;
	  perror("[Wrn] Cutoff Freq Too Large.\n");
	}
      else if(Order==1)
	{
	  pTF->Construct(1,1);
	  double tW=2./ aDeltaT * tan( aDeltaT * PI * aHerz );
	  pTF->mB[0]= 2./(2.+aDeltaT*tW);
	  pTF->mB[1]= -2./(2.+aDeltaT*tW);
	  pTF->mA[0]= 1;
	  pTF->mA[1]= (-2.+aDeltaT*tW)/(2.+aDeltaT*tW);
	}
      else
	{
	  pTF->Construct(0,0);
	  pTF->mB[0] = 1.;
	  printf("not implemented yet");
	}
      pTF->Reset();
    }
};
#endif
