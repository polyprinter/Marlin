#ifndef THERMISTORTABLES_H_
#define THERMISTORTABLES_H_

#include "Marlin.h"

#define OVERSAMPLENR 16

#if (THERMISTORHEATER_0 == 1) || (THERMISTORHEATER_1 == 1)  || (THERMISTORHEATER_2 == 1) || (THERMISTORBED == 1) //100k bed thermistor

const short temptable_1[][2] PROGMEM = {
{       23*OVERSAMPLENR ,       300     },
{       25*OVERSAMPLENR ,       295     },
{       27*OVERSAMPLENR ,       290     },
{       28*OVERSAMPLENR ,       285     },
{       31*OVERSAMPLENR ,       280     },
{       33*OVERSAMPLENR ,       275     },
{       35*OVERSAMPLENR ,       270     },
{       38*OVERSAMPLENR ,       265     },
{       41*OVERSAMPLENR ,       260     },
{       44*OVERSAMPLENR ,       255     },
{       48*OVERSAMPLENR ,       250     },
{       52*OVERSAMPLENR ,       245     },
{       56*OVERSAMPLENR ,       240     },
{       61*OVERSAMPLENR ,       235     },
{       66*OVERSAMPLENR ,       230     },
{       71*OVERSAMPLENR ,       225     },
{       78*OVERSAMPLENR ,       220     },
{       84*OVERSAMPLENR ,       215     },
{       92*OVERSAMPLENR ,       210     },
{       100*OVERSAMPLENR        ,       205     },
{       109*OVERSAMPLENR        ,       200     },
{       120*OVERSAMPLENR        ,       195     },
{       131*OVERSAMPLENR        ,       190     },
{       143*OVERSAMPLENR        ,       185     },
{       156*OVERSAMPLENR        ,       180     },
{       171*OVERSAMPLENR        ,       175     },
{       187*OVERSAMPLENR        ,       170     },
{       205*OVERSAMPLENR        ,       165     },
{       224*OVERSAMPLENR        ,       160     },
{       245*OVERSAMPLENR        ,       155     },
{       268*OVERSAMPLENR        ,       150     },
{       293*OVERSAMPLENR        ,       145     },
{       320*OVERSAMPLENR        ,       140     },
{       348*OVERSAMPLENR        ,       135     },
{       379*OVERSAMPLENR        ,       130     },
{       411*OVERSAMPLENR        ,       125     },
{       445*OVERSAMPLENR        ,       120     },
{       480*OVERSAMPLENR        ,       115     },
{       516*OVERSAMPLENR        ,       110     },
{       553*OVERSAMPLENR        ,       105     },
{       591*OVERSAMPLENR        ,       100     },
{       628*OVERSAMPLENR        ,       95      },
{       665*OVERSAMPLENR        ,       90      },
{       702*OVERSAMPLENR        ,       85      },
{       737*OVERSAMPLENR        ,       80      },
{       770*OVERSAMPLENR        ,       75      },
{       801*OVERSAMPLENR        ,       70      },
{       830*OVERSAMPLENR        ,       65      },
{       857*OVERSAMPLENR        ,       60      },
{       881*OVERSAMPLENR        ,       55      },
{       903*OVERSAMPLENR        ,       50      },
{       922*OVERSAMPLENR        ,       45      },
{       939*OVERSAMPLENR        ,       40      },
{       954*OVERSAMPLENR        ,       35      },
{       966*OVERSAMPLENR        ,       30      },
{       977*OVERSAMPLENR        ,       25      },
{       985*OVERSAMPLENR        ,       20      },
{       993*OVERSAMPLENR        ,       15      },
{       999*OVERSAMPLENR        ,       10      },
{       1004*OVERSAMPLENR       ,       5       },
{       1008*OVERSAMPLENR       ,       0       } //safety
};
#endif
#if (THERMISTORHEATER_0 == 2) || (THERMISTORHEATER_1 == 2) || (THERMISTORHEATER_2 == 2) || (THERMISTORBED == 2) //200k bed thermistor
const short temptable_2[][2] PROGMEM = {
//200k ATC Semitec 204GT-2
//Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 4.7kohm pullup, voltage divider math, and manufacturer provided temp/resistance
   {1*OVERSAMPLENR, 848},
   {30*OVERSAMPLENR, 300}, //top rating 300C
   {34*OVERSAMPLENR, 290},
   {39*OVERSAMPLENR, 280},
   {46*OVERSAMPLENR, 270},
   {53*OVERSAMPLENR, 260},
   {63*OVERSAMPLENR, 250},
   {74*OVERSAMPLENR, 240},
   {87*OVERSAMPLENR, 230},
   {104*OVERSAMPLENR, 220},
   {124*OVERSAMPLENR, 210},
   {148*OVERSAMPLENR, 200},
   {176*OVERSAMPLENR, 190},
   {211*OVERSAMPLENR, 180},
   {252*OVERSAMPLENR, 170},
   {301*OVERSAMPLENR, 160},
   {357*OVERSAMPLENR, 150},
   {420*OVERSAMPLENR, 140},
   {489*OVERSAMPLENR, 130},
   {562*OVERSAMPLENR, 120},
   {636*OVERSAMPLENR, 110},
   {708*OVERSAMPLENR, 100},
   {775*OVERSAMPLENR, 90},
   {835*OVERSAMPLENR, 80},
   {884*OVERSAMPLENR, 70},
   {924*OVERSAMPLENR, 60},
   {955*OVERSAMPLENR, 50},
   {977*OVERSAMPLENR, 40},
   {993*OVERSAMPLENR, 30},
   {1004*OVERSAMPLENR, 20},
   {1012*OVERSAMPLENR, 10},
   {1016*OVERSAMPLENR, 0},
};

#endif
#if (THERMISTORHEATER_0 == 3) || (THERMISTORHEATER_1 == 3) || (THERMISTORHEATER_2 == 3) || (THERMISTORBED == 3) //mendel-parts
const short temptable_3[][2] PROGMEM = {
                {1*OVERSAMPLENR,864},
                {21*OVERSAMPLENR,300},
                {25*OVERSAMPLENR,290},
                {29*OVERSAMPLENR,280},
                {33*OVERSAMPLENR,270},
                {39*OVERSAMPLENR,260},
                {46*OVERSAMPLENR,250},
                {54*OVERSAMPLENR,240},
                {64*OVERSAMPLENR,230},
                {75*OVERSAMPLENR,220},
                {90*OVERSAMPLENR,210},
                {107*OVERSAMPLENR,200},
                {128*OVERSAMPLENR,190},
                {154*OVERSAMPLENR,180},
                {184*OVERSAMPLENR,170},
                {221*OVERSAMPLENR,160},
                {265*OVERSAMPLENR,150},
                {316*OVERSAMPLENR,140},
                {375*OVERSAMPLENR,130},
                {441*OVERSAMPLENR,120},
                {513*OVERSAMPLENR,110},
                {588*OVERSAMPLENR,100},
                {734*OVERSAMPLENR,80},
                {856*OVERSAMPLENR,60},
                {938*OVERSAMPLENR,40},
                {986*OVERSAMPLENR,20},
                {1008*OVERSAMPLENR,0},
                {1018*OVERSAMPLENR,-20}
        };

#endif
#if (THERMISTORHEATER_0 == 4) || (THERMISTORHEATER_1 == 4) || (THERMISTORHEATER_2 == 4) || (THERMISTORBED == 4) //10k thermistor
const short temptable_4[][2] PROGMEM = {
   {1*OVERSAMPLENR, 430},
   {54*OVERSAMPLENR, 137},
   {107*OVERSAMPLENR, 107},
   {160*OVERSAMPLENR, 91},
   {213*OVERSAMPLENR, 80},
   {266*OVERSAMPLENR, 71},
   {319*OVERSAMPLENR, 64},
   {372*OVERSAMPLENR, 57},
   {425*OVERSAMPLENR, 51},
   {478*OVERSAMPLENR, 46},
   {531*OVERSAMPLENR, 41},
   {584*OVERSAMPLENR, 35},
   {637*OVERSAMPLENR, 30},
   {690*OVERSAMPLENR, 25},
   {743*OVERSAMPLENR, 20},
   {796*OVERSAMPLENR, 14},
   {849*OVERSAMPLENR, 7},
   {902*OVERSAMPLENR, 0},
   {955*OVERSAMPLENR, -11},
   {1008*OVERSAMPLENR, -35}
};
#endif

#if (THERMISTORHEATER_0 == 5) || (THERMISTORHEATER_1 == 5) || (THERMISTORHEATER_2 == 5) || (THERMISTORBED == 5) //100k ParCan thermistor (104GT-2)
const short temptable_5[][2] PROGMEM = {
// ATC Semitec 104GT-2 (Used in ParCan)
// Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 4.7kohm pullup, voltage divider math, and manufacturer provided temp/resistance
{1*OVERSAMPLENR, 713},
   {17*OVERSAMPLENR, 300}, //top rating 300C
   {20*OVERSAMPLENR, 290},
   {23*OVERSAMPLENR, 280},
   {27*OVERSAMPLENR, 270},
   {31*OVERSAMPLENR, 260},
   {37*OVERSAMPLENR, 250},
   {43*OVERSAMPLENR, 240},
   {51*OVERSAMPLENR, 230},
   {61*OVERSAMPLENR, 220},
   {73*OVERSAMPLENR, 210},
   {87*OVERSAMPLENR, 200},
   {106*OVERSAMPLENR, 190},
   {128*OVERSAMPLENR, 180},
   {155*OVERSAMPLENR, 170},
   {189*OVERSAMPLENR, 160},
   {230*OVERSAMPLENR, 150},
   {278*OVERSAMPLENR, 140},
   {336*OVERSAMPLENR, 130},
   {402*OVERSAMPLENR, 120},
   {476*OVERSAMPLENR, 110},
   {554*OVERSAMPLENR, 100},
   {635*OVERSAMPLENR, 90},
   {713*OVERSAMPLENR, 80},
   {784*OVERSAMPLENR, 70},
   {846*OVERSAMPLENR, 60},
   {897*OVERSAMPLENR, 50},
   {937*OVERSAMPLENR, 40},
   {966*OVERSAMPLENR, 30},
   {986*OVERSAMPLENR, 20},
   {1000*OVERSAMPLENR, 10},
   {1010*OVERSAMPLENR, 0}
};
#endif

#if (THERMISTORHEATER_0 == 6) || (THERMISTORHEATER_1 == 6) || (THERMISTORHEATER_2 == 6) || (THERMISTORBED == 6) // 100k Epcos thermistor
const short temptable_6[][2] PROGMEM = {
   {1*OVERSAMPLENR, 350},
   {28*OVERSAMPLENR, 250}, //top rating 250C
   {31*OVERSAMPLENR, 245},
   {35*OVERSAMPLENR, 240},
   {39*OVERSAMPLENR, 235},
   {42*OVERSAMPLENR, 230},
   {44*OVERSAMPLENR, 225},
   {49*OVERSAMPLENR, 220},
   {53*OVERSAMPLENR, 215},
   {62*OVERSAMPLENR, 210},
   {71*OVERSAMPLENR, 205}, //fitted graphically
   {78*OVERSAMPLENR, 200}, //fitted graphically
   {94*OVERSAMPLENR, 190},
   {102*OVERSAMPLENR, 185},
   {116*OVERSAMPLENR, 170},
   {143*OVERSAMPLENR, 160},
   {183*OVERSAMPLENR, 150},
   {223*OVERSAMPLENR, 140},
   {270*OVERSAMPLENR, 130},
   {318*OVERSAMPLENR, 120},
   {383*OVERSAMPLENR, 110},
   {413*OVERSAMPLENR, 105},
   {439*OVERSAMPLENR, 100},
   {484*OVERSAMPLENR, 95},
   {513*OVERSAMPLENR, 90},
   {607*OVERSAMPLENR, 80},
   {664*OVERSAMPLENR, 70},
   {781*OVERSAMPLENR, 60},
   {810*OVERSAMPLENR, 55},
   {849*OVERSAMPLENR, 50},
   {914*OVERSAMPLENR, 45},
   {914*OVERSAMPLENR, 40},
   {935*OVERSAMPLENR, 35},
   {954*OVERSAMPLENR, 30},
   {970*OVERSAMPLENR, 25},
   {978*OVERSAMPLENR, 22},
   {1008*OVERSAMPLENR, 3}
};
#endif

#if (THERMISTORHEATER_0 == 7) || (THERMISTORHEATER_1 == 7) || (THERMISTORHEATER_2 == 7) || (THERMISTORBED == 7) // 100k Honeywell 135-104LAG-J01
const short temptable_7[][2] PROGMEM = {
   {1*OVERSAMPLENR, 500},
   {46*OVERSAMPLENR, 270}, //top rating 300C
   {50*OVERSAMPLENR, 265},
   {54*OVERSAMPLENR, 260},
   {58*OVERSAMPLENR, 255},
   {62*OVERSAMPLENR, 250},
   {67*OVERSAMPLENR, 245},
   {72*OVERSAMPLENR, 240},
   {79*OVERSAMPLENR, 235},
   {85*OVERSAMPLENR, 230},
   {91*OVERSAMPLENR, 225},
   {99*OVERSAMPLENR, 220},
   {107*OVERSAMPLENR, 215},
   {116*OVERSAMPLENR, 210},
   {126*OVERSAMPLENR, 205},
   {136*OVERSAMPLENR, 200},
   {149*OVERSAMPLENR, 195},
   {160*OVERSAMPLENR, 190},
   {175*OVERSAMPLENR, 185},
   {191*OVERSAMPLENR, 180},
   {209*OVERSAMPLENR, 175},
   {224*OVERSAMPLENR, 170},
   {246*OVERSAMPLENR, 165},
   {267*OVERSAMPLENR, 160},
   {293*OVERSAMPLENR, 155},
   {316*OVERSAMPLENR, 150},
   {340*OVERSAMPLENR, 145},
   {364*OVERSAMPLENR, 140},
   {396*OVERSAMPLENR, 135},
   {425*OVERSAMPLENR, 130},
   {460*OVERSAMPLENR, 125},
   {489*OVERSAMPLENR, 120},
   {526*OVERSAMPLENR, 115},
   {558*OVERSAMPLENR, 110},
   {591*OVERSAMPLENR, 105},
   {628*OVERSAMPLENR, 100},
   {660*OVERSAMPLENR, 95},
   {696*OVERSAMPLENR, 90},
   {733*OVERSAMPLENR, 85},
   {761*OVERSAMPLENR, 80},
   {794*OVERSAMPLENR, 75},
   {819*OVERSAMPLENR, 70},
   {847*OVERSAMPLENR, 65},
   {870*OVERSAMPLENR, 60},
   {892*OVERSAMPLENR, 55},
   {911*OVERSAMPLENR, 50},
   {929*OVERSAMPLENR, 45},
   {944*OVERSAMPLENR, 40},
   {959*OVERSAMPLENR, 35},
   {971*OVERSAMPLENR, 30},
   {981*OVERSAMPLENR, 25},
   {989*OVERSAMPLENR, 20},
   {994*OVERSAMPLENR, 15},
   {1001*OVERSAMPLENR, 10},
   {1005*OVERSAMPLENR, 5}
};
#endif

#if (THERMISTORHEATER_0 == 51) || (THERMISTORHEATER_1 == 51) || (THERMISTORHEATER_2 == 51) || (THERMISTORBED == 51) 
// 100k EPCOS (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
// Verified by linagee.
// Calculated using 1kohm pullup, voltage divider math, and manufacturer provided temp/resistance
// Advantage: Twice the resolution and better linearity from 150C to 200C
const short temptable_51[][2] PROGMEM = {
   {1*OVERSAMPLENR, 350},
   {190*OVERSAMPLENR, 250}, //top rating 250C
   {203*OVERSAMPLENR, 245},
   {217*OVERSAMPLENR, 240},
   {232*OVERSAMPLENR, 235},
   {248*OVERSAMPLENR, 230},
   {265*OVERSAMPLENR, 225},
   {283*OVERSAMPLENR, 220},
   {302*OVERSAMPLENR, 215},
   {322*OVERSAMPLENR, 210},
   {344*OVERSAMPLENR, 205},
   {366*OVERSAMPLENR, 200},
   {390*OVERSAMPLENR, 195},
   {415*OVERSAMPLENR, 190},
   {440*OVERSAMPLENR, 185},
   {467*OVERSAMPLENR, 180},
   {494*OVERSAMPLENR, 175},
   {522*OVERSAMPLENR, 170},
   {551*OVERSAMPLENR, 165},
   {580*OVERSAMPLENR, 160},
   {609*OVERSAMPLENR, 155},
   {638*OVERSAMPLENR, 150},
   {666*OVERSAMPLENR, 145},
   {695*OVERSAMPLENR, 140},
   {722*OVERSAMPLENR, 135},
   {749*OVERSAMPLENR, 130},
   {775*OVERSAMPLENR, 125},
   {800*OVERSAMPLENR, 120},
   {823*OVERSAMPLENR, 115},
   {845*OVERSAMPLENR, 110},
   {865*OVERSAMPLENR, 105},
   {884*OVERSAMPLENR, 100},
   {901*OVERSAMPLENR, 95},
   {917*OVERSAMPLENR, 90},
   {932*OVERSAMPLENR, 85},
   {944*OVERSAMPLENR, 80},
   {956*OVERSAMPLENR, 75},
   {966*OVERSAMPLENR, 70},
   {975*OVERSAMPLENR, 65},
   {982*OVERSAMPLENR, 60},
   {989*OVERSAMPLENR, 55},
   {995*OVERSAMPLENR, 50},
   {1000*OVERSAMPLENR, 45},
   {1004*OVERSAMPLENR, 40},
   {1007*OVERSAMPLENR, 35},
   {1010*OVERSAMPLENR, 30},
   {1013*OVERSAMPLENR, 25},
   {1015*OVERSAMPLENR, 20},
   {1017*OVERSAMPLENR, 15},
   {1018*OVERSAMPLENR, 10},
   {1019*OVERSAMPLENR, 5},
   {1020*OVERSAMPLENR, 0},
   {1021*OVERSAMPLENR, -5}
};
#endif

#if (THERMISTORHEATER_0 == 52) || (THERMISTORHEATER_1 == 52) || (THERMISTORHEATER_2 == 52) || (THERMISTORBED == 52) 
// 200k ATC Semitec 204GT-2 (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
// Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 1kohm pullup, voltage divider math, and manufacturer provided temp/resistance
// Advantage: More resolution and better linearity from 150C to 200C
const short temptable_52[][2] PROGMEM = {
   {1*OVERSAMPLENR, 500},
   {125*OVERSAMPLENR, 300}, //top rating 300C
   {142*OVERSAMPLENR, 290},
   {162*OVERSAMPLENR, 280},
   {185*OVERSAMPLENR, 270},
   {211*OVERSAMPLENR, 260},
   {240*OVERSAMPLENR, 250},
   {274*OVERSAMPLENR, 240},
   {312*OVERSAMPLENR, 230},
   {355*OVERSAMPLENR, 220},
   {401*OVERSAMPLENR, 210},
   {452*OVERSAMPLENR, 200},
   {506*OVERSAMPLENR, 190},
   {563*OVERSAMPLENR, 180},
   {620*OVERSAMPLENR, 170},
   {677*OVERSAMPLENR, 160},
   {732*OVERSAMPLENR, 150},
   {783*OVERSAMPLENR, 140},
   {830*OVERSAMPLENR, 130},
   {871*OVERSAMPLENR, 120},
   {906*OVERSAMPLENR, 110},
   {935*OVERSAMPLENR, 100},
   {958*OVERSAMPLENR, 90},
   {976*OVERSAMPLENR, 80},
   {990*OVERSAMPLENR, 70},
   {1000*OVERSAMPLENR, 60},
   {1008*OVERSAMPLENR, 50},
   {1013*OVERSAMPLENR, 40},
   {1017*OVERSAMPLENR, 30},
   {1019*OVERSAMPLENR, 20},
   {1021*OVERSAMPLENR, 10},
   {1022*OVERSAMPLENR, 0}
};
#endif

#if (THERMISTORHEATER_0 == 55) || (THERMISTORHEATER_1 == 55) || (THERMISTORHEATER_2 == 55) || (THERMISTORBED == 55) 
// 100k ATC Semitec 104GT-2 (Used on ParCan) (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
// Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 1kohm pullup, voltage divider math, and manufacturer provided temp/resistance
// Advantage: More resolution and better linearity from 150C to 200C
const short temptable_55[][2] PROGMEM = {
   {1*OVERSAMPLENR, 500},
   {76*OVERSAMPLENR, 300},
   {87*OVERSAMPLENR, 290},
   {100*OVERSAMPLENR, 280},
   {114*OVERSAMPLENR, 270},
   {131*OVERSAMPLENR, 260},
   {152*OVERSAMPLENR, 250},
   {175*OVERSAMPLENR, 240},
   {202*OVERSAMPLENR, 230},
   {234*OVERSAMPLENR, 220},
   {271*OVERSAMPLENR, 210},
   {312*OVERSAMPLENR, 200},
   {359*OVERSAMPLENR, 190},
   {411*OVERSAMPLENR, 180},
   {467*OVERSAMPLENR, 170},
   {527*OVERSAMPLENR, 160},
   {590*OVERSAMPLENR, 150},
   {652*OVERSAMPLENR, 140},
   {713*OVERSAMPLENR, 130},
   {770*OVERSAMPLENR, 120},
   {822*OVERSAMPLENR, 110},
   {867*OVERSAMPLENR, 100},
   {905*OVERSAMPLENR, 90},
   {936*OVERSAMPLENR, 80},
   {961*OVERSAMPLENR, 70},
   {979*OVERSAMPLENR, 60},
   {993*OVERSAMPLENR, 50},
   {1003*OVERSAMPLENR, 40},
   {1010*OVERSAMPLENR, 30},
   {1015*OVERSAMPLENR, 20},
   {1018*OVERSAMPLENR, 10},
   {1020*OVERSAMPLENR, 0}
};
#endif

#if ( THERMISTORHEATER_0 == 100474092 ) || ( THERMISTORHEATER_1 == 100474092) || ( THERMISTORHEATER_2 == 100474092 ) || ( THERMISTORBED == 100474092 ) 
// 100k with 4.7k pullup, Beta of 4092.
// Corresponds to EPCOS B57540G104F
// Calculated using 4.7kohm pullup, voltage divider math, and manufacturer provided beta value
const short temptable_100474092[][2] PROGMEM = {
	{17*OVERSAMPLENR, 350     },
	{18*OVERSAMPLENR, 345     },
	{19*OVERSAMPLENR, 340     },
	{20*OVERSAMPLENR, 335     },
	{21*OVERSAMPLENR, 330     },
	{22*OVERSAMPLENR, 325     },
	{23*OVERSAMPLENR, 320     },
	{24*OVERSAMPLENR, 315     },
	{26*OVERSAMPLENR, 310     },
	{27*OVERSAMPLENR, 305     },
	{29*OVERSAMPLENR, 300     },
	{31*OVERSAMPLENR, 295     },
	{33*OVERSAMPLENR, 290     },
	{35*OVERSAMPLENR, 285     },
	{37*OVERSAMPLENR, 280     },
	{40*OVERSAMPLENR, 275     },
	{43*OVERSAMPLENR, 270     },
	{46*OVERSAMPLENR, 265     },
	{49*OVERSAMPLENR, 260     },
	{52*OVERSAMPLENR, 255     },
	{56*OVERSAMPLENR, 250     },
	{60*OVERSAMPLENR, 245     },
	{65*OVERSAMPLENR, 240     },
	{70*OVERSAMPLENR, 235     },
	{75*OVERSAMPLENR, 230     },
	{81*OVERSAMPLENR, 225     },
	{88*OVERSAMPLENR, 220     },
	{95*OVERSAMPLENR, 215     },
	{102*OVERSAMPLENR, 210     },
	{111*OVERSAMPLENR, 205     },
	{120*OVERSAMPLENR, 200     },
	{130*OVERSAMPLENR, 195     },
	{141*OVERSAMPLENR, 190     },
	{153*OVERSAMPLENR, 185     },
	{167*OVERSAMPLENR, 180     },
	{181*OVERSAMPLENR, 175     },
	{197*OVERSAMPLENR, 170     },
	{214*OVERSAMPLENR, 165     },
	{233*OVERSAMPLENR, 160     },
	{254*OVERSAMPLENR, 155     },
	{276*OVERSAMPLENR, 150     },
	{300*OVERSAMPLENR, 145     },
	{325*OVERSAMPLENR, 140     },
	{353*OVERSAMPLENR, 135     },
	{382*OVERSAMPLENR, 130     },
	{413*OVERSAMPLENR, 125     },
	{446*OVERSAMPLENR, 120     },
	{480*OVERSAMPLENR, 115     },
	{515*OVERSAMPLENR, 110     },
	{551*OVERSAMPLENR, 105     },
	{587*OVERSAMPLENR, 100     },
	{624*OVERSAMPLENR, 95     },
	{661*OVERSAMPLENR, 90     },
	{697*OVERSAMPLENR, 85     },
	{732*OVERSAMPLENR, 80     },
	{765*OVERSAMPLENR, 75     },
	{797*OVERSAMPLENR, 70     },
	{826*OVERSAMPLENR, 65     },
	{853*OVERSAMPLENR, 60     },
	{878*OVERSAMPLENR, 55     },
	{901*OVERSAMPLENR, 50     },
	{920*OVERSAMPLENR, 45     },
	{938*OVERSAMPLENR, 40     },
	{953*OVERSAMPLENR, 35     },
	{966*OVERSAMPLENR, 30     },
	{977*OVERSAMPLENR, 25     },
	{987*OVERSAMPLENR, 20     },
	{994*OVERSAMPLENR, 15     },
	{1001*OVERSAMPLENR, 10     },
	{1006*OVERSAMPLENR, 5     },
	{1010*OVERSAMPLENR, 0     },
	{1013*OVERSAMPLENR, -5     },
	{1016*OVERSAMPLENR, -10     },
	{1018*OVERSAMPLENR, -15     },
	{1019*OVERSAMPLENR, -20     },
	{1020*OVERSAMPLENR, -25     },
	{1021*OVERSAMPLENR, -30     },
	{1022*OVERSAMPLENR, -35     },
	{1022*OVERSAMPLENR, -40     },

	};
#endif

#if ( THERMISTORHEATER_0 == 100474275 ) || ( THERMISTORHEATER_1 == 100474275) || ( THERMISTORHEATER_2 == 100474275 ) || ( THERMISTORBED == 100474275 ) 
// 100k with 4.7k pullup, Beta of 4275.
// Corresponds to MuRata NXFT15WF104FA2B025
// Calculated using 4.7kohm pullup, voltage divider math, and manufacturer provided beta value
const short temptable_100474275[][2] PROGMEM = {
	{12*OVERSAMPLENR, 350     },
	{13*OVERSAMPLENR, 345     },
	{14*OVERSAMPLENR, 340     },
	{14*OVERSAMPLENR, 335     },
	{15*OVERSAMPLENR, 330     },
	{16*OVERSAMPLENR, 325     },
	{17*OVERSAMPLENR, 320     },
	{18*OVERSAMPLENR, 315     },
	{19*OVERSAMPLENR, 310     },
	{21*OVERSAMPLENR, 305     },
	{22*OVERSAMPLENR, 300     },
	{23*OVERSAMPLENR, 295     },
	{25*OVERSAMPLENR, 290     },
	{27*OVERSAMPLENR, 285     },
	{28*OVERSAMPLENR, 280     },
	{31*OVERSAMPLENR, 275     },
	{33*OVERSAMPLENR, 270     },
	{35*OVERSAMPLENR, 265     },
	{38*OVERSAMPLENR, 260     },
	{41*OVERSAMPLENR, 255     },
	{44*OVERSAMPLENR, 250     },
	{47*OVERSAMPLENR, 245     },
	{51*OVERSAMPLENR, 240     },
	{55*OVERSAMPLENR, 235     },
	{60*OVERSAMPLENR, 230     },
	{64*OVERSAMPLENR, 225     },
	{70*OVERSAMPLENR, 220     },
	{76*OVERSAMPLENR, 215     },
	{83*OVERSAMPLENR, 210     },
	{90*OVERSAMPLENR, 205     },
	{98*OVERSAMPLENR, 200     },
	{107*OVERSAMPLENR, 195     },
	{117*OVERSAMPLENR, 190     },
	{127*OVERSAMPLENR, 185     },
	{139*OVERSAMPLENR, 180     },
	{153*OVERSAMPLENR, 175     },
	{167*OVERSAMPLENR, 170     },
	{183*OVERSAMPLENR, 165     },
	{201*OVERSAMPLENR, 160     },
	{220*OVERSAMPLENR, 155     },
	{241*OVERSAMPLENR, 150     },
	{264*OVERSAMPLENR, 145     },
	{289*OVERSAMPLENR, 140     },
	{316*OVERSAMPLENR, 135     },
	{345*OVERSAMPLENR, 130     },
	{376*OVERSAMPLENR, 125     },
	{409*OVERSAMPLENR, 120     },
	{444*OVERSAMPLENR, 115     },
	{480*OVERSAMPLENR, 110     },
	{518*OVERSAMPLENR, 105     },
	{556*OVERSAMPLENR, 100     },
	{595*OVERSAMPLENR, 95     },
	{635*OVERSAMPLENR, 90     },
	{673*OVERSAMPLENR, 85     },
	{711*OVERSAMPLENR, 80     },
	{748*OVERSAMPLENR, 75     },
	{782*OVERSAMPLENR, 70     },
	{814*OVERSAMPLENR, 65     },
	{844*OVERSAMPLENR, 60     },
	{871*OVERSAMPLENR, 55     },
	{895*OVERSAMPLENR, 50     },
	{917*OVERSAMPLENR, 45     },
	{936*OVERSAMPLENR, 40     },
	{952*OVERSAMPLENR, 35     },
	{966*OVERSAMPLENR, 30     },
	{977*OVERSAMPLENR, 25     },
	{987*OVERSAMPLENR, 20     },
	{994*OVERSAMPLENR, 15     },
	{1001*OVERSAMPLENR, 10     },
	{1006*OVERSAMPLENR, 5     },
	{1010*OVERSAMPLENR, 0     },
	{1013*OVERSAMPLENR, -5     },
	{1016*OVERSAMPLENR, -10     },
	{1018*OVERSAMPLENR, -15     },
	{1019*OVERSAMPLENR, -20     },
	{1020*OVERSAMPLENR, -25     },
	{1021*OVERSAMPLENR, -30     },
	{1022*OVERSAMPLENR, -35     },
	{1022*OVERSAMPLENR, -40     },
	};
#endif

#if ( THERMISTORHEATER_0 == 100474240 ) || ( THERMISTORHEATER_1 == 100474240) || ( THERMISTORHEATER_2 == 100474240 ) || ( THERMISTORBED == 100474240 ) 
// 100k with 4.7k pullup, Beta of 4240, which is a fudged value based on measuring the head temperature.
// Corresponds to EPCOS B57540G104F
// Calculated using 4.7kohm pullup, voltage divider math, and fudged beta value
const short temptable_100474240[][2] PROGMEM = {
	{13*OVERSAMPLENR, 350     },
	{14*OVERSAMPLENR, 345     },
	{14*OVERSAMPLENR, 340     },
	{15*OVERSAMPLENR, 335     },
	{16*OVERSAMPLENR, 330     },
	{17*OVERSAMPLENR, 325     },
	{18*OVERSAMPLENR, 320     },
	{19*OVERSAMPLENR, 315     },
	{20*OVERSAMPLENR, 310     },
	{22*OVERSAMPLENR, 305     },
	{23*OVERSAMPLENR, 300     },
	{25*OVERSAMPLENR, 295     },
	{26*OVERSAMPLENR, 290     },
	{28*OVERSAMPLENR, 285     },
	{30*OVERSAMPLENR, 280     },
	{32*OVERSAMPLENR, 275     },
	{34*OVERSAMPLENR, 270     },
	{37*OVERSAMPLENR, 265     },
	{40*OVERSAMPLENR, 260     },
	{43*OVERSAMPLENR, 255     },
	{46*OVERSAMPLENR, 250     },
	{49*OVERSAMPLENR, 245     },
	{53*OVERSAMPLENR, 240     },
	{58*OVERSAMPLENR, 235     },
	{62*OVERSAMPLENR, 230     },
	{67*OVERSAMPLENR, 225     },
	{73*OVERSAMPLENR, 220     },
	{79*OVERSAMPLENR, 215     },
	{86*OVERSAMPLENR, 210     },
	{94*OVERSAMPLENR, 205     },
	{102*OVERSAMPLENR, 200     },
	{111*OVERSAMPLENR, 195     },
	{121*OVERSAMPLENR, 190     },
	{132*OVERSAMPLENR, 185     },
	{144*OVERSAMPLENR, 180     },
	{158*OVERSAMPLENR, 175     },
	{172*OVERSAMPLENR, 170     },
	{189*OVERSAMPLENR, 165     },
	{207*OVERSAMPLENR, 160     },
	{226*OVERSAMPLENR, 155     },
	{247*OVERSAMPLENR, 150     },
	{270*OVERSAMPLENR, 145     },
	{296*OVERSAMPLENR, 140     },
	{323*OVERSAMPLENR, 135     },
	{352*OVERSAMPLENR, 130     },
	{383*OVERSAMPLENR, 125     },
	{416*OVERSAMPLENR, 120     },
	{450*OVERSAMPLENR, 115     },
	{487*OVERSAMPLENR, 110     },
	{524*OVERSAMPLENR, 105     },
	{562*OVERSAMPLENR, 100     },
	{601*OVERSAMPLENR, 95     },
	{640*OVERSAMPLENR, 90     },
	{678*OVERSAMPLENR, 85     },
	{715*OVERSAMPLENR, 80     },
	{751*OVERSAMPLENR, 75     },
	{785*OVERSAMPLENR, 70     },
	{817*OVERSAMPLENR, 65     },
	{846*OVERSAMPLENR, 60     },
	{873*OVERSAMPLENR, 55     },
	{896*OVERSAMPLENR, 50     },
	{918*OVERSAMPLENR, 45     },
	{936*OVERSAMPLENR, 40     },
	{952*OVERSAMPLENR, 35     },
	{966*OVERSAMPLENR, 30     },
	{977*OVERSAMPLENR, 25     },
	{987*OVERSAMPLENR, 20     },
	{994*OVERSAMPLENR, 15     },
	{1001*OVERSAMPLENR, 10     },
	{1006*OVERSAMPLENR, 5     },
	{1010*OVERSAMPLENR, 0     },
	{1013*OVERSAMPLENR, -5     },
	{1016*OVERSAMPLENR, -10     },
	{1018*OVERSAMPLENR, -15     },
	{1019*OVERSAMPLENR, -20     },
	{1020*OVERSAMPLENR, -25     },
	{1021*OVERSAMPLENR, -30     },
	{1022*OVERSAMPLENR, -35     },
	{1022*OVERSAMPLENR, -40     },

	};
#endif

#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)

#ifdef THERMISTORHEATER_0
  #define heater_0_temptable TT_NAME(THERMISTORHEATER_0)
  #define heater_0_temptable_len (sizeof(heater_0_temptable)/sizeof(*heater_0_temptable))
#else
#ifdef HEATER_0_USES_THERMISTOR
  #error No heater 0 thermistor table specified
#else  // HEATER_0_USES_THERMISTOR
  #define heater_0_temptable 0
  #define heater_0_temptable_len 0
#endif // HEATER_0_USES_THERMISTOR
#endif

#ifdef THERMISTORHEATER_1
  #define heater_1_temptable TT_NAME(THERMISTORHEATER_1)
  #define heater_1_temptable_len (sizeof(heater_1_temptable)/sizeof(*heater_1_temptable))
#else
#ifdef HEATER_1_USES_THERMISTOR
  #error No heater 1 thermistor table specified
#else  // HEATER_1_USES_THERMISTOR
  #define heater_1_temptable 0
  #define heater_1_temptable_len 0
#endif // HEATER_1_USES_THERMISTOR
#endif

#ifdef THERMISTORHEATER_2
  #define heater_2_temptable TT_NAME(THERMISTORHEATER_2)
  #define heater_2_temptable_len (sizeof(heater_2_temptable)/sizeof(*heater_2_temptable))
#else
#ifdef HEATER_2_USES_THERMISTOR
  #error No heater 2 thermistor table specified
#else  // HEATER_2_USES_THERMISTOR
  #define heater_2_temptable 0
  #define heater_2_temptable_len 0
#endif // HEATER_2_USES_THERMISTOR
#endif

#ifdef THERMISTORBED
  #define bedtemptable TT_NAME(THERMISTORBED)
  #define bedtemptable_len (sizeof(bedtemptable)/sizeof(*bedtemptable))
#else
#ifdef BED_USES_THERMISTOR
  #error No bed thermistor table specified
#endif // BED_USES_THERMISTOR
#endif

#endif //THERMISTORTABLES_H_

