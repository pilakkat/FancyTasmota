/*
  xsns_120_ssofterctl.ino - Customized softner and garden control based on Tasmota

  Copyright (C) 2024  Deepak Pilakkattu

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_SOFTNERCTL
#define XSNS_120                   120
#define PULSERATE_TABLE_SIZE       6
#define FLOWRATE_TABLE_SIZE        6
#define TANKVOLUME                 1000
#define MAXROUTINES                4
#define FLOWSTATEDEB               5
#define MAXFILLTRY                 5
#define REGENDEFLIM                2500
#define WAITAFTERREGEN             900 //resume operations 15 minutes after regen
#define SALTPERFLUSH               4.5 // salt usage: kg/regen 
#define MINFLOWRATE                5
#define SALTCAPACITY               50  // brine tank capacity: kg
#define REGENLOCKSTART             1   //Start of REGEN LOCK - 1AM
#define REGENLOCKEND               6   //REGEN LOCK Release - 6AM
#define TOPUPHOUR                  22  //Hour window for tank fill up 

/* DEB16 Masks */
#define DEB16MSK_DEBVALUE   0x8000
#define DEB16MSK_DEBONCE    0x4000
#define DEB16MSK_NEWEVENT   0x2000
#define DEB16MSK_PREVINP    0x1000
#define DEB16MSK_TIMER      0x0FFF

const char HTTP_SOFTNERCTL_SNSR_INFO[]           PROGMEM = "{s}%s{m}%d %d %d %d{e}";
const char HTTP_SOFTNERCTL_INFO[]           PROGMEM = "{s}%s{m} %d%s{e}";

const char kSoftnerCommands[] PROGMEM = "|"  // Prefix
  "SoftnerCommands|SensorDebounce|FlowFactor|"
  "TankStats|TankLimit|ValveLimit|SoftnerInit|"
  "TimerEvent|Routine|ReadRoutine|SoftnerConfig|TestMode" ;

void (* const SoftnerCommand[])(void) PROGMEM = {
  &CmndSoftnerCommands,&CmndSensorDeb,&CmndFlowFactor,
  &CmndTankStats,&CmndTankLimit,&CmndValveLimit,&CmndSoftnerInit,
  &CmndTimerEvent,&CmndRoutine,&CmndReadRoutine,&CmndSoftnerConfig,&CmndTestMode};

enum SoftnerStates {
    ERROR_DRYRUN=0,
    ERROR_DRYRUNPERM,
    ERROR_ADCSRCMIN,
    ERROR_ADCSRCMAX,
    WARN_TIMEWINLOCK,
    WARN_MINLOCK,
    WARN_REGENLOCK
};

enum SoftnerCtlSensors {
    TEMPTY = 0,
    TMED,   
    TFULL,
    SOFTNER
};

enum ValveRelays {
    RELAY_SOFTNER,
    RELAY_UTILITY,
    RELAY_FRONT,
    RELAY_BACK
};

enum SoftnerCtlParamFlags { //32bits
    DIGSNS1DEB=0, //Debounced at least once
    DIGSNS2DEB,
    DIGSNS3DEB,
    DIGSNS4DEB,
    OFFLIM=4,    // 4-5: Validity
    ONLIM,
    AUTOMODE=6, //automode state
    MAXON1=7,  //7-19: validity
    MAXON2,
    MAXON3,
    MAXON4,
    MINOFF1=11, 
    MINOFF2,  
    MINOFF3,  
    MINOFF4,
    REGENLIM=15,
    TANKVOL,
    LOWLVL=17,
    MEDLVL,
    HIGHLVL,
    TOPUPMODE=20, //topup feature enable state
    SALTLVL,    //validity
    SALTCAPAVALID,
    SALTUSAGE
};

struct SensorLocals {
    float total_volume=0; 
    float mass_flow_rate=0;
    float total_volume_old=0; 
    float mass_flow_rate_old=0;
    float watervolume; //modeled volume
    float pulse[2]; //[pulse/s, pulse/l]
    float voldelta;
    float salt_volume;
    float salt_volume_old;
    uint32_t counter_old=0;
    uint16_t relay_countdown[4];
    uint16_t adc_value=0;
    uint16_t wait[3];
    uint16_t sensordebstates[4]; 
    uint16_t dryrundebstate=0;
    uint8_t raw_state;//bit coded 0:L 1:M 2:H 3:REG 4..7: old value of 0...3
    uint8_t deb_state; //bit coded 0:L 1:M 2:H 3:REG 4..7: old value of 0...3
    uint8_t waterlevel=0;
    uint8_t waterlevel_old=0;
    uint8_t relaystate_old;
    uint8_t relaylock;
    uint8_t dryruntimer;
    uint8_t errorstates_old=0;
    uint8_t errorstates=0;
    uint8_t routine[3];
    uint8_t timeout[3];
    uint8_t routineactive=0;
    uint8_t routinestate=0; /* bit 0..2: activated via assigned routine, 
                            bit3..5: timeout active due to manual operation */
    uint8_t flowstate=(FLOWSTATEDEB<<2); /* bit 0: charge:1/discharge:0 (raw), bit 1: bit0 debouced, 2-7: timer (5sec debounce)*/
    uint8_t valvestate = 0; /* bit0: last state, bit1: command accepted on last call, bit2: locked ON on last call, bit3: auto fill desired  */
    uint8_t filltries = 0;
    uint8_t specialtimes = 0; /* bit0: regen inhibit, bit1: topup, bit4=bit0 old, bit5=bit1 processed */
    bool flushstate_old;
    bool valid=false;
    bool tankfilling=false;
    bool tanklevelvalid=false;
    bool ext_override_active = false;
    bool abortfill=false;
    bool firstinit=true;
    bool nofillwindow=false;
} softnersensors;

struct SensorParams {
    float disch_rate_T[(FLOWRATE_TABLE_SIZE*2+1)];
    float ctPulsePerLNorm_T[(PULSERATE_TABLE_SIZE*2+1)];
    float charge_rate_factor; //for test purpose
    float discharge_rate_factor; //for test purpose
    float offlimit;
    float onlimit;
    float regenlimit;
    float tank_volume;
    float saltusage;
    float saltcapacity;
    uint16_t deb_limit[4];      //multiples of 50ms
    uint16_t relay_limit[4];    //max ~18hrs
    uint16_t relay_minlock[4];
    uint16_t sensorlevels[3];
    uint8_t maxfillwait=10;
    uint8_t testctrdelta; //for testing
    bool testmode=false; //for testing
} softnerparams;

#define GET_RAWSTATE(id) (softnersensors.raw_state>>(id) & 1)
#define GET_DEBSTATE(id) ((softnersensors.sensordebstates[id] & DEB16MSK_DEBVALUE)!=0)
#define IS_TESTED(id) ((softnersensors.sensordebstates[id] & DEB16MSK_DEBONCE)!=0)
#define IS_NEWEVENT(id) ((softnersensors.sensordebstates[id] & DEB16MSK_NEWEVENT)!=0)
#define SOFTNER_FLAGS(bit) bitRead(Settings->softner_flags,(bit))

float LinearIpo(float x, float* T) {
    /* simple linear ipo rountine 
        T = [n,x1,x2,....,xn,y1,y2,....,yn]    */
    uint8_t size = (uint8_t)T[0];
    if (x <= T[1]) {
        return T[size+1];
    } else if (x >= T[size]) {
        return T[size*2];
    } else {
        for (uint8_t i=1;i<=size;i++) {
            if (x == T[i]) { //axis point
                return T[size+i];
            } else if (x < T[i]) { //between T[i-1] and T[i] 
                /* Do linear ipo : y = mx+c = (y2-y1)/(x2-x1)*(x-x1)+y1
                    (x1,x2): (T[i-1],T[i])
                    (y1,y2): (T[size+i-1],T[size+i])
                */
                return ((float)(T[size+i]-T[size+i-1])*(x-T[i-1])/(T[i]-T[i-1]))+T[size+i-1];
            }
        }
    }
    return 0; // should not reach here
}

float NoIpo(float x, float* T) {
    /* simple lookup table routine 
        T = [n,x1,x2,....,xn,y1,y2,....,yn]    */
    uint8_t size = (uint8_t)T[0];
    if (x <= T[1]) {
        return T[size+1];
    } else if (x >= T[size]) {
        return T[size*2];
    } else {
        for (uint8_t i=1;i<=size;i++) {
            if (x == T[i]) { //axis point
                return T[size+i];
            } else if (x < T[i]) { //between T[i-1] and T[i] 
                return T[size+i-1];
            }
        }
    }
    return 0; // should not reach here
}

float GetDischargeRate()
{
    if (RtcTime.valid) {
        return NoIpo(RtcTime.hour, softnerparams.disch_rate_T)*softnerparams.discharge_rate_factor;
    } else {
        return softnerparams.disch_rate_T[(uint8_t)softnerparams.disch_rate_T[FLOWRATE_TABLE_SIZE+1]]*softnerparams.discharge_rate_factor; //default value
    }
}
bool GetRoutine(uint8_t id) {
    /* id:
         0 = cancel (fill 0)
         1..4 = routine X (fill settings)
         255 = manual trigger (no fill)
    */
    uint16_t waittime=0; //calculated wait time for activation
    sint8_t overlap=2; //valve operation overlaps. use negative values for delay between valves
    bool active=false;
    if(id==0) { //stop routine
        bool clearrest=false;
        for (uint8_t i=0;i<3;i++) {
            softnersensors.routine[i]=0;
            softnersensors.wait[i]=0;
            softnersensors.timeout[i]=0;
        }
        Response_P(PSTR("{\"id\":%d}"), softnersensors.routineactive);
        MqttPublishPrefixTopic_P(STAT, PSTR("STOPROUTINE"), Settings->flag5.mqtt_state_retain);
        active=true;
    } else if (id==255) { //manual routine. data is already filled from command
        for(uint8_t i=0;i<3;i++) {
            if(softnersensors.routine[i]) {
                InitTimeouts(i);
                softnersensors.wait[i]=waittime;
                if (softnersensors.routine[i]>softnersensors.timeout[i]) {
                    waittime = waittime+softnersensors.timeout[i]-overlap;
                } else {
                    waittime = waittime+softnersensors.routine[i]-overlap;
                }
                softnersensors.routine[i]++; //to compensate last raster
                active=true;
                softnersensors.routineactive=id;
            }
        }
        if(active) {
            Response_P(PSTR("{\"id\":%d,\"upv\":%d,\"fgv\":%d,\"kgv\":%d}"), softnersensors.routineactive,
                    softnersensors.routine[0],softnersensors.routine[1],softnersensors.routine[2]);
            MqttPublishPrefixTopic_P(STAT, PSTR("STARTROUTINE"), Settings->flag5.mqtt_state_retain);
        }
    } else {
        for (uint8_t i=0;i<3;i++) {
            softnersensors.routine[i]=0;
            softnersensors.wait[i]=0;
        }
        if (id <= MAXROUTINES) {
            if (bitRead(Settings->garden_routines[0],id-1)) {
                for(uint8_t i=0;i<3;i++) {
                    softnersensors.routine[i]=Settings->garden_routines[(id-1)*3+i+1];
                    if(softnersensors.routine[i]) {
                        InitTimeouts(i);
                        softnersensors.wait[i]=waittime;
                        if (softnersensors.routine[i]>softnersensors.timeout[i]) {
                            waittime = waittime+softnersensors.timeout[i]-overlap;
                        } else {
                            waittime = waittime+softnersensors.routine[i]-overlap;
                        }
                        softnersensors.routine[i]++; //to compensate last raster
                        active=true;
                        softnersensors.routineactive=id;
                    }
                }
                if(active) {
                    Response_P(PSTR("{\"id\":%d,\"upv\":%d,\"fgv\":%d,\"kgv\":%d}"), softnersensors.routineactive,
                            softnersensors.routine[0],softnersensors.routine[1],softnersensors.routine[2]);
                    MqttPublishPrefixTopic_P(STAT, PSTR("STARTROUTINE"), Settings->flag5.mqtt_state_retain);
                }
            }
        }
    }
    return active;
}

void SetValveState(uint8_t id, bool val) {
    if ((val == GetValveState(id)) || // state change not needed
        (id > RELAY_BACK) || (TasmotaGlobal.uptime < 4) || //valid index and initial timout
        (bitRead(softnersensors.relaylock, id) && val)) { return; } //block turn on due to lock
    ExecuteCommandPower(id+1, (val?POWER_ON:POWER_OFF), SRC_IGNORE); 
}

bool GetValveState(uint8_t id) {
    if (id <= RELAY_BACK) {
        return (bool)((TasmotaGlobal.power>>(id)) & 1);
    } else {
        return false;
    }
}

bool GetValveOldState(uint8_t id) {
    if (id <= RELAY_BACK) {
        return bitRead(softnersensors.relaystate_old, id);
    } else {
        return false;
    }
}

bool SoftDebounce16(bool x, uint16_t tion, uint16_t tioff, uint16_t* state) {
    /* state.15 --> last deb state 
       state.14 --> debounced once 
       state.13 --> new event
       state.12 --> target state being debounced to
       state.11 .. 0 --> counter = limit: 0 .. 4098 => ~ >200s@50ms, ~ >1hr@1s */
    bool prev = (*state & DEB16MSK_DEBVALUE) != 0;
    bool debonce = (*state & DEB16MSK_DEBONCE) != 0;
    bool newevent = false;
    bool xold = (*state & DEB16MSK_PREVINP) != 0;
    uint16_t tmr = *state & DEB16MSK_TIMER;

    if (x!=xold) {tmr=0;}
    if(x == prev){        
        if(!debonce) { //not yet debounced
            if (++tmr>=(x?tion:tioff)) {
                debonce = true;
                newevent = true;
            }
        }      
    } else {
        if (++tmr>=(x?tion:tioff)) {  
            prev = x;
            newevent = true;
            debonce = true;
        }
    }
    *state =  (prev?DEB16MSK_DEBVALUE:0)
            | (debonce?DEB16MSK_DEBONCE:0) 
            | (newevent?DEB16MSK_NEWEVENT:0) 
            | (x?DEB16MSK_PREVINP:0) 
            | (tmr & DEB16MSK_TIMER);
    return prev;
}




/*********************************************************************************************/
void SoftnerCtlInit(void) {
    uint8_t scount = 0;
    if (PinUsed(GPIO_TEMPTY)) {pinMode(Pin(GPIO_TEMPTY), INPUT_PULLUP);scount++;}
    if (PinUsed(GPIO_TMED)) {pinMode(Pin(GPIO_TMED), INPUT_PULLUP);scount++;}
    if (PinUsed(GPIO_TFULL)) {pinMode(Pin(GPIO_TFULL), INPUT_PULLUP);scount++;}
   /* if (PinUsed(GPIO_SOFTNER)) {AdcAttach(Pin(GPIO_SOFTNER), ADC_INPUT);scount++;}
    for (uint8_t i=0;i<=3;i++) {
        if (PinUsed(GPIO_LVL, i)) {pinMode(Pin(GPIO_LVL), INPUT_PULLUP);scount++;}
    }*/
    softnersensors.valid = (bool)(scount>0);
    softnersensors.flushstate_old=false;
    /* Default debounce values */
    softnerparams.deb_limit[TEMPTY] = (5<<8) | 5;     //2s both directions
    softnerparams.deb_limit[TMED] = (5<<8) | 5; 
    softnerparams.deb_limit[TFULL] = (5<<8) | 5; 
    softnerparams.deb_limit[SOFTNER] = (30<<8) | 10;    //30s Turnon, 10s turoff 
    softnerparams.relay_limit[RELAY_SOFTNER]=600;     //10 min max 
    softnerparams.relay_limit[RELAY_UTILITY]=60;     
    softnerparams.relay_limit[RELAY_FRONT]=60;     
    softnerparams.relay_limit[RELAY_BACK]=90;
    softnerparams.relay_minlock[RELAY_SOFTNER]=120;  //2 min off minimum  
    softnerparams.relay_minlock[RELAY_UTILITY]=0;     
    softnerparams.relay_minlock[RELAY_FRONT]=0;     
    softnerparams.relay_minlock[RELAY_BACK]=0;
    for(uint8_t i=0;i<=DIGSNS4DEB;i++) {
        if (SOFTNER_FLAGS(i)) {
            uint8_t ton = ((Settings->softner_deblim[i] & 0xF0)>>4) * 5; // MSB: 0 - F, res. 5 => 0- 75s
            uint8_t toff = (Settings->softner_deblim[i] & 0xF) * 5; // LSB: 0 - 75s 
            softnerparams.deb_limit[i] = (ton << 8 | toff);
        }  
        softnersensors.sensordebstates[i]=0;
    }
    for(uint8_t i=MAXON1;i<=MAXON4;i++) {
        if (SOFTNER_FLAGS(i)) {
            softnerparams.relay_limit[i-MAXON1] = Settings->softner_maxon[i-MAXON1];
        }  
    }
    for(uint8_t i=MINOFF1;i<=MINOFF4;i++) {
        if (SOFTNER_FLAGS(i)) {
            softnerparams.relay_minlock[i-MINOFF1] = Settings->softner_minoff[i-MINOFF1];
        }  
    }

    softnersensors.total_volume = (float)Settings->softner_totalvolume*0.1;
    softnersensors.watervolume = (float)Settings->softner_watervolume*0.1;
    
    softnersensors.total_volume_old = softnersensors.total_volume;
    softnersensors.counter_old = CounterPinRead(1);
    /* Interpolation setting for flow sensor : pulse/sec --> pulse/liter */
    softnerparams.ctPulsePerLNorm_T[0] = PULSERATE_TABLE_SIZE;
    softnerparams.ctPulsePerLNorm_T[1] = 10;  softnerparams.ctPulsePerLNorm_T[PULSERATE_TABLE_SIZE+1] = 230;
    softnerparams.ctPulsePerLNorm_T[2] = 60;  softnerparams.ctPulsePerLNorm_T[PULSERATE_TABLE_SIZE+2] = 250;
    softnerparams.ctPulsePerLNorm_T[3] = 110; softnerparams.ctPulsePerLNorm_T[PULSERATE_TABLE_SIZE+3] = 300;
    softnerparams.ctPulsePerLNorm_T[4] = 160; softnerparams.ctPulsePerLNorm_T[PULSERATE_TABLE_SIZE+4] = 380;
    softnerparams.ctPulsePerLNorm_T[5] = 210; softnerparams.ctPulsePerLNorm_T[PULSERATE_TABLE_SIZE+5] = 430;
    softnerparams.ctPulsePerLNorm_T[6] = 260; softnerparams.ctPulsePerLNorm_T[PULSERATE_TABLE_SIZE+6] = 455;
    /* Average discharge rate for tank */
    softnerparams.disch_rate_T[0]=FLOWRATE_TABLE_SIZE;
    softnerparams.disch_rate_T[1]=5; softnerparams.disch_rate_T[FLOWRATE_TABLE_SIZE+1]=0.001;     //12AM to 5:59AM
    softnerparams.disch_rate_T[2]=6; softnerparams.disch_rate_T[FLOWRATE_TABLE_SIZE+2]=0.02;      //6 to 9:59AM
    softnerparams.disch_rate_T[3]=10;softnerparams.disch_rate_T[FLOWRATE_TABLE_SIZE+3]=0.004;     //10AM to 2:59PM
    softnerparams.disch_rate_T[4]=15;softnerparams.disch_rate_T[FLOWRATE_TABLE_SIZE+4]=0.015;    //3PM to 5:59PM
    softnerparams.disch_rate_T[5]=18;softnerparams.disch_rate_T[FLOWRATE_TABLE_SIZE+5]=0.01;     //6PM to 10:59PM
    softnerparams.disch_rate_T[6]=23;softnerparams.disch_rate_T[FLOWRATE_TABLE_SIZE+6]=0.001;    //11PM to 11:59PM

    softnersensors.pulse[0]=0;
    softnersensors.pulse[1]=0;
    softnerparams.charge_rate_factor=1;
    softnerparams.discharge_rate_factor=1;
    
    if (SOFTNER_FLAGS(TANKVOL)) {
        softnerparams.tank_volume = (float)(Settings->softner_tankvolume);
        if (softnerparams.tank_volume < 500) {softnerparams.tank_volume = 500;} //minimum volume : 500L
    } else {
        softnerparams.tank_volume = (float)TANKVOLUME;
    }
    if (SOFTNER_FLAGS(OFFLIM)) {
        softnerparams.offlimit = (float)(Settings->softner_offlimit);
    } else {
        softnerparams.offlimit = softnerparams.tank_volume;
    }
    if (SOFTNER_FLAGS(ONLIM)) {
        softnerparams.onlimit = (float)(Settings->softner_onlimit);
    } else {
        softnerparams.onlimit = 0.6*softnerparams.tank_volume;
    }
    if (SOFTNER_FLAGS(REGENLIM)) {
        softnerparams.regenlimit = (float)(Settings->softner_regenlimit);
    } else {
        softnerparams.regenlimit = REGENDEFLIM;
    }
    for (uint8_t i=0;i<3;i++) {
        softnersensors.routine[i]=0;
        softnersensors.wait[i]=0;
        InitTimeouts(i);
    }
    softnersensors.routineactive=0;

    softnerparams.sensorlevels[0]=(uint16_t)(0.2*softnerparams.tank_volume);
    softnerparams.sensorlevels[1]=(uint16_t)(0.6*softnerparams.tank_volume);
    softnerparams.sensorlevels[2]=(uint16_t)(softnerparams.tank_volume);
    for(uint8_t i=LOWLVL;i<=HIGHLVL;i++) {
        if (SOFTNER_FLAGS(i)) {
            softnerparams.sensorlevels[i-LOWLVL] = Settings->softner_sensorlevels[i-LOWLVL];
        }
    }
    if (SOFTNER_FLAGS(SALTLVL)) {
        softnersensors.salt_volume = (float)(Settings->softner_saltlevel)/100.0; //to Kg, resolution 0.01
    } else {
        softnersensors.salt_volume = 0;
    }
    if (SOFTNER_FLAGS(SALTUSAGE)) {
        softnerparams.saltusage = (float)(Settings->softner_saltusage)/10.0; //in Kg, resolution 0.1
    } else {
        softnerparams.saltusage = SALTPERFLUSH;
    }
    if (SOFTNER_FLAGS(SALTCAPAVALID) && (Settings->softner_saltcapacity > 0)) {
        softnerparams.saltcapacity = (float)(Settings->softner_saltcapacity); //in Kg
    } else {
        softnerparams.saltcapacity = SALTCAPACITY;
    }

    Response_P(PSTR("{\"level\":%d,\"total\":%d,\"counter\":%d}"),
         (uint16_t)softnersensors.watervolume,(uint16_t)softnersensors.total_volume,
         softnersensors.counter_old);
    MqttPublishPrefixTopic_P(STAT, PSTR("INITCMPL"), Settings->flag5.mqtt_state_retain);
            
}

void InitTimeouts(uint8_t idx) {
    softnersensors.timeout[idx]=180;
}

void MonitorCountDown(void) {
    for (uint8_t idx=0;idx<RELAY_BACK;idx++) {
        if(softnersensors.relay_countdown[idx]) { //timer active
            if(--softnersensors.relay_countdown[idx]==0) { //finished
                if (bitRead(softnersensors.relaylock, idx)) { //lock timer
                    bitClear(softnersensors.relaylock, idx); //release lock
                } else {  // off timer
                    SetValveState(idx,false);
                }
            }
        }

        bool state=GetValveState(idx);
        if (state) {bitClear(softnersensors.relaylock, idx);} // No need anymore
        if(GetValveOldState(idx) != state) { //changed internally or externally
            bitWrite(softnersensors.relaystate_old, idx, state);
            if (state) { //turning on
                softnersensors.relay_countdown[idx]=softnerparams.relay_limit[idx];
            } else {     //turning off
                if(softnerparams.relay_minlock[idx]) { //min off time configured?
                    softnersensors.relay_countdown[idx]=softnerparams.relay_minlock[idx];
                    bitSet(softnersensors.relaylock, idx);
                }
            }
        }
    }
}

void TankSensorProbe50ms(void) {
    uint8_t idx;
    if (!softnerparams.testmode) {
        if (PinUsed(GPIO_TEMPTY)) {bitWrite(softnersensors.raw_state,TEMPTY,!digitalRead(Pin(GPIO_TEMPTY)));} 
        if (PinUsed(GPIO_TMED)) {bitWrite(softnersensors.raw_state,TMED,!digitalRead(Pin(GPIO_TMED)));} 
        if (PinUsed(GPIO_TFULL)) {bitWrite(softnersensors.raw_state,TFULL,!digitalRead(Pin(GPIO_TFULL)));} 

        if (PinUsed(GPIO_ADC_INPUT)) {
            uint16_t newvalue = analogRead(Pin(GPIO_ADC_INPUT));
            softnersensors.adc_value = newvalue;
            /* 2 DIG input reading via ADC and resistor array */
            if (newvalue < 180) {
                /* SCG */           
                bitSet(softnersensors.errorstates,ERROR_ADCSRCMIN);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMAX);
                bitClear(softnersensors.raw_state,TEMPTY);
                bitClear(softnersensors.raw_state,SOFTNER);
            } else if (newvalue < 330) {
                bitSet(softnersensors.raw_state,TEMPTY);
                bitClear(softnersensors.raw_state,SOFTNER);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMIN);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMAX);
            } else if (newvalue < 525) {
                bitClear(softnersensors.raw_state,TEMPTY);
                bitClear(softnersensors.raw_state,SOFTNER);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMIN);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMAX);
            } else if (newvalue < 727) {
                bitSet(softnersensors.raw_state,TEMPTY);
                bitSet(softnersensors.raw_state,SOFTNER);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMIN);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMAX);
            } else if (newvalue < 920) {
                bitClear(softnersensors.raw_state,TEMPTY);
                bitSet(softnersensors.raw_state,SOFTNER);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMIN);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMAX);
            } else {
                /* SCB */
                bitClear(softnersensors.raw_state,TEMPTY);
                bitClear(softnersensors.raw_state,SOFTNER);
                bitClear(softnersensors.errorstates,ERROR_ADCSRCMIN);
                bitSet(softnersensors.errorstates,ERROR_ADCSRCMAX);
            }
        }
    } else {
        bitClear(softnersensors.errorstates,ERROR_ADCSRCMIN);
        bitClear(softnersensors.errorstates,ERROR_ADCSRCMAX);
    }
    for(idx=0;idx<4;idx++) {
        bool raw = GET_RAWSTATE(idx);
        uint16_t tion = ((softnerparams.deb_limit[idx] >> 8) & 0xFF) * 20;
        uint16_t tioff = (softnerparams.deb_limit[idx] & 0xFF) * 20;

        bool deb = SoftDebounce16(raw,tion, tioff,&softnersensors.sensordebstates[idx]);
        bitWrite(softnersensors.deb_state,idx,deb);

        if (idx<SOFTNER && IS_NEWEVENT(idx)) {
            WaterModelInitVolume(idx,deb);
        }
        softnersensors.tanklevelvalid = ((softnersensors.errorstates & (1<<ERROR_ADCSRCMIN | 1<<ERROR_ADCSRCMAX)) == 0) &&
                                            (IS_TESTED(TEMPTY) && IS_TESTED(TMED) && IS_TESTED(TFULL)); //all are confirmed

    }
}

void SenseFlush250ms() {
    if (!GET_DEBSTATE(SOFTNER) && softnersensors.flushstate_old) {//count from edge fall 
        CounterPinSet(1,0,0);
        softnersensors.counter_old = 0;
        softnersensors.total_volume = 0;
        softnersensors.salt_volume -= softnerparams.saltusage;
        if (softnersensors.salt_volume<0) {softnersensors.salt_volume=0;}
    }
    softnersensors.flushstate_old = GET_DEBSTATE(SOFTNER);
}

void WaterModelInitVolume(uint8_t idx, bool deb) {
    /* Just got a new level sensor update. Check all to make sure calculation is done properly */ 
    uint16_t volold = (uint16_t)softnersensors.watervolume;   
    if (IS_TESTED(TEMPTY) && IS_TESTED(TMED) && IS_TESTED(TFULL)) { //all are debounced
        if (softnersensors.firstinit) {
            Response_P(PSTR("{\"id\":%d,\"deb\":%d,\"vol\":%d,\"firstinit\":%d}"), idx,deb,volold,softnersensors.firstinit);
            MqttPublishPrefixTopic_P(STAT, PSTR("SENSEVENT"), Settings->flag5.mqtt_state_retain);
            softnersensors.firstinit = false; //first call, keep saved value
        } else {
            if (softnersensors.tankfilling) {
                /* Increasing water volume. One sensor just activated */
                if (GET_DEBSTATE(TFULL)) {
                    softnersensors.watervolume = (float)softnerparams.sensorlevels[2];
                    softnersensors.filltries = 0;
                    softnersensors.abortfill = false;
                } else if (GET_DEBSTATE(TMED)) {
                    softnersensors.watervolume = (float)softnerparams.sensorlevels[1];
                    softnersensors.filltries = 0;
                    softnersensors.abortfill = false;
                } else if (GET_DEBSTATE(TEMPTY)) {
                    softnersensors.watervolume = (float)softnerparams.sensorlevels[0];
                } else {
                    softnersensors.watervolume = 0; //practically not supposed to reach here
                }
            } else {
                /* Reducing water volume. Just dropped below one of the sensor */
                if (GET_DEBSTATE(TFULL)) {
                    softnersensors.filltries = 0;
                    softnersensors.abortfill = false; // test purpose
                    softnersensors.watervolume = (float)softnerparams.sensorlevels[2]; //practically not supposed to reach here
                } else if (GET_DEBSTATE(TMED)) {
                    softnersensors.watervolume = (float)softnerparams.sensorlevels[2];
                } else if (GET_DEBSTATE(TEMPTY)) {
                    softnersensors.watervolume = (float)softnerparams.sensorlevels[1];
                } else {
                    softnersensors.watervolume = (float)softnerparams.sensorlevels[0];
                }
            }
            
            Response_P(PSTR("{\"id\":%d,\"deb\":%d,\"filling\":%d,\"OldVolume\":%d,\"NewVolume\":%d}"), idx,deb,
                            softnersensors.tankfilling,volold,(uint16_t)softnersensors.watervolume);
            MqttPublishPrefixTopic_P(STAT, PSTR("SENSEVENT"), Settings->flag5.mqtt_state_retain);

            WaterLevelController();
        }
    } else {        
        Response_P(PSTR("{\"id\":%d,\"deb\":%d,\"vol\":%d,\"debst\":\"%d %d %d\"}"), idx,deb,volold,IS_TESTED(TEMPTY),IS_TESTED(TMED),IS_TESTED(TFULL));
        MqttPublishPrefixTopic_P(STAT, PSTR("SENSEVENT"), Settings->flag5.mqtt_state_retain);
    }
}

void WaterFlowSensor(void) {
    uint32_t newcounter=CounterPinRead(1);
    if (softnerparams.testmode) {
        if (GetValveState(RELAY_SOFTNER)) {
            newcounter=softnersensors.counter_old+softnerparams.testctrdelta;
        } else {
            newcounter=softnersensors.counter_old;
        }
    }
    uint16_t ctrdelta = (uint16_t)(newcounter - softnersensors.counter_old);
    float pulse_per_lit = LinearIpo((float)ctrdelta,softnerparams.ctPulsePerLNorm_T);  //pulse per liter
    softnersensors.tankfilling = (bool)ctrdelta>0;
    bitWrite(softnersensors.flowstate,0,ctrdelta>0);
    if(bitRead(softnersensors.flowstate,1)!=(ctrdelta>0)) {
        //debounce required. check timers
        uint8_t tval = softnersensors.flowstate>>2;
        if (tval) {//timer running. this should be always true when reaches here.
            if(!--tval) {//finished
                bitWrite(softnersensors.flowstate,1,ctrdelta>0);
                tval = FLOWSTATEDEB;    //reset timer 
            }
            //write back timer value
            softnersensors.flowstate = (tval<<2 | (softnersensors.flowstate & 0x3));
        }
    } else {
        //reset debounce timer. TIME_IN_ROW method.
        softnersensors.flowstate = (FLOWSTATEDEB<<2 | (softnersensors.flowstate & 0x3));
    }
    softnersensors.pulse[0]=ctrdelta;
    softnersensors.pulse[1]=pulse_per_lit;
    if (ctrdelta>0 && pulse_per_lit>0) {
        float vol_delta = (float)ctrdelta/pulse_per_lit*softnerparams.charge_rate_factor;
        softnersensors.total_volume += vol_delta;
        softnersensors.mass_flow_rate = vol_delta*60; //since this is @1s raster
        softnersensors.watervolume += vol_delta;
        softnersensors.voldelta = vol_delta;

        /* Do not allow model go above non-reporting sensor, provided all are debounced */
        if (IS_TESTED(TEMPTY) && IS_TESTED(TMED) && IS_TESTED(TFULL)) {
            if (!GET_DEBSTATE(0)) {
                if (softnersensors.watervolume >= (float)softnerparams.sensorlevels[0]) {softnersensors.watervolume=(float)softnerparams.sensorlevels[0];}
            } else if (!GET_DEBSTATE(1)) {
                if (softnersensors.watervolume >= (float)softnerparams.sensorlevels[1]) {softnersensors.watervolume=(float)softnerparams.sensorlevels[1];}
            } else if (!GET_DEBSTATE(2)) {
                if (softnersensors.watervolume >= (float)softnerparams.sensorlevels[2]) {softnersensors.watervolume=(float)softnerparams.sensorlevels[2];}
            } else if (softnersensors.watervolume > softnerparams.tank_volume) { 
                softnersensors.watervolume = softnerparams.tank_volume;          
            }
        }
    } else {
        softnersensors.mass_flow_rate = 0;
        softnersensors.voldelta = 0;
    }
    softnersensors.counter_old = newcounter;
}

void ModelWaterDischarge(void) {
    if (!softnersensors.tankfilling) {
        if (IS_TESTED(TEMPTY) && IS_TESTED(TMED) && IS_TESTED(TFULL)) {
            /* Model based on average discharge rate */
            softnersensors.watervolume -= GetDischargeRate();
            /* Do not allow model go below reporting sensor */
            if (GET_DEBSTATE(2)) {
                if (softnersensors.watervolume <= (float)softnerparams.sensorlevels[2]) {softnersensors.watervolume=(float)softnerparams.sensorlevels[2];}
            } else if (GET_DEBSTATE(1)) {
                if (softnersensors.watervolume <= (float)softnerparams.sensorlevels[1]) {softnersensors.watervolume=(float)softnerparams.sensorlevels[1];}
            } else if (GET_DEBSTATE(0)) {
                if (softnersensors.watervolume <= (float)softnerparams.sensorlevels[0]) {softnersensors.watervolume=(float)softnerparams.sensorlevels[0];}
            } else {
                if (softnersensors.watervolume <= 0) {softnersensors.watervolume=0;}
            }
        }
    }   
}

void CheckRegenWindow(void) {
    if (RtcTime.valid) {
        bitWrite(softnersensors.specialtimes,4,bitRead(softnersensors.specialtimes,0)); //bit4 = bit0_old
        
        bitWrite(softnersensors.specialtimes,0,(RtcTime.hour >= REGENLOCKSTART) && (RtcTime.hour <= REGENLOCKEND));                     //bit0 = regen window - no filling (12 - 6AM)
        bitWrite(softnersensors.specialtimes,1,(RtcTime.hour == TOPUPHOUR));                   //bit21 = topup (10:00-10:59pm everyday)
        if (!bitRead(softnersensors.specialtimes,1)) {bitClear(softnersensors.specialtimes,5);} //reset bit5 out of window
    } else {
        softnersensors.specialtimes = 0;
    }
    bitWrite(softnersensors.errorstates,WARN_TIMEWINLOCK,bitRead(softnersensors.specialtimes,0));
}

void WaterLevelController(void) {
    softnersensors.waterlevel = (uint8_t)((float)(softnersensors.watervolume*100/softnerparams.tank_volume)+0.5);

    /* Detect external actions */
    if (GetValveState(RELAY_SOFTNER) != bitRead(softnersensors.valvestate,0)) {
        if (bitRead(softnersensors.valvestate,1)) {// previous run was accepted
            if(softnersensors.relay_countdown[RELAY_SOFTNER]>0 || GetValveState(RELAY_SOFTNER)) {
                /* Changed while countdown is still active (possible only when on --> off externally) ||
                   Turned on while counter is 0 (possible only when off --> on externally) */
                if(GetValveState(RELAY_SOFTNER)) {
                    bitSet(softnersensors.valvestate,0);
                } else {
                    bitClear(softnersensors.valvestate,0);
                    bitClear(softnersensors.valvestate,3); // force stop hysteresis. prevents turning on after cooling period
                }

                softnersensors.dryrundebstate = softnersensors.dryrundebstate ^ DEB16MSK_PREVINP; // force restart debounce timer
                softnersensors.dryrundebstate = softnersensors.dryrundebstate & ~DEB16MSK_DEBONCE; //force dryrun reevaluation via new event on debounce completion
                Response_P(PSTR("{\"StateInfo\":\"%X\",\"StateTmr\":\"%X\"}"),softnersensors.dryrundebstate>>11,softnersensors.dryrundebstate&0xFFF);
                MqttPublishPrefixTopic_P(STAT, PSTR("EXTEVENT"), Settings->flag5.mqtt_state_retain);
            }
        } else if (bitRead(softnersensors.valvestate,2) && GetValveState(RELAY_SOFTNER)) {
            /* it was locked, but now turned on (only possible from external trigger) */
            bitSet(softnersensors.valvestate,0);
            softnersensors.dryrundebstate = softnersensors.dryrundebstate ^ DEB16MSK_PREVINP; // force restart debounce timer
            softnersensors.dryrundebstate = softnersensors.dryrundebstate & ~DEB16MSK_DEBONCE; //force dryrun reevaluation via new event on debounce completion
            Response_P(PSTR("{\"StateInfo\":\"%X\",\"StateTmr\":\"%X\"}"),softnersensors.dryrundebstate>>11,softnersensors.dryrundebstate&0xFFF);
            MqttPublishPrefixTopic_P(STAT, PSTR("EXTLOCKOVRD"), Settings->flag5.mqtt_state_retain);
        }
    }

    bool flgregeninhib = softnersensors.total_volume >= softnerparams.regenlimit;
    bitWrite(softnersensors.errorstates,WARN_REGENLOCK,flgregeninhib);
    bool flgfilldes = bitRead(softnersensors.valvestate,3) || bitRead(softnersensors.valvestate,0);     
    /* Hysteresis for valve action. */ 
    if (!softnersensors.tanklevelvalid || flgregeninhib || (softnersensors.watervolume >= softnerparams.offlimit)) {
        flgfilldes = false;
    } else if (softnersensors.watervolume < softnerparams.onlimit) {
        flgfilldes = true;
    }
    bitWrite(softnersensors.valvestate,3,flgfilldes);

     /*Lock & state checks are handled in calling function */
    if (SOFTNER_FLAGS(AUTOMODE)) {               //automatic control active
        if (flgfilldes &&                       //fill demand is active (even after auto off due to valve timer)
            !softnersensors.abortfill)          //fill not aborted after multiple failed attempts    
        {     
            bitSet(softnersensors.valvestate,0);
        } else if (!flgfilldes ||               //auto off request
                    bitRead(softnersensors.specialtimes,0))  //tank fill is not allowed at this time
        {               
            bitClear(softnersensors.valvestate,0);     
        } 
        
        if (SOFTNER_FLAGS(TOPUPMODE) &&                  //top-up option enabled
            !bitRead(softnersensors.valvestate,0) &&     //not going to turn on
            bitRead(softnersensors.specialtimes,1) &&    //topup window 
            !bitRead(softnersensors.specialtimes,5))     //not handled yet      
        {
            /* Topup is enabled if level is at least 100L less than the auto-off limit */
            if (softnersensors.tanklevelvalid && !softnersensors.abortfill &&  //pre-conditions 
                !GetValveState(RELAY_SOFTNER) &&                //already not filling
                softnersensors.watervolume < softnerparams.offlimit-100 && //limit satisfies
                !bitRead(softnersensors.relaylock, RELAY_SOFTNER)) //relay lock not present
            {
                bitSet(softnersensors.valvestate,0);
                bitSet(softnersensors.specialtimes,5); //top-up only once
            }
        }
    }
    
    bool oldst = GetValveState(RELAY_SOFTNER);
    SetValveState(RELAY_SOFTNER, bitRead(softnersensors.valvestate,0)); 
    bool newst = GetValveState(RELAY_SOFTNER); 
    bitWrite(softnersensors.valvestate,0,newst);
    bitWrite(softnersensors.valvestate,1,(newst == bitRead(softnersensors.valvestate,0))); 
    bitWrite(softnersensors.valvestate,2,bitRead(softnersensors.relaylock, RELAY_SOFTNER));
    if (bitRead(softnersensors.valvestate,1)) { //not processed
        bitWrite(softnersensors.errorstates,WARN_MINLOCK,bitRead(softnersensors.relaylock, RELAY_SOFTNER));
    } else {
        bitClear(softnersensors.errorstates,WARN_MINLOCK);
    }

    if (newst && !oldst) { //just turned on
        softnersensors.dryrundebstate = softnersensors.dryrundebstate ^ DEB16MSK_PREVINP;  
        softnersensors.dryrundebstate = softnersensors.dryrundebstate & ~DEB16MSK_DEBONCE;
        Response_P(PSTR("{\"StateInfo\":\"%X\",\"StateTmr\":\"%X\"}"),softnersensors.dryrundebstate>>11,softnersensors.dryrundebstate&0xFFF);
        MqttPublishPrefixTopic_P(STAT, PSTR("INTEVENT"), Settings->flag5.mqtt_state_retain);
    } //for dry run reevaluation

    if (!GetValveState(RELAY_SOFTNER) && flgregeninhib) {
        //enforce lock for some time before reengaging autofill after a regen
        softnersensors.relay_countdown[RELAY_SOFTNER]=WAITAFTERREGEN;
        bitSet(softnersensors.relaylock, RELAY_SOFTNER);
    }    
}

void DryRunMonitor(void) {
    bool flow = (bool)(softnersensors.mass_flow_rate>MINFLOWRATE);  
    bool flowdeb = SoftDebounce16(flow,softnerparams.maxfillwait, softnerparams.maxfillwait,&softnersensors.dryrundebstate);
     
    if ((softnersensors.dryrundebstate & DEB16MSK_NEWEVENT)!=0) {
        Response_P(PSTR("{\"DryRun\":%d,\"StateInfo\":\"%X\",\"StateTmr\":\"%X\"}"),!flowdeb,softnersensors.dryrundebstate>>11,softnersensors.dryrundebstate&0xFFF);
        MqttPublishPrefixTopic_P(STAT, PSTR("DRYRUNEVENT"), Settings->flag5.mqtt_state_retain);
    }
    if (flowdeb) { 
        /* Continous flow of x LPM for > 10sec will clear DRYRUN fault */
        bitClear(softnersensors.errorstates,ERROR_DRYRUN);
        bitClear(softnersensors.errorstates,ERROR_DRYRUNPERM);
    } else if (GetValveState(RELAY_SOFTNER)) {
        /* Valve is open, but reporting dry */
        if ((softnersensors.dryrundebstate & DEB16MSK_NEWEVENT)!=0) {
            /* A new confirmed state of dry (<xLPM): Either remained dry throughout or <xLPM event now.
               During external operation of valve, the event is forced via DEBONCE flag reset from WaterLevelController */
            SetValveState(RELAY_SOFTNER,false);
            bitSet(softnersensors.errorstates,ERROR_DRYRUN);                   
            if (++softnersensors.filltries>=MAXFILLTRY) {
                softnersensors.abortfill = true;
                bitSet(softnersensors.errorstates,ERROR_DRYRUNPERM);
            }
        }
    }
}

void ShowLevelAndStates(bool json) {
    if(json) {
      uint8_t ssr = (uint8_t)(softnersensors.raw_state & 0xF);
      uint8_t ssd = (uint8_t)(softnersensors.deb_state & 0xF);
      uint8_t saltvol = (uint8_t)(softnersensors.salt_volume*100.0/softnerparams.saltcapacity + 0.5);    // to % 
      ResponseAppend_P(PSTR(",\"SensorStates\":{\"LoSwt\":%d,\"MidSwt\":%d,\"HiSwt\":%d,\"RegenSwt\":%d}"),
                                    (((ssd>>TEMPTY) &1)<<1 | (ssr>>TEMPTY)&1),      //xxx1 + xxx1 --> 00000011
                                    (((ssd>>TMED) &1)<<1 | (ssr>>TMED)&1),          //xx1x + xx1x --> 00000011
                                    (((ssd>>TFULL) &1)<<1 | (ssr>>TFULL)&1),        //x1xx + x1xx --> 00000011
                                    (((ssd>>SOFTNER) &1)<<1 | (ssr>>SOFTNER)&1));   //1xxx + 1xxx --> 00000011
      ResponseAppend_P(PSTR(",\"Tank\":{\"Level\":%d}"),softnersensors.waterlevel);
      ResponseAppend_P(PSTR(",\"Softner\":{\"FlowRateLPM\":%1_f, \"Volume\":%1_f, \"Regeneration\":%d, \"Salt\":%d}"),
                                        &softnersensors.mass_flow_rate,&softnersensors.total_volume,GET_DEBSTATE(3),saltvol);
      if (softnersensors.errorstates == 0) {
        ResponseAppend_P(PSTR(",\"Diagnostics\":{\"Code\":%d,\"Message\":\"%s\"}"),softnersensors.errorstates,PSTR("NO WARNING OR FAULTS"));
      } else {
        /* At least one error present */
        ResponseAppend_P(PSTR(",\"Diagnostics\":{\"Code\":%d,\"Message\":\""),softnersensors.errorstates);
        bool first=true;
        if (bitRead(softnersensors.errorstates,ERROR_DRYRUNPERM)) {
            ResponseAppend_P(PSTR("%s"),PSTR("NO WATER"));
            first=false;
        } else if (bitRead(softnersensors.errorstates,ERROR_DRYRUN)){
            ResponseAppend_P(PSTR("%s"),PSTR("LOW FLOW"));
            first=false;
        }
        
        if (bitRead(softnersensors.errorstates, ERROR_ADCSRCMIN)) {
            if (!first) {ResponseAppend_P(PSTR("%s"),PSTR(","));}
            ResponseAppend_P(PSTR("%s"),PSTR("ADC SCG"));
            first=false;
        } else if (bitRead(softnersensors.errorstates,ERROR_ADCSRCMAX)) {
            if (!first) {ResponseAppend_P(PSTR("%s"),PSTR(","));}
            ResponseAppend_P(PSTR("%s"),PSTR("ADC SCB"));
            first=false;
        }

        if (bitRead(softnersensors.errorstates, WARN_TIMEWINLOCK)) {
            if (!first) {ResponseAppend_P(PSTR("%s"),PSTR(","));}
            ResponseAppend_P(PSTR("%s"),PSTR("TIME WINDOW LOCK"));
            first=false;
        } 
        if (bitRead(softnersensors.errorstates,WARN_MINLOCK)) {
            if (!first) {ResponseAppend_P(PSTR("%s"),PSTR(","));}
            ResponseAppend_P(PSTR("%s"),PSTR("COOLING LOCK"));
            first=false;
        }
        if (bitRead(softnersensors.errorstates,WARN_REGENLOCK)) {
            if (!first) {ResponseAppend_P(PSTR("%s"),PSTR(","));}
            ResponseAppend_P(PSTR("%s"),PSTR("HARD WATER"));
            first=false;
        }

        if(first) {
            ResponseAppend_P(PSTR("Unknown Fault"));
        }

        ResponseAppend_P(PSTR("\"}"));
      }
#ifdef USE_WEBSERVER
    } else {
      WSContentSend_P(HTTP_SOFTNERCTL_SNSR_INFO, "Softner Sensors: Raw Value", GET_RAWSTATE(0),GET_RAWSTATE(1),GET_RAWSTATE(2),GET_RAWSTATE(3));
      WSContentSend_P(HTTP_SOFTNERCTL_SNSR_INFO, "Softner Sensors: Debounced Value", GET_DEBSTATE(0),GET_DEBSTATE(1),GET_DEBSTATE(2),GET_DEBSTATE(3));
      WSContentSend_P(HTTP_SOFTNERCTL_INFO, "Water Level", softnersensors.waterlevel, "%");
      WSContentSend_P(HTTP_SOFTNERCTL_INFO, "Adc input", softnersensors.adc_value, ""); 
      
      char  pr[FLOATSZ], pl[FLOATSZ],vd[FLOATSZ],fr[FLOATSZ];
      dtostrfd((double)(softnersensors.pulse[0]), 0, pr);
      dtostrfd((double)(softnersensors.pulse[1]), 0, pl);
      dtostrfd((double)(softnersensors.voldelta), 3, vd);
      dtostrfd((double)(softnersensors.mass_flow_rate), 1, fr);
      WSContentSend_P(PSTR("{s}%s{m} %s%s{e}"), "Pulse Rate (Measured)", pr, "/s");
      WSContentSend_P(PSTR("{s}%s{m} %s%s{e}"), "Pulse Rate (Interpolated) ", pl, "/L");
      WSContentSend_P(PSTR("{s}%s{m} %s%s{e}"), "Delta Volume ", vd, "L");
      WSContentSend_P(PSTR("{s}%s{m} %s%s{e}"), "Water Flow ", fr, "LPM");
      WSContentSend_P(PSTR("{s}%s{m} %d %d %d{e}"), "Valve State", bitRead(softnersensors.valvestate,2),bitRead(softnersensors.valvestate,1),bitRead(softnersensors.valvestate,0));
#endif  // USE_WEBSERVER
    }
}

void UpdateSensors250ms(void) {
    bool update = false;  //set to trigger MQTT
    bool savelevel = false; //set to save values to flash
    float vol_delta=5;
    float mf_delta=5;
    if ((softnersensors.raw_state & 0xF) != (softnersensors.raw_state>>4 & 0xF)) {
        update=true;
    }
    if ((softnersensors.deb_state & 0xF) != (softnersensors.deb_state>>4 & 0xF)) {
        update=true; //only for regen state
    }
    if (fabs(softnersensors.total_volume-softnersensors.total_volume_old)>vol_delta) {
        update=true;
        savelevel=true; 
        softnersensors.total_volume_old=softnersensors.total_volume; //kept inside to avoid missing triggers       
    }
    if ((fabs(softnersensors.mass_flow_rate-softnersensors.mass_flow_rate_old)>mf_delta) || 
       (((uint8_t)softnersensors.mass_flow_rate == 0) && (softnersensors.mass_flow_rate != softnersensors.mass_flow_rate_old)))
    {        
        update=true;
    }
    if (softnersensors.waterlevel_old != softnersensors.waterlevel) {        
        update=true;
        savelevel=true;
        softnersensors.waterlevel_old=softnersensors.waterlevel;//kept inside to avoid possible save trigger miss
    }
    if (softnersensors.errorstates_old != softnersensors.errorstates) {
        update=true;
    }
    if(softnersensors.salt_volume_old != softnersensors.salt_volume) {
        softnersensors.salt_volume_old = softnersensors.salt_volume;
        savelevel=true;
        update = true;
    }
    if (update) {
        char scmnd[20];
        snprintf_P(scmnd, sizeof(scmnd), PSTR(D_CMND_STATUS " 10"));
        ExecuteCommand(scmnd, SRC_SENSOR);

        softnersensors.raw_state = (softnersensors.raw_state & 0xF)<<4 | (softnersensors.raw_state & 0xF);
        softnersensors.deb_state = (softnersensors.deb_state & 0xF)<<4 | (softnersensors.deb_state & 0xF);        
        softnersensors.mass_flow_rate_old=softnersensors.mass_flow_rate;        
        softnersensors.errorstates_old = softnersensors.errorstates;
        if (savelevel) {SaveSoftnerStates();}
    }
}

void SaveSoftnerStates() { //save volumes
    Settings->softner_totalvolume = (uint16_t)(softnersensors.total_volume*10);
    Settings->softner_watervolume = (uint16_t)(softnersensors.watervolume*10);
    Settings->softner_saltlevel = (uint16_t)(softnersensors.salt_volume*100.0);
}

bool ActivateRoutine(uint8_t id) {
    /* id:
         0 = cancel (fill 0)
         1..4 = routine X (fill settings)
         255 = manual trigger (no fill)
    */
    bool state=false;
    if(softnersensors.routineactive==0) {
        state=GetRoutine(id); 
    } else if (id==0) {
        //cancel routine
        GetRoutine(id);
        state=true;
    }
    return state;
}

void EvaluateRelayStates(void) {
    for(uint8_t i=RELAY_UTILITY;i<=RELAY_BACK;i++) {
        if(GetValveState(i)) {
            if(!bitRead(softnersensors.routinestate,i) && !bitRead(softnersensors.routinestate,i+3)) {
                /* Manual switch on */
                InitTimeouts(i-1);
                bitSet(softnersensors.routinestate,i+3);
                Response_P(PSTR("{\"id\":%d,\"action\":\"on\",\"remaining\":%d}"),100+i,softnersensors.timeout[i-1]);
                MqttPublishPrefixTopic_P(STAT, PSTR("VALVESTATE"), Settings->flag5.mqtt_state_retain);

                softnersensors.wait[i-1]=0; /* any activation can abort wait */
                softnersensors.routine[i-1]=0; /* and routine */
            }       
        } else {
            if(bitRead(softnersensors.routinestate,i)) {
                /* Manual switch off during routine */
                bitClear(softnersensors.routinestate,i);
                softnersensors.wait[i-1]=0; /* any activation can abort wait */
                softnersensors.routine[i-1]=0; /* and routine */
                Response_P(PSTR("{\"id\":%d,\"action\":\"off\"}"),i);
                MqttPublishPrefixTopic_P(STAT, PSTR("VALVESTATE"), Settings->flag5.mqtt_state_retain);
            } else if(bitRead(softnersensors.routinestate,i+3)) {
                /* Manual switch off */
                bitClear(softnersensors.routinestate,i+3);
                Response_P(PSTR("{\"id\":%d,\"action\":\"off\"}"),100+i);
                MqttPublishPrefixTopic_P(STAT, PSTR("VALVESTATE"), Settings->flag5.mqtt_state_retain);
            }

            if (softnersensors.wait[i-1]) {                
                softnersensors.wait[i-1]--; /* waiting for activation */
            } else if (softnersensors.routine[i-1]) {
                bitSet(softnersensors.routinestate,i); /* ready for count down */
            }
        }
    }
}
void RoutineMonitor(void) {
    /* Activates relays in sequence as per count down.
       To reduce the pressure on pipes during activation, next relay is 
       activated before first one finishes. This allows lower back-pressure
       on the closing relay, since new water path will be open (if any).
    */
    EvaluateRelayStates();
    if(softnersensors.routinestate) {
        for(uint8_t i=RELAY_UTILITY;i<=RELAY_BACK;i++) {
            if(bitRead(softnersensors.routinestate,i)) {
                /* Normal routine is active for this valve */            
                if(softnersensors.routine[i-1]) {softnersensors.routine[i-1]--;}
                if(softnersensors.timeout[i-1]) {softnersensors.timeout[i-1]--;}
                if(softnersensors.routine[i-1] == 0 || softnersensors.timeout[i-1]==0) {
                    if (GetValveState(i)) {
                        Response_P(PSTR("{\"id\":%d,\"action\":\"off\"}"),i);
                        MqttPublishPrefixTopic_P(STAT, PSTR("VALVESTATE"), Settings->flag5.mqtt_state_retain);
                    }
                    SetValveState(i,false);
                    bitClear(softnersensors.routinestate,i);
                } else {
                    if (!GetValveState(i)) {
                        Response_P(PSTR("{\"id\":%d,\"action\":\"on\",\"remaining\":%d}"),i,softnersensors.routine[i-1]);
                        MqttPublishPrefixTopic_P(STAT, PSTR("VALVESTATE"), Settings->flag5.mqtt_state_retain);
                    }
                    SetValveState(i,true);
                }
            } else if (bitRead(softnersensors.routinestate,i+3)) {
                /* Timeout delay is active for this valve. */
                if(softnersensors.timeout[i-1]) {softnersensors.timeout[i-1]--;}
                if(softnersensors.timeout[i-1]==0) {
                    if (GetValveState(i)) {
                        Response_P(PSTR("{\"id\":%d,\"action\":\"off\"}"),100+i);
                        MqttPublishPrefixTopic_P(STAT, PSTR("VALVESTATE"), Settings->flag5.mqtt_state_retain);
                    }
                    SetValveState(i,false);
                    bitClear(softnersensors.routinestate,i+3);
                }
            }        
        }
    } else {
        softnersensors.routineactive=0;
    }
}




/***************************** Commands ******************************/
void CmndReadRoutine()
{
    if (XdrvMailbox.data_len) { 
        char argument[XdrvMailbox.data_len];
        if (ArgC() == 1) { //timer id = 1...4
            uint8_t timer = (uint8_t)CharToFloat(ArgV(argument, 1))-1; 
            if (timer>=0 && timer<MAXROUTINES) {
                char payload[100];
                Response_P(PSTR("{\"Routine\":%d,\"Days\":["), timer+1);
                Timer xtimer = Settings->timer[timer];
                bool days[7] = { false };                
                for (uint32_t i = 0; i < 7; i++) {
                    days[i]=bitRead(xtimer.days,i);
                    ResponseAppend_P(PSTR("%d"),days[i]);
                    if(i<6) {ResponseAppend_P(PSTR(","));}
                }
                ResponseAppend_P(PSTR("]"));
            /*  fixed action: rule
                char soutput[80];
                soutput[0] = '\0';
                if (TasmotaGlobal.devices_present) {
                    snprintf_P(soutput, sizeof(soutput), PSTR(",\"" D_JSON_TIMER_OUTPUT "\":%d"), xtimer.device +1);
                } */

               /* Read routines */
                uint8_t routine[3];
                bool active=false;
                if (bitRead(Settings->garden_routines[0],timer)) {
                    for(uint8_t i=0;i<3;i++) {
                        routine[i]=Settings->garden_routines[timer*3+i+1];
                        if(routine[i]) {
                            active=true;
                        }
                    }
                }
            #ifdef USE_SUNRISE
                char sign[2] = { 0 };
                int16_t hour = xtimer.time / 60;
                if ((1 == xtimer.mode) || (2 == xtimer.mode)) {  // Sunrise or Sunset
                    if (hour > 11) {
                    hour -= 12;
                    sign[0] = '-';
                    }
                }
                ResponseAppend_P(PSTR(",\"status\":{\"trigger\":%d,\"setting\":%d},\"mode\":%d,\"hour\":%d,\"minute\":%d"),
                                    xtimer.arm,active,xtimer.mode,hour,xtimer.time%60);
           #else
                ResponseAppend_P(PSTR(",\"status\":{\"trigger\":%d,\"setting\":%d},\"hour\":%d,\"minute\":%d"),
                                    xtimer.arm,active,hour,xtimer.time%60);
            #endif  // USE_SUNRISE           
                ResponseAppend_P(PSTR(",\"State\":%d,\"Sequence\":[%d,%d,%d]"),bitRead(Settings->garden_routines[0],timer+1),
                            Settings->garden_routines[timer*3+1],Settings->garden_routines[timer*3+2],Settings->garden_routines[timer*3+3]);
                ResponseJsonEnd();
                MqttPublishPrefixTopic_P(STAT, PSTR("ROUTINE"), Settings->flag5.mqtt_state_retain);
            }
        }
    }
}

void CmndSoftnerConfig() {
    /* WriteSoftnerConfig 
        1: automode          --> 0: Manual, 1: Auto, 2:Auto with topup
        [2,3]: maxon,cooldn  --> for valve operations
        [4,5]: lolim,hilim   --> for auto fill
        6: regenlimit        --> for inhibit autofill (hard water)
        7: tankcapacity      --> for % calculations
        [8,9,10]: sensor levels --> [low,mid,hi]
        [11,12,13]: salt marker --> [usage,marker volume,marker level]
    */        
    if (XdrvMailbox.data_len) {
        char argument[XdrvMailbox.data_len];
        if (ArgC() >= 1) {   
            uint8_t automode = (uint8_t)CharToFloat(ArgV(argument, 1));
            if (automode == 0 || automode == 1 || automode == 2) {
                bitWrite(Settings->softner_flags,AUTOMODE,(bool)(automode != 0));
                bitWrite(Settings->softner_flags,TOPUPMODE,(bool)(automode == 2));
            } 
             
            if (ArgC() >= 3) {   
                uint16_t maxon = (uint16_t)CharToFloat(ArgV(argument, 2));
                uint16_t cooldn = (uint16_t)CharToFloat(ArgV(argument, 3));
                if (maxon <= 1800) {                    
                    Settings->softner_maxon[RELAY_SOFTNER]=maxon;
                    softnerparams.relay_limit[RELAY_SOFTNER]=maxon;
                    bitSet(Settings->softner_flags,MAXON1);
                }
                if (cooldn <= 3600) {
                    Settings->softner_minoff[RELAY_SOFTNER]=cooldn;
                    softnerparams.relay_minlock[RELAY_SOFTNER]=cooldn;
                    bitSet(Settings->softner_flags,MINOFF1);
                }
            }  

            //old % limit values
            float onlimit = softnerparams.onlimit*100/softnerparams.tank_volume;
            float offlimit = softnerparams.offlimit*100/softnerparams.tank_volume;
            if (ArgC() >= 7) { // tank capacity
                float capa = softnerparams.tank_volume;
                float newcapa = CharToFloat(ArgV(argument, 7));
                if (newcapa>=500 && newcapa <=5000) 
                {
                    softnerparams.tank_volume = newcapa; 
                    Settings->softner_tankvolume = (uint16_t)newcapa;
                    bitSet(Settings->softner_flags,TANKVOL);

                    
                    /* Recalcualte limits based on new volume - convert back to abs */
                    onlimit = onlimit*softnerparams.tank_volume/100; //absolute values
                    offlimit = offlimit*softnerparams.tank_volume/100;                    
                    Settings->softner_onlimit = (uint16_t)onlimit;
                    Settings->softner_offlimit = (uint16_t)offlimit;
                    bitSet(Settings->softner_flags,ONLIM);
                    bitSet(Settings->softner_flags,OFFLIM);
                    softnerparams.onlimit = onlimit;
                    softnerparams.offlimit = offlimit;
                }
            }

            if (ArgC() >= 5) {
                float newonlimit = CharToFloat(ArgV(argument, 4)); 
                float newofflimit = CharToFloat(ArgV(argument, 5));
                if ((newofflimit - newonlimit >= 20) &&
                    (newonlimit >=15 && newonlimit <=70) &&
                    (newofflimit >= 40) && (newofflimit <=100))
                {
                    newonlimit = newonlimit*softnerparams.tank_volume/100; //absolute values
                    newofflimit = newofflimit*softnerparams.tank_volume/100;
                    Settings->softner_onlimit = (uint16_t)newonlimit;
                    softnerparams.onlimit = newonlimit;
                    Settings->softner_offlimit = (uint16_t)newofflimit;
                    softnerparams.offlimit = newofflimit;
                    bitSet(Settings->softner_flags,ONLIM);
                    bitSet(Settings->softner_flags,OFFLIM);
                }
            }
        

            if (ArgC() >= 6) { // regen limit to prevent autofill
                float regen = softnerparams.regenlimit;
                float newregen = CharToFloat(ArgV(argument, 6));
                if (newregen>500 && newregen <5000) 
                {
                    softnerparams.regenlimit = newregen; 
                    Settings->softner_regenlimit = (uint16_t)newregen;
                    bitSet(Settings->softner_flags,REGENLIM);
                }
            }  

            if (ArgC() >= 10) { // tank sensor levels
                float lo = CharToFloat(ArgV(argument, 8));
                float mid = CharToFloat(ArgV(argument, 9));
                float hi = CharToFloat(ArgV(argument, 10));
                float capa = softnerparams.tank_volume;

                if ((lo>0.05*capa && lo<0.35*capa) &&
                   (mid>0.35*capa && mid<0.7*capa) &&
                   (hi>0.7*capa && hi<=capa)) {
                    Settings->softner_sensorlevels[0] = (uint16_t)lo;
                    Settings->softner_sensorlevels[1] = (uint16_t)mid;
                    Settings->softner_sensorlevels[2] = (uint16_t)hi;
                    softnerparams.sensorlevels[0] = (uint16_t)lo;
                    softnerparams.sensorlevels[1] = (uint16_t)mid;
                    softnerparams.sensorlevels[2] = (uint16_t)hi;
                    bitSet(Settings->softner_flags,LOWLVL);
                    bitSet(Settings->softner_flags,MEDLVL);
                    bitSet(Settings->softner_flags,HIGHLVL);
                } else {
                    softnerparams.sensorlevels[0]=(uint16_t)(0.2*capa);
                    softnerparams.sensorlevels[1]=(uint16_t)(0.6*capa);
                    softnerparams.sensorlevels[2]=(uint16_t)(capa);
                    bitClear(Settings->softner_flags,LOWLVL);
                    bitClear(Settings->softner_flags,MEDLVL);
                    bitClear(Settings->softner_flags,HIGHLVL);
                }
            }

            if (ArgC() >= 13) { 
                float usagev = CharToFloat(ArgV(argument, 11)); //in grams
                if (usagev>=1000 && usagev <=5000) 
                {
                    softnerparams.saltusage = usagev/1000;  //in Kg
                    Settings->softner_saltusage = (uint8_t)(usagev/100.0); //Kg, resolution 0.1
                    bitSet(Settings->softner_flags,SALTUSAGE);
                }
                
                float markerv = CharToFloat(ArgV(argument, 12)); //in Kg
                if (markerv>=10 && markerv <=75) 
                {
                    softnerparams.saltcapacity = markerv; //in Kg
                    Settings->softner_saltcapacity = (uint8_t)(markerv); //Kg
                    bitSet(Settings->softner_flags,SALTCAPAVALID);
                }

                float newval = CharToFloat(ArgV(argument, 13)); //in level marker %
                if (newval>=0 && newval <=100) {                    
                    softnersensors.salt_volume = newval*softnerparams.saltcapacity/100;                
                    bitSet(Settings->softner_flags,SALTLVL);
                }
            }         
        }
        ReadSoftnerConfig(false); 
    } else {
        ReadSoftnerConfig(true);   
    }
}

void ReadSoftnerConfig(bool help) {
    /* Global settings */
    Response_P(PSTR("{\"automode\":%d,\"topup\":%d"),SOFTNER_FLAGS(AUTOMODE),SOFTNER_FLAGS(TOPUPMODE));
    /* Valve settings*/
    ResponseAppend_P(PSTR(",\"valve\":{\"maxon\":%d,\"minoff\":%d}"),
                        softnerparams.relay_limit[RELAY_SOFTNER],softnerparams.relay_minlock[RELAY_SOFTNER]);
    /* Tank limit settings */    
    ResponseAppend_P(PSTR(",\"tank\":{\"capa\":%d"),(uint16_t)softnerparams.tank_volume);
 
   // dtostrfd((double)(softnerparams.onlimit*100/softnerparams.tank_volume), 0, arr2);  
    ResponseAppend_P(PSTR(",\"autoon\":%d"),(uint16_t)(softnerparams.onlimit*100/softnerparams.tank_volume));

   // dtostrfd((double)(softnerparams.offlimit*100/softnerparams.tank_volume), 0, arr3);
    ResponseAppend_P(PSTR(",\"autooff\":%d"),(uint16_t)(softnerparams.offlimit*100/softnerparams.tank_volume));

    ResponseAppend_P(PSTR(",\"regenlim\":%d"),(uint16_t)softnerparams.regenlimit);

    ResponseAppend_P(PSTR(",\"sensorlevels\":{\"low\":%d,\"mid\":%d,\"high\":%d}}"),
                                softnerparams.sensorlevels[0],softnerparams.sensorlevels[1],softnerparams.sensorlevels[2]);

    ResponseAppend_P(PSTR(",\"saltcapacity\":%d,\"saltusage\":%d,\"saltlevel\":%d"),
                            (uint16_t)(softnerparams.saltcapacity),
                            (uint16_t)(softnerparams.saltusage*1000),
                            (uint8_t)(softnersensors.salt_volume*100/softnerparams.saltcapacity));

    /* Ipo values */
    /* ResponseAppend_P(PSTR(",\"sensTX\":[%s,%s,%s,%s,%s],sensTZ\":[%s,%s,%s,%s,%s]"),
                        F2Str(softnerparams.ctPulsePerLNorm_T[1]),F2Str(softnerparams.ctPulsePerLNorm_T[2]),F2Str(softnerparams.ctPulsePerLNorm_T[3]),
                        F2Str(softnerparams.ctPulsePerLNorm_T[4]),F2Str(softnerparams.ctPulsePerLNorm_T[5]),
                        F2Str(softnerparams.ctPulsePerLNorm_T[6]),F2Str(softnerparams.ctPulsePerLNorm_T[7]),F2Str(softnerparams.ctPulsePerLNorm_T[8]),
                        F2Str(softnerparams.ctPulsePerLNorm_T[9]),F2Str(softnerparams.ctPulsePerLNorm_T[10]));*/
    ResponseJsonEnd();
    MqttPublishPrefixTopic_P(STAT, PSTR("SOFTNERCONF"), Settings->flag5.mqtt_state_retain);
    if(help) {
        Response_P(PSTR("{\"%s\":{\"Usage\":\"SoftnerConf automode(0|1|2), [maxon:<1800, minoff<3600], [autoon:20-80, autooff:75-100], regen:1k-5k, tankvol:500-5K, [sensorlow ,sensormed, sensorhigh]:in L, [salt_usage,marker_vol - in g,marker_level in %]\"}}"), XdrvMailbox.command);
    }
}

void CmndTestMode() {
    if (XdrvMailbox.data_len) {
        char argument[XdrvMailbox.data_len];
        if (ArgC() == 1 || ArgC() == 5) {
            softnerparams.testmode = true; 
            softnerparams.testctrdelta = (uint8_t)CharToFloat(ArgV(argument, 1));
            if (ArgC() == 5) {
                for(uint8_t i=TEMPTY;i<=SOFTNER;i++) {
                    bitWrite(softnersensors.raw_state,i,(bool)(CharToFloat(ArgV(argument, i+2))!=0));
                }
            }
            Response_P(PSTR("{\"%s\":\"TestMode Active CTRD:%d, L:%d, M:%d, H:%d, R:%d\"}"), XdrvMailbox.command,softnerparams.testctrdelta,
                                                            bitRead(softnersensors.raw_state,0),bitRead(softnersensors.raw_state,1),
                                                            bitRead(softnersensors.raw_state,2),bitRead(softnersensors.raw_state,3));
        } else {
            softnerparams.testmode = false;
            Response_P(PSTR("{\"%s\":{\"TestMode\":\"Not Active\",\"Usage\":\"TestMode ctrdelta,[loswt,medswt,highswt,regen]\"}}"), XdrvMailbox.command);
        }
    } else {
        softnerparams.testmode = false;
        Response_P(PSTR("{\"%s\":{\"TestMode\":\"Not Active\",\"Usage\":\"TestMode ctrdelta,[loswt,medswt,highswt,regen]\"}}"), XdrvMailbox.command);
    }
}

void CmndTimerEvent() {
    /* These are triggered by configured schedules in the timers via script / rule.
     Current support for 4 events : Timer1 ... 4 
     Special events: 0 = cancel active, 255 = manual */     
     if (XdrvMailbox.data_len) { 
        char argument[XdrvMailbox.data_len];
        if (ArgC() == 1 || ArgC() == 4) { //timer id or manual values
            uint8_t timer = (uint8_t)CharToFloat(ArgV(argument, 1));
            bool valid = (bool)(timer!=255);
            if (timer==255 && ArgC() == 4 && softnersensors.routineactive==0) {
                uint8_t mantmr[4];
                mantmr[0]=0; //bit0: at least one timer >=5, bit 1..3: timer is valid nonzero value
                for (uint8_t i=1;i<=3;i++) {
                    mantmr[i] = (uint8_t)CharToFloat(ArgV(argument, i+1));
                    bitWrite(mantmr[0],i,(bool)(mantmr[i]>=5));                    
                    if (mantmr[i]>=5) {bitSet(mantmr[0],0);}
                }
                if (bitRead(mantmr[0],0)) {
                    for (uint8_t i=0;i<3;i++) {
                        softnersensors.routine[i] = (bitRead(mantmr[0],i+1))?mantmr[i+1]:0;
                    }
                    valid = true;
                }
            }
            if (valid && ActivateRoutine(timer)) {
                if (timer==0) {
                    Response_P(PSTR("{\"%s\":\"Canceled active routine %d\"}"), XdrvMailbox.command,softnersensors.routineactive);
                } else {
                    Response_P(PSTR("{\"%s\":\"Activated routine %d\"}"), XdrvMailbox.command,timer);
                }
            } else {
                Response_P(PSTR("{\"%s\":\"Failed to activate routine %d\", \"routine active\":%d,\"remaining\":[%d,%d,%d]}"), 
                                    XdrvMailbox.command,timer,softnersensors.routineactive,
                                    softnersensors.routine[0],softnersensors.routine[1],softnersensors.routine[2]);
            }
        }
     }
}

void CmndRoutine(void) {
    /* Routine 1, 30, 20, 40 */
    if (XdrvMailbox.data_len) {
        char argument[XdrvMailbox.data_len];
        if (ArgC() > 0) {            
            uint8_t timer = (uint8_t)CharToFloat(ArgV(argument, 1))-1; 
            if (timer < MAXROUTINES) {
                if (ArgC() > 3) {
                    bool success=false;
                    uint8_t r1t = (uint8_t)CharToFloat(ArgV(argument, 2));    
                    uint8_t r2t = (uint8_t)CharToFloat(ArgV(argument, 3));    
                    uint8_t r3t = (uint8_t)CharToFloat(ArgV(argument, 4));
                    if(timer < MAXROUTINES && (r1t || r2t || r3t)) {
                        Settings->garden_routines[timer*3+1] = r1t;
                        Settings->garden_routines[timer*3+2] = r2t;
                        Settings->garden_routines[timer*3+3] = r3t;
                        bitSet(Settings->garden_routines[0],timer);
                        success=true;
                    } else if(timer < MAXROUTINES) {
                        Settings->garden_routines[timer*3+1] = 0;
                        Settings->garden_routines[timer*3+2] = 0;
                        Settings->garden_routines[timer*3+3] = 0;
                        bitClear(Settings->garden_routines[0],timer);
                        success=true;
                    }
                    if(success) {
                        Response_P(PSTR("{\"%s\":{\"Routine\":%d,\"Sequence\":{\"Utility\":%d,\"Front\":%d,\"Back\":%d}}}"),
                                            XdrvMailbox.command,timer+1,r1t,r2t,r3t);
                    } else {
                        Response_P(PSTR("{\"%s(not updated)\":{\"Usage\":\"Routine <seqid(1..4)>, <utility>, <front>, <back> (in seconds, range 0 - 255)\"}}"), XdrvMailbox.command);
                    }
                } else {
                    Response_P(PSTR("{\"%s\":{\"Routine id\":%d,\"State\":%d,\"Sequence[U,F,B]\":[%d,%d,%d]}}"), XdrvMailbox.command,
                            timer, bitRead(Settings->garden_routines[0],timer+1),
                            Settings->garden_routines[timer*3+1],Settings->garden_routines[timer*3+2],Settings->garden_routines[timer*3+3]);
                }
            } else {
                Response_P(PSTR("{\"%s\":{\"Error\":\"(routine %d not supported\"}}"), XdrvMailbox.command,timer+1);
            }
        } else {
            Response_P(PSTR("{\"%s\":{\"Usage\":\"Routine <seqid(1..4)>, <utility>, <front>, <back> (in seconds, range 0 - 255)\"}}"), XdrvMailbox.command);
        }
    } else {
        Response_P(PSTR("{\"%s\":{\"Usage\":\"Routine <seqid(1..4)>, <utility>, <front>, <back> (in seconds, range 0 - 255)\"}}"), XdrvMailbox.command);
    }
}

void CmndSoftnerCommands(void) {
    Response_P(PSTR("{\"%s\":[\"SensorDebounceX(2)\",\"FlowFactor(2)\",\"TankLimit(2)\",\"TankStats(0)\",\"ValveLimit(4)\",\"Routine(4)\",\"SoftnerConfig(10)\",\"SoftnerInitX(1)\"]}"), XdrvMailbox.command);
}


void CmndSensorDeb(void) {
    /*  SensorDebounceX <on>,<off>  
        SensorDebounce0 5,10,5,10,5,10,30,10 --> all at once
        SensorDebounce1 5,5 --> for <empty> */
    if ((XdrvMailbox.index >= TEMPTY) && (XdrvMailbox.index <= SOFTNER+1)) {
        uint8_t sensorid = XdrvMailbox.index;
        char argument[XdrvMailbox.data_len];

        if (sensorid>0 && ArgC() > 1) {                           
            uint8_t tonr = (uint8_t)CharToFloat(ArgV(argument, 1));
            uint8_t toffr = (uint8_t)CharToFloat(ArgV(argument, 2));
            uint8_t ton=(tonr<=75)?tonr:75;
            uint8_t toff=(toffr<=75)?toffr:75;
            sensorid=sensorid-1;

            Settings->softner_deblim[sensorid] = (uint8_t)(ton/5)<<4 | (uint8_t)(toff/5);

            ton = ((Settings->softner_deblim[sensorid] & 0xF0)>>4) * 5;  
            toff = (Settings->softner_deblim[sensorid] & 0xF) * 5;  
            softnerparams.deb_limit[sensorid] = (uint16_t)(ton << 8) | (uint16_t)toff;

            bitSet(Settings->softner_flags,sensorid);
        } else if (sensorid==0 && ArgC() >= 8) {
            uint8_t tonr[4];
            uint8_t toffr[4];
            for (uint8_t i=0;i<=SOFTNER;i++) {
                tonr[i] = (uint8_t)CharToFloat(ArgV(argument, 2*i+1));
                toffr[i] = (uint8_t)CharToFloat(ArgV(argument, 2*i+2));
                tonr[i] = (tonr[i]<=75)?tonr[i]:75;
                toffr[i] = (toffr[i]<=75)?toffr[i]:75;

                Settings->softner_deblim[i] = (uint8_t)(tonr[i]/5)<<4 | (uint8_t)(toffr[i]/5);
                bitSet(Settings->softner_flags,sensorid);

                tonr[i] = ((Settings->softner_deblim[i] & 0xF0)>>4) * 5;  
                toffr[i] = (Settings->softner_deblim[i] & 0xF) * 5;

                softnerparams.deb_limit[i] = (uint16_t)(tonr[i] << 8) | (uint16_t)toffr[i];                
            }
        }
        
        /* Send response */
        if (XdrvMailbox.index>0) {
            sensorid=XdrvMailbox.index-1;
            Response_P(PSTR("{\"%s\":{\"SensorIndex\":%d,\"Sensor\":\"%s\",\"DebTOn\":%d,\"DebTOff\":%d}}"), XdrvMailbox.command,sensorid,
                            sensorid==0?PSTR("LOW"):sensorid==1?PSTR("MID"):sensorid==2?PSTR("HIGH"):PSTR("REGEN"),
                            (uint16_t)(softnerparams.deb_limit[sensorid] >> 8),
                            (uint16_t)(softnerparams.deb_limit[sensorid] & 0xFF));
        } else if (XdrvMailbox.index==0) { 
            Response_P(PSTR("{\"%s\":["), XdrvMailbox.command);
            for (uint8_t i=0;i<=SOFTNER;i++) {
                ResponseAppend_P(PSTR("{\"Sensor\":\"%s\",\"Id\":%d,\"DebTOn\":%d,\"DebTOff\":%d}"), 
                            i==0?PSTR("LOW"):i==1?PSTR("MID"):i==2?PSTR("HIGH"):PSTR("REGEN"),i,
                            (uint16_t)(softnerparams.deb_limit[i] >> 8),
                            (uint16_t)(softnerparams.deb_limit[i] & 0xFF));
                if(i<SOFTNER){ResponseAppend_P(PSTR(","));} else {ResponseAppend_P(PSTR("]"));}            
            }
            ResponseJsonEnd();
            MqttPublishPrefixTopic_P(STAT, PSTR("SENSORDEB"), Settings->flag5.mqtt_state_retain);
        }          
    } else {
        Response_P(PSTR("{\"%s(error)\":{\"Usage\":\"e.g. SensorDebounceX <TOn>, <TOff> (X 0:all, 1(Low)...4(Softner), T in seconds, range 0 - 75)\"}}"), XdrvMailbox.command);
    }
}


void CmndValveLimit(void) {
    if (XdrvMailbox.data_len) { 
        if (ArgC() > 3) {
            char argument[XdrvMailbox.data_len];
            for(uint8_t idx=0;idx<=3;idx++) {
                float newlim = CharToFloat(ArgV(argument, idx+1));
                if ((int)newlim>=0 && (int)newlim<=1800) {
                    softnerparams.relay_limit[idx]=(uint16_t)newlim;
                }
            }
            Response_P(PSTR("{\"%s\":{\"RelayLimits\":[%d,%d,%d,%d]}}"), XdrvMailbox.command,
                                softnerparams.relay_limit[0],softnerparams.relay_limit[1],
                                softnerparams.relay_limit[2],softnerparams.relay_limit[3]);
        } else {
            Response_P(PSTR("{\"%s(error)\":{\"Usage\":\"e.g. ValveLimit <softner>, <utility>, <front>, <back> (in seconds, range 0 - 1800)\"}}"), XdrvMailbox.command);    
        }
    } else {
        Response_P(PSTR("{\"%s\":{\"RelayLimits\":[%d,%d,%d,%d]}}"), XdrvMailbox.command,
                                softnerparams.relay_limit[0],softnerparams.relay_limit[1],
                                softnerparams.relay_limit[2],softnerparams.relay_limit[3]);
    }
}

void CmndFlowFactor(void){
    if (XdrvMailbox.data_len) { 
        if (ArgC() > 1) {
            char argument[XdrvMailbox.data_len];
            softnerparams.charge_rate_factor = CharToFloat(ArgV(argument, 1));
            softnerparams.discharge_rate_factor = CharToFloat(ArgV(argument, 2));
            char  cr[FLOATSZ], dr[FLOATSZ];
            dtostrfd((double)(softnerparams.charge_rate_factor), 2, cr);
            dtostrfd((double)(softnerparams.discharge_rate_factor), 2, dr);
            Response_P(PSTR("{\"%s(updated)\":{\"MassFlowRateFactor\":\"%s\",\"StaticDischargeFactor\":\"%s\"}}"), XdrvMailbox.command,cr,dr);
        } else {
            Response_P(PSTR("{\"%s(error)\":{\"Usage\":\"e.g. FlowFactor <inflow factor>,<discharge factor>\"}}"), XdrvMailbox.command);
        }
    } else {
        char  cr[FLOATSZ], dr[FLOATSZ];
        dtostrfd((double)(softnerparams.charge_rate_factor), 2, cr);
        dtostrfd((double)(softnerparams.discharge_rate_factor), 2, dr);
        Response_P(PSTR("{\"%s(no update)\":{\"Hint\":\"e.g. FlowFactor <inflow factor>,<discharge factor>\","
                        "\"MassFlowRateFactor\":\"%s\",\"StaticDischargeFactor\":\"%s\"}}"), XdrvMailbox.command,cr,dr);
    }
}

void CmndSoftnerInit() {
    /* To be used only after reflashing completely. Use values from HA history expected. 
     Softnerinit1 <x> : Water in tank in L
     Softnerinit2 <x> : Water since Regen in L */
    if ((XdrvMailbox.index ==1) || (XdrvMailbox.index ==2) && ArgC() == 1) {
        char argument[XdrvMailbox.data_len];
        float newval = CharToFloat(ArgV(argument, 1));
        bool updated = false;
        if (XdrvMailbox.index==1)
        {
            if ((newval > 0) && (newval <= softnerparams.tank_volume)) {
                softnersensors.watervolume = newval;
                updated=true;
            }
        } else if (XdrvMailbox.index==2) {
            if ((newval > 0) && (newval < 5000)) {
                softnersensors.total_volume = newval;
                updated=true;
            }                 
        } 

        if(updated) {
            SaveSoftnerStates(); 

            Response_P(PSTR("{\"%s\":{\"WaterVolume\":%1_f,\"VolumeSinceRegen\":%1_f}}"), XdrvMailbox.command,
                &softnersensors.watervolume,&softnersensors.total_volume);   
        }                   
    }  else {
        Response_P(PSTR("{\"%s\":\"Usage : SoftnerInitX <Y>, X=1(tank) or 2(regen)\"}"), XdrvMailbox.command);
    }
}

void CmndTankLimit(void)
{
    if (XdrvMailbox.data_len) { 
        if (ArgC() > 1) {
            char argument[XdrvMailbox.data_len];
            float onlimit = softnerparams.onlimit*100/softnerparams.tank_volume; //absolute --> %
            float offlimit = softnerparams.offlimit*100/softnerparams.tank_volume;
            float newonlimit = CharToFloat(ArgV(argument, 1)); 
            float newofflimit = CharToFloat(ArgV(argument, 2));
            bool err=true; 
            if ((newofflimit - newonlimit > 20) &&
                (newonlimit >=20 && newonlimit <=80) &&
                (newofflimit >= 75) && (newofflimit <=100))
            {
                newonlimit = newonlimit*softnerparams.tank_volume/100; //% --> absolute values
                newofflimit = newofflimit*softnerparams.tank_volume/100;
                Settings->softner_onlimit = (uint16_t)newonlimit;
                softnerparams.onlimit = newonlimit;
                Settings->softner_offlimit = (uint16_t)newofflimit;
                softnerparams.offlimit = newofflimit;                
                bitSet(Settings->softner_flags,ONLIM);
                bitSet(Settings->softner_flags,OFFLIM);
                err=false;
            }
            char onl[FLOATSZ],offl[FLOATSZ];
            char onla[FLOATSZ],offla[FLOATSZ];
            dtostrfd((double)(softnerparams.onlimit*100/softnerparams.tank_volume), 0, onl);
            dtostrfd((double)(softnerparams.offlimit*100/softnerparams.tank_volume), 0, offl);
            dtostrfd((double)(softnerparams.onlimit), 1, onla);
            dtostrfd((double)(softnerparams.offlimit), 1, offla);
            if (err) {
                Response_P(PSTR("{\"%s(failed)\":{\"Restrictions\":\"OnLimit(20 - 80%), OffLimit(75 - 100%, minimum gap 20%)\",\"On%\":%s,\"Off%\":%s,\"OnVolume\":\"%s L\",\"OffVolume\":\"%s L\"}}"), XdrvMailbox.command, onl,offl,onla,offla);
            } else {
                Response_P(PSTR("{\"%s\":{\"On%\":%s,\"Off%\":%s,\"OnVolume\":\"%s L\",\"OffVolume\":\"%s L\"}}"), XdrvMailbox.command, onl,offl,onla,offla);
            }
        } else {
            Response_P(PSTR("{\"%s(error)\":{\"Usage\":\"e.g. TankLimit <OnLimit> (20 - 80%), <OffLimit> (75 - 100%, minimum gap 20%)\"}}"), XdrvMailbox.command);
        }
    } else {
        char onl[FLOATSZ],offl[FLOATSZ];
        char onla[FLOATSZ],offla[FLOATSZ];
        dtostrfd((double)(softnerparams.onlimit*100/softnerparams.tank_volume), 0, onl);
        dtostrfd((double)(softnerparams.offlimit*100/softnerparams.tank_volume), 0, offl);
        dtostrfd((double)(softnerparams.onlimit), 1, onla);
        dtostrfd((double)(softnerparams.offlimit), 1, offla);
        Response_P(PSTR("{\"%s\":{\"On%\":%s,\"Off%\":%s,\"OnVolume\":\"%s L\",\"OffVolume\":\"%s L\"}}"), XdrvMailbox.command, onl,offl,onla,offla);
    }
}

void CmndTankStats(void)
{
    char dr[FLOATSZ];
    dtostrfd((double)(GetDischargeRate()*60), 4, dr);
    char mf[FLOATSZ];
    dtostrfd((double)softnersensors.mass_flow_rate, 1, mf);
    char vol[FLOATSZ];
    dtostrfd((double)softnersensors.watervolume, 3, vol);
    Response_P(PSTR("{\"%s\":{\"StaticDischarge\":\"%s lpm\",\"MassflowRate\":\"%s lpm\",\"WaterInTank\":\"%s L\",\"WaterLevel\":\"%d %\",\"FlowState\":{\"Raw\":%d,\"Deb\":%d,\"Time\":%d}}}"), 
                    XdrvMailbox.command, dr, mf, vol, softnersensors.waterlevel,
                    (bool)softnersensors.flowstate&1,(bool)softnersensors.flowstate&2,softnersensors.flowstate>>2);
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns120(uint32_t function) {
    bool result = false;
    switch (function) {
        case FUNC_INIT:
            SoftnerCtlInit();
            return true;
            break;
        case FUNC_EVERY_50_MSECOND:
            TankSensorProbe50ms();
            return true;
            break;
        case FUNC_SAVE_SETTINGS:
        case FUNC_SAVE_AT_MIDNIGHT:
            SaveSoftnerStates();
            return true;
            break;
        case FUNC_EVERY_250_MSECOND:
            SenseFlush250ms();
            UpdateSensors250ms();
            return true;
            break;
        case FUNC_EVERY_SECOND:
            MonitorCountDown();
            RoutineMonitor();  
            WaterFlowSensor();
            ModelWaterDischarge();
            CheckRegenWindow();
            WaterLevelController();
            DryRunMonitor();
            result = true;
            break;
        case FUNC_JSON_APPEND:
            ShowLevelAndStates(1);
            break;
    #ifdef USE_WEBSERVER
        case FUNC_WEB_SENSOR:
            ShowLevelAndStates(0);
            break;
    #endif  // USE_WEBSERVER    
        case FUNC_COMMAND:
            result = DecodeCommand(kSoftnerCommands, SoftnerCommand);
            break;
    }
  
  return result;
}

#endif  // USE_SOFTNERCTL
