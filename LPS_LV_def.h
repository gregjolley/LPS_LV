#include <xc.h>
#include "Delay_TMR2.h"

// I/O pin signal names
#define CS_V_DAC        LATBbits.LATB13
#define CS_Ilimit_DAC   LATBbits.LATB14
#define ADC_CONVST      LATBbits.LATB4      // active high
#define comm_ready      LATBbits.LATB9      // 
#define ISO_enable      LATBbits.LATB6      // 

// Exponential time constant is 10.35 mS. The delay to exponential ramping is 15.6 mS
// Note, the frequency tolorance of LPRC used to derive delays is 20% max

// small voltage changes less than 8.192*exp_threshold / 2^16 do not employ exponential ramping
#define exp_threshold   2000    // must be unsigned int

#define exp_mag         0.1656  // previous_voltage_setting + (new_voltage_setting - previous_voltage_setting)*(1.0 - exp_mag*exp(-t/tau))
#define t_V0            485     // These values are loaded into delay_TMR2_ms32
#define t_V1            34      // The delay values must be unsigned int
#define t_V2            39  
#define t_V3            45     
#define t_V4            52     
#define t_V5            61
#define t_V6            75
#define t_V7            96
#define t_V8            135
#define t_V9            102
#define t_V10           134
#define t_V11           242
#define t_V12           280


void load_dummy(){  // Prepare the SPI port to recieve the next instruction from the control board
    
    while (!SPI3STATLbits.SRMT); // wait for all transmissions to complete
    SPI3BUFL = 0x0000;
}

// Write a single word to either the Vset or Ilimit DACs
void SPI2_TX(unsigned int data){
    while (!SPI2STATLbits.SRMT); // wait for all transmissions to complete
    SPI2BUFL = data;
    while (!SPI2STATLbits.SRMT); //
    data = SPI2BUFL;            // dummy read
}

// Write a single work to the Vset DAC
void write_voltage(unsigned int voltage){
    CS_V_DAC = 0;
    SPI2_TX(voltage);
    CS_V_DAC = 1;
}

// Write a single word to the Ilimit DAC
void write_Ilimit(unsigned int Ilimit){
    CS_Ilimit_DAC = 0;
    SPI2_TX(Ilimit);
    CS_Ilimit_DAC = 1;
}

// Determine which instruction has been received from the control board
unsigned int decode_request(){
    
    unsigned int instruction;
    if(SPI3STATLbits.SPIRBE == 0){  // Test for the existence of an instruction in the buffer
        instruction = SPI3BUFL;
        if(instruction == 0xF0F0){  // Control board reads the voltage and load current
            return 1;
        }
        else if(instruction == 0xFF00){ // Set new voltage
            return 2;
        }
        else if(instruction == 0x00FF){ // Set new Ilimit
            return 3;
        }
    }   
    return 0;
}

void read_IV(unsigned int *voltage, unsigned int *current){
    unsigned int    i;
    while (!SPI1STATLbits.SRMT); // wait for all transmissions to complete
    ADC_CONVST = 1;             // start conversion
    for(i=0; i<10; i++){         // wait at least 8us
        
    }
    SPI1BUFL = 0x00;            // Start 2 dummy transmissions
    SPI1BUFL = 0x00;
    while (!SPI1STATLbits.SRMT); // wait for all transmissions to complete
    
    current[0] = SPI1BUFL;
    voltage[0] = SPI1BUFL;
    ADC_CONVST = 0;
    voltage[0] = voltage[0]&0xFFFC; // 14-bit ADC
    
}

void voltage_ramp(unsigned int* current_Vset, unsigned int final_voltage){
    
    unsigned int V_function[14];
    unsigned int temp;
    
    if(final_voltage > current_Vset[0]){
        temp = final_voltage - current_Vset[0];
    }
    else{
        temp = current_Vset[0] - final_voltage;
    }
    
    
    if(final_voltage > current_Vset[0]){
        if(temp > exp_threshold){
            V_function[0] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag));
            V_function[1] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.9)); 
            V_function[2] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.8));
            V_function[3] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.7));
            V_function[4] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.6));
            V_function[5] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.5));
            V_function[6] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.4));
            V_function[7] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.3));
            V_function[8] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.2));
            V_function[9] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.15));
            V_function[10] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.1));
            V_function[11] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.05));
            V_function[12] = (unsigned int)(((float)final_voltage)*(1.0-exp_mag*0.025));
            V_function[13] = final_voltage;
    
            write_voltage(V_function[0]);
            delay_TMR2_mS32(t_V0);
            write_voltage(V_function[1]);
            delay_TMR2_mS32(t_V1);
            write_voltage(V_function[2]);
            delay_TMR2_mS32(t_V2);
            write_voltage(V_function[3]);
            delay_TMR2_mS32(t_V3);
            write_voltage(V_function[4]);
            delay_TMR2_mS32(t_V4);
            write_voltage(V_function[5]);
            delay_TMR2_mS32(t_V5);
            write_voltage(V_function[6]);
            delay_TMR2_mS32(t_V6);
            write_voltage(V_function[7]);
            delay_TMR2_mS32(t_V7);
            write_voltage(V_function[8]);
            delay_TMR2_mS32(t_V8);
            write_voltage(V_function[9]);
            delay_TMR2_mS32(t_V9);
            write_voltage(V_function[10]);
            delay_TMR2_mS32(t_V10);
            write_voltage(V_function[11]);
            delay_TMR2_mS32(t_V11);
            write_voltage(V_function[12]);
            delay_TMR2_mS32(t_V12);
            write_voltage(V_function[13]);
        }
        else{
            write_voltage(final_voltage);
        }
    }
    else{
        if(temp > exp_threshold){
            V_function[0] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag));
            V_function[1] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.9)); 
            V_function[2] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.8));
            V_function[3] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.7));
            V_function[4] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.6));
            V_function[5] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.5));
            V_function[6] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.4));
            V_function[7] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.3));
            V_function[8] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.2));
            V_function[9] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.15));
            V_function[10] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.1));
            V_function[11] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.05));
            V_function[12] = (unsigned int)(((float)final_voltage)*(1.0+exp_mag*0.025));
            V_function[13] = final_voltage;
    
            write_voltage(V_function[0]);
            delay_TMR2_mS32(t_V0);
            write_voltage(V_function[1]);
            delay_TMR2_mS32(t_V1);
            write_voltage(V_function[2]);
            delay_TMR2_mS32(t_V2);
            write_voltage(V_function[3]);
            delay_TMR2_mS32(t_V3);
            write_voltage(V_function[4]);
            delay_TMR2_mS32(t_V4);
            write_voltage(V_function[5]);
            delay_TMR2_mS32(t_V5);
            write_voltage(V_function[6]);
            delay_TMR2_mS32(t_V6);
            write_voltage(V_function[7]);
            delay_TMR2_mS32(t_V7);
            write_voltage(V_function[8]);
            delay_TMR2_mS32(t_V8);
            write_voltage(V_function[9]);
            delay_TMR2_mS32(t_V9);
            write_voltage(V_function[10]);
            delay_TMR2_mS32(t_V10);
            write_voltage(V_function[11]);
            delay_TMR2_mS32(t_V11);
            write_voltage(V_function[12]);
            delay_TMR2_mS32(t_V12);
            write_voltage(V_function[13]);
        }
        else{
            write_voltage(final_voltage);
        }
    }    
}


void set_new_voltage(unsigned int* current_Vset){     // Read a new voltage from the control board and send it to the voltage DAC
    
    unsigned int new_Vset;
    SPI3BUFL = 0x0000;          // Dummy write
    comm_ready = 1;
    while(!SPI3STATLbits.SRMT); // wait for the control board to send data
    
    new_Vset = SPI3BUFL;
    new_Vset = (new_Vset<<3);   
    
    //write_voltage(voltage);
    
    voltage_ramp(current_Vset, new_Vset);
    
    comm_ready = 0;
    
    current_Vset[0] = new_Vset;
}

void set_new_Ilimit(){
    
    unsigned int Ilimit;
    SPI3BUFL = 0x0000;  // Dummy write
    comm_ready = 1;
    while(!SPI3STATLbits.SRMT); // wait for the data to be sent
    
    Ilimit = SPI3BUFL;
    Ilimit = Ilimit<<4;
    
    //SPI3BUFL = 0x0000;
    
    write_Ilimit(Ilimit);
    
    comm_ready = 0;
}

void write_VI(unsigned int *voltage, unsigned int *Iload){
    
    unsigned int    dummy;
    SPI3BUFL = voltage[0];
    SPI3BUFL = Iload[0];
    comm_ready = 1;
    while(!SPI3STATLbits.SRMT); // wait for the control board to read the data from the SPI port
    
    dummy = SPI3BUFL;   // Prevent read buffer overflow
    dummy = SPI3BUFL;
    
    comm_ready = 0;
}

