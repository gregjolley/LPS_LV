#pragma config BWRP = OFF    // Boot Segment Write-Protect bit->Boot Segment may be written
#pragma config BSS = DISABLED    // Boot Segment Code-Protect Level bits->No Protection (other than BWRP)
#pragma config BSEN = OFF    // Boot Segment Control bit->No Boot Segment
#pragma config GWRP = OFF    // General Segment Write-Protect bit->General Segment may be written
#pragma config GSS = DISABLED    // General Segment Code-Protect Level bits->No Protection (other than GWRP)
#pragma config CWRP = OFF    // Configuration Segment Write-Protect bit->Configuration Segment may be written
#pragma config CSS = DISABLED    // Configuration Segment Code-Protect Level bits->No Protection (other than CWRP)
#pragma config AIVTDIS = OFF    // Alternate Interrupt Vector Table bit->Disabled AIVT

// FOSCSEL
#pragma config FNOSC = FRC    // Oscillator Source Selection->Internal Fast RC (FRC)
#pragma config PLLMODE = DISABLED    // PLL Mode Selection->No PLL used; PLLEN bit is not available
#pragma config IESO = ON    // Two-speed Oscillator Start-up Enable bit->Start up device with FRC, then switch to user-selected oscillator source

// FOSC
#pragma config POSCMD = NONE    // Primary Oscillator Mode Select bits->Primary Oscillator disabled
#pragma config OSCIOFCN = ON    // OSC2 Pin Function bit->OSC2 is general purpose digital I/O pin
#pragma config SOSCSEL = OFF    // SOSC Power Selection Configuration bits->Digital (SCLKI) mode
#pragma config PLLSS = PLL_FRC    // PLL Secondary Selection Configuration bit->PLL is fed by the on-chip Fast RC (FRC) oscillator
#pragma config IOL1WAY = OFF    // Peripheral pin select configuration bit->Allow multiple reconfigurations
#pragma config FCKSM = CSDCMD    // Clock Switching Mode bits->Both Clock switching and Fail-safe Clock Monitor are disabled

// FWDT
#pragma config WDTPS = PS32768    // Watchdog Timer Postscaler bits->1:32768
#pragma config FWPSA = PR128    // Watchdog Timer Prescaler bit->1:128
#pragma config FWDTEN = OFF    // Watchdog Timer Enable bits->WDT and SWDTEN disabled
#pragma config WINDIS = OFF    // Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode
#pragma config WDTWIN = WIN25    // Watchdog Timer Window Select bits->WDT Window is 25% of WDT period
#pragma config WDTCMX = WDTCLK    // WDT MUX Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config WDTCLK = LPRC    // WDT Clock Source Select bits->WDT uses LPRC

// FPOR
#pragma config BOREN = ON    // Brown Out Enable bit->Brown Out Enable Bit
#pragma config LPCFG = OFF    // Low power regulator control->No Retention Sleep
#pragma config DNVPEN = ENABLE    // Downside Voltage Protection Enable bit->Downside protection enabled using ZPBOR when BOR is inactive

// FICD
#pragma config ICS = NONE    // ICD Communication Channel Select bits->Reserved, do not use
#pragma config JTAGEN = OFF    // JTAG Enable bit->JTAG is disabled

// FDEVOPT1
#pragma config ALTCMPI = DISABLE    // Alternate Comparator Input Enable bit->C1INC, C2INC, and C3INC are on their standard pin locations
#pragma config TMPRPIN = OFF    // Tamper Pin Enable bit->TMPRN pin function is disabled
#pragma config SOSCHP = ON    // SOSC High Power Enable bit (valid only when SOSCSEL = 1->Enable SOSC high power mode (default)
#pragma config ALTI2C1 = ALTI2CEN    // Alternate I2C pin Location->SDA1 and SCL1 on RB9 and RB8


#include <xc.h>
#include "LPS_LV_def.h"
#include "Delay_TMR2.h"

void pin_configuration(){
    // CF no clock failure; NOSC FRC; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
    __builtin_write_OSCCONL((uint8_t) (0x0000 & 0x00FF));
    // CPDIV 1:1; PLLEN disabled; DOZE 1:1; RCDIV FRC; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x0000;
    // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; STEN disabled; TUN Center frequency; 
    OSCTUN = 0x0000;
    // ROEN disabled; ROSEL FOSC; ROSIDL disabled; ROSWEN disabled; ROOUT disabled; ROSLP disabled; 
    REFOCONL = 0x0000;
    // RODIV 0; 
    REFOCONH = 0x0000;
    // ROTRIM 0; 
    REFOTRIML = 0x0000;
    // DCOTUN 0; 
    DCOTUN = 0x0000;
    // DCOFSEL 8; DCOEN disabled; 
    DCOCON = 0x0700;
    // DIV 0; 
    OSCDIV = 0x0000;
    // TRIM 0; 
    OSCFDIV = 0x0000;
    
        
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x6000;  // ADC and DAC CS RB14, RB13

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0x0000;
    TRISB = 0x01A0;  // RB8 Dout from power board, RB5 ADC Dout

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    IOCPDA = 0x0000;
    IOCPDB = 0x0000;
    IOCPUA = 0x0000;
    IOCPUB = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSA = 0x0000;
    ANSB = 0x0000;


    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

    // ADC
    RPINR20bits.SDI1R = 0x0005;     //RB5->SPI1:SDI1;
    RPOR1bits.RP2R = 0x0008;        //RB2->SPI1:SCK1OUT;
    
    // DAC
    RPOR6bits.RP12R = 0x000B;       //RB12->SPI2:SCK2OUT;
    RPOR5bits.RP11R = 0x000A;       //RB11->SPI2:SDO2;
    
    // Control board communications
    RPOR5bits.RP10R = 0x0017;       //RB10->SPI3:SDO3;
    RPINR28bits.SDI3R = 0x0008;     //RB8->SPI3:SDI3;
    RPINR28bits.SCK3R = 0x0007;     //RB7->SPI3:SCK3IN;

    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
   
}

void SPI_configuration(){
    
    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
    SPI1CON1H = 0x0000;
    // WLENGTH 0; 
    SPI1CON2L = 0x0000;
    // SPIROV disabled; FRMERR disabled; 
    SPI1STATL = 0x0000;
    // SPI1BRGL 2; 
    SPI1BRGL = 0x0002;
    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
    SPI1IMSKL = 0x0000;
    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
    SPI1IMSKH = 0x0000;
    // SPI1URDTL 0; 
    SPI1URDTL = 0x0000;
    // SPI1URDTH 0; 
    SPI1URDTH = 0x0000;
    // SPIEN enabled; DISSDO enabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Master; MODE16 enabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; ENHBUF enabled; DISSDI disabled; 
    SPI1CON1L = 0x1421;
    SPI1CON1Lbits.SPIEN = 1;
    
    
    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Master; MODE16 enabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; ENHBUF enabled; DISSDI enabled; 
    SPI2CON1L = 0x0531;
    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
    SPI2CON1H = 0x0000;
    // WLENGTH 0; 
    SPI2CON2L = 0x0000;
    // SPIROV disabled; FRMERR disabled; 
    SPI2STATL = 0x0000;
    // SPI2BRGL 2; 
    SPI2BRGL = 0x0002;
    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
    SPI2IMSKL = 0x0000;
    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
    SPI2IMSKH = 0x0000;
    // SPI2URDTL 0; 
    SPI2URDTL = 0x0000;
    // SPI2URDTH 0; 
    SPI2URDTH = 0x0000;
    SPI2CON1Lbits.SPIEN = 1;
    
    
    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
    SPI3CON1H = 0x0000;
    // WLENGTH 0; 
    SPI3CON2L = 0x0000;
    // SPIROV disabled; FRMERR disabled; 
    SPI3STATL = 0x0000;
    // SPI3BRGL 0; 
    SPI3BRGL = 0x0000;
    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
    SPI3IMSKL = 0x0000;
    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
    SPI3IMSKH = 0x0000;
    // SPI3URDTL 0; 
    SPI3URDTL = 0x0000;
    // SPI3URDTH 0; 
    SPI3URDTH = 0x0000;
    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Slave; MODE16 enabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; ENHBUF enabled; DISSDI disabled; 
    SPI3CON1L = 0x0401;
    SPI3CON1Lbits.SPIEN = 1;    
}

int main(void) {

    unsigned int voltage, Ilimit, instruction;
    unsigned int read_voltage[1], Iload[1], Vset[1];
    
    pin_configuration();
    ISO_enable = 1;
    SPI_configuration();
    TMR2_conf_delay();
    
    
    voltage = 0x2000;
    Ilimit = 0x1000;
    
    write_voltage(voltage);     
    write_Ilimit(Ilimit); 
    
    delay_TMR2_mS(2000);   
    delay_TMR2_mS(2000);   
        
    load_dummy();   // Prevent TX underrun. There should always be a dummy word in the buffer ready for the control board to exchange with an instruction
    
    comm_ready = 1;
    delay_TMR2_mS(100);   
    comm_ready = 0;
    
    while(1){
        read_IV(read_voltage, Iload);       // Read the ADCs
        
        instruction = decode_request();     // Check if there is an instruction in the SPI buffer
        if(instruction == 1){               // Control board reads the voltage and load current
            write_VI(read_voltage, Iload);       // 
            load_dummy();                   // 
        }
        else if(instruction == 2){          // Set new voltage
            set_new_voltage(Vset);
            load_dummy();
        }
        else if(instruction == 3){          // Set new Ilimit
            set_new_Ilimit();
            load_dummy();
        }
        delay_TMR2_mS(50);   
        
    }
    
    return 0;
}
