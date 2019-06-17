/*This programme takes analog input from 4 variable resistors, and two switches, set them up into 12 bits of information and appends an address of 4 bits on them. (i.e 16bits), them transmits them throuch a 4 bits encoder operating at a frequency of 3khz. 
 *
 *
 * The HT-12E appends an 8 bit address to each 4 bit of information sending out 12bits through the 433mhz fm radio transmitter, at the recieving end, the address is cut off and the 4 bit data is put out on a 4 bit output port.
 *
 *
 *
 *
 * THIS PROGRAM WORKS AT THE RECEIVING END 
 *each variable resistor and switch making up 6 devices gets a unique 4 bit address and the information is 12 bits in length.

 START CONDITION
if all 16 bits low then, then 16 bits complete information has been sent, and the next 16 bits is about to be sent.


so first, is device is pressed, the change in state of that switch is what assigns an 4 bit address to it, then the new value is repressented in 12 bits of data.

an 12bit left shift is done on the address which is saved in 16bit format then a bitwise or operation is done between the address and data bits. this is stored in 16 bit array. A ready_to_send function that send 4 low bits 4 times at a specified time interval x_miliseconds (eg 2 miliseconds) is initiated then another send_function sends the 16bits of information at 4 bits packets at time interval of x_miliseconds(eg 2 miliseconds) apart. 




At the receiving end, data is captured in a similar manner with this protocol in mind.
 *   
 */



#define F_CPU 1000000UL

#include <avr/io.h>
#include <stdint.h>

/***ADC Functions***/
void ADC_init();
uint16_t ADC_read(uint8_t chnl);

/********/

/*****Communication to HT-12E*****/
void Send(uint16_t AnalogData);
/**********/


#define _send_delay 45 //45microseconds




int main(void)
{
   
   DDRC=0x00 // Sets portC as inputs or DDRC = 0b00000000;
   DDRD |=0x0F //sets lower nibble of portB as output 
   
    DDRD & = ~((1<<PD0)|(1<<PD1)) //"0b(*00)(11 11)000 set as input pins
    PORTD | = ((1<<PD0)|(1<<PD1)) //set portd0 and D1 as pulled up input

   uint8_t i=6;
   //enum dev={K11=1,K12,RP1=0,RP2,RP3,RP4};
   
   /*
   device address (6 bits)

   RP1=0b000000 
   RP2=0b000100
   RP2=0b001000
   RP2=0b001100
   K11=0b010000
   K12=0b010100
   
  */



    // Insert code
    while(1){
        uint16_t Analog_data;

        for (uint8_t t=0; t<=3; t++){//A0 to A3
            ADC_init();
            Analog_Data = ADC_read(t);
        }

        if (PIND & (1<<0)){
            uint16_t Analog_Data = 0b0100000000000011; 
        }else if (PIND & (1<<1)){
            Analog_Data = 0b0101000000001111;
        }
         Send(Analog_data);
    };

    return 0;
}


void ADC_init(){
    PRR & = ~(1<<PRADC); // clears the Power Reduction ADC register
    DIDR0=0X00; //clears the Digital Input Data Register 0
    ADCSRA |= ((1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1))// 0b10001110 enables ADC,ADCinterrupt & division factor64-125khz 
    ADMUX & = ~((1<<REFS1)|(1<<REFS0)); //AREF, internal Vref turned off 
};

uint16_t ADC_read(uint8_t chnl){
    //chnl = (chnl>>4); not needed
    ADMUX |=chnl;
    ADCSRA |=(1<<ADSC);

    while (!((ADCSRA)&(1<<ADIF)));

    ADCSRA|=(1<<ADIF);

    uint16_t AD_low = ADCL;
    uint16_t AD_hi = ADCH;
    uint16_t chnl_16 = chnl;

    uint16_t AnalogData_ADC_read =((ADCL)|(ADCH)|(chnl_16<<12));
 
    return AnalogData_ADC_read;
}    



bool Send(uint16_t AnalogData){
        DDRD | = ((1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)) //"0b00(11 11)000 set as output pins
        //char Send_data[]=// string value of AnalogData
        uint8_t Send_data[16];
        
            for(int8_t i=0; i<=15; i++)
            {
                if(AnalogData & (1<<i)){
                    Send_data[i] = 1;
                }else if (!(AnalogData & (1<<i)){
                    Send_data[i] = 0;
                }else return 0;
            }




        uint8_t k=i=h=0;

        


        while (k<=3) {
            uint8_t Send_4bits;
            uint8_t Send_4bits[4];
            uint8_t Send_8bits=0;

            for (i=0; i<=3; i++){
                Send_4bits[i] = Send_data[i+h];
            }

            for (i=1; i<=4; i++){
                 Send_8bits |= (Send_4bits[(i-1)]<<i);     //information carried by B1 to B3
            }

            PORTD = Send_8bits;   /*-//Send_4bits; PORTD.2 = Send_4bits[0]; PORTD.3 = Send_4bits[1]; PORTD.4 = Send_4bits[2]; PORTD.5 = Send_4bits*/

        h+=4;
        k++;    
        _delay_ms(_send_delay);
        } 

        //_delay_ms(2000);
}
