/*This programme takes analog input from 4 variable resistors (POTs), and two switches, set them up into 10 bits of information and appends an address of 4 bits on the analog values and two bits out the 16 bits are reserved for the switches (i.e 16bits), then transmits them through a 4 bits encoder operating at a frequency of 3khz. 
 *
 *
 * The HT-12E appends an 8 bit address to each 4 bit of information sending out 12bits through the 433mhz fm radio transmitter, at the recieving end, the address is cut off and the 4 bit data is put out on a 4 bit output port.
 *
 *
 *
 *
 * THIS PROGRAM WORKS AT THE TRANSMITTING END 
 *each variable resistor and switch making up 6 devices, the four POTs gets a unique 4 bit address and the information is 10 bits in length,  and two out of 16 bits at all times are reserved for the switch state.
 START CONDITION
if all 16 bits low then, then 16 bits complete information has been sent, and the next 16 bits is about to be sent.
so first, if device is pressed, the change in state of that switch is what assigns an 4 bit address to it, then the new value is repressented in 12 bits of data.
*********Still in development...
************************************************************an 12bit left shift is done on the address which is saved in 16bit format then a bitwise or operation is done between the address and data bits. this is stored in 16 bit array. A ready_to_send function that send 4 low bits 4 times at a specified time interval x_miliseconds (eg 2 miliseconds) is initiated then another send_function sends the 16bits of information at 4 bits packets at time interval of x_miliseconds(eg 2 miliseconds) apart.***************************************/ 


/*
At the receiving end, data is captured in a similar manner with this protocol in mind.
bits of AnalogData     (0,1,2,3)            (4,5)              (6,7,8,9,A,B,C,D,   E,F)
                      Address bits         Switches                 ADCL          ADCH
bit4:5  01 switch one
        10 switch two
bit0:3  0001  A0  address
        0010  A1  address
        0011  A2  address
       0100   A3  address
bit6:F   ADC 10 bit data
 *   
 
 BY NATOR VERINUMBE
 25th/JUNE/2019
 */


//#ifndef F_CPU 1000000UL
//#define F_CPU 1000000UL //internal clock used
//#endif /*F_CPU 1000000UL*/


#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
//#include <avr/interrupt.h>


void ADC_init();
uint16_t ADC_read(uint8_t chnl);
void Send(uint16_t AnalogData);


int main(void)
{

    //DDRC &= ~((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)); //0x00; // Sets portC as inputs or DDRC = 0b0000(*0000); 
    DDRD &= ~((1<<PD0)|(1<<PD1)); //"0b00(11 11)0(*00) set as input pins
    PORTD |= ((1<<PD0)|(1<<PD1)); //set portd0 and D1 as pulled up input

    

    while(1){
        uint16_t Analog_data;
          uint16_t channel=0;
       
        for (channel=0; channel<=3; channel++){//reads    A0 to A3
        
                ADC_init();
                Analog_data &= 0x0000;
                Analog_data = ADC_read(channel);
                Analog_data |= (channel<<12); ;
                _delay_ms(50);


                    if (!(PIND & (1<<0)))
                    {  //appends switches
                        Analog_data |= (1<<10); 
                    }
                    else if (!(PIND & (1<<1)))
                    {
                        Analog_data |=  (1<<11);
                    }


                    Send(Analog_data);
        }

    };

    return 0;
}


void ADC_init(){
    //PRR &= ~(1<<PRADC); // clears the Power Reduction ADC register
   // DIDR0 |=((1<<ADC0D)|(1<<ADC1D)|(1<<ADC2D)|(1<<ADC3D));  //0X00; //sets the Digital Input Disable Register0
    ADCSRA |= ((1<<ADEN)|(1<<ADPS2)|(1<<ADPS1));// 0b10001110 enables ADC & division factor64-125khz 
    ADMUX &= ~((1<<REFS1)|(1<<REFS0)); //AREF, internal Vref turned off 
    ADCSRA |=(1<<ADSC);
};

uint16_t ADC_read(uint8_t chnl){
    //chnl = (chnl>>4); not needed
    ADMUX &= 0xF0; //clear channel bits
    ADMUX |=chnl; // set ADC channel to value of chnl
    ADCSRA |=(1<<ADSC); // Starts A

    while (!((ADCSRA)&(1<<ADSC)));

    ADCSRA|=(1<<ADSC);

    //uint16_t AD_low = ADCL;
    //uint16_t AD_hi = ADCH;
//    uint16_t chnl_16 = chnl;

    //uint16_t AnalogData_ADC_read =((AD_low)|(AD_hi<<8)|(chnl_16<<12));
 
    return (ADC); //AnalogData_ADC_read;
}    



void Send(uint16_t AnalogData){
        DDRD |= ((1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)); //"0b000(1 111)00 set as output pins
        //char Send_data[]=// string value of AnalogData
        uint8_t Send_data[16];
        uint16_t i=0;
        
            for(i=0; i<=15; i++)
            {
                if(AnalogData & (1<<i)){
                    Send_data[i] = 1;
                }else if (!(AnalogData & (1<<i))){
                    Send_data[i] = 0;
                };
            }
 
 //Essentially Send_data[] == AnalogData



        uint8_t k=0, h=0; /*i=0,*/

        


        while (k<=3) {
           // uint8_t Send_4bits;
            uint8_t Send_4bits[4];
            uint8_t Send_8bits=0;
            uint8_t q=0, w=0;
            
            for (q=0; q<=3; q++){
                Send_4bits[q] = Send_data[q+h];
            }

            for (w=2; w<=5; w++){

                if (Send_4bits[w-2]){

                   Send_8bits |= (1<<w);
                  // PORTD = (1<<w);//Send_8bits |= (1<<w); //(Send_4bits[(w-2)]<<w);     //information carried by B0 to B3
                } else{
                    Send_8bits &= ~(1<<w);
                    //PORTD = Send_8bits; //Send_8bits &= ~(1<<w);
                }
            }

            PORTD = Send_8bits;
            _delay_ms(100);   /*-//Send_4bits; PORTD.2 = Send_4bits[0]; PORTD.3 = Send_4bits[1]; PORTD.4 = Send_4bits[2]; PORTD.5 = Send_4bits*/

        h+=4;
        k++;    
        //_delay_ms(45);
        } 

        //_delay_ms(2000);
}
