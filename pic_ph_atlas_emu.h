/* 
 * File:   pic_ph.h
 * Author: Leonardo
 *
 * Created on 8 de Maio de 2019, 22:56
 */

#ifndef PIC_PH_H_ATLAS
#define	PIC_PH_H_ATLAS

// #define debug /*depuracao*/

#use delay(internal=16MHZ) // CPU rodando em 16MHZ (clock interno)


// configuracoes de hardware do PIC (fuses)
 #fuses RSTOSC_HFINTRC, WRT, PROTECT, CPD, NOLVP, NOWDT  

/*
 Def para acesso dos registradores (ambiente CCS) relacionado aos I/O do PORTA
 */
#byte TRISA= getenv("SFR:TRISA") 
#bit TRISA1= TRISA.1 

#byte PORTA= getenv("SFR:PORTA") 
#bit  PORTA1= PORTA.1 

#byte LATA=getenv("SFR:LATA") 
#bit  LATA1= LATA.1 




// Descrição fuses do PIC16F18326 que estão configurados 
/* 
RSTOSC_HFINTRC   // On Power-up clock running from HFINTRC
PUT             // Power Up Timer
LPBOR           // Low-Power Brownout reset is enabled
WRT            //  Program Memory Write Protected
PROTECT        //  Code protected from reads
CPD             // Data EEPROM Code Protected
NOLVP           // No low voltage programing, B3(PIC16) or B5(PIC18) used for I/O
WDT           //   Watch Dog Timer
*/ 

#define BUF_SIZE 40 // tamanho dos Buffers do I2C1
#define PIC_ADDRESS 0xA0 // endereco da I2C1 do pic (modo slave) (em 8 bits=> o valor verdadeiro do endereco vai ser PIC_ADDRESS>>1 (endereco em 7 bits)
#define LED_PIN PIN_A1 // pino onde está conectado o LED pino 12 RA1
#define LED_STATUS  PIN_A1
#define TAM_MAX  25

#define LEN_MAX_CMD 7 // o numero maximo de caracteres dos comandos da lista_comandos  ( não contando '\0') para comandos da forma : "CMD"
#define LEN_MAX_CMD2 5  // tamanho max do cmd2 ( não contando '\0') para um comando da forma CMD,CMD2,VALOR
#define LEN_MAX_VALOR 28 // tamanho max de  valor (não contando '\0') para comandos da forma CMD,VALOR ouu CMD,CMD2,VALOR pode variar, mas não passara de 20

// Macros de reset dos buffers do I2C1 (buffers não físicos)
#define RESET_out_buffer  for(int i = 0; i <= BUF_SIZE; i++) out_buffer[i] =0;  // reseta o buffer de entrada do I2C1
#define RESET_in_buffer   for(int i = 0; i <= BUF_SIZE; i++) in_buffer[i] =0;  // reseta o buffer de saida do I2C1
#define RESET_out_buffers for(int i = 0; i <= BUF_SIZE; i++) { out_buffer[i] =0; if(i<=LEN_MAX_CMD)CMD[i] =0; if(i<=LEN_MAX_CMD2) CMD2[i] =0; if(i<=LEN_MAX_VALOR)VALOR[i]=0;} // reseta os buffers de entrada e saída do PIC (modo Slave)


// associando os pinos do pic com a funcao que ele desempenha ( #pin_select) para uso da I2C1
// PPS do PIC
 #pin_select SCL1IN=PIN_C0 
 #pin_select SCL1OUT=PIN_C0 // Modo slave não controla clock
 #pin_select SDA1IN=PIN_C1
 #pin_select SDA1OUT=PIN_C1
/*configura a I2C1(MSSP1) para o modo slave, com endereco PIC_ADDRESS,  usando hardware I2C 
 functions(FORCE_HW). Uma ID= I2C_PIC_SLAVE  representa o canal(stream) i2c criado 
 A i2c é iniciado com i2c_init() (NOINIT) , */ 
#use i2c(SLAVE, I2C1, address= PIC_ADDRESS , stream= I2C_PIC_SLAVE, NOINIT,FORCE_HW) 

/*
#pin_select U1TX= PIN_A4 // PIN_C1 SDA/TX
#pin_select U1RX= PIN_A5 // PIN_C0 SCL/RX
#use rs232(baud=9600, xmit = PIN_A4 , rcv = PIN_A5, bits=8 , stream= UART_PIC ,NOINIT)

*/

// C5 SDA MCP
// C4 SCL MCP

//unsigned int8 tamanho = 0;
char in_buffer[BUF_SIZE+1]; // buffer de recepcao de dados vindos do Master para o PIC
char out_buffer[BUF_SIZE+1]; // buffer de saida dados do slave(PIC) para o master

char CMD[LEN_MAX_CMD +1]={};
char CMD2[LEN_MAX_CMD2+1]={};
char VALOR[LEN_MAX_VALOR+1]={} ;


// unsigned int8  index_in_buffer_uart =0; 

unsigned int1 lendo_str_master = FALSE ; // indica se o PIC está preenchendo in_buffer (I2C)
unsigned int8  index_out_buffer;
unsigned int8  index_in_buffer ;

typedef enum {cmd_err, cmd_Baud ,cmd_Cal, cmd_Export, cmd_Factory, cmd_Find, cmd_i, cmd_I2c, cmd_Import, cmd_L, cmd_Plock, cmd_R, cmd_Sleep, cmd_Slope,cmd_Status, cmd_T, cmd_RT} comandos;
char lista_comandos[17][LEN_MAX_CMD +1]= {"ERR","BAUD","CAL","EXPORT","FACTORY","FIND","I","I2C","IMPORT","L","PLOCK","R","SLEEP","SLOPE","STATUS","T","RT"} ;
/* variaveis PH */

void config_PIC(void) ;

/*
 Faz o parseamento de in_buffer, conforme o protocolo definido abaixo:
 in_buffer pode ser da forma "CMD", "CMD,VALOR" ou "CMD,CMD2,VALOR"
 Monta as string CMD(comando), CMD2(comando secundário (nem todos os comandos tem um CMD2) ) e VALOR( parâmetro de algum comando)
 Obs: o separador padrão é ',' mas pode ser alterado para separador=%c
 retorna TRUE se o parseamento foi correto (nº esperado de separadores)
 retorna FALSE se o parseamento foi inválido(erro de sintaxe) (nº inesperado de separadores)
 */
int1 parsing_in_buffer(char *rcv_buffer) ; 

/* Retorna qual  comando(CMD) foi enviado pelo usuário
 O retorno é um valor de enum comandos
 */
int8 identifica_comando( char * comando) ;

void monta_out_buffer( int8 num_comando ) ; // monta o vetor out_buffer de resposta as requisicoes do mestre


/*
 Variáveis e definições ANPH 
 */

//fator_k= ;
//fator_b= ;
 float32 temp_solucao= 25.0; // armazena a temperatura em ºC para compensacao de temperatura na leitura do ph ( Usado nos comandos T,RT e R) (padrão 25ºC (sem compensacao) )

 /*
 Protótipos das funções do ANPH
 */

/*
   retorna o valor ph com ou sem compensacao de temperatura  
 comp_temp=TRUE => retorna o valor ph com compensacao de temperatura (temp_C é a temperatura da solucao)
 comp_temp=FALSE => retorna o valor ph sem compensacao de temperatura (temp_C= 25ºC)
     
Default temperature = 25°C
Temperature is always in Celsius
Temperature is not retained if power is cut
 */
int1 isStr_float(char *str_teste) ;// verifica se uma string representa um float válido
float32 get_ph_value(float32 temp_C) ;


/*Retorna uma única Leitura*/
void ANPH_R(void) ; // retorna uma única leitura do valor de ph (%.2f) 
void ANPH_T(void); // Define a compensacao de temperatura
void ANPH_RT(void) ; // /*Retorna uma única Leitura e define a compensacao de temperatura (Faz R e T em um mesmo comando)
void ANPH_FIND(void); //Find: LED rapidly blinks white, used to help find device
void ANPH_L(void); // LED CONTROL
/* */



#ifndef debug

//////////////////// Driver for MCP3421 A/D Converter ///////////////////
////                                                                 ////
////  adc_init() - initializes the MCP3421, call after power up      ////
////                                                                 ////
////  read_adc_mcp3421() - initiates a conversion and reads adc,     ////
////                       returns raw reading.                      ////
////                                                                 ////
////  read_adc_volts_mcp3421() - initiates a conversion and read     ////
////                             adc, returns voltage as float       ////
////                             (-2.048 to 2.048).                  ////
////                                                                 ////
////  Defines used to setup MCP3421:                                 ////
////                                                                 ////
////    MCP3421_SCL  - specifies the I2C clock pin to use, default   ////
////                   if not specified is PIN_C3.                   ////
////                                                                 ////
////    MCP3421_SDA  - specifies the I2C data pin to use, default if ////
////                   not specified is PIN_C4.                      ////
////                                                                 ////
////    MCP3421_MODE - specifies the mode to use, default if not     ////
////                   specified is MCP3421_CONTINUOUS.  Valid       ////
////                   options are:                                  ////
////                     MCP3421_CONTINUOUS                          ////
////                     MCP3421_ONE_SHOT                            ////
////                                                                 ////
////    MCP3421_BITS - specifies the number of conversion bits to    ////
////                   use, default if not specified is              ////
////                   MCP3421_18BITS.  Valid options are:           ////
////                     MCP3421_18BITS                              ////
////                     MCP3421_16BITS                              ////
////                     MCP3421_14BITS                              ////
////                     MCP3421_12BITS                              ////
////                                                                 ////
////    MCP3421_GAIN - specifies the PGA gain to use, default if not ////
////                   specified is MCP3421_1X_GAIN.  Valid options  ////
////                   are:                                          ////
////                     MCP3421_8X_GAIN                             ////
////                     MCP3421_4X_GAIN                             ////
////                     MCP3421_2X_GAIN                             ////
////                     MCP3421_1X_GAIN                             ////
////                                                                 ////
////    MCP3421_ADDRESS - specifies the address of MCP3421 devices,  ////
////                      default if not specified is 0.  Valid      ////
////                      options are 0-7.                           ////
////                                                                 ////
////    USE_HW_I2C - specifies that the MCP3421_SCL and MCP3421_SDA  ////
////                 pins are HW I2C pins and to use PIC's HW I2C    ////
////                 peripheral.                                     ////
////                                                                 ////
/////////////////////////////////////////////////////////////////////////
////        (C) Copyright 1996,2013 Custom Computer Services         ////
//// This source code may only be used by licensed users of the CCS  ////
//// C compiler.  This source code may only be distributed to other  ////
//// licensed users of the CCS C compiler.  No other use,            ////
//// reproduction or distribution is permitted without written       ////
//// permission.  Derivative programs created using this software    ////
//// in object code form are not restricted in any way.              ////
/////////////////////////////////////////////////////////////////////////

#define MCP3421_CONTINUOUS 0x10
#define MCP3421_ONE_SHOT   0x00

#define MCP3421_18BITS     0x0C
#define MCP3421_16BITS     0x08
#define MCP3421_14BITS     0x04
#define MCP3421_12BITS     0x00

#define MCP3421_8X_GAIN    0x03
#define MCP3421_4X_GAIN    0x02
#define MCP3421_2X_GAIN    0x01
#define MCP3421_1X_GAIN    0x00

#define MCP3421_DEVICE_CODE       0xD0
#define MCP3421_START_CONVERSTION 0x80

#ifndef MCP3421_SCL
 #define MCP3421_SCL  PIN_C4
#endif

#ifndef MCP3421_SDA
 #define MCP3421_SDA  PIN_C5
#endif


#ifndef MCP3421_MODE
 #define MCP3421_MODE /*MCP3421_ONE_SHOT*/ MCP3421_CONTINUOUS
#endif

#ifndef MCP3421_BITS
 #define MCP3421_BITS MCP3421_18BITS
#endif

#ifndef MCP3421_GAIN
 #define MCP3421_GAIN MCP3421_1X_GAIN
#endif

#ifndef MCP3421_ADDRESS
 #define MCP3421_ADDRESS 1 // endereco MCP3421 0x69 => 0xD0|0x01<<1  ; 0x69 => 0xD0|0x01<<1 
#endif


 #pin_select SCL2IN = MCP3421_SCL 
 #pin_select SCL2OUT = MCP3421_SCL
 #pin_select SDA2IN = MCP3421_SDA
 #pin_select SDA2OUT = MCP3421_SDA


#use i2c(MASTER,FAST,SCL=MCP3421_SCL, SDA=MCP3421_SDA, stream= MCP3421_STREAM,NOINIT)


/* Protótipos*/
void adc_init(unsigned int8 address=MCP3421_ADDRESS) ;

#if MCP3421_BITS == MCP3421_18BITS
signed int32 read_adc_mcp3421(unsigned int8 address=MCP3421_ADDRESS) ;
#else
signed int16 read_adc_mcp3421(unsigned int8 address=MCP3421_ADDRESS) ;
#endif

float32 read_adc_volts_mcp3421(unsigned int8 address=MCP3421_ADDRESS) ;

#endif // debug 


#endif	/* PIC_PH_H_ATLAS */



