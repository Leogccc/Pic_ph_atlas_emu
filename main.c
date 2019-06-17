
#include <16F18326.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h> 
#include<string.h>
#include<ctype.h>

#include "pic_ph_atlas_emu.h"

//------------------Serial data receive interrupt------------------------------------------------- 
/*
#INT_RDA 
void  RDA_isr(void) 
{   
   if(index_in_buffer_uart<=BUF_SIZE) {
        
      
    if ( ( in_buffer[index_in_buffer_uart++]= getc() ) =='\0') 
 
    {   
        index_in_buffer_uart =0; 
        flag_monta_out_buffer=TRUE; 
        
    } 
      
   }

   else index_in_buffer_uart =0; // retorna o index no inicio do vetor (sobreescreve o BUFFER)
           
} 

*/

#INT_SSP // interrupcao I2C1 ( dispara a cada dado recebido ou dado enviado por SSP, chamando SSP_isr_slave )
void SSP_isr_slave(void) // ISR associado a interrupcao INT_SSP 
{
   
   unsigned int8 state;
   state = i2c_isr_state(I2C_PIC_SLAVE);
       
if(state == 0x00 ) /*recebou o endereco do master( bit R/W =0 escrita), slave ira armazenar os dados vindos pelo master*/
    { 
      i2c_read(I2C_PIC_SLAVE); 
      lendo_str_master = TRUE ; // usado para controlar  a flag_monta_out_buffer que é setada dps de um delay do TMR0
      index_in_buffer=0; 
    }
   
if(state == 0x80)  { i2c_read(I2C_PIC_SLAVE,2); index_out_buffer=0 ; }// recebeu o endereco do master (bit R/W =1 leitura), slave deve responder a requisicao de leitura do master; e reinicia o indice do buffer tbm         

if(state >= 0x80) { 
    
    if(index_out_buffer<=BUF_SIZE) {
        
    // slave respondendo a requisicao do master
    i2c_write(I2C_PIC_SLAVE, out_buffer[index_out_buffer] );
    index_out_buffer++ ; 
    
    }
}

   
else{
 if(state > 0x00) { // master escrevendo em slave
    
     if(index_in_buffer<=BUF_SIZE) {
    in_buffer[index_in_buffer] = i2c_read(I2C_PIC_SLAVE) ;// o slave lê só até o byte BUF_SIZE-1 ou até receber um caracter nulo; O slave envia  um nack para o mestre na leitura desse byte(ultimo byte) informando para o mestre gerar um stop no protocolo(parar de enviar dados)) 
    index_in_buffer++ ;
         }  
     }
  }
} // fim isr_slave

void main()
{   

    config_PIC() ;
   
    
    unsigned int1 flag_monta_out_buffer= FALSE ;// flag para montar o vetor out_buffer
    
    while(TRUE)
    { 
     if(in_buffer[0]!='\0'){ // analisa in_buffer apenas se ele for não nulo
         
      // delay de leitura do in_buffer I2C
      if(lendo_str_master) { 
          set_timer0(0);// timer0 comeca a contar do 0 (incrementando a cada 512us; para um delay de 30ms=512us*N_incrementos => N_incrementos=58.59375~ 59)  
          while(get_timer0()<=59){;} // delay de ~30ms
          
          flag_monta_out_buffer= TRUE ;
          in_buffer[BUF_SIZE]='\0'; // adiciona o caracter nulo para formar a string in_buffer
          
          lendo_str_master= FALSE ; 
      } // habilita a montagem do vetor out_buffer quando tiver preenchido o buffer de entrada (recebido todo o comando do mestre )get_timer0() >=59
              
      
      if(flag_monta_out_buffer==TRUE) 
      {
            flag_monta_out_buffer=FALSE ; 
            RESET_out_buffers ; // reseta out_buffer, CMD,CMD2 e VALOR

            int1 parseamento_ok= parsing_in_buffer(in_buffer);

            if(parseamento_ok)
            {
              int8 cmd_identificado = identifica_comando(CMD) ;
              monta_out_buffer(cmd_identificado); 
            }

            // fprintf(UART_PIC,"%s", in_buffer) ;
           RESET_in_buffer ; // limpa o in_buffer (deixa limpo  para um proximo comando)
    
      } // fim if flag_monta_out_buffer  
      
     } //fim de monitora se in_buffer não nulo
    } // fim loop   
} // fim main

void config_PIC(void) 
{   
   
    enable_interrupts(INT_SSP); // habilita a interrupcao de uma atividade em MSSP1 (interrupcao da I2C PIC SLAVE) 
    // enable_interrupts(INT_RDA); // habilita a interrupcao recieve data rs232
    enable_interrupts(GLOBAL); // habilita todas as interrupcoes unmasked, que foram habilitadas anteriormente dessa chamada
    
    i2c_init(I2C_PIC_SLAVE,1); // inicia I2C1 (modo slave) 
    
    i2c_init( MCP3421_STREAM,1); // inicia I2C2 (MCP3421) 
    adc_init(MCP3421_ADDRESS);// envia a configuração inicial para o MCP3421
     
   // setup_uart(TRUE,UART_PIC); // inicia a UART
              
   //setup_dac(DAC_VSS_VDD | DAC_OUTPUT);                // setup conversor digital para analógico (5 bits)
  
   //dac_write(4);//(5/31)*4 V                                    // Write DAC value 0-31 (5 bits)/*           
    
    /*Cálculo de parametros associados a interrupcao de TMR0(por overflow)
    FCLK- frequência do clock que o pic utiliza ; Neste caso FCLK= internal=16MHZ
    Fout? The output frequency after the division. 
    Tout ? The Cycle Time after the division. (periodo da interrupcao)
    4 - The division of the original clock (4 MHz) by 4, when using internal crystal as clock (and not external oscillator). 
    Count - A numeric value to be placed to obtain the desired output frequency - Fout. (fator necessário para ajustar Tout/Fout desejado)
    (256 - TMR0) - The number of times in the timer will count based on the register TMR0. (Ajusta até onde o timer iira contar)
     // 256 usando o timer no modo 8 bits, ou T0_16_BIT
     
     // Fout= FCLK/(4*Prescaler*(2^8-TMR0)*Count); // modo 8 bits
     // Fout= FCLK/(4*Prescaler*(2^16-TMR0)*Count); // modo 16 bits
  
     
     Para:
     TMR0=0; //valor inicial
     FCLK= internal=16MHZ
     tempo_TMR0_incremento=1/(FCLK/(4*Prescaler) ) =>  // tempo_TMR0_incremento= (Prescaler/4)us
     TMR0(max modo 16 bits)=2^16 -1 (16bits) ( 2^16 incrementos para o overflow=>TMR0=0
     
     */
    setup_timer_0(T0_INTERNAL|T0_DIV_2048| T0_16_BIT); //TMR0 incrementa a cada 512us
    
    
TRISA1=0; // configura o pino do led como saída
LATA1=0; // led A1 comeca desligado

   
    RESET_in_buffer ;
}


int1 parsing_in_buffer(char *rcv_buffer) {
   
 unsigned int8 num_separadores=0;
 unsigned int8 index_separador_1 ;
 unsigned int8 index_separador_2 ;
 const char separador=',';
 
 for( int i=0; rcv_buffer[i]!='\0'; i++) { 
     rcv_buffer[i]=toupper(rcv_buffer[i]) ; /*coloca todo os caracteres
 de in_buffer pra maiusculo para que o usuario possa mandar comandos com caracteres minusculos e/ou maiusculos*/ 
    if(rcv_buffer[i]==separador) {
        num_separadores++;  // conta o numero de separadores do comando enviado para o pic
        // guarda o indice de onde ocorrem os separadores
        if(num_separadores==1) index_separador_1=i;
        if(num_separadores==2) index_separador_2=i;
          }     
 }
 

 switch(num_separadores){
     case 0:  // não tem separador=> CMD= in_buffer
           int i ;
           for( i=0; i<=LEN_MAX_CMD; i++) CMD[i]= rcv_buffer[i] ;    
           CMD[i]='\0';
           break;  
     case 1: // só tem um separadores => comando é da forma CMD,VALOR
            {
              int i ;
              for( i=0; rcv_buffer[i]!='\0'; i++) {
                  if(i<index_separador_1) CMD[i]= rcv_buffer[i] ;
                  if(i==index_separador_1) CMD[i]='\0';
                  if(i>index_separador_1) VALOR[i-(index_separador_1 +1)]= rcv_buffer[i] ;
                       }
              VALOR[i-(index_separador_1 +1)]= '\0' ;
              }
         break; 
     case 2: // tem dois  separadores => comando é da forma CMD,CMD2,VALOR
            {
             int i ;
             for( i=0; rcv_buffer[i]!='\0'; i++) {
                         if(i<index_separador_1) CMD[i]= rcv_buffer[i] ;
                         if(i==index_separador_1) CMD[i]='\0';
                         if((i>index_separador_1)&&(i<index_separador_2)) CMD2[i-(index_separador_1 +1)]= rcv_buffer[i];
                         if(i==index_separador_2) CMD2[i-(index_separador_1 +1)]='\0'; 
                         if(i>index_separador_2) VALOR[i-(index_separador_2 +1)]= rcv_buffer[i] ;    
                    }
             VALOR[i-(index_separador_2 +1) ]= '\0' ;
         }
         break;      
     default: // o comando não é de nenhuma forma esperada=> erro de sintaxe
         out_buffer[0]= 2; // 2 sintax error  "Excesso de separador"
         return FALSE ; // erro de sintaxe durante o parseamento (número inesperado de separadores)
         break;
 }
 
 return TRUE ; // caso o parseamento ocorra com sucesso (sem erro de sintaxe em relação a separação (não detecta se os argumentos dos comandos são corretos, somente a forma do comando))
 //#endif

}


int8 identifica_comando( char * comando) {
/*Identiicacao de qual comando foi recebido pelo PIC*/
for( int8 cmd= cmd_err; cmd<=cmd_RT  ; cmd++ ) { // varre do primeiro comando ao ultimo da lista
  if ( strcmp(comando, lista_comandos[cmd] )== 0 ){ return cmd ;   }
}
 // comando invalido, já que o comando não esta na lista_de_comandos  
   return cmd_Err ;   
 }


void monta_out_buffer( int8 num_comando) {
 
  // Os comandos podem ser(ou qualquer outro acrescentado em lista_comandos[] e em enum comandos): cmd_err, cmd_Baud ,cmd_Cal, cmd_Export, cmd_Factory, cmd_Find, cmd_i, cmd_I2c, cmd_Import, cmd_L, cmd_Plock, cmd_R, cmd_Sleep, cmd_Slope,cmd_Status, cmd_T, cmd_RT
   switch(num_comando) {
     
       case cmd_Baud:
           
           break;
           
       case cmd_Cal:
             
           break;
           
       case cmd_R:
                  ANPH_R();// retorna uma única leitura do valor de ph (%.2f) 
           break;   
       case cmd_T:
                  ANPH_T(); // Define a compensacao de temperatura
           break ;
           
         case cmd_RT:
                 ANPH_RT(); // Faz R e T simultaneamente
           break;
        case cmd_Find:
                  ANPH_FIND(); //Find: LED rapidly blinks white, used to help find device
           break ; 
       case cmd_L:
                  ANPH_L(); // LED CONTROL
           break;
       case cmd_err: 
                    out_buffer[0]= 2; // 2 sintax error "Comando invalido
           break;
   }       
       
}


// Implementações de cada comando
int1 isStr_float(char *str_teste)  {
    
    int8 num_ponto=0;
    int8 num_sinal_menos=0;
    
// retorna TRUE se a str for uma string float válida(número ponto flutuante positivo ou negativo Ex: 323.124, -2.32, 0.533, .32 ,-.32) ou FALSE caso contrário
    for(int i=0; str_teste[i]!='\0'; i++){

        if(i==0){ // o primeiro caracter pode ser digito(0-9) ou '-'
            if (isdigit(str_teste[0])||(str_teste[0]=='-') )  {
                 if(str_teste[0]=='-') num_sinal_menos++;
                 continue; 
            }
            else return FALSE; //caracter não válido
        }
        else{
            if ((isdigit(str_teste[i])==TRUE)||(str_teste[i]=='.')||(str_teste[i]=='-') ) {
           
                     if(str_teste[i]=='.') {num_ponto++; if(num_ponto>1) return FALSE; } // só pode ter um ponto
                     if(str_teste[i]=='-') return FALSE; //o sinal de menos só pode ocorrer se i=0
                     if(str_teste[strlen(str_teste)-1]=='.') return FALSE; // o ultimo caracter não nulo não pode ser '.'
                     if (isdigit(str_teste[i]) ) continue ; 
                  }    
            else return FALSE; //caracter não válido
        }
    }
 return TRUE;      
}

float32 get_ph_value(float32 temp_C) {
          
            float32 mcp_value_mV = read_adc_volts_mcp3421(MCP3421_ADDRESS)*1000 ;
            float32 valor_ph = (-0.01672*mcp_value_mV) + 6.804; // curva do sensor de ph
            valor_ph = (((7-valor_ph)*temp_C)/300) - ((7-valor_ph)/12) + valor_ph; // compensacao de temperatura
            //valor_ph = fator_k*valor_ph + fator_b; // ajuste do valor_ph usando os fatores de calibração k e beta (calibração feita pelo usuário)
            return valor_ph ;
}

void ANPH_R(void){
        if( (strcmp(CMD,in_buffer)==0)&&(CMD2[0]=='\0')&&(VALOR[0]=='\0')) { // comando passado é da forma R
        sprintf(out_buffer,"%.2f",read_adc_volts_mcp3421(MCP3421_ADDRESS)*1000 /*get_ph_value(temp_solucao)*/) ;
        }
        else out_buffer[0]= 2; // 2 sintax error
}

void ANPH_T(void){
    
    if( (CMD2[0]=='\0')&&(VALOR[0]!='\0') ) { // comando passado é da forma T,VALOR
        
        if(isStr_float(VALOR)) 
             {
              temp_solucao= atof(VALOR) ; sprintf(out_buffer,"OK") ; 
             }
        else out_buffer[0]= 2; // 2 sintax errorfloat invalido
    }
    else out_buffer[0]= 2; // 2 sintax error
}

void ANPH_RT(void) {
    
    if( (CMD2[0]=='\0')&&(VALOR[0]!='\0') ) {
             if( isStr_float(VALOR) ) 
             {
              temp_solucao= atof(VALOR) ;
              sprintf(out_buffer,"%.2f", get_ph_value(temp_solucao) ) ;
             }
             else out_buffer[0]= 2;  // ErroSintaxe: float invalido  
    }
    else out_buffer[0]= 2; // 2 sintax error ; O comando passado não é da forma RT,VALOR
}

void ANPH_FIND(void){
    
    if( (strcmp(CMD,in_buffer)==0)&&(CMD2[0]=='\0')&&(VALOR[0]=='\0')) { // comando passado é da forma FIND
        
        //Response: DEC NULL
        //           1  0
        out_buffer[0]=1;
        in_buffer[0]=0; //sentinela do comando FIND
        
        while(in_buffer[0]==0){// fica piscando o led até o usuário enviar um caracter(in_buffer[0]=!0)
            LATA1=!LATA1 ;
            delay_ms(100);
            LATA1=!LATA1 ;
            delay_ms(100);
        }
        
        
        }
    
    else out_buffer[0]= 2; // 2 sintax error  
}

void ANPH_L(void){
/* Comando sintaxe
 L,1 // LED on ; Response: 1(DEC) 0(NULL)
 L,0  //  LED off ; Response: 1(DEC) 0(NULL)
 L,? // LED state on/off? ; Response: 1(DEC) ?L,1 0(NULL) ou 1(DEC) ?L,0 0(NULL)
 */  
 if( (CMD2[0]=='\0')&&(VALOR[0]!='\0') ){
 
 

     
     
 }
 
 else out_buffer[0]= 2; // 2 sintax error ; O comando passado não é da forma RT,VALOR
}


// Funções de comunicação com o MCP3421
#ifndef debug 

//////////////////////////////////////////////////////////////////////////////////
// adc_init()
// Purpose: To initialize the MCP3421.
// Parameters: address - Optional parameter for specifying the address of the
//                       MCP3421 to initialize.  Allows for initializing multiple
//                       devices on same bus.  Driver only supports one device
//                       configuration.  Defaults to MCP3421_ADDRESS if not
//                       specified.
// Returns:    Nothing.
//////////////////////////////////////////////////////////////////////////////////

void adc_init(unsigned int8 address=MCP3421_ADDRESS)
{
  i2c_start(MCP3421_STREAM);  //send I2C start
  i2c_write(MCP3421_STREAM, MCP3421_DEVICE_CODE | (address << 1));  //send write command
  i2c_write(MCP3421_STREAM, MCP3421_MODE | MCP3421_BITS | MCP3421_GAIN);  //send device configuration
  i2c_stop(MCP3421_STREAM);  //send I2C stop
}

//////////////////////////////////////////////////////////////////////////////////
// read_adc_mcp3421()
// Purpose: To read the last adc conversion from device, raw value read from
//          device. If device configured for One-Shot mode, it will initiate the
//          conversion.  Function will wait for a new conversion before returning.
// Parameter: address - Optional parameter for specifying the address of the
//                      MCP3421 to read.  Allows for reading multiple devices on
//                      same bus.  Defaults to MCP3421_ADDRESS if not specified.
// Returns:   signed int32 or signed int16 value depending MCP3421_BITS value.
//////////////////////////////////////////////////////////////////////////////////
#if MCP3421_BITS == MCP3421_18BITS
signed int32 read_adc_mcp3421(unsigned int8 address=MCP3421_ADDRESS)
#else
signed int16 read_adc_mcp3421(unsigned int8 address=MCP3421_ADDRESS)
#endif
{
  union
  {
   #if MCP3421_BITS == MCP3421_18BITS
    signed int32 sint32;
    unsigned int8 b[4];
   #else
    signed int16 sint16;
    unsigned int8 b[2];
   #endif
  } result;
  unsigned int8 status = 0x80;

  #if MCP3421_MODE == MCP3421_ONE_SHOT
   i2c_start(MCP3421_STREAM);  //send I2C start
   i2c_write(MCP3421_STREAM, MCP3421_DEVICE_CODE | (address << 1));  //send write command
   i2c_write(MCP3421_STREAM, MCP3421_START_CONVERSTION | MCP3421_MODE | MCP3421_BITS | MCP3421_GAIN);  //initiate conversion
   i2c_stop(MCP3421_STREAM);  //send I2C stop
  #endif

   i2c_start(MCP3421_STREAM);  //send I2C start
   i2c_write(MCP3421_STREAM, MCP3421_DEVICE_CODE | (address << 1) | 1);  //send read command

  #if MCP3421_BITS == MCP3421_18BITS
   result.b[2] = i2c_read(MCP3421_STREAM, 1);  //read MSB 18 Bit mode
  #endif
   result.b[1] = i2c_read(MCP3421_STREAM, 1);  //read 2nd MSB 18 Bit mode, read MSB 16, 14 or 12 Bit mode
   result.b[0] = i2c_read(MCP3421_STREAM, 1);  //read LSB
   status = i2c_read(MCP3421_STREAM, 1);       //read Status

   if(bit_test(status,7))  //if RDY = 1, New conversion not ready
   {
     do
     {
       status = i2c_read(MCP3421_STREAM, 1);  //read Status
     } while(bit_test(status, 7)); //until RDY = 0

     status = i2c_read(MCP3421_STREAM, 0);  //read Status, do nack
     i2c_stop();  //send I2C stop

     i2c_start(MCP3421_STREAM);  //send I2C start
     i2c_write(MCP3421_STREAM, MCP3421_DEVICE_CODE | (address << 1) | 1);  //send read command

    #if MCP3421_BITS == MCP3421_18BITS
     result.b[2] = i2c_read(MCP3421_STREAM, 1);  //read MSB 18 Bit mode
    #endif
     result.b[1] = i2c_read(MCP3421_STREAM, 1);  //read 2nd MSB 18 Bit mode, read MSB 16, 14 or 12 Bit mode
     result.b[0] = i2c_read(MCP3421_STREAM, 1);  //read LSB
   }

   status = i2c_read(MCP3421_STREAM, 0);  //read Status, do nack
   i2c_stop();  //send I2C stop

  #if MCP3421_BITS == MCP3421_18BITS
   if(bit_test(result.b[2],1))  //if 18 Bit mode check sign bit
     result.b[3] = 0xFF;
   else
     result.b[3] = 0;
  #endif

  #if MCP3421_BITS == MCP3421_18BITS
   return(result.sint32);
  #else
   return(result.sint16);
  #endif
}

//////////////////////////////////////////////////////////////////////////////////
// read_adc_volts_mcp3421()
// Purpose: To read the last adc conversion from device, actual volt value read
//          from device.
// Parameter: address - Optional parameter for specifying the address of the
//                      MCP3421 to read.  Allows for reading multiple devices on
//                      same bus.  Defaults to MCP3421_ADDRESS if not specified.
// Returns:   float32
//////////////////////////////////////////////////////////////////////////////////
float32 read_adc_volts_mcp3421(unsigned int8 address=MCP3421_ADDRESS)
{
  #if MCP3421_BITS == MCP3421_18BITS
   signed int32 result;
  #else
   signed int16 result;
  #endif

  float32 fresult;

  result = read_adc_mcp3421(address);

  #if MCP3421_BITS == MCP3421_12BITS
   fresult = (float32)result * 0.001;
  #elif MCP3421_BITS == MCP3421_14BITS
   fresult = (float32)result * 0.00025;
  #elif MCP3421_BITS == MCP3421_16BITS
   fresult = (float32)result * 0.0000625;
  #else
   fresult = (float32)result * 0.000015625;
  #endif

  #if MCP3421_GAIN == MCP3421_8X_GAIN
   fresult /= 8;
  #elif MCP3421_GAIN == MCP3421_4X_GAIN
   fresult /= 4;
  #elif MCP3421_gain == MCP3421_2X_GAIN
   fresult /= 2;
  #endif

   return(fresult);
}


#endif
