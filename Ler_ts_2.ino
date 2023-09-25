#define ENCODER_A         2 
#define TOTAL_AMOSTRAS  1600
#define AMOSTRAS_DELAY  0

const int amostras_efetivas = TOTAL_AMOSTRAS - AMOSTRAS_DELAY; 

volatile bool encA[amostras_efetivas];
volatile int amostra_number=0;
volatile bool valor_passado;
volatile bool habilita=0;
volatile bool printa=0;
bool para_de_printar=0;

//fclk = 16Mhz
//Tclk = 0,0000625 ms = 0,0625 us = 62,5  ns
//Ts = 0,1 ms (10 vezes maior que o período medido no osciloscópio com pwm de 200)
//Ts/Tclk = 1600 contagens -> pode ser 25 contagens com precaler de 64.

ISR(TIMER2_COMPA_vect){ // A cada 0,1ms
  TCNT2=0;  // REINICIALIZA COUNTER
  if(!habilita){
    encA[amostra_number] = digitalRead(ENCODER_A);
    if(valor_passado == encA[amostra_number]){ //Se ainda não saiu do estado inicial ...
      valor_passado = encA[amostra_number];
    } else { //Se saiu do estado inicial 
      amostra_number++; 
      habilita = 1;
    }      
  } 
  ////////////// COM DELAY /////////////////// 
  if(habilita){// A partir daqui começa a amostrar
    if(amostra_number<AMOSTRAS_DELAY){ //Se o tempo da amostra é menor que o tempo de iniciar a amostragem ...
      amostra_number++;     
    }
    else{ //Se o tempo da amostra é maior que o tempo de iniciar a amostragem ...
      if(amostra_number < TOTAL_AMOSTRAS ){
      encA[amostra_number-AMOSTRAS_DELAY] = digitalRead(ENCODER_A);
      amostra_number++;         
      }else{ //Se já terminou o período de amostragem, pode printar na tela.
        printa = 1;
      }
    }
  }
  ////////////// SEM DELAY ///////////////////
  /*if(habilita){
    if(amostra_number < TOTAL_AMOSTRAS ){
      TCNT2=0;  // REINICIALIZA COUNTER
      encA[amostra_number] = digitalRead(ENCODER_A);
      amostra_number++;         
    }else{ //Se já terminou o período de amostragem, pode printar na tela.
      printa = 1;
    }
  }*/
  
}//FIM DA ISR

void setup(){
  
  encA[0]=0;
  Serial.begin(9600);
  //Encoders como entradas
  pinMode(ENCODER_A, INPUT);

  valor_passado = digitalRead(ENCODER_A);

  TCCR2A |= 0; //normal
  TCCR2B |= _BV(CS22); //prescaler de 64
  TCNT2=0;
  OCR2A = 25;//64*25=1600
  
  //Habilita Interrupção do Timer2
  TIMSK2 = (1 << OCIE2A); //ESSA LINHA IMPEDE DE ACIONAR O MOTOR
}

void loop(){

  if(printa && !para_de_printar){
    for(int i=0;i<amostras_efetivas;i++){
      Serial.println(encA[i]);
    }
    para_de_printar = 1; 
  }
    
}
