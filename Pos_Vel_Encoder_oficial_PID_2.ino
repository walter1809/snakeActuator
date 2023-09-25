#include <util/atomic.h>
#include <AFMotor.h>

#define ENCODER_A         3 
#define ENCODER_B         2 
#define HALL              A1 

////// parametros PID ////////
#define T         0.01  //período de amostragem

//#define K         20  //ganho proporcional
//#define Ti        0.311   //constante de tempo integral
//#define Td        0.047  //constante de tempo derivativo
#define K         0.785  //ganho proporcional
#define Ti        0.311   //constante de tempo integral
#define Td        0.047  //constante de tempo derivativo
#define p         100   //polo em tempo tendendo ao infinito para tornar o derivativo em causal

  //Inicialização das variáveis de PID
  float erro_ant;
  float y;
  float erro;
  float ref;
  float P;
  float I;
  float D;
  float sinal_controle;
  float I_ant;
  float D_ant;
  float u;
  float dif_ant;
  float dif;


const float Tt = sqrt(Ti*Td);

const unsigned int MAX_MESSAGE_LENGTH = 7; // v ou p + até 4 números

//msg de erro
volatile bool err=0;
//incrementa apenas uma vez por comando o valor da variável
volatile bool adicionado_pre = 0;
volatile bool adicionado=0;

//variável global de posição compartilhada com a interrupção 
volatile double theta = 0;
//variável para calcular quantos pulsos houveram a T.
volatile int theta_10ms=0;

//variável global de pulsos compartilhada com a interrupção 
volatile int pulsos = 0;
unsigned long timeold;

//ppr = pulsos por revolução antes da redução no canal A
//ppr_saída = pulsos por revolução após a redução no canal A
//rt = relação de transmissão = 78 
//ppr_saida = ppr*rt = 11*78 = 858;
float resolution = 858;

//Variável global de velocidade
uint8_t  vel = 0;

//Variável global de posicao
float ang = 0;
float lin = 0;

//Variável global de modo (velocidade =false ou posição=true)
bool mode = false;

volatile bool habilita_pid=0; //variável para controlar o cálculo do PID 1 vez a cada período de amostragem
int counter=0; // contador para calcular o períodos de amostragem.

//Criando variável motor
AF_DCMotor motor(4); // define motor on channel 4 with 1KHz default PWM

//fclk = 16Mhz
//Tclk = 0,0000625 ms = 0,0625 us = 62,5  ns
//Ts = 0,1 ms (10 vezes maior que o período medido no osciloscópio com pwm de 200)
//Ts/Tclk = 1600 contagens -> pode ser 25 contagens com precaler de 64.

ISR(TIMER2_COMPA_vect){ // Interrupção a cada 0,1 ms
  TCNT2=0;  // REINICIALIZA COUNTER
  counter++; // contador para contar até 100 ms
  if(counter>=100){
    counter = 0;
    theta_10ms = 0;
    //habilita calculo do PID
    habilita_pid = 1;
  }
}

void setup(){

  //Habilita comunicação serial
  Serial.begin(9600);
  
  //Encoders como entradas
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(HALL, INPUT_PULLUP);
  
  //configura interrupção
  timeold = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A),ler_encoder, RISING);
  
  /////////////Configuracao timer 2 (AMOSTRAGEM)/////////////
  TCCR2A |= 0; //normal
  TCCR2B |= _BV(CS22); //prescaler de 64
  TCNT2=0;
  OCR2A = 25;//64*25=1600 contagens de 62,5 ns
  
  //Habilita Interrupção do Timer2
  TIMSK2 = (1 << OCIE2A);
  
  //Como operar
  Serial.println("Para velocidade digite: 'v NNNN'. ");
  Serial.println("Para posição (ângulo) digite: 'p NNNN'. ");
  Serial.println("Para posição (mm) digite: 'l NNNN'. ");
  Serial.println("NNNN é o valor de PWM para velocidade ou o ângulo/distância desejada para posição. ");
  Serial.println("Para parar o motor digite q ");
  Serial.println("O comando de velocidade implica na perda da referência. ");
  
  
}

void loop(){
  float posicao;
  float pos_lin;
  float rpm;
  uint8_t i;
  char comando = ler_serial()[0];
  int value = value_serial(ler_serial()).toInt();

  if(!digitalRead(HALL)){
    ang = 0;
    lin = 0;
    theta = 0;
  }

  //////////////// VELOCIDADE ////////////////
  if(comando=='v'){  
    mode = false;
    vel = value;
    //Ativa o motor direção Forward com a velocidade
    if(value>=0){
      motor.run(FORWARD);
      motor.setSpeed(vel);
    } else{
      motor.run(BACKWARD);
      motor.setSpeed(-vel);
    }

    //Espera um segundo para o cálculo das RPM
    if (millis() - timeold >= 2000)
    {
      //Modifica as variáveis da interrupção de forma atômica (o código atômico não pode ser interrompido)
      Serial.print("pulsos: ");
      Serial.println(pulsos);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){//Evita interrupção no meio deste cálculo e retoma a interrupção do estado que tinha parado antes deste cálculo
        rpm = float((60.0 * 1000.0 / resolution ) / (millis() - timeold) * pulsos);
        pulsos = 0;
      }
      timeold = millis();
      Serial.print("RPM: ");
      Serial.println(rpm);
      Serial.print("PWM: ");
      Serial.println(vel);
    }
  }
  //////////////////////// POSICAO ANGULAR ////////////////////////
  else if(comando=='p'){//Controle de posição angular
    mode = true;
    //Transforma o valor do potenciômetro em ângulo
    if (!adicionado){
      ang += value;
      lin += value*((3.14*7)/(180*30));
      adicionado_pre = 1;
      adicionado = 1;
    }

    ref = ang;
    //////////////////// PID ///////////////////// 
    if(habilita_pid){
      habilita_pid = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        posicao = (float(theta * 360.0 /resolution)); 
      }
      //a cada 10 ms//
      y = posicao;
      erro = ref-y;
      if(abs(erro)<1){
        erro =0;// travamento devido ao atrito
        erro_ant = 0;
        I = 0;
        I_ant = 0;
        D = 0;
        D_ant = 0;
      }
      P=K*erro;
      I = I_ant + (K*T*(erro+erro_ant)/(2*Ti)) + T*(dif+dif_ant)/(2*Tt);
      D = (D_ant*(2-p*T))/(2+p*T) + ((2*p*K*Td)/(2+p*T))*(erro-erro_ant);
      sinal_controle = P+I+D;
      erro_ant = erro;
      I_ant=I;
      D_ant=D;
      u = saturador(sinal_controle);
      dif_ant = dif;
      dif = u-sinal_controle;
    } 
    ///////////////////////////////////////////// 
    if (millis() - timeold >= 2000)
    {
      timeold = millis();
      if (adicionado_pre == 0){
        adicionado = 0;
      }
      Serial.print("posicao desejada: ");
      Serial.println(ang);
      Serial.print("Posição: ");
      Serial.println(posicao);
    }
  }
  
  //////////////////////// POSICAO LINEAR ////////////////////////
  else if(comando=='l'){//Controle de posição linear
    //após a saída do motor há uma redução de 1:30 para movimentação da coroa -> theta = theta_motor/30
    //o cabo será puxado por um carretel de 7 mm de raio -> S=theta*R
    mode = true;
    
    //Transforma o valor linear em ângulo
    if (!adicionado){
      ang += value*((180*30)/(3.14*7));
      lin += value;
      adicionado_pre = 1;
      adicionado = 1;
    }
    
    ref = ang;
    //////////////////// PID /////////////////////  
    if(habilita_pid){
      habilita_pid = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        posicao = (float(theta * 360.0 /resolution));
      }
      pos_lin = posicao*3.14*7/(180*30);
      
      //a cada 10 ms//
      y = posicao;
      erro = ref-y;
      if(abs(erro)<1){
        erro =0;// travamento devido ao atrito
        erro_ant = 0;
        I = 0;
        I_ant = 0;
        D = 0;
        D_ant = 0;
      }
      P=K*erro;
      I = I_ant + K*T*(erro+erro_ant)/(2*Ti)+ T*(dif+dif_ant)/(2*Tt);
      D = D_ant*(2-p*T)/(2+p*T) + 2*p*K*Td/(2+p*T)*(erro-erro_ant);
      sinal_controle = P+I+D;
      erro_ant = erro;
      I_ant=I;
      D_ant=D;
      u = saturador(sinal_controle);
      dif_ant = dif;
      dif = u-sinal_controle;
    } 
    ///////////////////////////////////////////////   
    //para 70 mm, o motor precisa dar .. 70mm = theta*7mm -> theta == 10rad -> 573º da coroa => 573*30=17.190º do sem fim => 47,75 voltas
    //para 7 mm, o motor precisa dar .. 7mm = theta*7mm -> theta == 10rad -> 57,3º da coroa => 57,3*30=17.19º do sem fim => 4,775 voltas
    
    if (millis() - timeold >= 2000)
    {
      timeold = millis();
      if (adicionado_pre == 0){
        adicionado = 0;
      }
      Serial.print("posicao desejada: ");
      Serial.println(String(lin)+ " mm");
      Serial.print("Posição: ");
      Serial.println(String(pos_lin)+ " mm");
    }
  }
  else {
    mode = true;
    vel = 0;
    motor.setSpeed(vel);
    motor.run(RELEASE); //parados
  }
}


void ler_encoder(){
  //Leitura de velocidade
  if(!mode){
    pulsos++; //Incrementa uma revolução
  }
  //Leitura de posição
  else{
    int b = digitalRead(ENCODER_B);

    if(b>0){
    //Incrementa variável global (sentido anti-horário)
    theta++; // rotação de (1/11)*360º na parte de trás do motor
    theta_10ms++; 
    }
    else{
    //Decrementa variável global
    theta--; // rotação de -(1/11)*360º na parte de trás do motor
    theta_10ms--;
    } 
  }
}

String ler_serial(){
 //Create a place to hold the incoming message
 static char message[MAX_MESSAGE_LENGTH]="n 000"; //valor inicial
 static unsigned int message_pos = 0;
 while (Serial.available() > 0)
 {
   //theta = 0; // resetar theta a cada novo comando?
   adicionado_pre = 0;
   err=0; //resta mensagem de erro
   //Lê o próximo byte disponível no buffer de recebimento do serial
   char inByte = Serial.read();

   //Mensagem chegando (check se não tem o caracter de término) e aguarda até o tamanho total da mensagem
   if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
   {
     //Adiciona o byte que está chegando à mensagem
     message[message_pos] = inByte;
     message_pos++;
   }
   //Mensagem recebida por completo...
   else
   {
     //Adiciona caracter null à string
     message[message_pos] = '\0';

     //Reseta para a próxima mensagem
     message_pos = 0;
   }
 }
 return message;
}

String value_serial(String s){
  String ultimosCaracteres;
  int nUltimosCaracteres = MAX_MESSAGE_LENGTH-2; // Defina o número de caracteres que você deseja pegar do final da string

  // Verifica se a string é maior ou igual ao número de caracteres que queremos pegar
  if (s.length() >= nUltimosCaracteres && !err) {
    ultimosCaracteres = s.substring(s.length() - nUltimosCaracteres);
  } else if(err) {
    ultimosCaracteres = "000";
  } else {
    Serial.println("Velocidade setada para 0.");
    ultimosCaracteres = "000";
    err=1;
  }
  return ultimosCaracteres;
}

float saturador(float sinal_controle){
    float u = sinal_controle;
    if(u>255){
      u=255;
      motor.run(FORWARD); //horario
      motor.setSpeed(u);
    } else if(u>0 && u< 255){
      u=u;
      motor.run(FORWARD); //horario
      motor.setSpeed(u);
    }else if(u<0 && u> -255){
      u=u;
      motor.run(BACKWARD); //anti-horario
      motor.setSpeed(-u);
    } else if(u<-255){
      u=-255;
      motor.run(BACKWARD); //anti-horario
      motor.setSpeed(-u);
    }else {
      motor.run(RELEASE); //para
      motor.setSpeed(u);
    }
    return u;
}
