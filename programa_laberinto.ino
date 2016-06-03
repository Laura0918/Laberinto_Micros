#include <QTRSensors.h>


int boton = 8;
int motor_derecha_ad = 9;   
int motor_izquierda_ad = 11; 
int motor_derecha_at = 10;    
int motor_izquierda_at = 12;  
int pwm_iz = 5;             
int pwm_der = 6;             
int ledOn = 4;
int ledPlay = 2;

// Pines analogos donde se conectan los sensores

//definen la posicion
int sensor1 = 0;
int sensor2 = 1;
int sensor3 = 2;
int sensor4 = 3;

// definen cambio de celda
int sensor_cd = 4;
int sensor_ci = 5;



// Variables iniciadas en cero
float proporcional = 0;
float proporcional_pasado = 0;
int cuadricula = 0;
int posicion = 0;
int ultima_posicion = 0;
float integral = 0;
float derivativo = 0;
float valor_pwm = 0;

// Constantes del PID
float kp=0.2; 
float kd=12;
float ki=0.1;

int valorCel = 6;     //valor inicial para la celda
int celda[6][6];      // matriz de posiciones de las celdas

// variables que definen la posicion de una celda
int i = 0;
int i1 = 0;

// variables que definen la orientacion del robot 
boolean norte = false;
boolean sur = false;
boolean este = true;
boolean oeste = false;




int posi(){  // funcion que calcula y retorna la posicion
  
  int s1 = analogRead(sensor1);
  int s2 = analogRead(sensor2);
  int s3 = analogRead(sensor3);
  int s4 = analogRead(sensor4);

  /*Serial.println(s2);
  delay(100);
  Serial.print(',');
  Serial.println(s2);*/
 

int s11 = 0; // Variables usadas para la conversion
int s12 = 0;
int s13 = 0;
int s14 = 0;




// Calibracion de sensores para convertirlos a binario
  if(s1<200){
  s11=1;
  }
  if(s2<300){
  s12=1;
  }
  if(s3<300){
  s13=1;
  }
  if(s4<200){
  s14=1;
  }

    
posicion = (1*s11)+(2*s12)+(4*s13)+(8*s14); 

return posicion;
}

int cuad(){
  
int s5 = analogRead(sensor_cd);  // sensores encargados de detectar cambio de celda
int s6 = analogRead(sensor_ci);

int s15 = 0;
int s16 = 0;
/*delay(500);
Serial.print(s5);
Serial.print(',');
Serial.println(s6);*/

// condiciones para cambiar las lecturas de analogas a digitales
  if(s5<200){
  s15=1;
  }
  if(s6<200){
  s16=1;
  }
int cuadricula = s15 + s16;

return cuadricula;
}


void setup() {
  
  delay(2000);   // Hacemos una espera de 2 segundos
  Serial.begin(9600);  // iniciamos comunicacion serial con 9600 bouds para enviar datos a processing

  // Configuramos los pines como entrada o salida
  pinMode(boton,INPUT_PULLUP);
  pinMode(motor_derecha_ad,OUTPUT);
  pinMode(motor_izquierda_ad,OUTPUT);
  pinMode(motor_derecha_at,OUTPUT);
  pinMode(motor_izquierda_at,OUTPUT);
  pinMode(sensor3,INPUT);
  pinMode(ledOn,OUTPUT);
  pinMode(ledPlay,OUTPUT);
  pinMode(pwm_iz,OUTPUT);
  pinMode(pwm_der,OUTPUT);

  // ponemos a girar ambos motores hacia adelante por dos segundos para verificar su correcta conexion
  digitalWrite(motor_derecha_ad,HIGH);
    digitalWrite(motor_derecha_at,LOW);
    analogWrite(pwm_der,85);
    digitalWrite(motor_izquierda_at,HIGH);
    digitalWrite(motor_izquierda_ad,LOW);
    analogWrite(pwm_iz,85);
    delay(2000);
      digitalWrite(motor_derecha_ad,LOW);
    digitalWrite(motor_derecha_at,LOW);
    analogWrite(pwm_der,85);
    digitalWrite(motor_izquierda_at,LOW);
    digitalWrite(motor_izquierda_ad,LOW);
    analogWrite(pwm_iz,85);

    celda[i][i1] = valorCel;  // asignamos a la posicion inicial [0,0] el valor 6
    

  int b = digitalRead(boton); // Leemos el valor en el pin boton (8)

// Ciclo de espera del pulso del boton de inicio
 while(b==1){

    b = digitalRead(boton);
    digitalWrite(ledOn,90);
    delay(300);
   }
   digitalWrite(ledOn,0);
}





void loop() {

  Serial.write(0); //x
  Serial.write(2); //y
  Serial.write(10); //value
  Serial.write('\n');
  
 
 delay(1000);
  digitalWrite(ledPlay,HIGH); // led que indica que el robot esta listo para iniciar
  int b = digitalRead(boton);

while(b==1){ // bucle de inicio de recorrido del laberinto

b = digitalRead(boton);
posicion = posi(); // pedimos el valor de la posicion
cuadricula = cuad();



if(valorCel > 6){
  valorCel = 6;  
}


if(cuadricula == 0 & posicion == 9){ // si el robot va en linea recta y no hay interseccion o giros de 90 grados
  if(valorCel <= 7 & valorCel >= 0){
  if(norte){
    i1-=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
  }
  if(sur){
    i1+=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
  }
 if(este){
    i+=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
  }
 if(oeste){
    i-=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
  }
  }
}



   // Calculamos los valores proporcional, integral y derivativo
   proporcional = (posicion - 9);
   integral = (integral + proporcional_pasado);
   derivativo = (proporcional - proporcional_pasado);

   // Valor obtenidos del PID
   valor_pwm = (kp*proporcional)+(ki*integral)+(kd*derivativo);

   // Limitamos el valor de la integral
   if(integral>400){
    integral=400;
   }
   if(integral<-400){
    integral=-400;
   }

   // Almacenamos el ultimo valor de proporcional
   proporcional_pasado = proporcional;
   
   // datos que son enviados a processing
  Serial.print(i);
  Serial.print(',');
  Serial.print(i1);
  Serial.print(',');
  Serial.println(celda[i][i1]);
   //Serial.print(posicion);
   //Serial.print(',');
   //Serial.println(cuadricula);
   
// Condiciones de la lectura de los sensores
if(posicion == 9){ // Si esta en linea, avanza
  if(valorCel < 7 & valorCel >= 0){
    digitalWrite(motor_derecha_ad,HIGH);
    digitalWrite(motor_derecha_at,LOW);
    analogWrite(pwm_der,85);
    digitalWrite(motor_izquierda_ad,HIGH);
    digitalWrite(motor_izquierda_at,LOW);
    analogWrite(pwm_iz,85);
  }
   }
if(posicion == 8 & cuadricula == 0){ // Gira para la izquierda
  if(valorCel <= 7 & valorCel >= 0){
   if(este){
      norte=true;
      este=!este;
       i+=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
    if(oeste){
      sur=true;
      oeste=!oeste;
       i-=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
       if(norte){
      oeste=true;
      norte=!norte;
       i1-=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
    if(sur){
      este=true;
      sur=!sur;
       i1+=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }

  
      while(posicion!=9){ //el robot gira a la izquierda hasta encontrar la linea
        posicion = posi();
        
    b = digitalRead(boton);
    digitalWrite(motor_derecha_ad,HIGH);
    digitalWrite(motor_derecha_at,LOW);
    analogWrite(pwm_der,85);
    delay(500);
    digitalWrite(motor_izquierda_ad,LOW);
    digitalWrite(motor_izquierda_at,HIGH);
    analogWrite(pwm_iz,85);
    posicion = posi();
      }
  }
  }
if(posicion == 1 & cuadricula == 0){ // Gira para la derecha
   if(valorCel <= 7 & valorCel >= 0){
    if(este){
      sur=true;
      este=!este;
       i+=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
    if(oeste){
      norte=true;
      oeste=!oeste;
       i-=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
       if(norte){
      este=true;
      norte=!norte;
       i1-=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
    if(sur){
      oeste=true;
      sur=!sur;
       i1+=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
     
    while(posicion!=9){//el robot gira a la derecha hasta encontrar la linea
      posicion = posi();
      b = digitalRead(boton);
    digitalWrite(motor_izquierda_at,LOW);
    digitalWrite(motor_izquierda_ad,HIGH);
    analogWrite(pwm_iz,85);
    delay(500);
    digitalWrite(motor_derecha_at,HIGH);
    digitalWrite(motor_derecha_ad,LOW);
    analogWrite(pwm_der,90);
    posicion = posi(); 
    }
   }
  }
if((posicion<15)&(posicion!=9)&(posicion!=8)&(posicion!=1)){  // Si los sensores estan en otra posicion 
   if(valorCel < 7 & valorCel >= 0){
   if(valor_pwm < 0){  // Gira para la derecha
    digitalWrite(motor_derecha_at,HIGH);
    digitalWrite(motor_derecha_ad,LOW);
    analogWrite(pwm_der,40+(-1)*valor_pwm);
    digitalWrite(motor_izquierda_ad,HIGH);
    digitalWrite(motor_izquierda_at,LOW);
    analogWrite(pwm_iz,85);
   }
   if(valor_pwm > 0){ // Gira para la izquierda
    digitalWrite(motor_derecha_ad,HIGH);
    digitalWrite(motor_derecha_at,LOW);
    analogWrite(pwm_der,85);
    digitalWrite(motor_izquierda_at,HIGH);
    digitalWrite(motor_izquierda_ad,LOW);
    analogWrite(pwm_iz,40+valor_pwm);
   }
   }
}      
if(posicion == 0){  // si todos los sensores estan sobre la linea negra, el carro gira a la derecha.
     if(valorCel < 7 & valorCel >= 0){
    if(este){
      sur=true;
      este=!este;
       i+=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
    if(oeste){
      norte=true;
      oeste=!oeste;
       i-=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
       if(norte){
      este=true;
      norte=!norte;
       i1-=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
    if(sur){
      oeste=true;
      sur=!sur;
       i1+=1;
  valorCel-=1;
  celda[i][i1]=valorCel;
    }
    digitalWrite(motor_derecha_ad,HIGH);
    digitalWrite(motor_derecha_at,LOW);
    analogWrite(pwm_der,90);
    delay(500);
    digitalWrite(motor_izquierda_ad,LOW);
    digitalWrite(motor_izquierda_at,HIGH);
    analogWrite(pwm_iz,90);
   }
}
if(posicion == 15 & cuadricula == 2){  // si el camino termina hace un giro de 180 grados y actualiza el valor de la celda
  if(valorCel < 6 & valorCel >= 0){ 
      if(este){
      oeste=true;
      este=!este;
    }
    if(oeste){
      este=true;
      oeste=!oeste;
    }
       if(norte){
      sur=true;
    }
    if(sur){
      norte=true;
      sur=!sur;
    }
    valorCel+=2;
    celda[i][i1] = valorCel;
    digitalWrite(motor_derecha_at,LOW);
    digitalWrite(motor_derecha_ad,HIGH);
    analogWrite(pwm_der,90);
    digitalWrite(motor_izquierda_ad,LOW);
    digitalWrite(motor_izquierda_at,HIGH);
    analogWrite(pwm_iz,90);
  }
 }
}

// secuencia de espera
delay(300);
digitalWrite(ledPlay,LOW);
delay(3000);
b = digitalRead(boton);
digitalWrite(motor_derecha_at,LOW);
    digitalWrite(motor_derecha_ad,LOW);
    analogWrite(pwm_der,0);
    digitalWrite(motor_izquierda_ad,LOW);
    digitalWrite(motor_izquierda_at,LOW);
    analogWrite(pwm_iz,0);

while(b==1){
  
 b = digitalRead(boton);
digitalWrite(ledPlay,HIGH);
delay(300);
digitalWrite(ledPlay,LOW);
delay(300);
}

delay(2000);
digitalWrite(ledPlay,HIGH);


while(true){ 
  
}
}
