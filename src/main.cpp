#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

#define verde 2
#define amarelo 3
#define vermelho 4
#define bot 5
#define flag_dist 0

uint8_t flags = 0;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void dist();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  for(int i=2; i<5; i++){
      pinMode(i, OUTPUT);
  };

  if(!lox.begin()){
    Serial.print("Falha ao tentar inicializar o sensor de distância");
  }else{
    Serial.print("Ok");
  }

  pinMode (bot, INPUT_PULLUP);

  digitalWrite(vermelho, HIGH);
  digitalWrite(amarelo, LOW);
  digitalWrite(verde, LOW);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t tempo = millis();
  if(~flags & (1 << flag_dist)){
    if(!digitalRead(bot)){
      delay(50);
      while(digitalRead(bot) == 0){
        if(millis() - tempo >= 2000){
          digitalWrite(vermelho, HIGH);
          digitalWrite(amarelo, HIGH);
          digitalWrite(verde, HIGH);
          flags |= (1 << flag_dist);       
          delay(50);
          while(!digitalRead(bot)){
            delay(500);
            digitalWrite(vermelho, LOW);
            digitalWrite(amarelo, LOW);
            digitalWrite(verde, LOW);
            delay(500);
            digitalWrite(vermelho, HIGH);
            digitalWrite(amarelo, HIGH);
            digitalWrite(verde, HIGH);
            delay(500);
          }
          return;
        }
      };
        delay(3000);
        digitalWrite(vermelho, LOW);
        digitalWrite(amarelo, LOW);
        digitalWrite(verde, HIGH);
        delay(10000);
        digitalWrite(vermelho, LOW);
        digitalWrite(amarelo, HIGH);
        digitalWrite(verde, LOW);
        delay(2000);
        digitalWrite(vermelho, HIGH);
        digitalWrite(amarelo, LOW);
        digitalWrite(verde, LOW);
      }
      delay(10);
    }else{      
      if(!digitalRead(bot)){
        delay(50);
        while(!digitalRead(bot));
        flags &= ~(1 << flag_dist);
        digitalWrite(vermelho, HIGH);
        digitalWrite(amarelo, LOW);
        digitalWrite(verde, LOW);
        delay(50);
        return;
      }
      dist();
      delay(100);
    }   
}

void dist(){
  if(!lox.isRangeComplete()){    
    float dist = lox.readRange();
    Serial.println(dist);
    if(dist > 180){
      digitalWrite(vermelho, LOW);
      digitalWrite(amarelo, LOW);
      digitalWrite(verde, LOW);
      Serial.println("Dist>120");
    }else if(dist < 180 &&  dist >= 120){
      digitalWrite(vermelho, LOW);
      digitalWrite(amarelo, LOW);
      digitalWrite(verde, HIGH);
      Serial.println("dist < 120 &&  dist >= 80");
    }else if(dist < 120 &&  dist >= 80){
      digitalWrite(vermelho, LOW);
      digitalWrite(amarelo, HIGH);
      digitalWrite(verde, HIGH);
      Serial.println("dist < 80 &&  dist >= 20");
    }else if(dist < 80){
      digitalWrite(vermelho, HIGH);
      digitalWrite(amarelo, HIGH);
      digitalWrite(verde, HIGH);
      Serial.println("dist < 20");
    }
  }
}
