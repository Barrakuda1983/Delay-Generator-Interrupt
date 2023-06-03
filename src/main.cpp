/*
Delay-Generator mit LCD-Display und Keypad zur Einstellung der Delay- sowie Pulsdauer
*/

/*
LCD-Keypad-Shield von Sainsmart
Der Tasterdruck wird über den Analogwert, ausgegeben an A0 ermittelt
Dies wird sicherlich von Board zu Board variieren. Das aktuell verbaute weißt folgende Wert auf
Taste:              Analogwert
Kein Tastendruck    1023
 'Select'           ca. 741
 'Left'             ca. 504
 'Right'            ca. 0
 'Up'               ca. 145
 'Down'             ca. 329
*/

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <util/delay.h>

// Define >> Eingangspins LCD-Display
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );

// Define >> Signal-Eingangpin und Signal-Ausgangspi
const byte signal_input = 2;
const byte signal_output = 13;

// Startwert der einzustellenden Variablen
uint32_t delay_time = 20;            // in Millisekunden
uint32_t puls_time = 2;              // in Millisekunden
bool einstellung_ok = false;
uint8_t menu = 1;                   // Menü-Variable
unsigned long previousMillis = 0;   // Varaible zumspeichern des aktuell Millis-Wert, als Global, da in der Funktion gelöscht werden würde.

// Delay Funktion in der sich Generator im Normalbetrieb befindet.
void delay_begin(){
                /*
                unsigned long currentMillis;
                bool finish = true;
                previousMillis = millis();
                
                while (finish)
                {
                  currentMillis = millis();
                  if (currentMillis - previousMillis >= delay_time){
                      previousMillis = currentMillis;
                      digitalWrite(signal_output,HIGH);
                
                      if (currentMillis - previousMillis >= (delay_time+puls_time)){
                      previousMillis = currentMillis;
                      digitalWrite(signal_output,LOW);
                      finish = false;
                      }
                  }
                  
                }
                Serial.println(previousMillis);
*/
  
  uint32_t waitD = delay_time*570;
  uint32_t waitP = puls_time*570;
  while(waitD > 0){
    _delay_us(1);
    waitD--;
  }
  digitalWrite(signal_output,HIGH);
  while(waitP > 0){
    _delay_us(1);
    waitP--;
  }
  digitalWrite(signal_output,LOW);

}

void setup() {
  lcd.begin(16,2);
  pinMode(signal_input,INPUT);
  pinMode(signal_output,OUTPUT);
  pinMode(A0, INPUT);
  //Serial.begin(9600);
  
  // Schleife zum Einstellen der Variablen
  do {
    
    if(menu == 1){
      lcd.setCursor(0,0);
      lcd.print("Delay-Zeit (ms)");
      lcd.setCursor(0,1);
      lcd.print("= "+ String(delay_time));
        if ((analogRead(A0) == 145) && (delay_time < 95)){
          delay_time++;
          delay(200);
          lcd.clear();
        }
        else if ((analogRead(A0) == 329) && (delay_time > 1)){
          delay_time--;
          delay(200);
          lcd.clear();
        }
        else if (analogRead(A0) == 741){
          menu = 2;
          delay(500);
          lcd.clear();
        }
    }
    if(menu == 2){
      lcd.setCursor(0,0);
      lcd.print("Pulsdauer (ms)");
      lcd.setCursor(0,1);
      lcd.print("= "+ String(puls_time));
      if ((analogRead(A0) == 145) && (puls_time < 15)){
        puls_time++;
        delay(200);
        lcd.clear();
      }
      else if ((analogRead(A0) == 329) && (puls_time > 1)){
        puls_time--;
        delay(200);
        lcd.clear();
      }
      else if (analogRead(A0) == 741){
        menu = 3;
        delay(500);
        lcd.clear();
        lcd.print("Drueck select");
        lcd.setCursor(0,1);
        lcd.print("zum starten");
        //Serial.println(delay_time);
        //Serial.println(puls_time);
      }
    }
    else if ((analogRead(A0) == 741 ) && (menu == 3)){
        lcd.clear();
        lcd.setCursor(5,0);
        lcd.print("Aktiv");
        lcd.setCursor(0,1);
        lcd.print("Del.= "+ String(delay_time));
        lcd.setCursor(9,1);
        lcd.print("Pu.= "+ String(puls_time));
        einstellung_ok = true;
      }

  } while (!(einstellung_ok));

attachInterrupt(digitalPinToInterrupt(signal_input), delay_begin, RISING);
}

void loop() {

  /*
  Im Loop passiert nichts, da der Interrupt Pin2 genutzt wird.
    */
}

