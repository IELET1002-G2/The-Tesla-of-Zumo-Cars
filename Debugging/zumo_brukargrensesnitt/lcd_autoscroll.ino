/*
Under arbeid...
*/

#include <Zumo32U4.h>               //Importerer Zumo-biblioteket

Zumo32U4LCD lcd;                    //Oppretter instans av LCD-display

void setup() 
{
    lcd.clear();
    lcd.gotoXY(0, 1);
    lcd.print("<A B^ C>");
    lcd.gotoXY(0, 0;
    lcd.autoscroll();
}

void loop() {}

void print(char* string)
{
    static unsigned long timer = millis();              //Variabel som lagrer tida for siste print
    static int counter;
    static int* ptr = nullptr;

    if (millis() - timer > 100) {                       //Printer ikkje oftare enn kvart 100ms for lesbarhet
        if (counter == 0) {
            lcd.print(string);                               //Printer tal utan desimalar

            counter = 8;
            ptr = &string;
        }

        else if (counter > 7 && string[counter] != "\0") {
            lcd.print(string[counter]);
            counter++;
        }

        else counter = 0;
    
        timer = millis();                               //Lagrer tida
    }

    
    
    int size = sizeof(string)/8 - 1;

    if (size > 8) {

    }


}



char string1[] = "AdasdadS789abc";
char string2[] = "HalloSao123456";
char string3[] = "Hei";



void setup() { Serial.begin(9600); }

void loop() {
  if (millis()<4000) print2(string1);
  else print2(string3);
}

void print(char* str)
{
    static unsigned long timer = millis();              //Variabel som lagrer tida for siste print
    static int counter;
    static char* lastString = NULL;

    if (str != lastString) {
      counter = 0; 
      lastString = str;

      for (counter; counter < 8; counter++) {
        Serial.print(str[counter]);

        if (str[counter] == '\0') {
          counter = 0;

          break;
        }
      }
      Serial.print(str);                               //Printer tal utan desimalar

      counter = 8;
    }

    if (millis() - timer > 1000) {                       //Printer ikkje oftare enn kvart 100ms for lesbarhet
        if (counter == 0) {
            Serial.print(str);
            counter = 8;
        }

        else if (counter > 7 && str[counter] != '\0') {
            Serial.print(str[counter]);
            counter++;
        }

        if (str[counter] == '\0') counter = 0;
    
        timer = millis();                               //Lagrer tida
    }
}


void print2(char* str) 
{
    static unsigned long timer = millis();
    static char* lastStr = nullptr;
    static byte i;
    byte len;

    for (len = 1; str[len - 1] != '\0'; len++) {
        
    }
    if (str != lastStr) {
        //print(str, "<A B^ C>");
        i = 0; 
        lastStr = str;
    }

    if (i == 0) {
        Serial.println(str); //print(str, "<A B^ C>");

        if (len > 8) i = 8;
        else {i = 0; delay(1000);}
    }

    if (i >= 8) {
        if (millis() - timer > 1000) {
            if (str[i] != '\0') {
                //lcd.autoScroll();
                //lcd.print(str[i]);
                //lcd.noAutoScroll();
                Serial.println(str[i]); 
                
                i++;
            }
            else i = 0;

            timer = millis();  
        }
    }
}
