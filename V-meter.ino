/*
  Скетч для калибровки точного вольтметра и его использование
  КАЛИБРОВКА:
  0) Ставим vol_calibration 1, прошиваем
  1) Запускаем, открываем монитор. Будет выведено реальное значение Vсс в милливольтах
  из расчёта по стандартной константе 1.1
  2) Измеряем вольтметром напряжение на пинах 5V и GND, полученное значение отправляем в порт
  В МИЛЛИВОЛЬТАХ (то есть если у нас 4.54В то отправляем 4540). Новая константа будет рассчитана
  автоматически и запишется во внутреннюю память
  3) Ставим vol_calibration 0, прошиваем
  4) Наслаждайтесь точными измерениями!
  ИСПОЛЬЗОВАНИЕ:
  0) Функция readVCC возвращает ТОЧНОЕ опорное напряжение В МИЛЛИВОЛЬТАХ. В расчётах используем
  не 5 Вольт, а readVсс!
  1) При использовании analogRead() для перевода в вольты пишем:
  float voltage = analogRead(pin) * (readVсс() / 1023.0); - это точный вольтаж В МИЛЛИВОЛЬТАХ
*/

#include <LiquidCrystal.h>
int backlight = 3;
LiquidCrystal lcd(4, 6, 10, 11, 12, 13);

#define vol_calibration 0                                                 // калибровка вольтметра (если работа от АКБ) 1 - включить, 0 - выключить
float my_vcc_const = 1.1;                                                 // начальное значение константы должно быть 1.1
#include "EEPROMex.h"                                                     // библиотека для работы со внутренней памятью ардуино

void setup() {
  pinMode(backlight, OUTPUT);
  digitalWrite(backlight, HIGH);
  lcd.begin(16, 2);
  Serial.begin(9600);
  if (vol_calibration) calibration();                                     // калибровка, если разрешена
  my_vcc_const = EEPROM.readFloat(0);                                     // считать константу из памяти
}

void loop() {
  //lcd.print("Voltage:");
  //lcd.setCursor(0, 1);
  float voltage = analogRead(A0) * (readVcc() / 1023000.0);
  lcd.print("Ha\xbe""p\xc7\xb6""e\xbd\xb8""e: " + String(voltage));
  //lcd.print(voltage);
  delay(200);
  lcd.clear();
}

void calibration() {
  my_vcc_const = 1.1;                                                     // начальаня константа калибровки
  Serial.print("Real VCC is: "); Serial.println(readVcc());               // общаемся с пользователем
  Serial.println("Write your VCC (in millivolts)");
  while (Serial.available() == 0); int Vcc = Serial.parseInt();           // напряжение от пользователя
  float real_const = (float)1.1 * Vcc / readVcc();                        // расчёт константы
  Serial.print("New voltage constant: "); Serial.println(real_const, 3);
  Serial.println("Set vol_calibration 0, flash and enjoy!");
  EEPROM.writeFloat(0, real_const);                                       // запись в EEPROM
  while (1);                                                              // уйти в бесконечный цикл
}
// функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
long readVcc() {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2);                                       // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                            // Start conversion
  while (bit_is_set(ADCSRA, ADSC));               // measuring
  uint8_t low  = ADCL;                            // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH;                            // unlocks both
  long result = (high << 8) | low;
  result = my_vcc_const * 1023 * 1000 / result;   // расчёт реального VCC
  return result;                                  // возвращает VCC
}
