#define Pin1 A2
#define Pin2 A3
#define leftMotor 7
#define rightMotor 8

/*! Задает зоны резистивной дорожки.*/
enum Zones { ZONE1, ZONE2, ZONE3, ZONE4, ZONE5, ZONE6};

/*! Задает состояния, принимаемые системой при движении по зонам.
   FORWARD_MOVE_* - прямое движение (по часовой стрелке) в зоне *.
   BACKWARD_MOVE_* - обратное движение (против часовой стрелки) в зоне *.
*/
enum StatesOfZones {
  // Прямые движения (по часовой стрелке) по зонам.
  FORWARD_MOVE_ZONE1 = 0, FORWARD_MOVE_ZONE2 = 1,
  FORWARD_MOVE_ZONE3 = 2, FORWARD_MOVE_ZONE4 = 3,
  FORWARD_MOVE_ZONE5 = 4, FORWARD_MOVE_ZONE6 = 5,
  // Обратные движения (против часовой стрелки) по зонам.
  BACKWARD_MOVE_ZONE1 = 6, BACKWARD_MOVE_ZONE2 = 7,
  BACKWARD_MOVE_ZONE3 = 8, BACKWARD_MOVE_ZONE4 = 9,
  BACKWARD_MOVE_ZONE5 = 10, BACKWARD_MOVE_ZONE6 = 11
};

/*! Объявляет структуру функции обработчика смены зон резистивной дорожки.*/
typedef void (*transition_zone_callback)();

/*! Объявляет структуру переходных состояний микропрограммного
   автома для смены зон резистивной дорожки.
   Структура включает в себя новую зону, принимаемую микропрограммным автоматом и обработчик смены зон.
*/
struct transition
{
  enum Zones new_zone;
  transition_zone_callback worker;
};

// TODO: потом вернуть конечный автомат.
//struct transition FSM_table[5][5] = {
//  [0][1] = {1, null}
//}

/*! Setup. */
void setup() {
  // put your setup code here, to run once:
  InitPins();
}

/*! Loop. */
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    int i = Serial.parseInt();
    TakeTurns(i);
  }
}

/*! Выполняет иницилизацию пинов ардуино.*/
void InitPins() {
  Serial.begin(9600);
  pinMode(INPUT, Pin1);
  pinMode(INPUT, Pin2);
  pinMode(OUTPUT, rightMotor);
  pinMode(OUTPUT, leftMotor);
  digitalWrite(rightMotor, LOW);
  digitalWrite(leftMotor, LOW);
}

/* Выполняет заданное количество полных поворотов.
   param[in] countTurns Кол-во поворотов.
*/
void TakeTurns(int countTurns) {
  Zones currentZONE = GetCurrentZONE();


}

/* Определяет текущую зону устройства на основании значений АЦП, полученных с резистивной дорожки.
   Зона 1 - 0 значение АЦП;
   Зона 2 - 0..43;
   Зона 3 - 44..60;
   Зона 4 - 61..160;
   Зона 5 - 161..700;
   Зона 6 - 701..1023;
   return Зона в которой находится устройство в текущий момент.
*/
Zones GetCurrentZONE()
{
  int valueADC = analogRead(Pin1);
  if (valueADC == 0) {
    return ZONE1;
  } else if (valueADC < 44) {
    return ZONE2;
  } else if (valueADC < 61) {
    return ZONE3;
  } else if (valueADC < 161) {
    return ZONE4;
  } else if (valueADC < 701) {
    return ZONE5;
  } else {
    return ZONE6;
  }
}

// void record_add_handler(
