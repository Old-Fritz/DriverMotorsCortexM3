#define Pin1 A2
#define Pin2 A3
#define leftMotor 7
#define rightMotor 8

#pragma Enums
/*! Задает зоны резистивной дорожки.*/
enum Zones { ZONE1, ZONE2, ZONE3, ZONE4, ZONE5, ZONE6};

/*! Задает состояния, принимаемые системой при движении по зонам.
   FORWARD_MOVE_* - прямое движение (по часовой стрелке) в зоне *.
   BACKWARD_MOVE_* - обратное движение (против часовой стрелки) в зоне *.
*/
enum StatesOfZones {
  // Прямые движения (по часовой стрелке) по зонам.
  FORWARD_MOVE_ZONE1 = 0, FORWARD_MOVE_ZONE2,
  FORWARD_MOVE_ZONE3, FORWARD_MOVE_ZONE4,
  FORWARD_MOVE_ZONE5, FORWARD_MOVE_ZONE6,
  FORWARD_MOVE_UNKNOWN,
  // Обратные движения (против часовой стрелки) по зонам.
  BACKWARD_MOVE_ZONE1, BACKWARD_MOVE_ZONE2,
  BACKWARD_MOVE_ZONE3, BACKWARD_MOVE_ZONE4,
  BACKWARD_MOVE_ZONE5, BACKWARD_MOVE_ZONE6,
  BACKWARD_MOVE_UNKNOWN,
};

/*! Объявляет структуру функции обработчика смены зон резистивной дорожки.*/
typedef void (*transition_zone_callback)();

/*! Объявляет структуру переходных состояний микропрограммного
   автома для смены зон резистивной дорожки.
   Структура включает в себя новую зону, принимаемую микропрограммным автоматом и обработчик смены зон.
*/
struct transition {
  enum Zones new_zone;
  transition_zone_callback worker;
};

//// TODO: потом вернуть конечный автомат.
//struct transition FSM_table[5][5] = {
//  [0][1] = {1, null}
//}
#pragma endregion Содержит объявление структур и енумов для работы с устройством.

#pragma Fields
/*! Определяет текущее состояние устройства. */
StatesOfZones CURRENT_STATE;

/*! Определяет текущее количество поворотов, совершенных устройством. */
int COUNT_TURNS = 0;
#pragma endregion Содержит поля, необходимые для работы с устройством.

#pragma setup and loop
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
#pragma endregion Содержит методы setup и loop.

#pragma Methods
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
  // Zones currentZONE = GetCurrentZONE();
  int endTurns = COUNT_TURNS + countTurns;
  
  if (countTurns > 0) {
    CURRENT_STATE = FORWARD_MOVE_UNKNOWN;
    digitalWrite(rightMotor, HIGH);
    digitalWrite(leftMotor, LOW);

    Zones lastZONE = GetCurrentZONE();
    while (endTurns != COUNT_TURNS) {
      Zones currentZONE = GetCurrentZONE();

      if (lastZONE == ZONE6 && currentZONE == ZONE1) {
        AddTurn();
      } else if (lastZONE == ZONE1 && currentZONE == ZONE6) {
        RemoveTurn();
      }
    }

    digitalWrite(rightMotor, LOW);
    digitalWrite(leftMotor, LOW);
  } else {
    CURRENT_STATE = BACKWARD_MOVE_UNKNOWN;
    digitalWrite(rightMotor, LOW);
    digitalWrite(leftMotor, HIGH);

    Zones lastZONE = GetCurrentZONE();
    while (endTurns != COUNT_TURNS) {
      Zones currentZONE = GetCurrentZONE();

      if (lastZONE == ZONE1 && currentZONE == ZONE6) {
        RemoveTurn();
      } else if (lastZONE == ZONE6 && currentZONE == ZONE1) {
        AddTurn();
      }
    }

    digitalWrite(rightMotor, LOW);
    digitalWrite(leftMotor, LOW);
  }
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

/*! Добавляет один полный оборот в текущее количество поворотов. */
void AddTurn() {
   COUNT_TURNS++;
}

/*! Удаляет один полный оборот из текущего количество поворотов. */
void RemoveTurn() {
   COUNT_TURNS--;
}
#pragma endregion Содержит основые методы программы.
