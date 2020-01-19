#define Pin1 A2
#define Pin2 A3
#define leftMotor 7
#define rightMotor 8

int Fl, Circle, Turnovers;

void checkCircle(int i)
{
  if(i == 0)
  {
    switch (Fl)
    {
      case 1:
        Fl = 0;
        break;
      case -1:
        Fl = 0;
        break;
      case 2:
        Fl = 0;
        Circle += 1;
        break;
      case -2:
        Fl = 0;
        Circle -= 1;
        break;
      default:
        Fl = 0;
        break;
    }
  }

  if((i > 0) && (i <=43)) //1
  {
    switch (Fl)
    {
      case 0:
        Fl = 1;
        break;
      case 1:
        Fl = 1;
        break;
      case -1:
        Fl = -2;
        break;
      case -2:
        Fl = -2;
        break;
      case 5:
        Fl = 11;
        break;
      case 11:
        Fl = 11;
        break;
      case 12:
        Fl = -2;
        break;
    }
  }
  
  if((i > 43) && (i <=60))  //2
  {
    switch (Fl)
    {
      case 1:
        Fl = 1;
        break;
      case -1:
        Fl = -1;
        break;
      case 5:
        Fl = 12;
        break;
      case 11:
        Fl = 1;
        break;
      case 12:
        Fl = 12;
        break;
      case 13:
        Fl = -1;
        break;
    }
  }
  
  if((i > 60) && (i <=160)) //3
  {
    switch (Fl)
    {
      case 1:
        Fl = 1;
        break;
      case -1:
        Fl = -1;
        break;
      case 5:
        Fl = 13;
        break;
      case 12:
        Fl = 1;
        break;
      case 13:
        Fl = 13;
        break;
      case 14:
        Fl = -1;
        break;
    }
  }
  
  if((i > 160) && (i <=700))  //4
  {
    switch (Fl)
    {
      case 1:
        Fl = 1;
        break;
      case -1:
        Fl = -1;
        break;
      case 5:
        Fl = 14;
        break;
      case 13:
        Fl = 1;
        break;
      case 14:
        Fl = 14;
        break;
      case 15:
        Fl = -1;
        break;
    }
  }
  
  if(i > 700) //5
  {
    switch (Fl)
    {
      case 0:
        Fl = -1;
        break;
      case 1:
        Fl = 2;
        break;
      case -1:
        Fl = -1;
        break;
      case 2:
        Fl = 2;
        break;
      case 5:
        Fl = 15;
        break;
      case 15:
        Fl = 15;
        break;
      case 14:
        Fl = 2;
        break;
    }
  }
}

void motorDriver(int i)
{
  int endCircle = Circle + i;

  if(i > 0)
  {
    while(endCircle != Circle)
    {
      digitalWrite(rightMotor, HIGH);
      digitalWrite(leftMotor, LOW);
      checkCircle(analogRead(Pin1));
    }
  }
  
  if(i < 0)
  {
    while(endCircle != Circle)
    {
      digitalWrite(rightMotor, LOW);
      digitalWrite(leftMotor, HIGH);
      checkCircle(analogRead(Pin1));
    }
  } 
  
  digitalWrite(rightMotor, LOW);
  digitalWrite(leftMotor, LOW);
}

void initialization()
{
  Fl = 5;
  Turnovers = 0;
  Serial.begin(9600);
  pinMode(INPUT, Pin1);
  pinMode(INPUT, Pin2);
  pinMode(OUTPUT, rightMotor);
  pinMode(OUTPUT, leftMotor);
  digitalWrite(rightMotor, LOW);
  digitalWrite(leftMotor, LOW);
  checkCircle(analogRead(Pin1));  
}

void setup() 
{
  initialization();
}


void loop() 
{  
  if(Serial.available() > 0)
  {
    int i = Serial.parseInt();
    motorDriver(i);    
  }
}
