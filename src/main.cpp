#include <Arduino.h>
#include <Bounce2.h>

#define MAX_COUNT 9999
#define MIN_COUNT 0
#define SWITCHED_SPEED 6
#define TEXT_TIMER 500
#define DATA_PIN 5   // 3
#define CLOCK_PIN 4  // 2
#define LATCH_PIN1 3 // 1
#define LATCH_PIN2 2 // 0
#define UP_PIN 6     // 4
#define DOWN_PIN 10  // 5

// button panel
#define BUTTON_DATA_PIN 9
#define BUTTON_CLOCK_PIN 8
#define BUTTON_LATCH_PIN 7
// code
#define BUTTON_STANDBY 255
#define BUTTON_START 0
#define BUTTON_FORWARD 1
#define BUTTON_UP 2
#define BUTTON_DOWN 3
#define BUTTON_PROG 4
#define BUTTON_RESET 5
#define BUTTON_ENTER 6

const unsigned char char_code_set[][2] = {
    {'0', 0b11111100}, /* 0 */
    {'1', 0b01100000}, /* 1 */
    {'2', 0b11011010}, /* 2 */
    {'3', 0b11110010}, /* 3 */
    {'4', 0b01100110}, /* 4 */
    {'5', 0b10110110}, /* 5 */
    {'6', 0b10111110}, /* 6 */
    {'7', 0b11100000}, /* 7 */
    {'8', 0b11111110}, /* 8 */
    {'9', 0b11110110}, /* 9 */
    {' ', 0b00000000}, /* (space) */
    {'A', 0b11101110}, /* A */
    {'B', 0b00111110}, /* b */
    {'b', 0b00111110}, /* b */
    {'C', 0b10011100}, /* C */
    {'D', 0b01111010}, /* D */
    {'d', 0b01111010}, /* d */
    {'E', 0b10011110}, /* E */
    {'F', 0b10001110}, /* F */
    {'G', 0b10111100}, /* G */
    {'H', 0b01101110}, /* H */
    {'h', 0b00101110}, /* h */
    {'I', 0b01100000}, /* I */
    {'J', 0b10110000}, /* J */
    {'L', 0b00011100}, /* L */
    {'n', 0b00101010}, /* n */
    {'N', 0b00101010}, /* N */
    {'O', 0b11111100}, /* O */
    {'P', 0b11001110}, /* P */
    {'R', 0b00001010}, /* R */
    {'r', 0b00001010}, /* r */
    {'S', 0b10110110}, /* S */
    {'t', 0b00011110}, /* t */
    {'U', 0b01101100}, /* U */
    {'Y', 0b01110110}, /* Y */
};

void up_counter();
void down_counter();
void reset_counter();
void do_somthings();
void show_count();
void task_scrollText(char *text);

typedef void (*callback_function_char)(char *_c);

bool first_start = true;
int count, resistor = 0;
char buf[5];
unsigned long last_out = 0;
unsigned long button_panel_out = 0;
Bounce up_debouncer;
Bounce down_debouncer;
char message01[13] = "HELLO FRIEND";

class TM_Timer
{
public:
  TM_Timer() {};
  TM_Timer(unsigned long _prd)
  {
    setPeriod(_prd);
  }

  void setPeriod(unsigned long _prd)
  {
    _prd <= 0 ? _period = 1 : _period = _prd;
  }

  void start()
  {
    _state = true;
    start_time = millis();
  }
  void stop()
  {
    if (_state)
    {
      current_time = millis() - start_time;
    }
    _state = false;
  }
  void restart()
  {
    start();
  }

  bool tick()
  {
    if (_state)
    {
      current_time = millis() - start_time;
    }
    if (_state && current_time >= _period)
    {
      return true;
    }
    return false;
  }

  bool active()
  {
    return _state;
  }

private:
  unsigned long _period, start_time, current_time = 0;
  bool _state;
};

class TM_Task
{
public:
  TM_Task() {}
  TM_Task(unsigned long _prd)
  {
    task_timer.setPeriod(_prd);
  }
  void start_task()
  {
    complete = false;
    execution = true;
    task_timer.start();
  }
  void stop_task()
  {
    execution = false;
    task_timer.stop();
  }
  void attach(callback_function_char _handler)
  {
    this->_handler = _handler;
  }
  void invoke(char *_c, callback_function_char)
  {
    _handler(_c);
  }
  virtual void do_task() {};
  bool complete = false;
  bool execution = false;
  callback_function_char _handler = nullptr;
  TM_Timer task_timer;
};

class RuningLine : public TM_Task
{
public:
  RuningLine() {}
  RuningLine(unsigned long timer)
  {
    task_timer.setPeriod(timer);
  }
  RuningLine(unsigned long timer, char *text)
  {
    task_timer.setPeriod(timer);
    message = text;
    len = strlen(text);
  }
  void do_task()
  {
    if (task_timer.tick())
    {
      if (execution && !complete)
      {
        if (len == 0 || loop_line_count >= len)
        {
          complete = true;
          loop_line_count = 0;
          stop_task();
        }
        if (!complete)
        {
          char *tmp = new char[5];
          sprintf(tmp, "%4s", " ");
          sprintf(buf, "%4s", " ");

          for (int j = 0; j < 4; j++)
          {
            if ((loop_line_count + j) < len)
            {
              tmp[j] = message[loop_line_count + j];
              buf[j] = message[loop_line_count + j];
            }
          }
          invoke(tmp, _handler);
          loop_line_count++;
          task_timer.restart();
          tmp = nullptr;
        }
      }
    }
  }

private:
  char *message;
  int len = 0;
  int loop_line_count = 0;
};

class DisplayText : public TM_Task
{
public:
  DisplayText() {}
  DisplayText(unsigned long timer)
  {
    task_timer.setPeriod(timer);
  }
  DisplayText(unsigned long timer, char *text)
  {
    task_timer.setPeriod(timer);
    message = text;
    len = strlen(text);
  }

private:
  char *message;
  int len = 0;
  int loop_line_count = 0;
};

class DisplayChar : public TM_Task
{
};

class DisplayCount : public TM_Task
{
};

class FourDigitsDisplay
{
public:
  FourDigitsDisplay(int _clockPin, int _latchPin1, int _latchPin2, int _dataPin)
  {
    clockPin = _clockPin;
    latchPin1 = _latchPin1;
    latchPin2 = _latchPin2;
    dataPin = _dataPin;

    pinMode(clockPin, OUTPUT);
    pinMode(latchPin1, OUTPUT);
    pinMode(latchPin2, OUTPUT);
    pinMode(dataPin, OUTPUT);
  }

  void shiftOutText(char *text)
  {
    int len = strlen(text);
    int idx = 4;

    for (int i = len - 1; i >= 0; --i)
    {
      shiftOutChar(text[i], idx++);
    }
  }
  void ledOn()
  {
    uint8_t value = 0b11110000;
    uint8_t prog_led = 0b00001000;

    digitalWrite(latchPin1, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, value);
    digitalWrite(latchPin1, HIGH);

    digitalWrite(latchPin2, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, ~prog_led);
    digitalWrite(latchPin2, HIGH);
  }

  void scrollText(char *text, long delayTime)
  {
    int len = strlen(text);
    if (len == 0)
      return;

    char *tmp = new char[5];

    for (int i = 0; i < len; i++)
    {
      sprintf(tmp, "%4s", " ");

      for (int j = 0; j < 4; j++)
      {
        if ((i + j) < len)
          tmp[j] = text[i + j];
      }

      unsigned long tms = millis() + delayTime;

      while (millis() < tms)
      {
        shiftOutText(tmp);
        yield();
      }
    }
  }

private:
  int clockPin, latchPin1, latchPin2, dataPin;
  const int max = MAX_COUNT;
  const int min = MIN_COUNT;
  uint8_t bit_clear(uint8_t number, uint8_t n)
  {
    return number & ~((uint8_t)1 << n);
  }

  uint8_t getCodeSetChar(unsigned char c)
  {
    uint8_t result = 0;
    for (unsigned int i = 0; i < sizeof(char_code_set); i++)
    {
      if (c == char_code_set[i][0])
      {
        result = char_code_set[i][1];
        result = ~result;
        break;
      }
    }
    return result;
  }
  void softDelay(long delayTime)
  {
    unsigned long tms = millis() + delayTime;

    while (millis() < tms)
    {
      yield();
    }
  }

  void shiftOutChar(unsigned char _c, byte _digit)
  {
    byte sevseg = getCodeSetChar(_c);

    uint8_t value = 0b11110000;
    bitClear(value, (uint8_t)_digit);

    digitalWrite(latchPin1, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, value);
    digitalWrite(latchPin1, HIGH);

    digitalWrite(latchPin2, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, sevseg);
    digitalWrite(latchPin2, HIGH);

    softDelay(SWITCHED_SPEED);
  }
};

FourDigitsDisplay myDisplay(CLOCK_PIN, LATCH_PIN1, LATCH_PIN2, DATA_PIN);
RuningLine testLine(500, message01);

void button_panel_input()
{
  static uint8_t last_input_states = 0;
  uint8_t curr_states = 0;
  digitalWrite(BUTTON_LATCH_PIN, LOW);
  delayMicroseconds(5); //
  digitalWrite(BUTTON_LATCH_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(BUTTON_CLOCK_PIN, HIGH);
  curr_states = shiftIn(BUTTON_DATA_PIN, BUTTON_CLOCK_PIN, LSBFIRST);
  if (curr_states != last_input_states)
  {
    uint8_t changed = curr_states ^ last_input_states;
    last_input_states = curr_states;
    for (int i = 0; i < 8; ++i)
    {
      if (changed & 1)
      {
        Serial.print("#");
        Serial.print(i);
        Serial.print(" -> ");
        Serial.println(curr_states & 1);
        if (curr_states & 1 && !first_start)
        {
          if (i == BUTTON_UP)
          {
            up_counter();
            testLine.stop_task();
          }
          if (i == BUTTON_DOWN)
          {
            down_counter();
            testLine.stop_task();
          }
          if (i == BUTTON_RESET)
          {
            reset_counter();
            testLine.stop_task();
          }
          if (i == BUTTON_FORWARD)
          {
            testLine.start_task();
          }
          if (i == BUTTON_PROG)
          {
          }
        }
      }
      changed >>= 1;
      curr_states >>= 1;
    }
    first_start = false;
  }
}

void task_scrollText(char *text)
{
  myDisplay.shiftOutText(text);
}

void up_counter()
{
  count < MAX_COUNT ? ++count : count = MAX_COUNT;
}

void down_counter()
{
  count > MIN_COUNT ? --count : count = MIN_COUNT;
}

void reset_counter()
{
  count = 0;
}

void do_somthings()
{
  myDisplay.scrollText(message01, 200 + resistor);
}

void show_count()
{
  if (count < 10)
  {
    sprintf(buf, "%01d", count);
  }
  else if (count >= 10 && count < 100)
  {
    sprintf(buf, "%02d", count);
  }
  else if (count >= 10 && count < 1000)
  {
    sprintf(buf, "%03d", count);
  }
  else if (count > 1000)
  {
    sprintf(buf, "%04d", count);
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);

  pinMode(BUTTON_LATCH_PIN, OUTPUT);
  pinMode(BUTTON_CLOCK_PIN, OUTPUT);
  pinMode(BUTTON_DATA_PIN, INPUT);

  up_debouncer.attach(UP_PIN);
  up_debouncer.interval(5);
  down_debouncer.attach(DOWN_PIN);
  down_debouncer.interval(5);

  digitalWrite(BUTTON_CLOCK_PIN, HIGH);
  digitalWrite(BUTTON_LATCH_PIN, HIGH);

  show_count();

  testLine.attach(task_scrollText);
}

void loop()
{
  unsigned long tms = millis();

  up_debouncer.update();
  down_debouncer.update();

  if (up_debouncer.fell())
  {
    up_counter();
  }
  else if (down_debouncer.fell())
  {
    down_counter();
  }
  else
  {
    myDisplay.shiftOutText(buf);
    testLine.do_task();
  }

  if ((tms - last_out) > 100)
  {
    last_out = tms;
    button_panel_input();
    // show_count();
  }
}