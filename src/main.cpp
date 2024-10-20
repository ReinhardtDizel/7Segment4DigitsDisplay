#include <Arduino.h>
#include <Bounce2.h>

#define MAX_COUNT 9999
#define MIN_COUNT 0
#define SWITCHED_SPEED 7
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

// const char task_status_code[][2] = {
//     {'COMPLITE', 0b01100010},
//     {'EXECUTION', 0b01100000},
//     {'STOP', 0b01001000},
// };

void up_counter();
void down_counter();
void reset_counter();
void do_somthings();
void show_count();

void task_button_panel_input();

// typedef void (*callback_function_char)(char *_c);

bool first_start = true;
int count, resistor = 0;
// char buf[5];
unsigned long last_out = 0;
unsigned long button_panel_out = 0;
Bounce up_debouncer;
Bounce down_debouncer;
char message01[14] = "HELLO FRIEND ";

struct TM_Flag
{
  uint8_t error_code = 0;
  uint8_t task_id = 0;
  byte task_priority = 0;
  uint8_t task_status = 0;
};

class TM_Display_Buffer_Operations
{
public:
  TM_Display_Buffer_Operations()
  {
    bufferTextFormat();
  }
  ~TM_Display_Buffer_Operations()
  {
    buffer = nullptr;
  }
  void bufferTextFormat()
  {
    sprintf(buffer, "%4s", " ");
  }
  void buffer1DigitFormat()
  {
    sprintf(buffer, "%01d", count);
  }
  void buffer2DigitFormat()
  {
    sprintf(buffer, "%01d", count);
  }
  void buffer3DigitFormat()
  {
    sprintf(buffer, "%01d", count);
  }
  void buffer4DigitFormat()
  {
    sprintf(buffer, "%01d", count);
  }
  char *buffer = new char[5];
};

TM_Display_Buffer_Operations buffer_operation;

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
  unsigned long getPeriod()
  {
    return _period;
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
  virtual void start_task()
  {
    complete = false;
    execution = true;
    task_timer.start();
    flag.task_status = 0b01100000;
  }
  virtual void stop_task(uint8_t error_code)
  {
    execution = false;
    task_timer.stop();
    flag.task_status = 0b01001000;
    if (error_code != 0)
    {
      flag.error_code = error_code;
    }
  }
  virtual void do_task() {};
  virtual void special_task() {};

private:
  TM_Timer task_timer;
  TM_Flag flag;
  bool complete = false;
  bool execution = false;
};

class RuningLine : public TM_Task
{
public:
  RuningLine() {}
  RuningLine(unsigned long _prd, uint8_t id, byte priority)
  {
    task_timer.setPeriod(_prd);
    flag.task_priority = priority;
    flag.task_id = id;
  }
  void setMessage(char *message)
  {
    this->message = message;
    len = strlen(message);
  }
  void start_task() override
  {
    complete = false;
    execution = true;
    task_timer.start();
    flag.task_status = 0b01100000;
  }
  void stop_task(uint8_t error_code) override
  {
    execution = false;
    task_timer.stop();
    flag.task_status = 0b01001000;
    if (error_code != 0)
    {
      flag.error_code = error_code;
    }
  }
  void do_task() override
  {
    if (task_timer.tick())
    {
      if (execution)
      {
        if (len == 0 || loop_line_count >= len)
        {
          complete = true;
          loop_line_count = 0;
          flag.task_status = 0b01100010;
          stop_task(0);
        }
        if (!complete)
        {
          char *tmp = new char[5];
          sprintf(tmp, "%4s", " ");
          buffer_operation.bufferTextFormat();
          for (int j = 0; j < 4; j++)
          {
            if ((loop_line_count + j) < len)
            {
              tmp[j] = message[loop_line_count + j];
              buffer_operation.buffer[j] = message[loop_line_count + j];
            }
          }
          loop_line_count++;
          task_timer.restart();
          Serial.println(tmp);
          tmp = nullptr;
        }
      }
    }
  }

private:
  TM_Timer task_timer;
  TM_Flag flag;
  bool complete = false;
  bool execution = false;

  char *message;
  int len = 0;
  int loop_line_count = 0;
};

class DisplayChain : public TM_Task
{
public:
  DisplayChain() {}
  DisplayChain(unsigned long _prd, uint8_t id, byte priority)
  {
    task_timer.setPeriod(_prd);
    flag.task_priority = priority;
    flag.task_id = id;
    this->text = buffer_operation.buffer;
  }
  void start_task() override
  {
    complete = false;
    execution = true;
    task_timer.start();
    flag.task_status = 0b01100000;
  }
  void stop_task(uint8_t error_code) override
  {
    execution = false;
    task_timer.stop();
    flag.task_status = 0b01001000;
    if (error_code != 0)
    {
      flag.error_code = error_code;
    }
  }
  void do_task() override
  {
    if (execution)
    {
      if (!complete)
      {
        if (pos >= 0)
        {
          if (task_timer.tick())
          {
            task_display_char(text[pos], idx);
            pos--;
            idx++;
            task_timer.restart();
          }
        }
        else
        {
          pos = len - 1;
          idx = 4;
        }
      }
    }
  }

private:
  TM_Timer task_timer;
  TM_Flag flag;
  bool complete = false;
  bool execution = false;

  char *text;
  int len = 4;
  byte idx = 4;
  int pos = len-1;

  void task_display_char(unsigned char _c, byte _digit)
  {
    uint8_t sevseg = task_get_code_char(_c);

    uint8_t value = 0b11110000;
    bitClear(value, (uint8_t)_digit);

    digitalWrite(LATCH_PIN1, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, value);
    digitalWrite(LATCH_PIN1, HIGH);

    digitalWrite(LATCH_PIN2, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, sevseg);
    digitalWrite(LATCH_PIN2, HIGH);
  }

  uint8_t task_get_code_char(unsigned char c)
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

  uint8_t tas_bit_clear(uint8_t number, uint8_t n)
  {
    return number & ~((uint8_t)1 << n);
  }
};

class Counter : public TM_Task
{
};

class ButtonPanel : public TM_Task
{
public:
  ButtonPanel() {}
  ButtonPanel(unsigned long _prd, uint8_t id, byte priority)
  {
  }
};

RuningLine runing_line(TEXT_TIMER, 0b01100000, 8);
DisplayChain display_chain(SWITCHED_SPEED, 0b11011010, 1);

void setup()
{
  Serial.begin(9600);
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);

  pinMode(BUTTON_LATCH_PIN, OUTPUT);
  pinMode(BUTTON_CLOCK_PIN, OUTPUT);
  pinMode(BUTTON_DATA_PIN, INPUT);

  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN1, OUTPUT);
  pinMode(LATCH_PIN2, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  up_debouncer.attach(UP_PIN);
  up_debouncer.interval(5);
  down_debouncer.attach(DOWN_PIN);
  down_debouncer.interval(5);

  digitalWrite(BUTTON_CLOCK_PIN, HIGH);
  digitalWrite(BUTTON_LATCH_PIN, HIGH);

  // show_count();

  runing_line.setMessage(message01);
  display_chain.start_task();
}

//=============LOOP============================
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
    display_chain.do_task();
    runing_line.do_task();
    // myDisplay.shiftOutText(buf);
    // runing_line.do_task();
  }

  if ((tms - last_out) > 100)
  {
    last_out = tms;
    task_button_panel_input();
    // show_count();
  }
}
//===============================================

void task_button_panel_input()
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
            runing_line.stop_task(0);
          }
          if (i == BUTTON_DOWN)
          {
            down_counter();
            runing_line.stop_task(0);
          }
          if (i == BUTTON_RESET)
          {
            reset_counter();
            runing_line.stop_task(0);
          }
          if (i == BUTTON_FORWARD)
          {
            runing_line.start_task();
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

// void show_count()
// {
//   if (count < 10)
//   {
//     sprintf(buf, "%01d", count);
//   }
//   else if (count >= 10 && count < 100)
//   {
//     sprintf(buf, "%02d", count);
//   }
//   else if (count >= 10 && count < 1000)
//   {
//     sprintf(buf, "%03d", count);
//   }
//   else if (count > 1000)
//   {
//     sprintf(buf, "%04d", count);
//   }
// }