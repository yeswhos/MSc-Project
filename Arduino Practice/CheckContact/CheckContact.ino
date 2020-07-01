#include <avr/pgmspace.h>
#include <Wire.h>
#include <Zumo32U4.h>

#Need to figure out 07.01

// #define LOG_SERIAL // write log output to serial port

Zumo32U4LCD lcd;
Zumo32U4ButtonA button;

#define XY_ACCELERATION_THRESHOLD 2400
#define QTR_THRESHOLD  1000
unsigned long loop_start_time;
unsigned long last_turn_time;
unsigned long contact_made_time;
#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000
#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings
#define XY_ACCELERATION_THRESHOLD 2400

template <typename T>
class RunningAverage
{
  public:
    RunningAverage(void);
    RunningAverage(int);
    ~RunningAverage();
    void clear();
    void addValue(T);
    T getAverage() const;
    void fillValue(T, int);
  protected:
    int _size;
    int _cnt;
    int _idx;
    T _sum;
    T * _ar;
    static T zero;
};

class Accelerometer : public LSM303
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } acc_data_xy;

  public:
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE) {};
    ~Accelerometer() {};
    void enable(void);
    void getLogHeader(void);
    void readAcceleration(unsigned long timestamp);
    float len_xy() const;
    float dir_xy() const;
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
    float dir_xy_avg(void) const;
  private:
    acc_data_xy last;
    RunningAverage<int> ra_x;
    RunningAverage<int> ra_y;
};

Accelerometer lsm303;
boolean in_contact;

void setup() {
  lcd.clear();
  // put your setup code here, to run once:
  Wire.begin();

  // Initialize LSM303
  lsm303.init();
  lsm303.enable();

#ifdef LOG_SERIAL
  lsm303.getLogHeader();
#endif

  randomSeed((unsigned int) millis());
}

void loop() {
  // put your main code here, to run repeatedly:
  if (check_for_contact()) on_contact_made();
  loop_start_time = millis();
  lsm303.readAcceleration(loop_start_time);
}

bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (lsm303.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

// sound horn and accelerate on contact -- fight or flight
void on_contact_made()
{
#ifdef LOG_SERIAL
  Serial.print("contact made");
  Serial.println();
#endif
  in_contact = true;
  contact_made_time = loop_start_time;
  Serial.println("on contact mode");
  
}

// reset forward speed
void on_contact_lost()
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  Serial.println("on contact lost");
}

void Accelerometer::enable(void)
{
  // Enable Accelerometer
  // 0x27 = 0b00100111
  // Normal power mode, all axes enabled
  writeAccReg(LSM303::CTRL_REG1_A, 0x27);

  if (getDeviceType() == LSM303::device_DLHC)
  writeAccReg(LSM303::CTRL_REG4_A, 0x08); // DLHC: enable high resolution mode
}

void Accelerometer::getLogHeader(void)
{
  Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
  Serial.println();
}

void Accelerometer::readAcceleration(unsigned long timestamp)
{
  readAcc();
  if (a.x == last.x && a.y == last.y) return;

  last.timestamp = timestamp;
  last.x = a.x;
  last.y = a.y;

  ra_x.addValue(last.x);
  ra_y.addValue(last.y);

#ifdef LOG_SERIAL
 Serial.print(last.timestamp);
 Serial.print("  ");
 Serial.print(last.x);
 Serial.print("  ");
 Serial.print(last.y);
 Serial.print("  ");
 Serial.print(len_xy());
 Serial.print("  ");
 Serial.print(dir_xy());
 Serial.print("  |  ");
 Serial.print(sqrt(static_cast<float>(ss_xy_avg())));
 Serial.print("  ");
 Serial.print(dir_xy_avg());
 Serial.println();
#endif
}

float Accelerometer::len_xy() const
{
  return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
  return atan2(last.x, last.y) * 180.0 / M_PI;
}

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg());
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

float Accelerometer::dir_xy_avg(void) const
{
  return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}

template <typename T>
T RunningAverage<T>::zero = static_cast<T>(0);

template <typename T>
RunningAverage<T>::RunningAverage(int n)
{
  _size = n;
  _ar = (T*) malloc(_size * sizeof(T));
  clear();
}

template <typename T>
RunningAverage<T>::~RunningAverage()
{
  free(_ar);
}

// resets all counters
template <typename T>
void RunningAverage<T>::clear()
{
  _cnt = 0;
  _idx = 0;
  _sum = zero;
  for (int i = 0; i< _size; i++) _ar[i] = zero;  // needed to keep addValue simple
}

// adds a new value to the data-set
template <typename T>
void RunningAverage<T>::addValue(T f)
{
  _sum -= _ar[_idx];
  _ar[_idx] = f;
  _sum += _ar[_idx];
  _idx++;
  if (_idx == _size) _idx = 0;  // faster than %
  if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added so far
template <typename T>
T RunningAverage<T>::getAverage() const
{
  if (_cnt == 0) return zero; // NaN ?  math.h
  return _sum / _cnt;
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
template <typename T>
void RunningAverage<T>::fillValue(T value, int number)
{
  clear();
  for (int i = 0; i < number; i++)
  {
    addValue(value);
  }
}
