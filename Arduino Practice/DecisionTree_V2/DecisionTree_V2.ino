#include <ArduinoJson.h>
#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

#define XY_ACCELERATION_THRESHOLD 300
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

void setup() {
  // put your setup code here, to run once:
  proxSensors.initFrontSensor();
  Wire.begin();

  // Initialize LSM303
  lsm303.init();
  lsm303.enable();
  lcd.clear();
  lcd.print(F("Press A"));
  buttonA.waitForButton();
  lcd.clear();
}

void runAway()
{
  motors.setSpeeds(-200, -200);
}

void hitIt(){
  motors.setSpeeds(200, 200);
}
void stopIt(){
  motors.setSpeeds(0, 0);
}

void turnRight()
{
  motors.setSpeeds(100, -100);
}

void turnLeft()
{
  motors.setSpeeds(-100, 100);
}

void goCharge(){
    motors.setSpeeds(400, 400);
//  delay(500);
}

void loop() {

  Serial.begin(9600);
  while (!Serial) continue;
  const size_t capacity = 0 + 390;
  DynamicJsonDocument doc(capacity);  
  char json[] = "{\"Contact\": [{\"0\": [{\"0\": [{\"0\": \"No object seeing\", \"1\": \"No object seeing\", \"2\": \"Object seeing\", \"3\": \"Object seeing\"}]}, {\"1\": [{\"0\": \"No object seeing\", \"1\": \"No object seeing\", \"2\": \"Object seeing\", \"3\": \"Object seeing\"}]}, {\"2\": \"Object seeing\"}, {\"3\": \"Object seeing\"} ]}, {\"1\": \"Charge\"} ]}";
  //char json[] = "{\"Front left Sensor Left\": [{\"0\": \"Hit it\"}, {\"1\": [{\"0\": \"run away\", \"1\": \"hit it\", \"2\": \"hit it\", \"3\": \"Hit it\"}]}, {\"2\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"Hit it\", \"3\": \"Hit it\"}]}, {\"3\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"run away\", \"3\": \"hit it\"}]}]}";
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  //Serial.println("here");
  proxSensors.read();
  uint8_t a = map(proxSensors.countsFrontWithLeftLeds(), 0, 6, 0, 3);
  uint8_t b = map(proxSensors.countsFrontWithRightLeds(), 0, 6, 0, 3);
//  Serial.println(String(a));
//  Serial.println(String(b));
  //const char *value;
  const char *value = doc["Contact"][0]["0"][a]["1"][b]["1"];
  //const char *value = doc["Contact"][0]["0"][1]["1"][0]["0"];
//  loop_start_time = millis();
//  lsm303.readAcceleration(loop_start_time);
//  if (check_for_contact()){
//      //on_contact_made();
//    lcd.clear();
//    lcd.gotoXY(0,0);
//    lcd.println("Contact!!!, charge!");
//    ledYellow(1);
//    //goCharge();
//    delay(2000);
//  }
//  else{
//    lcd.clear();
//    lcd.gotoXY(0,0);
//    lcd.println("No charge");
//  }
  
  //const char *value = doc["Contact"][0]["0"][a][String(a)][b][String(b)];
  //Serial.println("here");
  Serial.println(*value);

  if(*value == 'N'){
    Serial.println("No object");  
  }
  else{
    Serial.println("Object");
  }
  //const char *value = doc["Contact"][1]["1"];
  //const char *a = doc["Contact"][0]["0"][0]["0"][0]["0"];
  //Serial.println(a);
  //Serial.println(*value);
  delay(100);
}

bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (lsm303.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
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
