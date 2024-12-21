#include <Arduino.h>
#include "KTransFunc.h"

// Ngưỡng tốc độ PWM tối thiểu và tối đa
const int MIN_PWM = 20;               
const int MAX_PWM = 255;

// Chân điều khiển động cơ
const int vrPinLeft = 10;    // Chân điều chỉnh tốc độ bánh 
const int zfPinLeft = 9;     // Chân điều khiển chiều quay bánh 
const int elPinLeft = 8;     // Chân bật/tắt động cơ

// Chân Hall cho bánh trái
#define hallA_l 2
#define hallB_l 3
#define hallZ_l 4

// Các biến trạng thái Hall và đếm xung cho bánh trái
//String binaryArray[6] = {"101", "100", "110", "010", "011", "001"};     //Bánh phải
String binaryArray[6] = {"001", "011", "010", "110", "100", "101"};   //Bánh trái
int stateIndex_l = -1;
int count_l = 0;
int previousCount_l = 0;
String previousState_l;

// Biến để tính vận tốc góc
const int pulsesPerRevolution = 90;            // Số xung trên mỗi vòng quay
const unsigned long interval = 50;            // Thời gian giữa các lần đọc (ms)
unsigned long previousTime = 0;

// Biến điều khiển PID cho vận tốc góc
float targetAngularVelocity = 0;              // Vận tốc góc mục tiêu
//float Kp = 8.55 , Ki = 9.55, Kd = 0.81;         
float Kp = 8.55 , Ki = 9.55, Kd = 0.81;     
float previousError = 0;
float integral = 0;
float integralLimit = 50;                      // Giới hạn tích phân để tránh windup

// Biến lưu giá trị tốc độ động cơ
int pwmValue = 0;
bool initialSet = false;  // Biến để xác định đã kích hoạt động cơ lần đầu

// Bộ lọc thông thấp 
KTransFunc LowPassFilter;
const double DeltaT = interval / 1000.0; // Chuyển đổi ms thành giây
const int Order = 3;                      // Bậc của bộ lọc
const double CutFreq = 2;               // Tần số cắt

void setup() {
  pinMode(vrPinLeft, OUTPUT);
  pinMode(zfPinLeft, OUTPUT);
  pinMode(elPinLeft, OUTPUT);
  //digitalWrite(zfPinLeft, LOW);               // Thiết lập chiều quay ban đầu PHAI
  digitalWrite(zfPinLeft, HIGH);               // Thiết lập chiều quay ban đầu TRAI
  digitalWrite(elPinLeft, HIGH);              // Bật động cơ
  pinMode(hallA_l, INPUT_PULLUP);
  pinMode(hallB_l, INPUT_PULLUP);
  pinMode(hallZ_l, INPUT_PULLUP);
  Serial.begin(115200);
  initializeWheel(hallA_l, hallB_l, hallZ_l, previousState_l, stateIndex_l);

  // Khởi tạo bộ lọc thông thấp
  KBWLPass::Make(&LowPassFilter, DeltaT, Order, CutFreq);
}

void loop() {
  // Nhận vận tốc góc mục tiêu từ Serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "k") {
      // Dừng động cơ ngay lập tức khi nhận "0"
      pwmValue = 0;
      analogWrite(vrPinLeft, pwmValue);        // Đặt PWM về 0
      digitalWrite(elPinLeft, LOW);            // Tắt động cơ
      integral = 0;                            // Đặt lại tích phân
      previousError = 0;                       // Đặt lại lỗi trước đó
      targetAngularVelocity = 0;               // Đặt vận tốc mục tiêu về 0
      initialSet = false;                      // Đặt lại trạng thái
    } else {
      targetAngularVelocity = input.toFloat();  // Đọc vận tốc góc mục tiêu từ Serial
      initialSet = true;                       // Đánh dấu rằng đã kích hoạt lần đầu
      digitalWrite(elPinLeft, HIGH);           // Bật động cơ
      if (targetAngularVelocity < 0) {
        //digitalWrite(zfPinLeft, HIGH);  // Quay ngược chiều nếu vận tốc âm
        digitalWrite(zfPinLeft, LOW);  // Quay ngược chiều nếu vận tốc âm
      } else {
        //digitalWrite(zfPinLeft, LOW);   // Quay theo chiều hiện tại
        digitalWrite(zfPinLeft, HIGH);   // Quay theo chiều hiện tại
      }
      analogWrite(vrPinLeft, pwmValue);  // Áp dụng PWM ban đầu
    }
  }

  // Cập nhật trạng thái Hall cho bánh trái
  updateWheel(hallA_l, hallB_l, hallZ_l, previousState_l, stateIndex_l, count_l);

  // Tính vận tốc góc hiện tại mỗi khoảng thời gian
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    int deltaCount = (count_l - previousCount_l);
    float deltaTime = (currentTime - previousTime) / 1000.0; // Chuyển đổi ms thành giây
    float currentAngularVelocity = (deltaCount / (float)pulsesPerRevolution) * 2 * PI / deltaTime;

    // Áp dụng bộ lọc thông thấp cho vận tốc góc
    float filteredAngularVelocity = LowPassFilter.Work(currentAngularVelocity);

    // In giá trị 
    //Serial.print("Target:");
    //Serial.println(targetAngularVelocity);
   
    //Serial.print("\t"); 

    //Serial.print("Current:");  
    Serial.println(filteredAngularVelocity);

    // Nếu động cơ đang chạy, thực hiện điều khiển PID
    if (initialSet) {
      float error = targetAngularVelocity - filteredAngularVelocity;
      integral += error * deltaTime;

      // Giới hạn giá trị tích phân để tránh windup
      integral = constrain(integral, -integralLimit, integralLimit);

      float derivative = (error - previousError) / deltaTime;
      float output = Kp * error + Ki * integral + Kd * derivative;
      previousError = error;

      // Cập nhật giá trị PWM từ PID và đảm bảo trong phạm vi hợp lệ
      pwmValue = constrain(output, MIN_PWM, MAX_PWM);
      //pwmValue = constrain(MIN_PWM + abs(error) * 10, MIN_PWM, MAX_PWM);
      analogWrite(vrPinLeft, pwmValue);
    }

    // Cập nhật giá trị trạng thái trước đó
    previousCount_l = count_l;
    previousTime = currentTime;
  }
}


// Hàm khởi tạo trạng thái ban đầu cho bánh xe
void initializeWheel(int hallA, int hallB, int hallZ, String &previousState, int &stateIndex) {
  int a_state = digitalRead(hallA);
  int b_state = digitalRead(hallB);
  int z_state = digitalRead(hallZ);
  String hall_state = String(a_state) + String(b_state) + String(z_state);
  previousState = hall_state;

  for (int i = 0; i < 6; i++) {
    if (hall_state == binaryArray[i]) {
      stateIndex = i;
      break;
    }
  }
}

// Hàm cập nhật trạng thái của bánh xe dựa vào tín hiệu Hall
void updateWheel(int hallA, int hallB, int hallZ, String &previousState, int &stateIndex, int &count) {
  int a_state = digitalRead(hallA);
  int b_state = digitalRead(hallB);
  int z_state = digitalRead(hallZ);
  String hall_state = String(a_state) + String(b_state) + String(z_state);

  if (hall_state != previousState) {
    for (int i = 0; i < 6; i++) {
      if (hall_state == binaryArray[i]) {
        if (i == (stateIndex + 1) % 6) {
          count++;
          stateIndex = i;
        } else if (i == (stateIndex + 5) % 6) {
          count--;
          stateIndex = i;
        }
        break;
      }
    }
    previousState = hall_state;
  }
}
