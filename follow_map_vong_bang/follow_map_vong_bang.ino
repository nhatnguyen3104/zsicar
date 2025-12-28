// --- KHAI BÁO CHÂN KẾT NỐI ---
#define PWMA 5
#define AIN1 7
#define AIN2 8
#define PWMB 6
#define BIN1 9
#define BIN2 10
#define STBY 4

// Cảm biến PID (5 Analog): A0 -> A4
const int sensorPins[5] = {A0, A1, A2, A3, A4};

// --- [MỚI] KHAI BÁO 2 CẢM BIẾN ĐẾM (DIGITAL) ---
#define SENSOR_COUNT_L 2  // Cảm biến đếm bên Trái (Chân 2)
#define SENSOR_COUNT_R 3  // Cảm biến đếm bên Phải (Chân 3)

#define TRIG_PIN 11
#define ECHO_PIN 12
// void calibrateSensors();
// int readLine();
// void setMotorSpeed(int left, int right);
// void turnLeft();
// void turnRight();

// --- BIẾN CHO VIỆC ĐẾM VẠCH ---
// int ReadRight = digitalRead(3);
// int ReadLeft = digitalRead(2);
int cntLeft = 0;
int cntRight =0;        // Biến lưu số vạch đã đếm được
int lastCntRight = 0;
int lastCntLeft =0;


bool isLeftOnLine = false;
bool isRightOnLine = false;
bool isCrossingLine = false; // Cờ đánh dấu đang đi qua vạch ngang

// --- CẤU HÌNH THÔNG SỐ PID ---
float Kp = 0.15;
float Ki = 0.00; // Ki nên rất nhỏ
float Kd = 2.5;

int THRESHOLD = 500;

int cntGap = 0;     
int lastCntGap =0;     // Biến đếm số lần mất vạch giữa (khoảng trắng)
bool isGapDetected = false; // Cờ đánh dấu đang trong vùng trắng
int RAW_THRESHOLD = 500; // Ngưỡng so sánh thô (0-1023), dưới mức này là Trắng

// Tốc độ
int BASE_SPEED = 80; 
int MAX_SPEED = 180;

// --- BIẾN TOÀN CỤC PID ---
int sensorMin[5] = {1023, 1023, 1023, 1023, 1023};
int sensorMax[5] = {0, 0, 0, 0, 0};
int sensorValues[5];
int lastError = 0;
float accumulatedError = 0; // [SỬA] Biến I phải là toàn cục để cộng dồn

void setup() {
  Serial.begin(9600); // Bật Serial để xem số đếm trên máy tính

  // Cấu hình chân Motor
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Cấu hình chân Cảm biến PID
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // --- [MỚI] CẤU HÌNH CẢM BIẾN ĐẾM ---
  // Dùng INPUT nếu module đã có trở kéo, hoặc INPUT_PULLUP nếu cần
  pinMode(SENSOR_COUNT_L, INPUT); 
  pinMode(SENSOR_COUNT_R, INPUT);

  pinMode(13, OUTPUT);

  // --- CALIBRATION ---
  digitalWrite(13, HIGH);
  unsigned long startTime = millis();
  while (millis() - startTime < 4000) {
    calibrateSensors();
  }
  digitalWrite(13, LOW);
  delay(1000);
}

void loop() {
  int ReadLeft = digitalRead(2);
  int ReadRight = digitalRead(3);
  if (ReadLeft == 1&&ReadRight==0)  {
    if (isLeftOnLine == false) {
      cntLeft++;     
      caseLeft();     
      isLeftOnLine = true;
    } else {
      isLeftOnLine = false;
    }
  }
  else if (ReadRight == 1&&ReadLeft==0) {
    if (isRightOnLine == false) {
      cntRight++;          
      caseRight();
      isRightOnLine = true;
    } else {
      isRightOnLine = false;
    }
  }
  else if (analogRead(sensorPins[1]) > RAW_THRESHOLD && 
           analogRead(sensorPins[2]) > RAW_THRESHOLD && 
           analogRead(sensorPins[3]) > RAW_THRESHOLD) {
    
    // Nếu chưa đánh dấu là đang ở vùng trắng
    if (isGapDetected == false) {
      cntGap++;           // Tăng biến đếm
      // Bạn có thể thêm hàm xử lý riêng ở đây, ví dụ: caseGap();
      caseGap();
      isGapDetected = true; // Đánh dấu đã nhận diện xong, không đếm nữa cho đến khi thoát ra
    } else {
      isGapDetected = false;

    }
           }
  else {
    // 2. PID LOGIC
  int position = readLine();
  int error = position - 2000;

  // Tính toán PID
  accumulatedError += error; 
  // Giới hạn tích phân để tránh vọt lố (Anti-windup)
  accumulatedError = constrain(accumulatedError, -10000, 10000); 

  int P = error;
  float I = accumulatedError; 
  int D = error - lastError;
  
  lastError = error;

  int motorSpeedAdjustment = (Kp * P) + (Ki * I) + (Kd * D);

  int leftSpeed = BASE_SPEED + motorSpeedAdjustment;
  int rightSpeed = BASE_SPEED - motorSpeedAdjustment;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotorSpeed(leftSpeed, rightSpeed);
  }


  
  // if(cntLeft==1 && cntRight==3){
  //  float dt= getDistanceCM();
  //  if(dt<=25)
  //  {
  //   cntLeft++;
  //   caseLeft();
  //  }
  // }
}


void caseRight() {
  if(lastCntRight +1 == cntRight) {
    switch (cntRight) {
    case 1: 
      runBlindForward(500);
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    
    case 2:
      //runBlindForward(300);
      turnLeft12();
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 3:
      runBlindForward(100);
      turnRight();
      setMotorSpeed(0, 0);
      delay(100);
      runBlindForward(400);
      setMotorSpeed(0, 0);
      delay(50);
      turnLeft30();
      setMotorSpeed(0, 0);
      delay(50);
      runBlindForward(260);//250
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 4:
      runBlindForward(300);//1500
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 5:
    runBlindForward(100);
      turnRight();
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 6:
      runBlindForward(300);
      if(cntLeft == 1) {
        cntLeft =2;
      }
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 7:
    runBlindForward(200);
      turnRight();
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 8:
    runBlindForward(200);
      turnLeft45();
      if(cntLeft == 2){
        cntLeft = 3;
      }
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 9:
      runBlindForward(300);
      if(cntLeft == 3) {
        cntLeft =4;
      }
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 10:
      runBlindForward(100);
      turnLeft();
      if(cntLeft==4){
        cntLeft = 5;
      }
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 11:
      runBlindForward(500);
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    case 12:
      runBlindForward(200);
      if(cntLeft==6) {
        cntLeft = 7;
      }
      if(cntRight!=lastCntRight) {
        lastCntRight = cntRight;
      }
      break;
    default:
    setMotorSpeed(0,0);
      break;
  }
  }
  
}
void caseLeft() {
  if(lastCntLeft+1==cntLeft){
    switch (cntLeft) {
    case 1:
      if(cntRight==1) {
        runBlindForward(200);
      }
      lastCntLeft = cntLeft;
      break;
    // case 2:
    //   int dt = getDistanceCM();
    //   if(dt<=25) {
    //     turnLeft45();
    //   } else {
    //     runBlindForward(600);
    //   }
    //   lastCntLeft = cntLeft;
    //   break;

    case 2:
      runBlindForward(300);
      if(cntRight==5) {
        cntRight =6;
      }
      lastCntLeft = cntLeft;
      break;
    case 3:
      runBlindForward(200);
      turnLeft45();
      if(cntRight==7){
        cntRight = 8;
      }
      lastCntLeft = cntLeft;
      break;
    case 4:
      runBlindForward(300);
      if(cntRight==8) {
        cntRight = 9;
      }
      lastCntLeft = cntLeft;
      break;
    case 5:
      runBlindForward(100);
      turnLeft45();
      if(cntRight ==9){
        cntRight=10;
      }
      lastCntLeft = cntLeft;
      break;
    case 6:
      runBlindForward(500);
      lastCntLeft = cntLeft;
      break;
    case 7:
      runBlindForward(200);
      if(cntRight==11){
        cntRight=12;
      }
      lastCntLeft = cntLeft;
      break;

    // case 4:
    // if(dt<=25)
    // {
    // turnLeft();
    // }
    // else 
    // {
    //   //di thang nos bat dc line
    // }
    //    if(cntLeft!=cntLeft) {
    //     lastCntLeft = cntLeft;
    //   }
    // break;


    } 
    
  }
}
void caseGap() {
  if(lastCntGap+1==cntGap){
    switch(cntGap){
    case 1:
      break;
    case 2:
      runBlindForward(900);////
      lastCntRight = cntRight;
      break;
  }
  }
  
}

float getDistanceCM() {
  long duration;

  // Phát xung TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Đo xung ECHO (timeout 30ms ~ 5m)
  duration = pulseIn(ECHO_PIN, HIGH, 30000);

  // Không nhận được sóng phản hồi
  if (duration == 0) {
    return -1;   // giá trị lỗi
  }

  // Tính khoảng cách (cm)
  return (duration * 0.034f) / 2.0f;
}

/*
// --- [MỚI] HÀM KIỂM TRA VÀ ĐẾM VẠCH ---
void checkCrossLine() {
  // Đọc giá trị 2 cảm biến ngoài cùng
  // Giả sử: Gặp vạch đen = 1 (HIGH). Nếu cảm biến bạn ngược lại (đen = 0), hãy đổi thành == LOW
  // Hầu hết module hồng ngoại digital: Có vạch = Đèn sáng = LOW (0). 
  // Hãy kiểm tra thực tế, ở đây mình để là LOW (0) tương ứng vạch đen.
  
  int leftVal = digitalRead(SENSOR_COUNT_L);
  int rightVal = digitalRead(SENSOR_COUNT_R);
  
  // Điều kiện phát hiện vạch ngang: Cả 2 cảm biến cùng chạm vạch đen (hoặc 1 trong 2 tùy bạn chỉnh)
  // Ở đây mình cài đặt: Nếu 1 trong 2 chạm vạch là tính
  bool detected = (leftVal == LOW) || (rightVal == LOW); 

  if (detected) {
    // Nếu phát hiện vạch và trước đó chưa báo là đang qua vạch
    if (!isCrossingLine) {
      lineCounter++; // Tăng biến đếm
      isCrossingLine = true; // Đánh dấu là đang nằm trên vạch
      
      // Có thể làm gì đó tại đây, ví dụ dừng xe nếu đếm đủ 3 vạch
      // if (lineCounter >= 3) { BASE_SPEED = 0; }
      
      // In ra Serial để kiểm tra
      Serial.print("Da dem duoc vach so: ");
      Serial.println(lineCounter);
    }
  } else {
    // Nếu không còn phát hiện vạch nữa -> Reset cờ để sẵn sàng cho vạch tiếp theo
    isCrossingLine = false; 
  }
}*/

// --- CÁC HÀM CŨ GIỮ NGUYÊN ---
int readLine() {
  long weightedSum = 0;
  long sum = 0;
  bool onLine = false;
  int threshold = 200;

  for (int i = 0; i < 5; i++) {
    int val = analogRead(sensorPins[i]);
    int calibratedVal = map(val, sensorMin[i], sensorMax[i], 0, 1000);
    calibratedVal = constrain(calibratedVal, 0, 1000);

    if (calibratedVal > 200) onLine = true; 

    sensorValues[i] = calibratedVal;
    weightedSum += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;
  }
  if (sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold) {
    return 2000; // Trả về vị trí trung tâm -> Error = 0 -> Robot đi thẳng
  }
  

  if (!onLine) {
    if (lastError < 0) return 0;
    else return 4000;
  }
  if (sum == 0) return 2000;
  return weightedSum / sum;
}

void runBlindForward(int duration) {
  setMotorSpeed(BASE_SPEED, BASE_SPEED);
  delay(duration); 
}

void calibrateSensors() {
  for (int i = 0; i < 5; i++) {
    int val = analogRead(sensorPins[i]);
    if (val > sensorMax[i]) sensorMax[i] = val;
    if (val < sensorMin[i]) sensorMin[i] = val;
  }
}

void turnRight() {
  setMotorSpeed(110, -110);
  delay(600);
  setMotorSpeed(0, 0);
  delay(100);
  lastError = 0; 
  accumulatedError = 0;
}
void turnLeft() {
  setMotorSpeed(-100, 100);
  delay(650);
  setMotorSpeed(0, 0);
  delay(100);
  lastError = 0; 
  accumulatedError = 0;
}
void turnLeft45(){
  setMotorSpeed(-100, 100);
  delay(300);
  setMotorSpeed(0, 0);
  delay(100);
  lastError = 0; 
  accumulatedError = 0;
}
void turnLeft30(){
  setMotorSpeed(-100, 100);
  delay(280);
  setMotorSpeed(0, 0);
  delay(100);
  lastError = 0; 
  accumulatedError = 0;
}
void turnLeft23(){
  setMotorSpeed(-100, 100);
  delay(150);
  setMotorSpeed(0, 0);
  delay(100);
  lastError = 0; 
  accumulatedError = 0;
}

void turnLeft12(){
  setMotorSpeed(-100, 100);
  delay(85);
  setMotorSpeed(0, 0);
  delay(100);
  lastError = 0; 
  accumulatedError = 0;
}

void setMotorSpeed(int left, int right) {
  if (left > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, left);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, abs(left));
  }
  if (right > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, right);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, abs(right));
  }
}