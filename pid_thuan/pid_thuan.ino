// --- KHAI BÁO CHÂN KẾT NỐI (Theo yêu cầu) ---
#define PWMA 5
#define AIN1 7
#define AIN2 8
#define PWMB 6
#define BIN1 9
#define BIN2 10
#define STBY 4

// Cảm biến từ Trái (0) qua Phải (4)
const int sensorPins[5] = {A0, A1, A2, A3, A4};

// --- CẤU HÌNH THÔNG SỐ PID & ĐỘNG CƠ ---
// Bạn hãy thay đổi các giá trị Kp, Ki, Kd dưới đây để xe chạy mượt nhất
float Kp = 0.15;  // Hệ số P: Phản ứng nhanh với lỗi (Quan trọng nhất)
float Ki = 0.00;  // Hệ số I: Cộng dồn lỗi (Thường để 0 với xe dò line đơn giản)
float Kd = 3.5;  // Hệ số D: Giảm rung lắc, dự đoán lỗi tương lai//2.5

// Tốc độ cơ bản
// Vì nguồn 13V khá mạnh, nên để BASE_SPEED thấp (khoảng 100-150)
int BASE_SPEED = 130; //100
int MAX_SPEED = 200;  // Giới hạn PWM (0-255)//180

// --- BIẾN TOÀN CỤC ---
int sensorMin[5] = {1023, 1023, 1023, 1023, 1023};
int sensorMax[5] = {0, 0, 0, 0, 0};
int sensorValues[5];
int lastError = 0;

void setup() {
  // Cấu hình chân Motor
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, HIGH); // Bật Driver TB6612FNG

  // Cấu hình chân Cảm biến
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Đèn LED báo hiệu trạng thái
  pinMode(13, OUTPUT);

  // --- QUÁ TRÌNH CALIBRATION (CÂN CHỈNH) ---
  // Trong 4 giây này, hãy quét cảm biến qua lại vạch đen và nền trắng
  digitalWrite(13, HIGH); // Bật đèn báo bắt đầu calib
  unsigned long startTime = millis();
  while (millis() - startTime < 4000) {
    calibrateSensors();
  }
  digitalWrite(13, LOW); // Tắt đèn, bắt đầu chạy
  delay(1000); // Chờ 1s trước khi chạy
}

void loop() {
  // 1. Đọc vị trí line
  int position = readLine();

  // 2. Tính toán vị trí lỗi (Error)
  // Vị trí mong muốn là 2000 (giữa line). 
  // Error = Vị trí hiện tại - Vị trí mong muốn
  int error = position - 2000;

  // 3. Tính toán PID
  int P = error;
  int I = I + error; 
  int D = error - lastError;
  
  lastError = error; // Lưu lỗi hiện tại cho lần sau

  int motorSpeedAdjustment = (Kp * P) + (Ki * I) + (Kd * D);

  // 4. Điều khiển động cơ
  int leftSpeed = BASE_SPEED + motorSpeedAdjustment;
  int rightSpeed = BASE_SPEED - motorSpeedAdjustment;

  // Kẹp giá trị tốc độ trong khoảng cho phép
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotorSpeed(leftSpeed, rightSpeed);
}

// --- HÀM ĐỌC GIÁ TRỊ VÀ TÍNH TOÁN VỊ TRÍ ---
int readLine() {
  long weightedSum = 0;
  long sum = 0;
  bool onLine = false;

  for (int i = 0; i < 5; i++) {
    int val = analogRead(sensorPins[i]);
    
    // Map giá trị từ 0 đến 1000 dựa trên min/max đã calib
    // Nếu val < min -> 0, val > max -> 1000
    int calibratedVal = map(val, sensorMin[i], sensorMax[i], 0, 1000);
    calibratedVal = constrain(calibratedVal, 0, 1000);

    // Với line đen nền trắng, giá trị cao là đen. 
    // Nếu cảm biến của bạn ngược lại (trắng cao, đen thấp), hãy đổi công thức map.
    // Giả sử: Đen (hấp thụ IR) -> Giá trị Analog cao (hoặc thấp tùy mạch).
    // Code này giả định giá trị càng CAO thì càng gần màu ĐEN.
    
    // Lọc nhiễu nhỏ
    if (calibratedVal > 200) onLine = true; 

    sensorValues[i] = calibratedVal;
    
    // Tính trung bình trọng số:
    // Sensor 0: 0 * value
    // Sensor 1: 1000 * value
    // Sensor 2: 2000 * value (Giữa)
    // Sensor 3: 3000 * value
    // Sensor 4: 4000 * value
    weightedSum += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;
  }

  // Xử lý trường hợp mất line hoàn toàn
  if (!onLine) {
    // Nếu lần cuối error < 0 (lệch trái) -> Giữ nguyên giá trị cực trái (0)
    // Nếu lần cuối error > 0 (lệch phải) -> Giữ nguyên giá trị cực phải (4000)
    if (lastError < 0) return 0;
    else return 4000;
  }

  // Trả về vị trí trung bình (0 đến 4000)
  return weightedSum / sum;
}

// --- HÀM CÂN CHỈNH CẢM BIẾN ---
void calibrateSensors() {
  for (int i = 0; i < 5; i++) {
    int val = analogRead(sensorPins[i]);
    if (val > sensorMax[i]) sensorMax[i] = val;
    if (val < sensorMin[i]) sensorMin[i] = val;
  }
}

// --- HÀM ĐIỀU KHIỂN MOTOR (DRIVER TB6612FNG) ---
void setMotorSpeed(int left, int right) {
  // Motor A (Trái)
  if (left > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, left);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(left));
  }

  // Motor B (Phải)
  if (right > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, right);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(right));
  }
}

