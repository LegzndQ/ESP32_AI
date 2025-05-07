#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6050.h>
#include <IRrecv.h>
#include <IRutils.h>

#define IR_RECV_PIN 19 // 红外接收端口
#define MOTOR_A_PWM1 47
#define MOTOR_A_PWM2 48
#define MOTOR_B_PWM1 38
#define MOTOR_B_PWM2 39
#define HALL_A1 3
#define HALL_A2 46
#define HALL_B1 40
#define HALL_B2 41
#define MPU_INT_PIN 12 // MPU6050 中断引脚连接到 ESP32 的 IO12

const char* ssid = "Legznd";
const char* password = "12345678";
const char* serverIP = "172.20.10.11";  // 电脑端 IP
const int udpPort = 12345;

WiFiUDP udp;
MPU6050 mpu;
IRrecv irrecv(IR_RECV_PIN);
decode_results results;

volatile bool mpuInterrupt = false; // 标志位，用于标记 MPU6050 中断触发
bool collectingData = false;        // 标志位，是否正在采集数据

int motorAPWM = 0; // 电机 A 的 PWM 值
int motorBPWM = 0; // 电机 B 的 PWM 值
const int pwmStep = 10; // 每次调整 PWM 的增量

// 中断服务程序（ISR）
void IRAM_ATTR onMotionDetected() {
    mpuInterrupt = true; // 设置中断标志
}

// 添加全局变量用于记录 HALL 传感器的脉冲计数
volatile unsigned long hallA1Count = 0;
volatile unsigned long hallA2Count = 0;
volatile unsigned long hallB1Count = 0;
volatile unsigned long hallB2Count = 0;

// 中断服务程序（ISR）用于记录 HALL 传感器的脉冲
void IRAM_ATTR onHallA1Pulse() {
    hallA1Count++;
}

void IRAM_ATTR onHallA2Pulse() {
    hallA2Count++;
}

void IRAM_ATTR onHallB1Pulse() {
    hallB1Count++;
}

void IRAM_ATTR onHallB2Pulse() {
    hallB2Count++;
}

// 卡尔曼滤波器结构体
struct KalmanFilter {
    float Q_angle; // 过程噪声协方差（角度）
    float Q_bias;  // 过程噪声协方差（偏差）
    float R_measure; // 测量噪声协方差

    float angle; // 估计的角度
    float bias;  // 估计的偏差
    float rate;  // 未校正的角速度

    float P[2][2]; // 误差协方差矩阵
};

// 初始化卡尔曼滤波器
void initKalmanFilter(KalmanFilter &kf) {
    kf.Q_angle = 0.001f;
    kf.Q_bias = 0.003f;
    kf.R_measure = 0.03f;

    kf.angle = 0.0f;
    kf.bias = 0.0f;
    kf.P[0][0] = 0.0f;
    kf.P[0][1] = 0.0f;
    kf.P[1][0] = 0.0f;
    kf.P[1][1] = 0.0f;
}

// 卡尔曼滤波更新函数
float updateKalmanFilter(KalmanFilter &kf, float newAngle, float newRate, float dt) {
    // 预测阶段
    kf.rate = newRate - kf.bias;
    kf.angle += dt * kf.rate;

    kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + kf.Q_angle);
    kf.P[0][1] -= dt * kf.P[1][1];
    kf.P[1][0] -= dt * kf.P[1][1];
    kf.P[1][1] += kf.Q_bias * dt;

    // 更新阶段
    float S = kf.P[0][0] + kf.R_measure; // 估计误差
    float K[2]; // 卡尔曼增益
    K[0] = kf.P[0][0] / S;
    K[1] = kf.P[1][0] / S;

    float y = newAngle - kf.angle; // 角度偏差
    kf.angle += K[0] * y;
    kf.bias += K[1] * y;

    float P00_temp = kf.P[0][0];
    float P01_temp = kf.P[0][1];

    kf.P[0][0] -= K[0] * P00_temp;
    kf.P[0][1] -= K[0] * P01_temp;
    kf.P[1][0] -= K[1] * P00_temp;
    kf.P[1][1] -= K[1] * P01_temp;

    return kf.angle;
}

// 初始化卡尔曼滤波器
KalmanFilter kalmanAccelX, kalmanAccelY, kalmanAccelZ;
KalmanFilter kalmanGyroX, kalmanGyroY, kalmanGyroZ;

// 控制小车运动的函数
void setMotorPWM(int pwmA, int pwmB) {
    motorAPWM = pwmA;
    motorBPWM = pwmB;

    if (pwmA >= 0) {
        analogWrite(MOTOR_A_PWM1, pwmA);
        analogWrite(MOTOR_A_PWM2, 0);
    } else {
        analogWrite(MOTOR_A_PWM1, 0);
        analogWrite(MOTOR_A_PWM2, -pwmA);
    }

    if (pwmB >= 0) {
        analogWrite(MOTOR_B_PWM1, pwmB);
        analogWrite(MOTOR_B_PWM2, 0);
    } else {
        analogWrite(MOTOR_B_PWM1, 0);
        analogWrite(MOTOR_B_PWM2, -pwmB);
    }
}

void stopCar() {
    setMotorPWM(0, 0);
}

void resetHallCounts() {
    hallA1Count = 0;
    hallA2Count = 0;
    hallB1Count = 0;
    hallB2Count = 0;
}

void sendData(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, 
    float motorASpeed, float motorBSpeed, int pwmA, int pwmB) {
    char buffer[200];
    snprintf(buffer, sizeof(buffer), "%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d",
    millis(), accelX, accelY, accelZ, gyroX, gyroY, gyroZ, motorASpeed, motorBSpeed, pwmA, pwmB);
    udp.beginPacket(serverIP, udpPort);
    udp.print(buffer);
    udp.endPacket();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 20);  // 指定 ESP32-S3 的 I2C 引脚
    mpu.initialize();
    irrecv.enableIRIn(); // 启动红外接收
    Serial.println("IR 接收器已启动");

    pinMode(MOTOR_A_PWM1, OUTPUT);
    pinMode(MOTOR_A_PWM2, OUTPUT);
    pinMode(MOTOR_B_PWM1, OUTPUT);
    pinMode(MOTOR_B_PWM2, OUTPUT);
    pinMode(HALL_A1, INPUT_PULLUP);
    pinMode(HALL_A2, INPUT_PULLUP);
    pinMode(HALL_B1, INPUT_PULLUP);
    pinMode(HALL_B2, INPUT_PULLUP);

    // 配置 HALL 传感器中断
    attachInterrupt(digitalPinToInterrupt(HALL_A1), onHallA1Pulse, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_A2), onHallA2Pulse, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_B1), onHallB1Pulse, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_B2), onHallB2Pulse, RISING);

    // 配置 MPU6050 中断
    pinMode(MPU_INT_PIN, INPUT_PULLUP); // 设置 MPU6050 INT 引脚为输入
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onMotionDetected, FALLING);

    // 配置 MPU6050 的运动检测中断
    mpu.setMotionDetectionThreshold(0); // 设置运动检测阈值
    mpu.setMotionDetectionDuration(5);  // 设置运动检测持续时间
    mpu.setIntMotionEnabled(true);      // 启用运动检测中断

    // if (mpu.testConnection()) {
    //     Serial.println("MPU6050 连接成功！");
    // } else {
    //     Serial.println("MPU6050 连接失败！");
    //     while (1); // 连接失败，停止程序
    // }

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi 连接成功");

    udp.begin(udpPort);

    // 初始化卡尔曼滤波器
    initKalmanFilter(kalmanAccelX);
    initKalmanFilter(kalmanAccelY);
    initKalmanFilter(kalmanAccelZ);
    initKalmanFilter(kalmanGyroX);
    initKalmanFilter(kalmanGyroY);
    initKalmanFilter(kalmanGyroZ);
}

void loop() {
    // 检查 MPU6050 中断标志
    if (mpuInterrupt) {
        mpuInterrupt = false; // 清除中断标志

        // 读取 MPU6050 数据
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // 将原始数据转换为标准单位
        float accelX = ax / 16384.0f; // 加速度计数据（单位：g）
        float accelY = ay / 16384.0f;
        float accelZ = az / 16384.0f;
        float gyroX = gx / 131.0f;    // 陀螺仪数据（单位：°/s）
        float gyroY = gy / 131.0f;
        float gyroZ = gz / 131.0f;

        // 计算时间间隔
        static unsigned long lastTime = 0;
        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;

        // 应用卡尔曼滤波
        float filteredAccelX = updateKalmanFilter(kalmanAccelX, accelX, gyroX, dt);
        float filteredAccelY = updateKalmanFilter(kalmanAccelY, accelY, gyroY, dt);
        float filteredAccelZ = updateKalmanFilter(kalmanAccelZ, accelZ, gyroZ, dt);
        float filteredGyroX = updateKalmanFilter(kalmanGyroX, accelX, gyroX, dt);
        float filteredGyroY = updateKalmanFilter(kalmanGyroY, accelY, gyroY, dt);
        float filteredGyroZ = updateKalmanFilter(kalmanGyroZ, accelZ, gyroZ, dt);

        // 打印滤波后的数据
        Serial.printf("滤波后数据: AccelX=%.2f, AccelY=%.2f, AccelZ=%.2f, GyroX=%.2f, GyroY=%.2f, GyroZ=%.2f\n",
                      filteredAccelX, filteredAccelY, filteredAccelZ, filteredGyroX, filteredGyroY, filteredGyroZ);
        
        static unsigned long lastHallTime = 0;
        //unsigned long currentTime = millis();
        unsigned long timeDiff = currentTime - lastHallTime;

        // 每 50ms 计算一次角速度
        if (timeDiff >= 50) {
            lastHallTime = currentTime;

            // 计算角速度（一圈7个脉冲）
            float motorASpeed = hallA1Count * (360.0 / 7 / timeDiff * 1000 / 30); // 单位：度/秒
            float motorBSpeed = hallB1Count * (360.0 / 7 / timeDiff * 1000 / 30); // 单位：度/秒

            // 清零脉冲计数
            resetHallCounts();

            // 如果正在采集数据，将数据通过 Wi-Fi 发送到电脑
            if (collectingData) {
                sendData(filteredAccelX, filteredAccelY, filteredAccelZ, filteredGyroX, filteredGyroY, filteredGyroZ, 
                    motorASpeed, motorBSpeed, motorAPWM, motorBPWM);
            }
        }
    }

    // 红外遥控处理
    if (irrecv.decode(&results)) {
        uint64_t value = results.value;
        Serial.printf("接收到的红外信号: 0x%llX\n", value);

        switch (value) {
            case 0xFF30CF: // 增加左轮 PWM
                collectingData = true;
                motorAPWM -= pwmStep;
                setMotorPWM(motorAPWM, motorBPWM);
                break;
            case 0xFF10EF: // 减少左轮 PWM
                collectingData = true;
                motorAPWM += pwmStep;
                setMotorPWM(motorAPWM, motorBPWM);
                break;
            case 0xFF7A85: // 增加右轮 PWM
                collectingData = true;
                motorBPWM -= pwmStep;
                setMotorPWM(motorAPWM, motorBPWM);
                break;
            case 0xFF5AA5: // 减少右轮 PWM
                collectingData = true;
                motorBPWM += pwmStep;
                setMotorPWM(motorAPWM, motorBPWM);
                break;
            case 0xFFA25D: // 停止
                collectingData = false;
                stopCar();
                break;
            case 0xFF6897: // 设置两轮 PWM 为 -150，负数为正转
                collectingData = true;
                motorAPWM = 150;
                motorBPWM = 150;
                setMotorPWM(-motorAPWM, -motorBPWM);
                break;
        }
        irrecv.resume();
    }

    delay(50); // 延迟以降低 CPU 占用率
}