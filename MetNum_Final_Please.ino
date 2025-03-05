#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

int led = 10; // LED yang akan digunakan

// Variabel untuk menyimpan sudut dan kecepatan sudut
float roll, pitch, yaw;
float roll_prev = 0;
float pitch_prev = 0;
float yaw_prev = 0;

// Variabel untuk menyimpan kecepatan dan akselerasi sebelumnya
float velocity = 0;
float previous_acceleration[3] = {0, 0, 0}; // Array untuk menyimpan akselerasi sebelumnya
float dt = 0.25; // Interval waktu (delay dalam loop)

// Tambahkan variabel untuk mendeteksi jika sensor berhenti
float threshold_stop = 0.1; // Ambang batas untuk mendeteksi berhenti
unsigned long stop_time = 0; // Untuk menghitung waktu berhenti

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer range to +-2G
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // Set gyro range to +- 250 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // Set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  pinMode(led, OUTPUT);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Hitung akselerasi dalam m/s^2
  float acceleration[3] = {
    a.acceleration.x / 9.8066, // Normalisasi ke m/s^2
    a.acceleration.y / 9.8066,
    (a.acceleration.z - 9.8066) / 9.8066
  };

  // Hitung total akselerasi
  float total_acceleration = sqrt(acceleration[0] * acceleration[0] + acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2]);

  // Hitung kecepatan menggunakan metode trapezoidal
  velocity += 0.5 * (total_acceleration + sqrt(previous_acceleration[0] * previous_acceleration[0] + previous_acceleration[1] * previous_acceleration[1] + previous_acceleration[2] * previous_acceleration[2])) * dt;

  // Perbarui akselerasi sebelumnya
  for (int i = 0; i < 3; i++) {
    previous_acceleration[i] = acceleration[i];
  }

  // Reset kecepatan jika akselerasi mendekati nol
  if (total_acceleration < threshold_stop) {
    stop_time += dt * 1000; // Konversi ke milidetik
    if (stop_time > 1000) { // Jika berhenti lebih dari 1 detik
      velocity = 0; // Reset kecepatan
    }
  } else {
    stop_time = 0; // Reset waktu berhenti
  }

  // Tampilkan hasil kecepatan
  Serial.print("Velocity: ");
  Serial.println(velocity);

  // Kontrol LED berdasarkan kecepatan
  float speed_threshold = 8.33; // Ambang batas kecepatan (30 km/h)
  digitalWrite(led, (velocity > speed_threshold) ? HIGH : LOW);

  // Menghitung roll, pitch, dan yaw
  roll = 180 * atan2(a.acceleration.y, sqrt(a.acceleration.x * acceleration[0] + acceleration[2] * acceleration[2])) / M_PI;
  pitch = 180 * atan2(-a.acceleration.x, sqrt(a.acceleration.y * acceleration[1] + acceleration[2] * acceleration[2])) / M_PI;
  yaw = 180 * atan2(a.acceleration.z, sqrt(a.acceleration.x * acceleration[0] + acceleration[1] * acceleration[1])) / M_PI;

  // Menghitung kecepatan sudut menggunakan metode selisih maju
  float roll_rate = (roll - roll_prev) / dt;
  float pitch_rate = (pitch - pitch_prev) / dt;
  float yaw_rate = (yaw - yaw_prev) / dt;

  // Menampilkan hasil roll, pitch, yaw, dan kecepatan sudut
  Serial.print("|Roll: |");
  Serial.print(roll);
  Serial.print("| Pitch: |");
  Serial.print(pitch);
  Serial.print("| Yaw: |");
  Serial.print(yaw);
  Serial.print(" | Roll Rate: ");
  Serial.print(roll_rate);
  Serial.print(" | Pitch Rate: ");
  Serial.print(pitch_rate);
  Serial.print(" | Yaw Rate: ");
  Serial.println(yaw_rate);

  // Simpan sudut saat ini untuk iterasi berikutnya
  roll_prev = roll;
  pitch_prev = pitch;
  yaw_prev = yaw;

  // Delay untuk interval waktu
  delay(500); // Delay untuk interval waktu
}
