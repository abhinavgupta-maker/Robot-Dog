#include <Robot_Dog_inferencing.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "driver/i2s.h"
#include <math.h>

// =====================================================
// PCA9685
// =====================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define PCA_SDA 18
#define PCA_SCL 19

#define SERVOMIN 110
#define SERVOMAX 560

// 0,1 = FR, 2,3 = RR, 4,5 = FL, 6,7 = RL
int servoChannels[8] = {
  0, 1, 2, 3, 4, 5, 6, 7
};


// =====================================================
// LEG DIMENSIONS
// =====================================================

const float L1 = 101.0;
const float L2 = 133.0;


// =====================================================
// INMP441
// =====================================================

#define I2S_PORT I2S_NUM_0

#define I2S_SCK 4
#define I2S_WS  5
#define I2S_SD  17

#define SAMPLE_RATE 16000
#define AUDIO_SAMPLES 16000

int16_t audio_buffer[AUDIO_SAMPLES];


// =====================================================
// IK
// =====================================================

void moveLeg(int hipChannel, int kneeChannel, float x, float y) {

  float D = sqrtf(x * x + y * y);

  // Check if target is reachable
  if (D > (L1 + L2) || D < fabsf(L1 - L2)) {
    Serial.println("IK target unreachable.");
    return;
  }

  // =========================
  // KNEE
  // =========================

  float cosTheta2 =
    (x * x + y * y - L1 * L1 - L2 * L2)
    / (2.0f * L1 * L2);

  cosTheta2 = constrain(cosTheta2, -1.0f, 1.0f);

  float theta2 = acosf(cosTheta2);

  // =========================
  // HIP
  // =========================

  float alpha = atan2f(y, x);

  float cosBeta =
    (D * D + L1 * L1 - L2 * L2)
    / (2.0f * L1 * D);

  cosBeta = constrain(cosBeta, -1.0f, 1.0f);

  float beta = acosf(cosBeta);

  float theta1 = alpha - beta;

  // Convert radians → degrees
  float hip = theta1 * 180.0f / PI;
  float knee = theta2 * 180.0f / PI;

  Serial.print("Hip: ");
  Serial.print(hip);

  Serial.print(" | Knee: ");
  Serial.println(knee);

  // =========================
  // SEND TO SERVOS
  // =========================

  setServoAngle(hipChannel, (int)hip);
  setServoAngle(kneeChannel, (int)knee);
}


// =====================================================
// SERVO
// =====================================================

void setServoAngle(int channel, int angle) {

  angle = constrain(angle, 0, 180);

  int pulse = map(
    angle,
    0,
    180,
    SERVOMIN,
    SERVOMAX
  );

  pwm.setPWM(
    channel,
    0,
    pulse
  );
}


// =====================================================
// STAND
// =====================================================

void stand() {

  Serial.println(">>> STAND");

  setServoAngle(0, 100);
  setServoAngle(1, 20);
  setServoAngle(2, 100);
  setServoAngle(3, 20);

  setServoAngle(4, 100);
  setServoAngle(5, 20);
  setServoAngle(6, 100);
  setServoAngle(7, 20); 


}


// =====================================================
// SIT
// =====================================================

void sit() {

  Serial.println(">>> SIT");

  setServoAngle(0, 100);
  setServoAngle(1, 30);
  setServoAngle(2, 100);
  setServoAngle(3, 30);

  setServoAngle(4, 180);
  setServoAngle(5, 30);
  setServoAngle(6, 100);
  setServoAngle(7, 30);
}


// =====================================================
// I2S SETUP
// =====================================================

void setupMicrophone() {

  i2s_config_t i2s_config = {

    .mode = (i2s_mode_t)(
      I2S_MODE_MASTER |
      I2S_MODE_RX
    ),

    .sample_rate = SAMPLE_RATE,

    .bits_per_sample =
      I2S_BITS_PER_SAMPLE_32BIT,

    .channel_format =
      I2S_CHANNEL_FMT_ONLY_LEFT,

    .communication_format =
      I2S_COMM_FORMAT_STAND_I2S,

    .intr_alloc_flags =
      ESP_INTR_FLAG_LEVEL1,

    .dma_buf_count = 8,

    .dma_buf_len = 256,

    .use_apll = false,

    .tx_desc_auto_clear = false,

    .fixed_mclk = 0
  };


  i2s_pin_config_t pin_config = {

    .bck_io_num = I2S_SCK,

    .ws_io_num = I2S_WS,

    .data_out_num =
      I2S_PIN_NO_CHANGE,

    .data_in_num = I2S_SD
  };


  i2s_driver_install(
    I2S_PORT,
    &i2s_config,
    0,
    NULL
  );


  i2s_set_pin(
    I2S_PORT,
    &pin_config
  );


  i2s_zero_dma_buffer(I2S_PORT);

  Serial.println("INMP441 ready");
}


// =====================================================
// RECORD AUDIO
// =====================================================

void recordAudio() {

  int32_t raw_samples[256];

  size_t bytes_read;

  int samples_recorded = 0;


  while (samples_recorded < AUDIO_SAMPLES) {

    i2s_read(
      I2S_PORT,
      raw_samples,
      sizeof(raw_samples),
      &bytes_read,
      portMAX_DELAY
    );


    int count =
      bytes_read / sizeof(int32_t);


    for (int i = 0; i < count; i++) {

      if (samples_recorded >= AUDIO_SAMPLES)
        break;


      int32_t sample =
        raw_samples[i] >> 8;


      sample =
        constrain(
          sample,
          -32768,
          32767
        );


      audio_buffer[samples_recorded] =
        (int16_t)sample;

      samples_recorded++;
    }
  }
}


// =====================================================
// EDGE IMPULSE CALLBACK
// =====================================================

int get_audio_data(
  size_t offset,
  size_t length,
  float *out_ptr
) {

  for (size_t i = 0; i < length; i++) {

    out_ptr[i] =
      (float)audio_buffer[offset + i];
  }

  return 0;
}


// =====================================================
// VOICE RECOGNITION
// =====================================================

void recognizeVoice() {

  Serial.println();
  Serial.println("Listening...");

  recordAudio();


  signal_t signal;

  signal.total_length =
    AUDIO_SAMPLES;

  signal.get_data =
    get_audio_data;


  ei_impulse_result_t result = { 0 };


  EI_IMPULSE_ERROR res =
    run_classifier(
      &signal,
      &result,
      false
    );


  if (res != EI_IMPULSE_OK) {

    Serial.print("Classifier error: ");
    Serial.println(res);

    return;
  }


  float sitConfidence = 0;
  float standConfidence = 0;


  for (
    size_t ix = 0;
    ix < EI_CLASSIFIER_LABEL_COUNT;
    ix++
  ) {

    Serial.print(
      result.classification[ix].label
    );

    Serial.print(": ");

    Serial.println(
      result.classification[ix].value,
      3
    );


    if (
      strcmp(
        result.classification[ix].label,
        "Sit"
      ) == 0
    ) {

      sitConfidence =
        result.classification[ix].value;
    }


    if (
      strcmp(
        result.classification[ix].label,
        "Stand"
      ) == 0
    ) {

      standConfidence =
        result.classification[ix].value;
    }
  }


  // =================================================
  // DECISION
  // =================================================

  const float THRESHOLD = 0.95;


  if (
    sitConfidence >= THRESHOLD &&
    sitConfidence > standConfidence
  ) {

    sit();
  }


  else if (
    standConfidence >= THRESHOLD &&
    standConfidence > sitConfidence
  ) {

    stand();
  }


  else {

    Serial.println(
      "No confident command."
    );
  }
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(1000);


  Serial.println();
  Serial.println(
    "=============================="
  );

  Serial.println(
    "VOICE CONTROLLED ROBOT"
  );

  Serial.println(
    "=============================="
  );


  // PCA9685
  Wire.begin(
    PCA_SDA,
    PCA_SCL
  );

  pwm.begin();

  pwm.setPWMFreq(50);


  // Microphone
  setupMicrophone();


  // Initial position
  stand();


  delay(1000);


  Serial.println(
    "SYSTEM READY"
  );
}

void smoothServo(int channel, int from, int to, int stepDelay = 15) {

  if (from < to) {
    for (int a = from; a <= to; a++) {
      setServoAngle(channel, a);
      delay(stepDelay);
    }
  } 
  else {
    for (int a = from; a >= to; a--) {
      setServoAngle(channel, a);
      delay(stepDelay);
    }
  }
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // =====================================================
  // STAND
  // =====================================================

  setServoAngle(0, 120);   // FR hip
  setServoAngle(1, 180);   // FR knee

  setServoAngle(2, 150);   // RR hip
  setServoAngle(3, 170);   // RR knee

  setServoAngle(4, 180);   // FL hip
  setServoAngle(5, 0);     // FL knee

  setServoAngle(6, 170);   // RL hip
  setServoAngle(7, 0);     // RL knee

  delay(1000);


  // =====================================================
  // FR
  // =====================================================

  // Lift
  smoothServo(1, 180, 165, 5);

  // FORWARD - reversed direction
  smoothServo(0, 120, 112, 5);

  // Down
  smoothServo(1, 165, 180, 5);

  delay(200);


  // =====================================================
  // RL
  // =====================================================

  smoothServo(7, 0, 10, 5);

  // FORWARD - reversed direction
  smoothServo(6, 170, 178, 5);

  smoothServo(7, 10, 0, 5);

  delay(200);


  // =====================================================
  // FL
  // =====================================================

  smoothServo(5, 0, 10, 5);

  // Try the opposite direction from before
  smoothServo(4, 180, 170, 5);

  smoothServo(5, 10, 0, 5);

  delay(200);


  // =====================================================
  // RR
  // =====================================================

  smoothServo(3, 170, 160, 5);

  // FORWARD - reversed direction
  smoothServo(2, 150, 158, 5);

  smoothServo(3, 160, 170, 5);

  delay(500);
}
