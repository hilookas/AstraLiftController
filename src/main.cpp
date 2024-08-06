#include <AccelStepper.h>
#include <Arduino.h>
#include <comm.h>
#include <lwip/sockets.h>

// #define STEPPER_CORRECT_DIR(x) ((x)) // Normal
#define STEPPER_CORRECT_DIR(x) (-(x)) // Reversed

#define STEPPER_PULSE_PER_REV 800 // 800pulse/rev
#define RAIL_MM_PER_REV 10 // 10mm/rev
#define RAIL_MAX_LENGTH_MM 500 // 1000mm length rail // 500mm on desk length
#define STEPPER_MAX_PULSE (RAIL_MAX_LENGTH_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV) // 80000

// Define some steppers and the pins the will use
AccelStepper stepper(AccelStepper::DRIVER, 2, 4);

bool reach_zero = false;

uint32_t last_action_time;

#define TIMER_TIMEOUT_US 40

esp_timer_handle_t timer;

void timer_callback(void *arg) {
  if (digitalRead(15)) {
    // Serial.println("limit reach");
    stepper.setCurrentPosition(0);
    reach_zero = true;
    stepper.moveTo(STEPPER_CORRECT_DIR(10));
  }
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  
  ESP_ERROR_CHECK(esp_timer_start_once(timer, TIMER_TIMEOUT_US));
}

void setup() {
  Serial.begin(921600);

  pinMode(15, INPUT);

  stepper.setMinPulseWidth(2); // 防止丢步 驱动器要求最小 2us 持续时间
  stepper.setMaxSpeed(STEPPER_PULSE_PER_REV * 10); // 空载最大 STEPPER_PULSE_PER_REV * 12.5 = 10000 = 750rpm
  stepper.setAcceleration(STEPPER_PULSE_PER_REV * 10);

  // https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/esp_timer/main/esp_timer_example_main.c
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_callback
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_once(timer, TIMER_TIMEOUT_US));

  stepper.setMaxSpeed(STEPPER_PULSE_PER_REV * 2);
  stepper.moveTo(STEPPER_CORRECT_DIR(-STEPPER_MAX_PULSE));
  while (!reach_zero) delay(1);
  Serial.println("reach zero");
  stepper.setMaxSpeed(STEPPER_PULSE_PER_REV * 10); // 空载最大 10000 750rpm

  comm_init();

  last_action_time = millis();
}

void loop() {
  comm_type_t type;
  uint8_t buf[20];
  bool ret = comm_recv_poll_last(&type, buf);

  if (!ret) {
    // TODO PING PONG
    if (type == COMM_TYPE_PING) {
      comm_send_blocking(COMM_TYPE_PONG, buf);
    } else if (type == COMM_TYPE_PONG) {
      ;
    } else if (type == COMM_TYPE_CTRL) {
      last_action_time = millis();

      uint32_t pos[4];
      memcpy(pos, buf, sizeof pos);
      for (int i = 0; i < 4; ++i) pos[i] = ntohl(pos[i]);

      if (pos[0] > STEPPER_MAX_PULSE) {
        Serial.println("out of limit");
      } else {
        stepper.moveTo(STEPPER_CORRECT_DIR(pos[0]));
      }
      pos[0] = STEPPER_CORRECT_DIR(stepper.currentPosition());

      for (int i = 0; i < 4; ++i) pos[i] = htonl(pos[i]);
      comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
    } else {
      Serial.println("Wrong Cmd");
    }
  }
  uint32_t this_action_time = millis();
  if (this_action_time - last_action_time > 10) { // 每隔 10ms执行一次
    last_action_time = this_action_time;
    
    // Serial.println("No ctrl");

    uint32_t pos[4];
    pos[0] = STEPPER_CORRECT_DIR(stepper.currentPosition());

    for (int i = 0; i < 4; ++i) pos[i] = htonl(pos[i]);
    comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
  }

  delay(1);
}