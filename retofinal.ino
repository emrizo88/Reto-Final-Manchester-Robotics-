#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <Arduino.h>

// Definición de pines para el control del motor y el encoder
#define IN1 27
#define IN2 14
#define ENA 12
#define encoderPinA 26
#define encoderPinB 25

// Variables para el conteo de pulsos del encoder
volatile int pulsos = 0;
const float pulsosPorRevolucion = 240.0;

// Variable para almacenar el valor deseado de RPM recibido del setpoint
volatile float refRPM = 0; // Variable para almacenar el valor deseado de RPM recibido del setpoint

// Declaración de estructuras para ROS2
rcl_publisher_t angular_speed_publisher;
std_msgs__msg__Float32 angular_speed_msg;

rcl_subscription_t setpoint_subscriber;
std_msgs__msg__Float32 setpoint_msg;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_timer_t timer;

// Configuración de PWM para control del motor
const int freq = 10000;
const int ledChannel = 0;
const int resolution = 8;

// Variable para el ciclo de trabajo del PWM
float pwm_cycle = 0.0;

// Parámetros para el control PID
const float Kp = 2.229;
const float Ki = 0.323;
const float Kd = 0.08075;
const float Ts = 0.1;

// Variables para el cálculo PID
double integral = 0;
double previousError = 0;

// Función ISR para contar los pulsos del encoder
void ISR_contadorPulsos() {
  if (digitalRead(encoderPinB)== HIGH) {
    pulsos++;
  } else {
    pulsos--;
  }
}

// Función para calcular el control PID
double computePID(double setpoint, double measuredValue) {
  double error = setpoint - measuredValue;
  double Pout = Kp * error;
  integral += error * Ts;
  double Iout = Ki * integral;
  double derivative = (error - previousError) / Ts;
  double Dout = Kd * derivative;
  double output = Pout + Iout + Dout;
  previousError = error;
  return output;
}

// Callback del timer para calcular y ajustar la velocidad del motor
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    unsigned long currentTime = millis();
    static unsigned long lastTime = 0;
    float deltaTime = (currentTime - lastTime) / 1000.0; // Conversión a segundos

  

    if (deltaTime == 0) return; // Prevenir división por cero
    float velocidadAngular = (pulsos / pulsosPorRevolucion) * (2.0 * PI) / deltaTime;
    lastTime = currentTime;

    pulsos = 0; // Reiniciar contador de pulsos para la próxima medición

    float refAng= (refRPM*140)*((2.0 * PI)/60);

    double controlSignal = computePID(refAng, velocidadAngular); // Asume que refRPM representa ahora un objetivo de velocidad angular


    // Limita el valor de controlSignal para evitar valores fuera del rango de PWM
    pwm_cycle = max(-1.0f, min(static_cast<float>(controlSignal) / 255.0f, 1.0f));
    
    // Establece la dirección del motor basado en refRPM

    if (refRPM >= 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }



    // Ajusta el ciclo de trabajo del PWM según la señal de control

    int dutyCycle = static_cast<int>((controlSignal)*125);
    dutyCycle = (int)((controlSignal*70)*225);
    ledcWrite(ledChannel, dutyCycle);

    // Publica la velocidad angular calculada

    angular_speed_msg.data = controlSignal;

    rcl_publish(&angular_speed_publisher, &angular_speed_msg, NULL);

  }
}


void setpoint_callback(const void* msgin) {
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
  refRPM = msg->data;
}

void setup() {
  set_microros_transports();


  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);


  attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR_contadorPulsos, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), ISR_contadorPulsos, CHANGE); // Usamos CHANGE para detectar ambos flancos.


  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ENA, ledChannel);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "control_node", "", &support);

  rclc_publisher_init_default(&angular_speed_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/controller/angular_speed");
  rclc_subscription_init_default(&setpoint_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/micro_ros_esp32/setpoint");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &setpoint_subscriber, &setpoint_msg, &setpoint_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}