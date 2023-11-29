#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2 // Pino de dados do sensor DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Configuração do PID
double setpoint = 25.0; // Temperatura desejada em graus Celsius
double input, output;
double Kp = 5; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error, lastError;
double cumError, rateError;

// Definindo os limites de saída do PID
double outMin = 0;
double outMax = 255;

void setup() {
  Serial.begin(9600);
  sensors.begin();
}

void loop() {
  sensors.requestTemperatures(); // Solicita a leitura da temperatura

  // Lê a temperatura atual do sensor DS18B20
  double currentTemperature = sensors.getTempCByIndex(0);

  // Chama a função do PID
  PID_Controller(currentTemperature);

  // Exibe informações no Serial Monitor
  Serial.print("Temperatura Atual: ");
  Serial.print(currentTemperature);
  Serial.print(" °C | Saída do PID: ");
  Serial.println(output);

  delay(1000); // Aguarda um segundo antes de realizar a próxima leitura
}

void PID_Controller(double currentTemperature) {
  // Calcula o erro
  error = setpoint - currentTemperature;

  // Calcula o tempo decorrido
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  // Calcula as componentes PID
  double P = Kp * error;
  cumError += error * elapsedTime;
  double I = Ki * cumError;
  rateError = (error - lastError) / elapsedTime;
  double D = Kd * rateError;

  // Calcula a saída PID
  output = P + I + D;

  // Limita a saída do PID dentro dos limites especificados
  output = constrain(output, outMin, outMax);

  // Aplica a saída do PID para controlar algum dispositivo (por exemplo, um atuador)
  // Neste exemplo, apenas exibimos a saída no Serial Monitor
  // Você deve adaptar esta parte para controlar seu sistema específico
  // (por exemplo, controle de um relé para ligar/desligar um aquecedor)
  // Substitua este trecho pelo código apropriado para o seu hardware específico.
  // digitalWrite(ACAO_PINO, output);

  // Atualiza variáveis para a próxima iteração
  lastError = error;
  previousTime = currentTime;
}
