// Controle de velocidade de motor DC – Arduino Uno + L298N + encoder em quadratura.
//
// Um único sketch para os 4 modos, com a MESMA aquisição, o MESMO período de amostragem
// e o MESMO acionamento – a única coisa que muda entre os ensaios é a lei de controle.
//
//   MODO_CONTROLE 0  ensaio em malha aberta (degraus de tensão) para identificar o motor
//   MODO_CONTROLE 1  PID
//   MODO_CONTROLE 2  Fuzzy PI incremental
//   MODO_CONTROLE 3  MLP (rede neural clonada do PID)
//
// Saída serial (115200 baud), uma linha CSV por amostra:
//   t_ms,ref,w_rad_s,u_V,exec_us,estouros
// exec_us = tempo de leitura do encoder + cálculo da lei de controle + escrita do PWM,
// medido com micros() (resolução de 4 us no Uno). É o número a citar como
// "tempo de execução do controlador" – medido, não estimado.
// estouros = nº acumulado de amostras em que o laço inteiro (inclusive a serial) não coube
// em TS. Se for > 0, o período de amostragem real não é o simulado: os resultados não valem.
//
// Comandos pela serial:  r<valor>  muda a referência [rad/s]  (ex.: r20.5)
//                        s         para o motor (referência 0)
//
// Ligações: ver docs/hardware.md.  Ganhos/tabela/pesos: parametros_gerados.h (gerado).

#include "parametros_gerados.h"
#include "controladores.h"

// ============================== CONFIGURAÇÃO =================================
#define MODO_CONTROLE 1

#define PINO_PWM   9     // ENA da L298N (retire o jumper de ENA)
#define PINO_IN1   7
#define PINO_IN2   8
#define PINO_ENC_A 2     // INT0 – canal A do encoder
#define PINO_ENC_B 3     // INT1 – canal B do encoder

// Contagens por volta do EIXO DE SAÍDA com decodificação x4 (as duas bordas dos dois canais):
//   ENC_CPR = 4 x PPR do encoder (eixo do motor) x relação de redução.  AJUSTE PARA O SEU MOTOR.
// Resolução de velocidade = 2*pi / (ENC_CPR * Ts). Ela precisa ser bem menor que a faixa de
// tolerância das métricas (ex.: 2 % da referência), senão a acomodação nunca é detectada:
// com 600 contagens/volta a 100 Hz, 1 contagem = 1,05 rad/s – 5 % de uma referência de 20 rad/s.
#define ENC_CPR 2400.0f
// Se a velocidade medida sair negativa com tensão positiva, troque para -1.
#define ENC_SENTIDO 1

// Ensaio de identificação (MODO 0): fração de V_MAX e duração de cada degrau [ms]
const float ENSAIO_FRAC[] = {0.0f, 0.5f, 0.9f, 0.3f, 0.0f};
const unsigned long ENSAIO_MS[] = {200, 2000, 2000, 2000, 2000};
// ============================================================================

volatile long contagem = 0;
volatile uint8_t estado_enc = 0;

// Decodificação em quadratura x4: tabela indexada por (estado anterior << 2) | estado atual.
// Transições inválidas (pulo de dois estados, ruído) contam 0.
const int8_t TAB_QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void isrEncoder() {
  uint8_t s = (digitalRead(PINO_ENC_A) << 1) | digitalRead(PINO_ENC_B);
  contagem += ENC_SENTIDO * TAB_QUAD[(estado_enc << 2) | s];
  estado_enc = s;
}

void aplicarTensao(float u) {
  int pwm = (int)(fabs(u) / V_MAX * 255.0f + 0.5f);
  if (pwm > 255) pwm = 255;
  if (u >= 0) { digitalWrite(PINO_IN1, HIGH); digitalWrite(PINO_IN2, LOW); }
  else        { digitalWrite(PINO_IN1, LOW);  digitalWrite(PINO_IN2, HIGH); }
  analogWrite(PINO_PWM, pwm);
}

EstadoPID   est_pid;
EstadoFuzzy est_fz;
EstadoMLP   est_mlp;

float ref = 0.0f;
long contagem_ant = 0;
unsigned long t0_us, proximo_us;
unsigned long estouros = 0;   // amostras em que o laço não coube em TS
bool degrau_aplicado = false;

void lerComandos() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') ref = Serial.parseFloat();
    else if (c == 's') ref = 0.0f;
  }
}

void setup() {
  pinMode(PINO_IN1, OUTPUT);
  pinMode(PINO_IN2, OUTPUT);
  pinMode(PINO_PWM, OUTPUT);
  pinMode(PINO_ENC_A, INPUT_PULLUP);
  pinMode(PINO_ENC_B, INPUT_PULLUP);
  estado_enc = (digitalRead(PINO_ENC_A) << 1) | digitalRead(PINO_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PINO_ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_ENC_B), isrEncoder, CHANGE);
  aplicarTensao(0);
  Serial.begin(115200);
  pid_reset(&est_pid);
  fuzzy_reset(&est_fz);
  mlp_reset(&est_mlp);
  Serial.print(F("# modo=")); Serial.print(MODO_CONTROLE);
  Serial.print(F(" Ts_ms=")); Serial.print(TS_S * 1000.0f);
  Serial.print(F(" cpr=")); Serial.print(ENC_CPR);
  Serial.print(F(" vmax=")); Serial.println(V_MAX);
  Serial.println(F("t_ms,ref,w_rad_s,u_V,exec_us,estouros"));
  t0_us = micros();
  proximo_us = t0_us + TS_US;
}

float tensaoEnsaio(unsigned long t_ms) {
  unsigned long acum = 0;
  for (unsigned int k = 0; k < sizeof(ENSAIO_MS) / sizeof(ENSAIO_MS[0]); k++) {
    acum += ENSAIO_MS[k];
    if (t_ms < acum) return ENSAIO_FRAC[k] * V_MAX;
  }
  return -1.0f;  // fim do ensaio
}

void loop() {
  // se a iteração anterior (controle + serial) passou do próximo instante, houve estouro
  if ((long)(micros() - proximo_us) > 0) estouros++;
  while ((long)(micros() - proximo_us) < 0) { }   // espera o próximo instante de amostragem
  unsigned long inicio = micros();
  proximo_us += TS_US;

  noInterrupts();
  long c = contagem;
  interrupts();
  float w = (float)(c - contagem_ant) * (2.0f * PI) / (ENC_CPR * TS_S);
  contagem_ant = c;
  unsigned long t_ms = (inicio - t0_us) / 1000UL;

  float u;
#if MODO_CONTROLE == 0
  u = tensaoEnsaio(t_ms);
  if (u < 0) { aplicarTensao(0); Serial.println(F("# fim do ensaio")); while (true) { } }
  ref = u;
#else
  if (t_ms < 500) ref = 0.0f;                          // 0,5 s parado antes do degrau
  else if (!degrau_aplicado) { ref = REF_PADRAO; degrau_aplicado = true; }
  else lerComandos();
#if MODO_CONTROLE == 1
  u = pid_update(&est_pid, ref, w);
#elif MODO_CONTROLE == 2
  u = fuzzy_update(&est_fz, ref, w);
#elif MODO_CONTROLE == 3
  u = mlp_update(&est_mlp, ref, w);
#else
#error "MODO_CONTROLE invalido"
#endif
#endif
  aplicarTensao(u);
  unsigned long exec_us = micros() - inicio;

  Serial.print(t_ms); Serial.print(',');
  Serial.print(ref, 4); Serial.print(',');
  Serial.print(w, 4); Serial.print(',');
  Serial.print(u, 4); Serial.print(',');
  Serial.print(exec_us); Serial.print(',');
  Serial.println(estouros);
}
