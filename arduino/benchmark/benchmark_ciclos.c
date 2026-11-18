// Mede, em ciclos de CPU do ATmega328P, o custo de cada lei de controle de controladores.h.
// Roda no simulador simavr (preciso em ciclos) ou numa placa real (leia a serial a 9600).
//
//   avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os -I../motor_control
//           benchmark_ciclos.c -o bench.elf -Wl,-u,vfprintf -lprintf_flt -lm
//   simavr -m atmega328p -f 16000000 bench.elf
//
// Mede SÓ a chamada *_update (cálculo da lei de controle). Não inclui leitura do encoder,
// analogWrite nem serial – esses custos aparecem no exec_us do sketch na bancada.
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>
#include "../motor_control/controladores.h"

static int uart_putchar(char c, FILE *f) {
  (void)f;
  while (!(UCSR0A & (1 << UDRE0))) { }
  UDR0 = c;
  return 0;
}
static FILE uart_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

#define N_CHAMADAS 300

static volatile float sumidouro;  // impede o compilador de eliminar as chamadas

typedef float (*FnUpdate)(void *estado, float r, float y);
static float w_pid(void *s, float r, float y) { return pid_update((EstadoPID *)s, r, y); }
static float w_fz(void *s, float r, float y) { return fuzzy_update((EstadoFuzzy *)s, r, y); }
static float w_mlp(void *s, float r, float y) { return mlp_update((EstadoMLP *)s, r, y); }

static void medir(const char *nome, FnUpdate fn, void *estado) {
  uint16_t cmin = 0xFFFF, cmax = 0;
  uint32_t soma = 0;
  float y = 0.0f;
  for (int k = 0; k < N_CHAMADAS; k++) {
    // trajetória plausível: sobe até a referência com ondulação, para exercitar todos os ramos
    y = REF_PADRAO * (1.0f - 1.0f / (1.0f + 0.03f * k)) + 0.02f * REF_PADRAO * ((k % 7) - 3);
    TCNT1 = 0;
    float u = fn(estado, REF_PADRAO, y);
    uint16_t c = TCNT1;
    sumidouro = u;
    if (c < cmin) cmin = c;
    if (c > cmax) cmax = c;
    soma += c;
  }
  // desconta o custo da própria medição (leitura/escrita de TCNT1 ~ alguns ciclos)
  printf("%s: ciclos min=%u med=%lu max=%u | us @16MHz: min=%u med=%lu max=%u\n", nome,
         cmin, soma / N_CHAMADAS, cmax, cmin / 16, soma / N_CHAMADAS / 16, cmax / 16);
}

int main(void) {
  UBRR0H = 0; UBRR0L = 103;              // 9600 baud @ 16 MHz
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  stdout = &uart_out;
  TCCR1A = 0; TCCR1B = (1 << CS10);      // Timer1 sem prescaler: 1 contagem = 1 ciclo

  EstadoPID p; pid_reset(&p);
  EstadoFuzzy f; fuzzy_reset(&f);
  EstadoMLP m; mlp_reset(&m);
  printf("# benchmark %d chamadas, F_CPU=%lu\n", N_CHAMADAS, (unsigned long)F_CPU);
  medir("PID  ", w_pid, &p);
  medir("Fuzzy", w_fz, &f);
  medir("MLP  ", w_mlp, &m);
  printf("# fim\n");
  while (!(UCSR0A & (1 << TXC0))) { }   // espera o último byte sair
  cli();
  sleep_mode();                          // no simavr: dormir sem interrupções encerra a simulação
  for (;;) { }
}
