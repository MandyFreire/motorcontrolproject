# Hardware

## Componentes

| Item | Observação |
|---|---|
| Arduino Uno (ATmega328P, 16 MHz) | 2 KB de SRAM, 32 KB de flash, sem FPU |
| Ponte H L298N | bipolar: perde ~2 V ou mais em carga – meça a tensão real no motor |
| Motor DC **com encoder em quadratura** | **obrigatório**: é a única medição de velocidade do sistema |
| Fonte 12 V | dimensionada para a corrente de partida do motor |

O encoder não é opcional. Sem ele não existe realimentação de velocidade e, portanto,
não existe controle em malha fechada. A versão anterior lia `analogRead(A0)/10` e
chamava o resultado de RPM, sem nenhum sensor ligado em A0.

## Ligações

| L298N / encoder | Arduino | Por quê |
|---|---|---|
| ENA | D9 (PWM, Timer1) | **retire o jumper de ENA**, senão o PWM não tem efeito |
| IN1 | D7 | sentido de giro – o firmware escreve HIGH/LOW a cada amostra |
| IN2 | D8 | com IN1 = IN2 a L298N freia o motor (a versão anterior nunca escrevia esses pinos) |
| Encoder canal A | D2 (INT0) | interrupção nas duas bordas |
| Encoder canal B | D3 (INT1) | interrupção nas duas bordas → decodificação ×4 |
| Encoder VCC / GND | 5 V / GND | |
| **GND da L298N** | **GND do Arduino** | **referência comum obrigatória**: sem ela os sinais IN1/IN2/ENA flutuam |
| 12 V da fonte | +12 V da L298N | não alimente o motor pelo 5 V do Arduino |

## Parâmetros a medir antes de qualquer ensaio

Estes números entram em `arduino/motor_control/motor_control.ino` e na identificação
(`simulations/identificacao/identify_motor.py`):

| Parâmetro | Como medir |
|---|---|
| `ENC_CPR` | 4 × PPR do encoder × relação de redução (datasheet do motor). Confirme girando o eixo de saída uma volta à mão e lendo a contagem. |
| `v_max` | tensão nos terminais do motor com PWM 100 %, motor girando (multímetro). Com L298N e fonte de 12 V, espere de 9,5 a 10,5 V. |
| `R` | multímetro nos terminais com o rotor travado, em várias posições; use a média |
| `K` | gire o eixo com outra fonte de rotação e meça a tensão em aberto e a velocidade: K = V/ω [V·s/rad]. Ou, energizado em regime, K = (V − R·I)/ω. |
| `L` | medidor LCR ou datasheet. Se desconhecido, use 1 mH: a 100 Hz o polo elétrico não afeta o controle. |

## Limites conhecidos desta montagem

- **Resolução de velocidade** = 2π / (`ENC_CPR` × Ts). Ela precisa ser bem menor que a
  tolerância das métricas. Com 600 contagens/volta a 100 Hz, 1 contagem = 1,05 rad/s,
  ou 5 % de uma referência de 20 rad/s: a "acomodação em 2 %" nunca seria detectada,
  só por quantização. A decodificação ×4 reduz esse passo 4 vezes.
- **Relação PWM → tensão**: o firmware assume tensão proporcional ao duty cycle. A L298N
  tem zona morta e queda de tensão dependente da corrente, que o modelo não inclui. O
  ensaio de identificação mostra quanto isso pesa (R² do ajuste).
- **PWM de 490 Hz** (padrão do Timer1 no Uno): audível e com ondulação de corrente.
  Não afeta o controle a 100 Hz, mas pode ser trocado por 31 kHz alterando o prescaler
  do Timer1.
- **Tempo de cálculo** (medido em ciclos no simavr, ATmega328P a 16 MHz, só a lei de
  controle): PID ≈ 0,24 ms, Fuzzy ≈ 0,37 ms, MLP ≈ 1,1 ms em média e 1,5 ms no pior
  caso. A MLP **não cabe** num período de 1 ms neste microcontrolador. O laço de 10 ms
  comporta todas, inclusive com a serial. O tempo real de cada laço é registrado na
  coluna `exec_us` do log. Reproduza com `arduino/benchmark/benchmark_ciclos.c`.
