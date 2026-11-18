# Rede neural no controle do motor: o que foi feito e o que dá para afirmar

## O que foi implementado

**Clonagem de comportamento** (*behavior cloning*): uma MLP 3-8-1 (ReLU, 41 parâmetros)
treinada para reproduzir a saída do PID sintonizado.

| Item | Escolha | Motivo |
|---|---|---|
| Entradas | erro, integral do erro, derivada filtrada da medição | são os estados internos do PID; sem o estado integral nenhuma rede reproduz a ação integral e sobra erro em regime |
| Alvo | tensão do PID antes da saturação | a saturação é aplicada fora da rede, igual ao PID |
| Dados | 20 episódios × 300 amostras (6.000), em malha fechada, com referência, carga e ruído aleatórios | cobrir o envelope de operação |
| Validação | 4 episódios inteiros separados | amostras do mesmo episódio são correlacionadas; misturá-las infla a métrica |
| Embarcado | pesos exportados para C (`parametros_gerados.h`), inferência em float no ATmega328P | uma MLP deste tamanho cabe em C puro, sem framework de inferência |

## Resultados (modelo CTMS, `results/ctms/metricas.md`)

- **A rede iguala o PID**, sem superá-lo: as métricas em malha fechada coincidem até a
  3ª ou 4ª casa decimal nos 4 cenários.
- **Uma regressão linear acerta o alvo com erro de 8×10⁻¹⁵ V** e recupera os ganhos do
  PID exatamente. A saída do PID é uma combinação linear dos três estados. A rede gasta
  41 parâmetros para aprender 3 coeficientes.
- **A variação entre inicializações é grande.** O erro de validação vai de 0,0003 V a
  0,27 V entre as sementes 0 e 4. A semente usada (0) é a melhor das cinco, e todas
  estão registradas em `metricas.md`.
- **Menos overshoot não é melhor desempenho.** A pior rede (semente 4) tem 0 % de
  overshoot contra 0,12 % do PID, mas acomoda mais devagar (1,04 s contra 0,98 s) e
  tem IAE maior. O erro de aproximação funcionou como uma redução de ganho. Se alguém
  medir "X % menos overshoot" num clone, a explicação mais provável é essa, e ela só
  aparece quando se olham as outras métricas.
- **Fora do envelope de treino, a rede falha.** Com 10 episódios, uma divisão deixou a
  validação com saídas abaixo do mínimo visto no treino, e o erro subiu de ~0,001 para
  0,45 V. Redes ReLU extrapolam mal. O PID não tem esse problema.
- **Custo embarcado** (simavr, ATmega328P a 16 MHz): ~1,1 ms em média e 1,5 ms no pior
  caso por cálculo, contra 0,24 ms do PID. Não cabe numa malha de 1 kHz neste
  microcontrolador.

## O que esta arquitetura NÃO faz

- **Não se adapta** a mudanças de carga ou de parâmetros: não há aprendizado online nem
  estado além dos do PID. No cenário J ×2, ela se comporta como o PID clonado, porque é
  o PID clonado.
- **Não supera o PID que imitou.** Por construção, o melhor resultado possível é igualá-lo.

## Quando faria sentido usar ML aqui

- **Controle adaptativo** de verdade: estimar parâmetros online (mínimos quadrados
  recursivos, MRAC) ou reajustar ganhos por agendamento (*gain scheduling*).
- **Aprendizado por reforço** com a planta identificada como simulador, comparado contra
  o PID sob o mesmo custo, e com garantias de estabilidade discutidas.
- **Compensar não linearidades** que o PID não trata (zona morta da L298N, atrito de
  Coulomb), com uma rede em paralelo ao PID, e não no lugar dele.

## Referências

- Sutton, R. S.; Barto, A. G. *Reinforcement Learning: An Introduction*, 2ª ed., MIT Press, 2018.
- Goodfellow, I.; Bengio, Y.; Courville, A. *Deep Learning*, MIT Press, 2016.
- Åström, K. J.; Hägglund, T. *Advanced PID Control*, ISA, 2006 (anti-windup e filtro da derivada).
- Control Tutorials for MATLAB & Simulink, University of Michigan – DC Motor Speed.
