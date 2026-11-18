# Sintonia do PID: cálculo à mão × otimização numérica

**Pergunta:** vale a pena sintonizar com otimização numérica ou rede neural, ou um método
clássico calculado à mão dá na mesma?

**Resposta curta:** para este motor (modelo linear conhecido, um ponto de operação), dá
praticamente na mesma. O método IMC/lambda, feito à mão em cinco minutos, chega a 9 % do
custo ótimo. O otimizador, depois de 2.432 simulações, chegou essencialmente **à mesma
estrutura do IMC**. Ziegler-Nichols é o único método que fica muito longe.

Todos os números abaixo são reproduzíveis com
`python simulations/auto_tuning/comparar_metodos.py`. São de **simulação**, sobre o modelo
didático CTMS, nas mesmas condições do restante do projeto: Ts = 10 ms, ±12 V,
referência de 0,799 rad/s.

---

## 1. Dados da planta (à mão)

```
G(s) = 0,01 / (0,005 s² + 0,06 s + 0,1001)
```

Dividindo numerador e denominador por 0,1001:

```
G(s) = 0,0999 / (0,04995 s² + 0,5994 s + 1)
```

**Ganho estático:** k = 0,01 / 0,1001 = **0,0999 (rad/s)/V**

**Constantes de tempo:** o denominador é (τ₁s + 1)(τ₂s + 1), então
τ₁ + τ₂ = 0,5994 e τ₁·τ₂ = 0,04995. Elas são as raízes de x² − 0,5994x + 0,04995 = 0:

```
Δ  = 0,5994² − 4·0,04995 = 0,35928 − 0,19980 = 0,15948      √Δ = 0,39935
τ₁ = (0,5994 + 0,39935)/2 = 0,4994 s     (polo lento, mecânico)
τ₂ = (0,5994 − 0,39935)/2 = 0,1000 s     (polo rápido, elétrico)
```

**Referência usada nos ensaios:** r = 2/3 · 12 V · 0,0999 = **0,799 rad/s**, que exige 8 V em regime.

---

## 2. Ziegler-Nichols, método da curva de reação

1. Aplique um degrau em malha aberta e trace a tangente no ponto de inflexão da curva
   normalizada. Os valores lidos do gráfico são L = 0,0535 s (atraso aparente) e
   T = 0,747 s (constante de tempo aparente). Aqui foram medidos numericamente sobre a
   mesma resposta; à mão, lê-se do gráfico.
2. Tabela de Ziegler-Nichols para PID:

```
Kp = 1,2·T/(k·L) = 1,2·0,747 / (0,0999·0,0535) = 0,8964 / 0,005345 = 167,7 V/(rad/s)
Ti = 2L    = 0,107 s   → Ki = Kp/Ti = 167,7/0,107 = 1.567,6
Td = 0,5L  = 0,0268 s  → Kd = Kp·Td = 4,49
```

3. **Verificação antes de usar:** no instante do degrau, Kp·r = 167,7 · 0,799 = 134 V. O
   driver entrega no máximo 12 V, então o controlador passa o transitório inteiro
   saturado. Com ruído de medição de σ = 0,01 rad/s, só o termo proporcional já injeta
   167,7 · 0,01 = 1,7 V de ruído na tensão **a cada amostra**.

Ziegler-Nichols foi formulado em 1942 para decaimento de 1/4 por ciclo. Ele não considera
saturação nem ruído, e para esta planta resulta em ganhos cerca de 12 vezes maiores que o
necessário.

---

## 3. IMC / lambda (Rivera, Morari e Skogestad, 1986)

**Ideia:** escolher o controlador para que o zero do PI **cancele o polo lento** da planta,
e definir diretamente o tempo de resposta desejado, τc.

PI em série: C(s) = Kc · (τI·s + 1)/(τI·s), com

```
τI = τ₁ = 0,4994 s          (cancela o polo lento)
Kc = τ₁ / (k·τc)
```

Em forma paralela, que é a usada no código:

```
Kp = Kc       = τ₁/(k·τc)
Ki = Kc/τI    = 1/(k·τc)
```

**Malha fechada resultante.** Como o polo lento foi cancelado, a malha aberta fica

```
L(s) = C(s)·G(s) = 1 / (τc·s·(τ₂s + 1))
```

e a malha fechada é um sistema de 2ª ordem:

```
T(s) = 1 / (τc·τ₂·s² + τc·s + 1)
ωn = 1/√(τc·τ₂)          ζ = ½·√(τc/τ₂)
```

**Escolha de τc.** É a única decisão de projeto, e se faz com uma conta de saturação:

```
u(0⁺) = Kp·r ≤ 12 V   →   τ₁·r/(k·τc) ≤ 12   →   τc ≥ τ₁·r/(k·12) = 0,4994·0,799/(0,0999·12) = 0,333 s
```

Com uma margem de 20 %, um número redondo: **τc = 0,4 s**.

```
Kp = 0,4994/(0,0999·0,4) = 12,50 V/(rad/s)
Ki = 1/(0,0999·0,4)      = 25,0  V/(rad·s⁻¹·s)
Kd = 0
u(0⁺) = 12,50·0,799 = 10,0 V    (dentro do limite)
```

**Previsão à mão:**

```
ζ  = ½·√(0,4/0,1) = 1,0          → amortecimento crítico: overshoot = 0 %
ωn = 1/√(0,4·0,1) = 5,0 rad/s
ts(2 %) ≈ 5,8/ωn = 1,17 s        (fórmula do amortecimento crítico)
```

**Simulação:** overshoot de 0,00 % e acomodação em 1,16 s. A conta à mão acertou o tempo
de acomodação com erro de 1 %.

---

## 4. Comparação no simulador

Mesmas condições e mesma função custo (`motorlib/tuning.py`). Os cenários de carga e de
ruído estão descritos em `docs/theory.md`.

| Método | Kp | Ki | Kd | u(0⁺) V | Overshoot % | Acomodação s | IAE | Recup. carga s | Acomod. c/ ruído s | Var. total u c/ ruído V | Custo |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Ziegler-Nichols (curva de reação) | 167,74 | 1567,85 | 4,487 | 12,0 (sat.) | 0,73 | 0,66 | 0,261 | 0,11 | 0,66 | **1998** | 2,065 |
| IMC PI, τc = 0,25 s | 19,99 | 40,04 | 0 | 12,0 (sat.) | 0,00 | 1,70 | 0,336 | 0,46 | 1,66 | 136 | 0,301 |
| IMC PI, τc = 0,333 s (limite) | 15,01 | 30,06 | 0 | 12,0 | 0,00 | 1,30 | 0,313 | 0,56 | 1,47 | 104 | 0,230 |
| **IMC PI, τc = 0,4 s (escolha à mão)** | **12,50** | **25,02** | 0 | 10,0 | **0,00** | **1,16** | 0,320 | 0,64 | 1,16 | 87 | **0,205** |
| IMC PI, τc = 0,5 s | 10,00 | 20,02 | 0 | 8,0 | 0,00 | 1,59 | 0,400 | 0,76 | 1,59 | 69 | 0,267 |
| IMC PID, τc = 0,365 s (cancela os 2 polos) | 16,44 | 27,42 | 1,370 | 12,0 | 0,07 | 1,34 | 0,341 | 0,58 | 1,47 | 479 | 0,623 |
| IMC PI, τc = 0,365 s ¹ | 13,70 | 27,42 | 0 | 10,9 | 0,00 | 1,06 | 0,299 | 0,59 | 1,21 | 95 | 0,199 |
| **Otimização numérica** (2.432 simulações, ~9 s) | **13,40** | **27,42** | 0,000 | 10,7 | 0,12 | **0,98** | **0,293** | 0,60 | 1,11 | 93 | **0,188** |

¹ τc = 0,365 s **não foi escolhido à mão**: é o valor implícito no resultado da otimização,
1/(k·Ki) = 0,365 s. Ele está na tabela só para mostrar o próximo ponto.

---

## 5. O que os números mostram

1. **O otimizador reencontrou o IMC.** O Ki ótimo (27,42) corresponde exatamente a
   τc = 0,365 s, e a razão Kp/Ki = 0,489 s está a 2 % de τ₁ = 0,499 s, que é a regra de
   cancelamento do polo lento. O otimizador também zerou o Kd, do mesmo jeito que o IMC PI.
   Em 2.432 simulações, ele descobriu numericamente a regra que a teoria dá em duas linhas.
2. **A escolha à mão (τc = 0,4 s) fica a 9 % do custo ótimo** (0,205 contra 0,188). Na
   prática a diferença é de 0,18 s na acomodação, e a escolha à mão fica 7 % melhor na
   atividade de controle sob ruído (87 V contra 93 V).
3. **Ziegler-Nichols vence no papel e perde na prática.** Ele tem o menor IAE nominal e a
   recuperação de carga mais rápida, mas porque opera saturado, quase em liga-desliga. A
   tensão de controle sob ruído varia **21 vezes mais** que a ótima (1998 V contra 93 V).
   Num motor real, isso é aquecimento, ruído audível e desgaste.
4. **A ação derivativa piora.** O IMC PID, que cancela também o polo rápido, multiplica
   por 5 a atividade de controle sob ruído e não melhora o rastreamento. Isso confirma o
   Kd ≈ 0 da otimização.
5. **τc mede a agressividade.** Abaixo do limite de saturação (0,25 s), o controlador
   satura e fica mais lento (1,70 s), não mais rápido. A conta de saturação da seção 3 é a
   parte mais importante do projeto.

---

## 6. E sintonizar com rede neural?

Para esta planta, **não traz ganho, e custa mais**:

- A otimização numérica já encontra o ótimo para a função custo escolhida. Uma rede que
  aprenda a escolher ganhos, otimizando o mesmo custo no mesmo modelo, chega no máximo ao
  mesmo ponto. E o IMC à mão já está a 9 % dele.
- Ela exige dataset, treino, validação e escolha de arquitetura. Além disso, como medido
  neste projeto (`docs/ml_theory.md`), varia muito com a semente de inicialização e falha
  fora do envelope de treino.
- Se a rede **for** o controlador, e não quem sintoniza, ela custa cerca de 4,5 vezes mais
  tempo de CPU que o PID no ATmega328P (`docs/hardware.md`).

**Quando passa a fazer sentido** usar sintonia adaptativa, por rede ou outro método:

- quando a planta **muda com o ponto de operação** (não linearidade forte, carga variável
  em larga faixa) e um único conjunto de ganhos não serve;
- quando **não há modelo** e não é viável identificar um.

Mesmo nesses casos, a alternativa clássica para comparar é o *gain scheduling*: uma tabela
de ganhos IMC calculados para alguns pontos de operação e interpolados. Ela é simples,
verificável e roda no Uno.

**Ordem recomendada:** identificar a planta → IMC à mão → validar em simulação com
saturação e ruído → só então, se não bastar, otimização numérica → só então ML.

---

## Ressalvas

- Os resultados são do **modelo linear CTMS**, sem zona morta, atrito de Coulomb nem folga.
  Na bancada, a diferença entre os métodos pode mudar. O mesmo script roda com o motor
  identificado se você trocar `MotorParams()` pelos parâmetros identificados.
- O "custo" depende dos pesos escolhidos (`docs/theory.md`, §5). Com outros pesos, o ótimo
  e as distâncias mudam, mas a estrutura (cancelar o polo lento, Kd ≈ 0) é robusta: vem da
  física da planta.
- As leituras de L e T para Ziegler-Nichols foram feitas numericamente. À mão, sobre um
  gráfico, espere ±10 % nesses valores, o que mudaria os ganhos de ZN na mesma proporção.

## Referências

- Ziegler, J. G.; Nichols, N. B. "Optimum settings for automatic controllers", *Trans. ASME*, 1942.
- Rivera, D. E.; Morari, M.; Skogestad, S. "Internal Model Control. 4. PID controller design", *Ind. Eng. Chem. Process Des. Dev.*, 1986.
- Skogestad, S. "Simple analytic rules for model reduction and PID controller tuning", *Journal of Process Control*, 2003.
- Åström, K. J.; Hägglund, T. *Advanced PID Control*, ISA, 2006.
