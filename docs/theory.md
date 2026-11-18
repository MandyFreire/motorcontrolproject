# Teoria e metodologia: PID e Fuzzy PI

## 1. Modelo do motor

Motor DC controlado pela armadura, com estados velocidade ω e corrente i:

```
J dω/dt = K·i − b·ω − T_L
L di/dt = V − R·i − K·ω

W(s)/V(s) = K / ( JL·s² + (JR + Lb)·s + (bR + K²) )
```

**Parâmetros nominais:** J = 0,01 kg·m², b = 0,1 N·m·s, K = 0,01 N·m/A, R = 1 Ω,
L = 0,5 H. Fonte: *Control Tutorials for MATLAB & Simulink* (CTMS), University of
Michigan, "DC Motor Speed: System Modeling".

Esse é um **modelo didático**: com 12 V ele chega a 1,2 rad/s (11,4 rpm), muito abaixo de
qualquer motor de bancada. Os números de `results/ctms/` valem para esse modelo, não para
um motor físico. Para o motor real, identifique os parâmetros (seção 6).

A planta é discretizada por **segurador de ordem zero exato** (`scipy.signal.cont2discrete`)
com Ts = 10 ms. Isso reproduz o que o microcontrolador faz: o PWM fica constante durante
cada período. O teste `test_discretizacao_zoh_exata_para_degrau` confere essa discretização
contra a resposta contínua, com erro abaixo de 10⁻¹².

## 2. Condições comuns a todos os controladores

| Item | Valor | Por quê |
|---|---|---|
| Período de amostragem | 10 ms (100 Hz) | igual ao firmware; folga para a MLP no ATmega328P |
| Saturação | ±v_max (12 V no modelo) | limite físico do driver |
| Referência | 2/3 de v_max·ganho DC (0,799 rad/s) | exige 8 V em regime, sobram 4 V de margem |
| Carga | 5 % do torque de partida, degrau em t = 3 s | rejeição de perturbação |
| Variação de planta | J ×2, b +20 % | robustez a uma planta diferente da de sintonia |
| Ruído de medição | gaussiano, σ = 1,25 % da referência | sensibilidade a ruído |

O cenário de variação de planta usava b +50 % na primeira versão. Nesse caso a referência
exigiria 12,01 V em regime, acima da saturação: o cenário só mediria saturação, e não
robustez. Um teste automatizado (`test_cenario_inercia_alcancavel`) impede que isso volte.

## 3. PID

Forma paralela discreta:

- **Derivada sobre a medição**, e não sobre o erro, para evitar o pico de tensão no
  degrau de referência (*derivative kick*). É filtrada por 1ª ordem, com Tf = 2·Ts.
- **Integral** por Euler regressivo.
- **Anti-windup por integração condicional:** o integrador congela quando a saída está
  saturada e o erro empurra no mesmo sentido da saturação.

## 4. Fuzzy PI incremental

```
e_n  = sat(Ke · e)            ∈ [−1, 1]
Δe_n = sat(Kde · (e_k − e_{k−1}))
u_k  = sat(u_{k−1} + Ku · F(e_n, Δe_n))
```

- **Mamdani**, 5 conjuntos por variável (NG, NP, ZE, PP, PG), triangulares com 50 % de
  sobreposição e ombros nos extremos. Defuzzificação por centróide.
- **Base de regras 5×5 completa** (25 regras, padrão MacVicar-Whelan): todo par
  (e, Δe) ativa ao menos uma regra. A versão anterior tinha 3 regras e deixava 25 % do
  espaço sem saída.
- **Convenção de sinal** única no projeto, e = r − y: erro positivo → aumentar tensão.
- A forma incremental dá **ação integral**, logo erro nulo em regime para degrau. A
  saturação de u_k funciona como anti-windup.
- A superfície F é gerada com scikit-fuzzy numa grade 21×21 e avaliada por **interpolação
  bilinear**. O firmware usa exatamente a mesma tabela, gravada na flash. Os testes conferem
  completude, sinal e antissimetria da superfície.

## 5. Sintonia: mesmo critério para os dois

A comparação só é justa se os dois controladores forem sintonizados da mesma forma.
Os dois usam o mesmo procedimento (`motorlib/tuning.py`):

```
J = ITAE(nominal)/ref + 10·max(0, overshoot% − 5)² + 10⁻³·VT(u sob ruído)
```

onde VT é a variação total da tensão de controle no cenário com ruído. O custo é minimizado por evolução diferencial (`scipy.optimize.differential_evolution`) com
semente fixa. Três sementes diferentes levam aos mesmos ganhos do PID com 4 algarismos
significativos.

Consequências que precisam ser ditas junto com os números:

- O limite de 5 % de overshoot é uma **restrição de projeto**. Um overshoot perto de 5 %
  não é uma virtude do controlador: é o otimizador encostando na restrição.
- Os pesos são escolhas. Pesos diferentes levam a ganhos diferentes e podem inverter a
  comparação em algumas métricas.
- No modelo CTMS, **o Kd ótimo é ≈ 0**: com ruído de medição, a ação derivativa só
  aumenta o custo. O PID sintonizado é, na prática, um PI.

## 6. Do modelo para a bancada

1. Ensaio em malha aberta (`MODO_CONTROLE 0`), gravado pelo dashboard em CSV.
2. Identificação de J e b com K, R e L medidos (`identify_motor.py`). Com L pequeno, a
   resposta ao degrau só revela dois números (ganho DC e constante de tempo); por isso
   K precisa ser medido, e não estimado. Validado com dados sintéticos: erro abaixo de
   1,5 % em J e de 4 % em b, com R² > 0,99.
3. `python run_all.py --params results/motor_identificado.json --arduino`: sintoniza no
   modelo identificado e gera `parametros_gerados.h`.
4. Ensaio em malha fechada de cada controlador, gravado em CSV.
5. `analisar_ensaio.py ensaio.csv --params ...` calcula as métricas da bancada **com as
   mesmas funções da simulação** e mostra as duas colunas lado a lado.

## 7. Garantias automatizadas (`python -m pytest`)

- A planta é idêntica à função de transferência do repositório original.
- A discretização é exata.
- As métricas estão corretas num sinal de 1ª ordem conhecido.
- O anti-windup funciona.
- A base fuzzy é completa, com sinal correto e simétrica.
- A rede reproduz o PID dentro de 1 %.
- A identificação recupera J e b.
- O código C do firmware dá o mesmo resultado que o Python, amostra a amostra
  (`tests/test_paridade_c.py`).
