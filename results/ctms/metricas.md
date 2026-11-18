# Resultados – modelo didático CTMS (U. Michigan) – não é um motor físico

Referência: 0.7992 rad/s · Ts = 10 ms · saturação ±12 V
Gerado por `run_all.py`. Valores de SIMULAÇÃO, não de bancada.

## Ganhos (mesma função custo e otimizador para PID e Fuzzy)

- PID: Kp = 13.4, Ki = 27.42, Kd = 3.08e-05
- Fuzzy PI: Ke = 1.343, Kde = 37.6, Ku = 0.6416

## Métricas

| Cenário | Métrica | PID | Fuzzy PI | MLP (clone do PID) |
|---|---|---|---|---|
| nominal | overshoot_pct | 0.124 | 2.711 | 0.124 |
| nominal | subida_10_90_s | 0.590 | 0.530 | 0.590 |
| nominal | acomodacao_2pct_s | 0.980 | 1.210 | 0.980 |
| nominal | erro_regime_pct | 0.000 | 0.000 | 0.000 |
| nominal | IAE | 0.293 | 0.377 | 0.293 |
| nominal | ITAE | 0.076 | 0.108 | 0.076 |
| nominal | u_max_V | 12.000 | 12.000 | 11.999 |
| nominal | tempo_saturado_pct | 0.000 | 1.833 | 0.000 |
| nominal | variacao_total_u_V | 5.068 | 16.349 | 5.069 |
| carga | queda_max_pct | 5.490 | 4.905 | 5.489 |
| carga | recuperacao_2pct_s | 0.600 | 0.360 | 0.600 |
| carga | IAE_carga | 0.022 | 0.014 | 0.022 |
| inercia_x2 | overshoot_pct | 0.000 | 2.765 | 0.000 |
| inercia_x2 | subida_10_90_s | 0.780 | 0.720 | 0.780 |
| inercia_x2 | acomodacao_2pct_s | 1.300 | 1.550 | 1.300 |
| inercia_x2 | erro_regime_pct | 0.000 | 0.001 | 0.000 |
| inercia_x2 | IAE | 0.400 | 0.479 | 0.400 |
| inercia_x2 | ITAE | 0.143 | 0.184 | 0.143 |
| inercia_x2 | u_max_V | 11.989 | 12.000 | 11.989 |
| inercia_x2 | tempo_saturado_pct | 0.000 | 7.833 | 0.000 |
| inercia_x2 | variacao_total_u_V | 4.826 | 15.505 | 4.826 |
| ruido | overshoot_pct | 0.251 | 5.126 | 0.251 |
| ruido | subida_10_90_s | 0.630 | 0.550 | 0.630 |
| ruido | acomodacao_2pct_s | 1.110 | 3.110 | 1.110 |
| ruido | erro_regime_pct | 0.072 | 0.429 | 0.072 |
| ruido | IAE | 0.309 | 0.447 | 0.309 |
| ruido | ITAE | 0.097 | 0.276 | 0.097 |
| ruido | u_max_V | 11.922 | 12.000 | 11.921 |
| ruido | tempo_saturado_pct | 0.000 | 0.500 | 0.000 |
| ruido | variacao_total_u_V | 93.282 | 142.117 | 93.425 |

## PID original do repositório (Kp=1, Ki=0,5, Kd=0,1) no cenário nominal

- overshoot_pct: 0.000
- subida_10_90_s: não atingido em 6 s
- acomodacao_2pct_s: não atingido em 6 s
- erro_regime_pct: 71.248
- IAE: 3.928
- ITAE: 11.139
- u_max_V: 2.524
- tempo_saturado_pct: 0.000
- variacao_total_u_V: 1.721

## Rede neural

- amostras_treino: 4800
- amostras_validacao: 1200
- episodios_validacao: [5, 9, 11, 14]
- rmse_treino_V: 0.002741640067605628
- rmse_validacao_V: 0.00034734741553783235
- parametros_treinaveis: 41
- rmse_validacao_regressao_linear_V: 8.072891634279006e-15
- malha_fechada_por_semente (semente 0 = modelo usado): [{'semente': 0, 'rmse_validacao_V': 0.000347, 'overshoot_pct': 0.1241, 'acomodacao_2pct_s': 0.98, 'IAE': 0.2933}, {'semente': 1, 'rmse_validacao_V': 0.001098, 'overshoot_pct': 0.1228, 'acomodacao_2pct_s': 0.98, 'IAE': 0.2933}, {'semente': 2, 'rmse_validacao_V': 0.018884, 'overshoot_pct': 0.1223, 'acomodacao_2pct_s': 0.98, 'IAE': 0.2933}, {'semente': 3, 'rmse_validacao_V': 0.009104, 'overshoot_pct': 0.126, 'acomodacao_2pct_s': 0.98, 'IAE': 0.2933}, {'semente': 4, 'rmse_validacao_V': 0.267594, 'overshoot_pct': 0.0, 'acomodacao_2pct_s': 1.04, 'IAE': 0.2988}]
- entradas: [erro, integral do erro, derivada filtrada da medição]
- arquitetura: 3-8-1, ReLU, sklearn MLPRegressor (lbfgs)
