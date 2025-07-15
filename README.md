# Projeto Completo: Controle de Motor DC (PID e Fuzzy)

## Visão Geral
Sistema de controle para motor DC implementado com controladores PID e Fuzzy, incluindo simulações, autotuning, implementação física com Arduino, visualização em dashboard e sugestões de projetos futuros (Machine Learning e IoT).

---

## Estrutura do Repositório
```
motorcontrolproject/
├── simulations/
│   ├── pid/
│   │   └── motor_pid_simulation.py
│   ├── fuzzy/
│   │   ├── fuzzy_controller.py
│   │   └── requirements.txt
│   └── auto_tuning/
│       └── auto_tune_pid.py
├── arduino/
│   ├── pid/
│   │   └── motor_pid.ino
│   └── fuzzy/
│       └── fuzzy_motor.ino
├── dashboard/
│   └── realtime_plot.py
├── docs/
│   └── ml_theory.md
└── README.md

```

---

## Como Reproduzir o Projeto

### 1️⃣ Simulações (Python ou MATLAB)
- Rodar `motor_pid_simulation.py` ou `fuzzy_controller.py`
- Visualizar gráficos de resposta ao degrau e comparar desempenho dos controladores.

### 2️⃣ Ajuste Automático do PID (Python)
- Executar `auto_tune_pid.py` para encontrar melhores valores de Kp, Ki, Kd.

### 3️⃣ Implementação no Arduino
- Subir `motor_pid.ino` (PID) ou `fuzzy_motor.ino` (Fuzzy).
- Conectar hardware conforme esquema em `docs/hardware.md`.
- Usar Serial Plotter para observar a resposta do motor.

### 4️⃣ Dashboard Python (Opcional)
- Rodar `realtime_plot.py` para visualizar RPM do motor em tempo real.

### 5️⃣ Documentação
- Consulte `docs/hardware.md` e `docs/theory.md` para detalhes técnicos.

---

## Tabela Comparativa dos Controladores
| Controlador | Overshoot | Tempo de Estabilização | Robustez |
|-------------|-----------|-------------------------|----------|
| PID         | 5%        | 2s                      | Boa      |
| Fuzzy       | 15%       | 4s                      | Muito Boa|

---

## Próximos Passos (Opcionais)
- **Machine Learning**: Usar TensorFlow Lite para substituir PID por rede neural. (em andamento)
- **Controle Adaptativo**: Ajuste dinâmico dos ganhos PID em tempo real.
- **IoT**: Controle remoto via ESP32 + MQTT.
- **Dashboard Web**: Visualização via Node-RED ou ThingsBoard.

---

## Resultados
- Controladores comparados em ambiente simulado e físico.
- Motor DC controlado de forma estável em malha fechada.
- Código modular e comentado, pronto para evolução do projeto.


## Créditos
Desenvolvido por Amanda Freire. Projeto educacional de automação e controle.
