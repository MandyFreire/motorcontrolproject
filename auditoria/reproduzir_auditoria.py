"""Reproduz, de forma independente, as evidências numéricas da auditoria.

Uso:
    pip install -r auditoria/requirements.txt
    python auditoria/reproduzir_auditoria.py

Gera auditoria/pid_degrau.png e imprime todas as métricas citadas em AUDITORIA.md.
Audita a versão ORIGINAL (commit e69d29a): as definições auditadas estão replicadas aqui,
e arquivos que já foram corrigidos/removidos são lidos do histórico do git.
"""
import pathlib
import subprocess
import warnings

import control as ct
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as ss
import skfuzzy as fuzz
from skfuzzy import control as ctrl

warnings.filterwarnings("ignore")
AQUI = pathlib.Path(__file__).resolve().parent


def secao(titulo):
    print("\n" + "=" * 70 + "\n" + titulo + "\n" + "=" * 70)


# ---------------------------------------------------------------------------
secao("1. A API usada em simulations/pid/pid e auto_tune_pid.py existe?")
print("scipy.signal.feedback existe?", hasattr(ss, "feedback"))
try:
    ss.TransferFunction([1], [1, 1]) * ss.TransferFunction([1], [1, 1])
    print("TransferFunction * TransferFunction: OK")
except TypeError as e:
    print("TransferFunction * TransferFunction -> TypeError:", e)

# ---------------------------------------------------------------------------
secao("2. Planta usada no repositório (parâmetros do tutorial CTMS / U. Michigan)")
J, b, K, R, L = 0.01, 0.1, 0.01, 1.0, 0.5
P = ct.tf([K], [J * L, J * R + L * b, b * R + K * K])
print("den =", [J * L, J * R + L * b, b * R + K * K], "(repo: [0.005, 0.06, 0.1001])")
print("ganho DC [rad/s por V] =", round(ct.dcgain(P), 4), "| polos =", np.round(P.poles(), 3))


def metricas(Kp, Ki, Kd, T):
    C = ct.tf([Kd, Kp, Ki], [1, 0])
    G = ct.feedback(C * P, 1)
    t = np.linspace(0, T, 200_001)
    t, y = ct.step_response(G, t)
    info = ct.step_info(G, T=t, SettlingTimeThreshold=0.02)
    return G, t, y, info


secao("3. PID do repositório (Kp=1, Ki=0.5, Kd=0.1) – malha fechada real")
G, t, y, info = metricas(1.0, 0.5, 0.1, 200)
print("polos MF =", np.round(G.poles(), 4))
print(f"overshoot = {info['Overshoot']:.2f} %  (README: 5 %)")
print(f"tempo de acomodação 2% = {info['SettlingTime']:.1f} s  (README: 2 s)")
print(f"tempo de subida 10-90% = {info['RiseTime']:.1f} s")
for tt in (2, 4, 10):
    print(f"y({tt:>2} s) = {np.interp(tt, t, y):.3f} do setpoint")

_, t2, y2, info2 = metricas(100, 200, 10, 2)
print(f"\nReferência (ganhos do tutorial CTMS 100/200/10): OS={info2['Overshoot']:.2f}% "
      f"ts={info2['SettlingTime']:.3f}s")

# ---------------------------------------------------------------------------
secao("4. auto_tune_pid.py com a API corrigida (mesma grade, mesmo critério)")
melhor, params, zeros, finais = np.inf, None, 0, []
for Kp in np.linspace(0.1, 2, 5):
    for Ki in np.linspace(0.01, 0.5, 5):
        for Kd in np.linspace(0, 0.2, 3):
            Gc = ct.feedback(ct.tf([Kd, Kp, Ki], [1, 0]) * P, 1)
            _, yy = ct.step_response(Gc, np.linspace(0, 10, 1000))
            ov = (np.max(yy) - yy[-1]) / yy[-1] * 100
            zeros += ov == 0
            finais.append(yy[-1])
            if ov < melhor:
                melhor, params = ov, (Kp, Ki, Kd, yy[-1])
print(f"candidatos com 'overshoot' = 0: {zeros}/75 (empate; vence o primeiro da grade)")
print(f"escolhido: Kp={params[0]}, Ki={params[1]}, Kd={params[2]} -> y(10 s) = {params[3]:.3f}")
print(f"melhor y(10 s) entre TODOS os 75 candidatos = {max(finais):.3f}")

# ---------------------------------------------------------------------------
secao("5. Fuzzy Python (simulations/fuzzy/fuzzy_controller.py) – cobertura das regras")
erro = ctrl.Antecedent(np.arange(-10, 10, 0.1), "erro")
de = ctrl.Antecedent(np.arange(-5, 5, 0.1), "variacao_erro")
saida = ctrl.Consequent(np.arange(-1, 1, 0.01), "saida")
erro["neg"] = fuzz.trimf(erro.universe, [-10, -10, 0])
erro["zero"] = fuzz.trimf(erro.universe, [-5, 0, 5])
erro["pos"] = fuzz.trimf(erro.universe, [0, 10, 10])
de["neg"] = fuzz.trimf(de.universe, [-5, -5, 0])
de["zero"] = fuzz.trimf(de.universe, [-1, 0, 1])
de["pos"] = fuzz.trimf(de.universe, [0, 5, 5])
saida["diminuir"] = fuzz.trimf(saida.universe, [-1, -1, 0])
saida["neutro"] = fuzz.trimf(saida.universe, [-0.5, 0, 0.5])
saida["aumentar"] = fuzz.trimf(saida.universe, [0, 1, 1])
sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem([
    ctrl.Rule(erro["neg"] & de["neg"], saida["aumentar"]),
    ctrl.Rule(erro["zero"], saida["neutro"]),
    ctrl.Rule(erro["pos"] & de["pos"], saida["diminuir"]),
]))


def u(e, d):
    sim.input["erro"], sim.input["variacao_erro"] = e, d
    try:
        sim.compute()
        return sim.output["saida"]
    except Exception as ex:  # skfuzzy não produz saída quando nenhuma regra dispara
        return f"EXCEÇÃO ({type(ex).__name__})"


for e, d in [(3, 0.5), (8, -3), (-8, 3), (8, 0), (-8, 0)]:
    print(f"e={e:+}, de={d:+} -> {u(e, d)}")
falhas = tot = 0
for e in np.arange(-9.9, 9.9, 0.5):
    for d in np.arange(-4.9, 4.9, 0.5):
        tot += 1
        falhas += isinstance(u(e, d), str)
print(f"pontos da grade sem saída: {falhas}/{tot} = {100 * falhas / tot:.1f} %")
uni = np.arange(-5, 5, 0.1)
print("pertinência de de['pos'] em de=0 (deveria ser 0):",
      fuzz.interp_membership(uni, fuzz.trimf(uni, [0, 5, 5]), 0.0))

# ---------------------------------------------------------------------------
secao("6. Fuzzy Arduino – saída quando a única regra dispara")
x = np.linspace(0, 150, 150_001)
mu = np.interp(x, [0, 100, 150], [1, 1, 0])
print(f"centróide do trapézio 'reduzir' (0,0,100,150) = {np.sum(x * mu) / np.sum(mu):.1f} (PWM)")
print("erro = 100 - analogRead/10 ∈ [-2.3, 100]; regra só dispara se erro <= 0 (rpm >= 100)")

# ---------------------------------------------------------------------------
secao("7. ML – dataset sintético de data_collection.py")
rng = np.random.default_rng(0)
v = rng.normal(95, 5, 1000)
dados = np.column_stack([np.full(1000, 100.0), v, 0.1 * (100 - v), 100 - v])
X, yml = dados[:, :3], dados[:, 3]
print("desvio-padrão do setpoint (feature 1):", X[:, 0].std())
print("max |alvo - 10*feature3| =", np.abs(yml - 10 * X[:, 2]).max(), "-> vazamento de rótulo")
print("autocorrelação lag-1 da 'velocidade':", round(np.corrcoef(v[:-1], v[1:])[0, 1], 3),
      "(sinal de um sistema dinâmico real seria próximo de 1)")
try:
    tam = subprocess.run(["git", "cat-file", "-s", "e69d29a:ml_control/trained_model.h5"],
                         cwd=AQUI, capture_output=True, text=True, check=True).stdout.strip()
except (subprocess.CalledProcessError, FileNotFoundError):
    tam = "indisponível (histórico git ausente)"
print("tamanho de ml_control/trained_model.h5 no commit e69d29a:", tam, "byte(s)")

# ---------------------------------------------------------------------------
secao("8. Dashboard – vazão serial a 9600 baud (8N1 = 10 bits/caractere)")
cps = 9600 / 10
print("linhas/s PID  :", round(cps / len("Setpoint: 100.00 | RPM: 12.30\r\n"), 1))
print("linhas/s Fuzzy:", round(cps / len("63.33\r\n"), 1))
print("linhas/s lidas pelo dashboard: 10 (QTimer de 100 ms, 1 readline por tick)")

# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
ax.plot(t, y, label="PID do repositório (1 / 0,5 / 0,1) – simulação correta")
ax.axvline(10, ls=":", c="gray")
ax.text(10.5, 0.1, "janela de 10 s\nusada no repositório", fontsize=8, color="gray")
ax.plot([2], [1.0], "rx", ms=10, label="README: acomoda em 2 s com 5 % de overshoot")
ax.axhspan(0.98, 1.02, color="green", alpha=0.15, label="faixa de ±2 %")
ax.set_xlim(0, 100)
ax.set_ylim(0, 1.15)
ax.set_xlabel("Tempo (s)")
ax.set_ylabel("Velocidade / setpoint")
ax.set_title("Resposta ao degrau: o que o README afirma × o que o modelo produz")
ax.grid(alpha=0.3)
ax.legend(loc="lower right", fontsize=8)
fig.tight_layout()
fig.savefig(AQUI / "pid_degrau.png")
print("\nFigura salva em", AQUI / "pid_degrau.png")
