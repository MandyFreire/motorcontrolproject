"""Resposta ao degrau do PID: ganhos do repositório original x ganhos sintonizados.

    python simulations/pid/motor_pid_simulation.py [--kp 13.4 --ki 27.4 --kd 0] [--salvar fig.png]

Correção em relação à versão original: scipy.signal não tem `feedback` nem multiplicação de
TransferFunction; a análise contínua usa python-control, e a simulação discreta (com
saturação e anti-windup, como no Arduino) usa motorlib.
"""
import argparse
import json

import _caminho  # noqa: F401
import control as ct
import matplotlib.pyplot as plt
import numpy as np

from motorlib.plant import MotorParams
from motorlib.simulate import cenarios, metricas_degrau, simular
from motorlib.tuning import make_pid


def main():
    ap = argparse.ArgumentParser()
    ganhos = _caminho.RAIZ / "results" / "ctms" / "ganhos.json"
    g = json.loads(ganhos.read_text())["pid"] if ganhos.exists() else {"Kp": 13.4, "Ki": 27.4, "Kd": 0.0}
    ap.add_argument("--kp", type=float, default=g["Kp"])
    ap.add_argument("--ki", type=float, default=g["Ki"])
    ap.add_argument("--kd", type=float, default=g["Kd"])
    ap.add_argument("--salvar", help="salva a figura em vez de abrir janela")
    a = ap.parse_args()

    p = MotorParams()
    planta = ct.tf(*p.tf_coeffs())
    print("Planta (CTMS):", planta)
    c = cenarios(p)["nominal"]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    for nome, k in [("original (1 / 0,5 / 0,1)", [1.0, 0.5, 0.1]),
                    (f"sintonizado ({a.kp:.3g} / {a.ki:.3g} / {a.kd:.2g})", [a.kp, a.ki, a.kd])]:
        # análise linear contínua (sem saturação): polos de malha fechada
        G = ct.feedback(ct.tf([k[2], k[0], k[1]], [1, 0]) * planta, 1)
        # simulação discreta realista (Ts = 10 ms, saturação ±12 V, anti-windup)
        t, y, u = simular(make_pid(k, p), c)
        m = metricas_degrau(t, y, u, c.ref, p.v_max)
        print(f"\n{nome}\n  polos MF (contínuo, sem saturação): {np.round(G.poles(), 3)}")
        for kk, v in m.items():
            print(f"  {kk:22s} {v:.4g}")
        ax.plot(t, y / c.ref, label=nome)
    ax.axhspan(0.98, 1.02, color="green", alpha=0.15, label="±2 %")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Velocidade / referência")
    ax.set_title("Resposta ao degrau – PID (modelo CTMS, Ts = 10 ms, ±12 V)")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    if a.salvar:
        fig.savefig(a.salvar, dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()
