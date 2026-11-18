"""Compara sintonias calculadas à mão (Ziegler-Nichols, IMC/lambda) com a otimização numérica,
no mesmo simulador, com as mesmas métricas e a mesma função custo.

    python simulations/auto_tuning/comparar_metodos.py

O passo a passo dos cálculos à mão, com todos os números, está em docs/comparacao_sintonia.md.
"""
import time

import _caminho  # noqa: F401
import numpy as np

from motorlib.plant import DCMotor, MotorParams
from motorlib.simulate import avaliar, ref_padrao
from motorlib.tuning import custo, make_pid, sintonizar_pid


def parametros_planta(p):
    """Ganho estático e constantes de tempo (lenta, rápida) da função de transferência."""
    taus = sorted(-1.0 / np.roots(p.tf_coeffs()[1]).real, reverse=True)
    return p.dc_gain, taus[0], taus[1]


def curva_de_reacao(p, dt=0.0005, n=20000):
    """L (atraso aparente) e T (constante de tempo aparente) pela tangente no ponto de inflexão
    da resposta ao degrau unitário normalizada – o que se faria à mão sobre o gráfico."""
    m = DCMotor(p, dt)
    y = np.array([m.step(1.0) for _ in range(n)]) / p.dc_gain
    t = np.arange(1, n + 1) * dt
    dy = np.gradient(y, t)
    i = int(np.argmax(dy))
    return t[i] - y[i] / dy[i], 1.0 / dy[i]


def main():
    p = MotorParams()
    k, t1, t2 = parametros_planta(p)
    ref = ref_padrao(p)
    L, T = curva_de_reacao(p)
    print(f"k = {k:.4f} (rad/s)/V   tau1 = {t1:.4f} s   tau2 = {t2:.4f} s   ref = {ref:.4f} rad/s")
    print(f"curva de reação: L = {L:.4f} s   T = {T:.4f} s")
    tc_min = t1 * ref / (k * p.v_max)
    print(f"tau_c mínimo para não saturar no degrau (Kp*ref <= v_max): {tc_min:.3f} s")

    kp_zn = 1.2 * T / (k * L)
    casos = {"Ziegler-Nichols (curva de reação)": [kp_zn, kp_zn / (2 * L), kp_zn * 0.5 * L]}
    for tc in (0.25, round(tc_min, 3), 0.365, 0.40, 0.50):
        casos[f"IMC/lambda PI, tau_c = {tc} s"] = [t1 / (k * tc), 1 / (k * tc), 0.0]
    tc = 0.365
    casos[f"IMC/lambda PID, tau_c = {tc} s"] = [(t1 + t2) / (k * tc), 1 / (k * tc), t1 * t2 / (k * tc)]
    t0 = time.time()
    g = sintonizar_pid(p)
    casos[f"Otimização numérica ({g['avaliacoes']} simulações, {time.time() - t0:.0f} s)"] = \
        [g["Kp"], g["Ki"], g["Kd"]]

    print(f"\n| Método | Kp | Ki | Kd | u(0⁺) V | Overshoot % | Acomodação s | IAE | "
          f"Recup. carga s | Acomod. c/ ruído s | Var. total u c/ ruído V | Custo |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for nome, x in casos.items():
        r, _ = avaliar(make_pid(x, p), p)
        n, c, z = r["nominal"], r["carga"], r["ruido"]
        print(f"| {nome} | {x[0]:.2f} | {x[1]:.2f} | {x[2]:.3f} | {min(x[0] * ref, p.v_max):.1f} | "
              f"{n['overshoot_pct']:.2f} | {n['acomodacao_2pct_s']:.2f} | {n['IAE']:.3f} | "
              f"{c['recuperacao_2pct_s']:.2f} | {z['acomodacao_2pct_s']:.2f} | "
              f"{z['variacao_total_u_V']:.0f} | {custo(make_pid(x, p), p):.3f} |")
    print(f"\ntau_c implícito na otimização: 1/(k*Ki) = {1 / (k * g['Ki']):.3f} s ; "
          f"Kp/Ki = {g['Kp'] / g['Ki']:.4f} s (IMC: tau1 = {t1:.4f} s)")


if __name__ == "__main__":
    main()
