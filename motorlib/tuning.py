"""Sintonia por otimização – MESMO critério, otimizador e semente para PID e Fuzzy.

Custo (minimizar), avaliado sobre o cenário nominal e o cenário com ruído:
    J = ITAE(nominal) / ref
        + 10 * max(0, overshoot% - 5)^2           restrição suave: overshoot <= 5 %
        + 1e-3 * variação total de u (ruído) / v_max * 12
                                                  penaliza ganho que amplifica ruído de medição

Otimizador: scipy.optimize.differential_evolution, semente fixa -> resultado determinístico.
Os pesos são escolhas de projeto declaradas aqui; mudar os pesos muda os ganhos.
Os limites de busca escalam com o motor (ganho DC e referência) para servir a qualquer planta.
"""
import numpy as np
from scipy.optimize import differential_evolution

from .controllers import PID, FuzzyPI
from .plant import MotorParams
from .simulate import TS, cenarios, metricas_degrau, simular

OS_MAX = 5.0
W_OS = 10.0
W_TV = 1e-3


def custo(ctrl, p: MotorParams):
    cs = cenarios(p)
    c = cs["nominal"]
    t, y, u = simular(ctrl, c)
    m = metricas_degrau(t, y, u, c.ref, p.v_max)
    _, _, u_r = simular(ctrl, cs["ruido"])
    tv = float(np.sum(np.abs(np.diff(u_r)))) / p.v_max * 12.0
    return m["ITAE"] / c.ref + W_OS * max(0.0, m["overshoot_pct"] - OS_MAX) ** 2 + W_TV * tv


def limites_pid(p: MotorParams):
    k = 1.0 / p.dc_gain  # [V/(rad/s)] – escala natural dos ganhos
    return [(0.0, 20 * k), (0.0, 40 * k), (0.0, 0.5 * k)]          # Kp, Ki, Kd


def limites_fuzzy(p: MotorParams):
    ref = 2.0 / 3.0 * p.v_max * p.dc_gain
    return [(0.1 / ref, 40 / ref), (0.1 / ref, 400 / ref), (0.001 * p.v_max, 0.25 * p.v_max)]  # Ke, Kde, Ku


def make_pid(x, p: MotorParams = MotorParams(), Ts=TS):
    return PID(x[0], x[1], x[2], Ts, -p.v_max, p.v_max)


def make_fuzzy(x, grid, table, p: MotorParams = MotorParams()):
    return FuzzyPI(table, grid, x[0], x[1], x[2], -p.v_max, p.v_max)


def _otimizar(fabrica, limites, p, seed, maxiter):
    return differential_evolution(lambda x: custo(fabrica(x), p), limites, seed=seed,
                                  maxiter=maxiter, popsize=12, tol=1e-6, polish=True)


def sintonizar_pid(p: MotorParams = MotorParams(), seed=1, maxiter=60):
    r = _otimizar(lambda x: make_pid(x, p), limites_pid(p), p, seed, maxiter)
    return {"Kp": float(r.x[0]), "Ki": float(r.x[1]), "Kd": float(r.x[2]),
            "custo": float(r.fun), "avaliacoes": int(r.nfev)}


def sintonizar_fuzzy(grid, table, p: MotorParams = MotorParams(), seed=1, maxiter=60):
    r = _otimizar(lambda x: make_fuzzy(x, grid, table, p), limites_fuzzy(p), p, seed, maxiter)
    return {"Ke": float(r.x[0]), "Kde": float(r.x[1]), "Ku": float(r.x[2]),
            "custo": float(r.fun), "avaliacoes": int(r.nfev)}
