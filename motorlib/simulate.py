"""Simulação em malha fechada e métricas – as mesmas para todos os controladores.

Cenários (mesmo período de amostragem e mesma saturação de tensão para todos):
    nominal     degrau de referência, parâmetros nominais
    carga       degrau de referência + degrau de torque de carga em T_CARGA
    inercia_x2  degrau com J x2 e b +20 % (planta diferente da usada na sintonia).
                Com b +50 % a referência ficaria inalcançável com 12 V (exigiria 12,01 V)
                e o cenário mediria só saturação – por isso +20 % (exige 9,6 V).
    ruido       degrau com ruído gaussiano na medição de velocidade

Tudo é escalado pelos parâmetros do motor, então o mesmo código serve para o modelo
didático e para um motor identificado na bancada:
    referência  = 2/3 da velocidade máxima em regime (v_max * ganho DC)
    carga       = 5 % do torque de partida (K * v_max / R)
    ruído       = 1,25 % da referência
"""
from dataclasses import dataclass

import numpy as np

from .plant import DCMotor, MotorParams

TS = 0.01             # período de amostragem [s] (100 Hz) – o mesmo do sketch Arduino
T_FINAL = 6.0         # duração [s]
T_CARGA = 3.0         # instante do degrau de carga [s]
FRAC_REF = 2.0 / 3.0
FRAC_CARGA = 0.05
FRAC_RUIDO = 0.0125
FAIXA = 0.02          # faixa de acomodação (2 %)


@dataclass
class Cenario:
    nome: str
    params: MotorParams
    ref: float
    torque: float = 0.0
    ruido: float = 0.0


def ref_padrao(p: MotorParams):
    return FRAC_REF * p.v_max * p.dc_gain


def cenarios(p: MotorParams = MotorParams()):
    ref = ref_padrao(p)
    return {
        "nominal": Cenario("nominal", p, ref),
        "carga": Cenario("carga", p, ref, torque=FRAC_CARGA * p.K * p.v_max / p.R),
        "inercia_x2": Cenario("inercia_x2", p.scaled(J=2.0, b=1.2), ref),
        "ruido": Cenario("ruido", p, ref, ruido=FRAC_RUIDO * ref),
    }


def simular(ctrl, c: Cenario, Ts=TS, t_final=T_FINAL, seed=0):
    motor = DCMotor(c.params, Ts)
    ctrl.reset()
    rng = np.random.default_rng(seed)
    n = int(round(t_final / Ts))
    t = np.arange(n) * Ts
    w = np.zeros(n)
    u = np.zeros(n)
    y = 0.0
    for k in range(n):
        y_med = y + (rng.normal(0.0, c.ruido) if c.ruido else 0.0)
        u[k] = ctrl.update(c.ref, y_med)
        w[k] = y
        y = motor.step(u[k], c.torque if t[k] >= T_CARGA else 0.0)
    return t, w, u


def _ultimo_fora(t, y, r, faixa=FAIXA):
    fora = np.nonzero(np.abs(y - r) > faixa * abs(r))[0]
    if len(fora) == 0:
        return t[0]
    if fora[-1] == len(t) - 1:
        return np.inf  # não acomodou dentro da janela
    return t[fora[-1] + 1]


def metricas_degrau(t, y, u, r, v_max):
    e = r - y
    i10 = np.argmax(y >= 0.1 * r) if np.any(y >= 0.1 * r) else None
    i90 = np.argmax(y >= 0.9 * r) if np.any(y >= 0.9 * r) else None
    ultimos = t >= t[-1] - 0.5
    dt = t[1] - t[0]
    return {
        "overshoot_pct": float(max(0.0, (y.max() - r) / r * 100)),
        "subida_10_90_s": float(t[i90] - t[i10]) if (i10 is not None and i90 is not None) else np.inf,
        "acomodacao_2pct_s": float(_ultimo_fora(t, y, r) - t[0]),
        "erro_regime_pct": float(np.mean(np.abs(e[ultimos])) / r * 100),
        "IAE": float(np.sum(np.abs(e)) * dt),
        "ITAE": float(np.sum(t * np.abs(e)) * dt),
        "u_max_V": float(np.max(np.abs(u))),
        "tempo_saturado_pct": float(np.mean(np.abs(u) >= v_max * (1 - 1e-6)) * 100),
        "variacao_total_u_V": float(np.sum(np.abs(np.diff(u)))),
    }


def metricas_carga(t, y, r, t0=T_CARGA):
    m = t >= t0
    tt, yy = t[m], y[m]
    return {
        "queda_max_pct": float(max(0.0, (r - yy.min()) / r * 100)),
        "recuperacao_2pct_s": float(_ultimo_fora(tt, yy, r) - t0),
        "IAE_carga": float(np.sum(np.abs(r - yy)) * (t[1] - t[0])),
    }


def avaliar(ctrl, p: MotorParams = MotorParams()):
    """Roda os 4 cenários; devolve ({cenario: {metrica: valor}}, {cenario: (t, y, u)})."""
    res, series = {}, {}
    for nome, c in cenarios(p).items():
        t, y, u = simular(ctrl, c)
        series[nome] = (t, y, u)
        if nome == "carga":
            res[nome] = metricas_carga(t[t < T_FINAL], y, c.ref)
        else:
            res[nome] = metricas_degrau(t, y, u, c.ref, c.params.v_max)
    return res, series
