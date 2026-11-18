"""Identificação dos parâmetros mecânicos J e b a partir de um ensaio em malha aberta.

Ensaio: o sketch Arduino com MODO_CONTROLE = 0 aplica degraus de tensão e registra
t_ms, ref, w_rad_s, u_V, exec_us (CSV).

Por que K, R e L entram como MEDIDAS e não como incógnitas: com L pequeno (típico de
motores reais), a resposta ao degrau só revela dois números – o ganho DC
G = K/(bR + K^2) e a constante de tempo mecânica tau = JR/(bR + K^2). Três incógnitas
(J, b, K) a partir de dois números não são identificáveis: infinitas combinações dão a
mesma curva (foi verificado com dados sintéticos: o ajuste das três converge para valores
errados). Com K, R e L medidos, J e b ficam determinados.

Como medir:
    R  multímetro nos terminais, rotor travado, várias posições (use a média)
    K  [V.s/rad] gire o eixo com outro motor/furadeira e meça a tensão em aberto e a
       velocidade (encoder): K = V / w.  Alternativa com o motor energizado em regime:
       K = (V - R*I) / w, medindo a corrente I.
    L  medidor LCR ou datasheet. Se desconhecido, use um valor pequeno (ex.: 1 mH);
       o polo elétrico é rápido demais para afetar o controle a 100 Hz.
    v_max  tensão efetivamente entregue ao motor com PWM 100 % (meça: a L298N perde ~2 V)

Ajuste: mínimos quadrados do erro de simulação usando a MESMA classe DCMotor do projeto
e a MESMA sequência de tensões registrada; parâmetros em escala log (positividade);
chute inicial calculado do próprio ensaio (ganho e constante de tempo do 1º degrau).
"""
import csv
import json

import numpy as np
from scipy.optimize import least_squares

from .plant import DCMotor, MotorParams


def ler_csv(caminho):
    t, w, u = [], [], []
    with open(caminho, newline="") as f:
        for linha in csv.DictReader(l for l in f if not l.startswith("#")):
            t.append(float(linha["t_ms"]) / 1000.0)
            w.append(float(linha["w_rad_s"]))
            u.append(float(linha["u_V"]))
    return np.array(t), np.array(w), np.array(u)


def simular_aberta(params, u, Ts, w0=0.0):
    m = DCMotor(params, Ts)
    m.reset(w0=w0)
    w = np.empty(len(u))
    for k, uk in enumerate(u):
        w[k] = m.w
        m.step(uk)
    return w


def chute_inicial(t, w, u, K, R):
    """Estima G e tau no primeiro degrau não nulo e converte para (J, b)."""
    i0 = int(np.argmax(np.abs(u) > 0))
    mud = np.nonzero(np.abs(np.diff(u[i0:])) > 1e-9)[0]
    i1 = i0 + (mud[0] + 1 if len(mud) else len(u) - i0)
    seg_w, seg_t, v = w[i0:i1], t[i0:i1], u[i0]
    w_ss = float(np.mean(seg_w[-max(3, len(seg_w) // 10):]))
    G = w_ss / v
    alvo = w[i0] + 0.632 * (w_ss - w[i0])
    k63 = int(np.argmax(seg_w >= alvo)) if w_ss > w[i0] else len(seg_w) // 3
    tau = max(float(seg_t[k63] - seg_t[0]), 2 * float(np.median(np.diff(t))))
    b = max((K / G - K ** 2) / R, 1e-9)
    J = tau * (b * R + K ** 2) / R
    return J, b


def identificar(t, w, u, K, R, L, v_max):
    Ts = float(np.median(np.diff(t)))
    if np.max(np.abs(np.diff(t) - Ts)) > 0.2 * Ts:
        raise ValueError("amostragem irregular no log (jitter > 20 % de Ts) – verifique o sketch")

    def montar(z):
        J, b = np.exp(z)
        return MotorParams(J=J, b=b, K=K, R=R, L=L, v_max=v_max)

    def resid(z):
        return simular_aberta(montar(z), u, Ts, w[0]) - w

    z0 = np.log(chute_inicial(t, w, u, K, R))
    r = least_squares(resid, z0, method="trf", x_scale="jac", max_nfev=2000)
    p = montar(r.x)
    w_fit = simular_aberta(p, u, Ts, w[0])
    rmse = float(np.sqrt(np.mean((w_fit - w) ** 2)))
    r2 = float(1 - np.sum((w_fit - w) ** 2) / np.sum((w - w.mean()) ** 2))
    return p, {"Ts_s": Ts, "rmse_rad_s": rmse, "R2": r2, "amostras": int(len(t)),
               "sucesso": bool(r.success), "mensagem": str(r.message)}, w_fit


def salvar_params(p: MotorParams, caminho, info=None):
    d = {k: getattr(p, k) for k in ("J", "b", "K", "R", "L", "v_max")}
    if info:
        d["_identificacao"] = info
    with open(caminho, "w") as f:
        json.dump(d, f, indent=2)


def carregar_params(caminho):
    with open(caminho) as f:
        d = json.load(f)
    return MotorParams(**{k: float(d[k]) for k in ("J", "b", "K", "R", "L", "v_max")})
