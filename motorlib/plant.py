"""Modelo do motor DC (velocidade) em espaço de estados, discretizado por ZOH.

Parâmetros nominais: tutorial "DC Motor Speed: System Modeling" dos Control Tutorials
for MATLAB & Simulink (CTMS), University of Michigan. É um modelo DIDÁTICO: não
representa um motor físico específico. Para o motor da bancada, identifique os
parâmetros com simulations/identificacao/identify_motor.py.

Estados x = [w, i]   (velocidade angular [rad/s], corrente de armadura [A])
Entradas  [V, T_L]   (tensão de armadura [V], torque de carga [N.m])

    J dw/dt = K i - b w - T_L
    L di/dt = V - R i - K w
"""
from dataclasses import dataclass, replace

import numpy as np
from scipy.signal import cont2discrete


@dataclass(frozen=True)
class MotorParams:
    J: float = 0.01     # inércia do rotor [kg.m^2]
    b: float = 0.1      # atrito viscoso [N.m.s]
    K: float = 0.01     # constante de torque = constante de FCEM [N.m/A] = [V.s/rad]
    R: float = 1.0      # resistência de armadura [ohm]
    L: float = 0.5      # indutância de armadura [H]
    v_max: float = 12.0  # limite de tensão do driver [V] (saturação simétrica)

    def scaled(self, **fatores):
        """Cópia com parâmetros multiplicados, ex.: p.scaled(J=2.0)."""
        return replace(self, **{k: getattr(self, k) * f for k, f in fatores.items()})

    @property
    def dc_gain(self):
        """Ganho estático tensão -> velocidade [(rad/s)/V]."""
        return self.K / (self.b * self.R + self.K ** 2)

    def tf_coeffs(self):
        """num, den de W(s)/V(s) – idênticos aos usados originalmente no repositório."""
        p = self
        return [p.K], [p.J * p.L, p.J * p.R + p.L * p.b, p.b * p.R + p.K ** 2]


class DCMotor:
    """Planta discreta exata (ZOH) com período Ts. Integração sem erro de discretização
    para entradas constantes durante o período, que é o caso de um PWM atualizado a cada Ts."""

    def __init__(self, params: MotorParams = MotorParams(), Ts: float = 0.01):
        p = params
        A = np.array([[-p.b / p.J, p.K / p.J],
                      [-p.K / p.L, -p.R / p.L]])
        B = np.array([[0.0, -1.0 / p.J],
                      [1.0 / p.L, 0.0]])
        Ad, Bd, *_ = cont2discrete((A, B, np.eye(2), np.zeros((2, 2))), Ts, method="zoh")
        # floats Python puros: ~10x mais rápido que numpy para 2x2 num laço escalar
        self._a = [[float(v) for v in row] for row in Ad]
        self._b = [[float(v) for v in row] for row in Bd]
        self.params, self.Ts = p, Ts
        self.reset()

    def reset(self, w0=0.0, i0=0.0):
        self.w, self.i = float(w0), float(i0)

    def step(self, v, load=0.0):
        """Aplica tensão v (já saturada pelo chamador) e torque de carga durante Ts."""
        a, b = self._a, self._b
        w = a[0][0] * self.w + a[0][1] * self.i + b[0][0] * v + b[0][1] * load
        i = a[1][0] * self.w + a[1][1] * self.i + b[1][0] * v + b[1][1] * load
        self.w, self.i = w, i
        return w
