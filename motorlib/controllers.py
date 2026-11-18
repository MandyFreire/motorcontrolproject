"""Controladores discretos. Todos expõem a mesma interface:

    reset()
    update(r, y) -> u        # r: referência, y: medição, u: tensão saturada [V]

As mesmas equações estão portadas, linha a linha, nos sketches de arduino/.
"""
import numpy as np


def _clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class PID:
    """PID paralelo discreto, pensado para ser idêntico à versão embarcada.

    - Derivada sobre a MEDIÇÃO (evita o "derivative kick" no degrau de referência),
      filtrada por 1ª ordem com constante Tf.
    - Integração por Euler regressivo.
    - Anti-windup por integração condicional (clamping): o integrador congela quando
      a saída está saturada e o erro empurra no mesmo sentido da saturação.
    """

    def __init__(self, Kp, Ki, Kd, Ts, u_min, u_max, Tf=None):
        self.Kp, self.Ki, self.Kd, self.Ts = Kp, Ki, Kd, Ts
        self.u_min, self.u_max = u_min, u_max
        self.Tf = 2 * Ts if Tf is None else Tf
        self.alpha = self.Tf / (self.Tf + Ts)
        self.reset()

    def reset(self):
        self.integ = 0.0     # integral do erro [unid.erro * s]
        self.d = 0.0         # derivada filtrada da medição
        self.y_prev = None
        self.u_raw = 0.0     # saída antes da saturação (usada como alvo do ML)

    def update(self, r, y):
        e = r - y
        if self.y_prev is None:
            self.y_prev = y
        self.d = self.alpha * self.d + (1 - self.alpha) * (y - self.y_prev) / self.Ts
        self.y_prev = y

        integ_new = self.integ + e * self.Ts
        u_raw = self.Kp * e + self.Ki * integ_new - self.Kd * self.d
        u = _clip(u_raw, self.u_min, self.u_max)
        saturado_e_piorando = (u_raw > self.u_max and e > 0) or (u_raw < self.u_min and e < 0)
        if not saturado_e_piorando:
            self.integ = integ_new
        else:  # recalcula sem integrar
            u_raw = self.Kp * e + self.Ki * self.integ - self.Kd * self.d
            u = _clip(u_raw, self.u_min, self.u_max)
        self.u_raw = u_raw
        self.e = e
        return u


class FuzzyPI:
    """Fuzzy tipo PI incremental (Mamdani, base 5x5 completa).

        e_n  = sat(Ke  * e)
        de_n = sat(Kde * (e_k - e_{k-1}))
        u_k  = sat(u_{k-1} + Ku * F(e_n, de_n))

    F é a superfície de controle gerada com scikit-fuzzy (motorlib.fuzzy) e avaliada aqui
    por interpolação bilinear numa tabela – a MESMA tabela exportada para o Arduino.
    A forma incremental dá ação integral, logo erro nulo em regime para degrau.
    A saturação de u_k é o próprio anti-windup.
    """

    def __init__(self, table, grid, Ke, Kde, Ku, u_min, u_max):
        self.table = np.asarray(table, dtype=float)
        self.grid = np.asarray(grid, dtype=float)
        self.n = len(grid)
        self.Ke, self.Kde, self.Ku = Ke, Kde, Ku
        self.u_min, self.u_max = u_min, u_max
        self._tab = self.table.tolist()
        self.reset()

    def reset(self):
        self.u = 0.0
        self.e_prev = None

    def surface(self, en, den):
        """Interpolação bilinear em grade uniforme [-1, 1] x [-1, 1]."""
        n = self.n
        fx = (_clip(en, -1.0, 1.0) + 1.0) * 0.5 * (n - 1)
        fy = (_clip(den, -1.0, 1.0) + 1.0) * 0.5 * (n - 1)
        i = min(int(fx), n - 2)
        j = min(int(fy), n - 2)
        tx, ty = fx - i, fy - j
        t = self._tab
        return ((1 - tx) * (1 - ty) * t[i][j] + tx * (1 - ty) * t[i + 1][j]
                + (1 - tx) * ty * t[i][j + 1] + tx * ty * t[i + 1][j + 1])

    def update(self, r, y):
        e = r - y
        if self.e_prev is None:
            self.e_prev = e
        du = self.Ku * self.surface(self.Ke * e, self.Kde * (e - self.e_prev))
        self.e_prev = e
        self.u = _clip(self.u + du, self.u_min, self.u_max)
        return self.u


class MLPController:
    """Rede neural treinada por clonagem de comportamento do PID (ver ml_control/).

    Entradas: [e, integral(e), derivada filtrada de y] – exatamente os estados internos do PID.
    Uma rede só com (e, de/dt) e sem estado integral NÃO consegue reproduzir a ação integral
    e deixaria erro em regime; por isso o integrador e o filtro ficam fora da rede, com a
    mesma lógica de anti-windup do PID. A rede aprende apenas o mapeamento estático.
    Consequência: por construção ela aproxima o PID; não há mecanismo que a faça superá-lo
    nem se adaptar a mudanças de carga (não há aprendizado online).
    """

    def __init__(self, weights, Ts, u_min, u_max, Tf=None):
        self.W1 = np.asarray(weights["W1"], float)  # (n_in, n_hidden)
        self.b1 = np.asarray(weights["b1"], float)
        self.W2 = np.asarray(weights["W2"], float)  # (n_hidden,)
        self.b2 = float(weights["b2"])
        self.x_mean = np.asarray(weights["x_mean"], float)
        self.x_std = np.asarray(weights["x_std"], float)
        self.y_mean, self.y_std = float(weights["y_mean"]), float(weights["y_std"])
        self.Ts, self.u_min, self.u_max = Ts, u_min, u_max
        self.Tf = 2 * Ts if Tf is None else Tf
        self.alpha = self.Tf / (self.Tf + Ts)
        self.reset()

    def reset(self):
        self.integ, self.d, self.y_prev = 0.0, 0.0, None

    def forward(self, x):
        z = (np.asarray(x, float) - self.x_mean) / self.x_std
        h = np.maximum(0.0, z @ self.W1 + self.b1)
        return float(h @ self.W2 + self.b2) * self.y_std + self.y_mean

    def update(self, r, y):
        e = r - y
        if self.y_prev is None:
            self.y_prev = y
        self.d = self.alpha * self.d + (1 - self.alpha) * (y - self.y_prev) / self.Ts
        self.y_prev = y
        integ_new = self.integ + e * self.Ts
        u_raw = self.forward([e, integ_new, self.d])
        u = _clip(u_raw, self.u_min, self.u_max)
        if (u_raw > self.u_max and e > 0) or (u_raw < self.u_min and e < 0):
            u_raw = self.forward([e, self.integ, self.d])
            u = _clip(u_raw, self.u_min, self.u_max)
        else:
            self.integ = integ_new
        return u
