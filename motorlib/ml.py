"""Clonagem de comportamento do PID por uma rede MLP pequena (3-8-1, ReLU).

O que isto É: um aproximador do PID sintonizado. A rede recebe os mesmos estados internos
que o PID usa (erro, integral do erro, derivada filtrada da medição) e aprende a saída
do PID antes da saturação.

O que isto NÃO É: um controlador adaptativo. Não há aprendizado online nem estado além
dos do PID; por construção, o melhor resultado possível é igualar o PID. Qualquer diferença
de desempenho em malha fechada é efeito do erro de aproximação – e é medida, não presumida.

Dados: episódios em malha fechada com o PID sintonizado sobre a planta simulada, com
referência, carga e ruído aleatórios. Divisão treino/validação POR EPISÓDIO (amostras do
mesmo episódio são correlacionadas no tempo; misturá-las inflaria a métrica de validação).
"""
import numpy as np
from sklearn.neural_network import MLPRegressor

from .plant import DCMotor, MotorParams
from .simulate import FRAC_RUIDO, TS, ref_padrao

N_EPISODIOS = 20
DURACAO_EP = 3.0  # s -> 300 amostras/episódio a 100 Hz -> 6000 amostras


def coletar(pid, n_episodios=N_EPISODIOS, duracao=DURACAO_EP, seed=0, params=MotorParams()):
    rng = np.random.default_rng(seed)
    n = int(round(duracao / TS))
    ref_nom = ref_padrao(params)
    torque_max = 0.08 * params.K * params.v_max / params.R
    sigma = FRAC_RUIDO * ref_nom
    X, y, ep = [], [], []
    for k_ep in range(n_episodios):
        motor = DCMotor(params, TS)
        pid.reset()
        ref = rng.uniform(0.25, 1.25) * ref_nom
        t_carga = rng.uniform(0.5, duracao)
        torque = rng.uniform(0.0, torque_max)
        w = rng.uniform(0.0, 0.6) * ref_nom
        motor.reset(w0=w)
        for k in range(n):
            u = pid.update(ref, w + rng.normal(0.0, sigma))
            X.append([pid.e, pid.integ, pid.d])
            y.append(pid.u_raw)
            ep.append(k_ep)
            w = motor.step(u, torque if k * TS >= t_carga else 0.0)
    return np.array(X), np.array(y), np.array(ep)


def treinar(X, y, ep, n_hidden=8, frac_val=0.2, seed=0):
    eps = np.unique(ep)
    n_val = max(1, int(round(frac_val * len(eps))))
    val_eps = np.random.default_rng(seed).choice(eps, n_val, replace=False)
    val = np.isin(ep, val_eps)

    x_mean, x_std = X[~val].mean(0), X[~val].std(0)
    y_mean, y_std = y[~val].mean(), y[~val].std()
    Xn, yn = (X - x_mean) / x_std, (y - y_mean) / y_std

    reg = MLPRegressor(hidden_layer_sizes=(n_hidden,), activation="relu", solver="lbfgs",
                       max_iter=5000, tol=1e-10, random_state=seed)
    reg.fit(Xn[~val], yn[~val])

    def rmse(m):
        return float(np.sqrt(np.mean((reg.predict(Xn[m]) * y_std + y_mean - y[m]) ** 2)))

    pesos = {
        "W1": reg.coefs_[0].tolist(), "b1": reg.intercepts_[0].tolist(),
        "W2": reg.coefs_[1][:, 0].tolist(), "b2": float(reg.intercepts_[1][0]),
        "x_mean": x_mean.tolist(), "x_std": x_std.tolist(),
        "y_mean": float(y_mean), "y_std": float(y_std),
    }
    info = {
        "amostras_treino": int((~val).sum()), "amostras_validacao": int(val.sum()),
        "episodios_validacao": sorted(int(v) for v in val_eps),
        "rmse_treino_V": rmse(~val), "rmse_validacao_V": rmse(val),
        "parametros_treinaveis": int(X.shape[1] * n_hidden + n_hidden + n_hidden + 1),
    }
    return pesos, info
