"""Treina a MLP 3-8-1 (ReLU) que clona o PID e compara com uma regressão linear.

    python ml_control/model_training.py   (depois de data_collection.py)
    ->  ml_control/mlp_pesos.json

Validação separada POR EPISÓDIO. A comparação com regressão linear não é enfeite: o alvo
(saída do PID) é uma combinação linear exata das entradas, então a regressão linear acerta
com erro de máquina. A rede é útil aqui como exercício de pipeline embarcado – não como
melhoria de desempenho.
"""
import json

import _caminho  # noqa: F401
import numpy as np

from motorlib.ml import treinar


def main():
    d = np.load(_caminho.RAIZ / "ml_control" / "motor_data.npz")
    X, y, ep = d["X"], d["y"], d["episodio"]
    pesos, info = treinar(X, y, ep)
    val = np.isin(ep, info["episodios_validacao"])
    A = np.c_[X, np.ones(len(X))]
    coef, *_ = np.linalg.lstsq(A[~val], y[~val], rcond=None)
    rmse_lin = float(np.sqrt(np.mean((A[val] @ coef - y[val]) ** 2)))
    for k, v in info.items():
        print(f"{k:28s} {v}")
    print(f"{'rmse_validacao_linear_V':28s} {rmse_lin:.3g}")
    print("coeficientes lineares [e, integral, derivada, bias]:", np.round(coef, 6))
    (_caminho.RAIZ / "ml_control" / "mlp_pesos.json").write_text(json.dumps(pesos, indent=2))


if __name__ == "__main__":
    main()
