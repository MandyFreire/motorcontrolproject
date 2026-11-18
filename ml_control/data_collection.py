"""Gera o dataset de clonagem do PID: episódios em malha fechada com referência, carga e
ruído aleatórios, sobre a planta simulada.

    python ml_control/data_collection.py   ->  ml_control/motor_data.npz

O que mudou em relação à versão original e por quê:
  - a "velocidade" era ruído N(95, 5) independente do controle, sem dinâmica; agora vem
    da planta em malha fechada;
  - o alvo era o ERRO (que é função exata das entradas: vazamento de rótulo); agora é a
    saída do PID antes da saturação;
  - entradas = estados internos do PID [e, integral(e), derivada filtrada de y]: sem o
    estado integral, nenhuma rede reproduz a ação integral e sobra erro em regime;
  - setpoint constante (variância zero) virou referência aleatória por episódio.
"""
import json

import _caminho  # noqa: F401
import numpy as np

from motorlib.ml import coletar
from motorlib.tuning import make_pid


def main():
    g = json.loads((_caminho.RAIZ / "results" / "ctms" / "ganhos.json").read_text())["pid"]
    X, y, ep = coletar(make_pid([g["Kp"], g["Ki"], g["Kd"]]))
    destino = _caminho.RAIZ / "ml_control" / "motor_data.npz"
    np.savez(destino, X=X, y=y, episodio=ep)
    print(f"{len(X)} amostras, {len(np.unique(ep))} episódios -> {destino}")


if __name__ == "__main__":
    main()
