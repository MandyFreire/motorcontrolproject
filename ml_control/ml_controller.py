"""Avalia a MLP em malha fechada contra o PID que ela clonou, nos 4 cenários.

    python ml_control/ml_controller.py   (depois de model_training.py)

Na versão original, a rede recebia 0 no lugar do sinal de controle (fora da distribuição de
treino) e sua saída – que era uma previsão do ERRO – era usada como tensão.
"""
import json

import _caminho  # noqa: F401

from motorlib.controllers import MLPController
from motorlib.plant import MotorParams
from motorlib.simulate import TS, avaliar
from motorlib.tuning import make_pid


def main():
    p = MotorParams()
    pesos = json.loads((_caminho.RAIZ / "ml_control" / "mlp_pesos.json").read_text())
    g = json.loads((_caminho.RAIZ / "results" / "ctms" / "ganhos.json").read_text())["pid"]
    r_pid, _ = avaliar(make_pid([g["Kp"], g["Ki"], g["Kd"]], p), p)
    r_mlp, _ = avaliar(MLPController(pesos, TS, -p.v_max, p.v_max), p)
    for cen in r_pid:
        print(f"\n[{cen}]  {'métrica':22s} {'PID':>10s} {'MLP':>10s}")
        for k in r_pid[cen]:
            print(f"         {k:22s} {r_pid[cen][k]:10.4g} {r_mlp[cen][k]:10.4g}")


if __name__ == "__main__":
    main()
