"""Sintonia automática do PID por otimização (evolução diferencial).

    python simulations/auto_tuning/auto_tune_pid.py [--params results/motor_identificado.json]

Por que a versão original não servia (além de não executar):
  - minimizava SÓ o overshoot, numa janela em que nenhuma resposta acomodava; 74 de 75
    candidatos empatavam em 0 % e vencia o primeiro da grade – o controlador mais lento;
  - a grade (Kp <= 2) ficava ~10x abaixo da região útil para esta planta.

Critério atual (motorlib/tuning.py): ITAE + penalidade de overshoot acima de 5 % +
penalidade de variação do sinal de controle sob ruído de medição. O mesmo critério é
usado para o Fuzzy, o que torna a comparação justa.
"""
import argparse

import _caminho  # noqa: F401

from motorlib.identificacao import carregar_params
from motorlib.plant import MotorParams
from motorlib.simulate import avaliar
from motorlib.tuning import limites_pid, make_pid, sintonizar_pid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", help="JSON de motor identificado (padrão: modelo CTMS)")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    p = carregar_params(a.params) if a.params else MotorParams()
    print("limites de busca (Kp, Ki, Kd):", [tuple(round(v, 3) for v in l) for l in limites_pid(p)])
    g = sintonizar_pid(p, seed=a.seed)
    print(f"Kp={g['Kp']:.5g}  Ki={g['Ki']:.5g}  Kd={g['Kd']:.3g}  custo={g['custo']:.4g}  "
          f"({g['avaliacoes']} simulações)")
    res, _ = avaliar(make_pid([g["Kp"], g["Ki"], g["Kd"]], p), p)
    for cen, m in res.items():
        print(cen, {k: round(v, 4) for k, v in m.items()})


if __name__ == "__main__":
    main()
