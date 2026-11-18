"""Métricas de um ensaio de BANCADA, calculadas com as MESMAS funções da simulação.

    python analisar_ensaio.py ensaio_pid.csv
    python analisar_ensaio.py ensaio_pid.csv --params results/motor_identificado.json --controlador pid

Com --params, simula o mesmo controlador no modelo identificado e mostra lado a lado
simulação x bancada: a diferença entre as duas colunas é o "buraco" entre teoria e prática,
medido em vez de narrado.

Reporta também o tempo de execução real do laço (exec_us) e os estouros de período.
"""
import argparse
import csv
import json
import pathlib

import numpy as np

from motorlib.controllers import MLPController
from motorlib.fuzzy import surface_table
from motorlib.identificacao import _linhas_ultima_execucao, carregar_params
from motorlib.simulate import TS, Cenario, metricas_degrau, simular
from motorlib.tuning import make_fuzzy, make_pid


def ler(caminho):
    cols = {k: [] for k in ("t_ms", "ref", "w_rad_s", "u_V", "exec_us", "estouros")}
    for linha in csv.DictReader(_linhas_ultima_execucao(caminho)):
        for k in cols:
            cols[k].append(float(linha[k]))
    return {k: np.array(v) for k, v in cols.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--params", help="JSON do motor identificado, para comparar com simulação")
    ap.add_argument("--controlador", choices=["pid", "fuzzy", "mlp"], default="pid")
    ap.add_argument("--resultados", default="results/identificado",
                    help="pasta com ganhos.json e mlp_pesos.json gerados por run_all.py --params")
    ap.add_argument("--duracao", type=float, default=6.0, help="janela analisada após o degrau [s]")
    a = ap.parse_args()

    d = ler(a.csv)
    degrau = np.nonzero(np.diff(d["ref"]) > 0)[0]
    if len(degrau) == 0:
        raise SystemExit("nenhum degrau de referência encontrado no log")
    k0 = degrau[0] + 1
    t = (d["t_ms"] - d["t_ms"][k0]) / 1000.0
    m = (t >= 0) & (t < a.duracao)
    ref = float(d["ref"][k0])
    t, w, u = t[m], d["w_rad_s"][m], d["u_V"][m]
    if np.any(np.diff(d["ref"][k0:][m[k0:]]) != 0):
        print("ATENÇÃO: a referência mudou dentro da janela analisada")

    dt = np.diff(d["t_ms"]) / 1000.0
    resultado = {"bancada": None}
    if a.params:
        p = carregar_params(a.params)
        v_max = p.v_max
    else:
        v_max = float(np.max(np.abs(d["u_V"])))  # sem params: usa o maior |u| registrado
    resultado["bancada"] = metricas_degrau(t, w, u, ref, v_max)

    ex = d["exec_us"]
    tempo = {
        "Ts_medio_ms": float(np.mean(dt) * 1000), "jitter_max_ms": float(np.max(np.abs(dt - np.median(dt))) * 1000),
        "exec_us_medio": float(np.mean(ex)), "exec_us_p99": float(np.percentile(ex, 99)),
        "exec_us_max": float(np.max(ex)), "estouros_total": int(d["estouros"][-1]),
    }

    if a.params:
        c = Cenario("bancada", p, ref)
        g = json.loads((pathlib.Path(a.resultados) / "ganhos.json").read_text())
        if a.controlador == "pid":
            ctrl = make_pid([g["pid"]["Kp"], g["pid"]["Ki"], g["pid"]["Kd"]], p)
        elif a.controlador == "fuzzy":
            grid, tab = surface_table()
            ctrl = make_fuzzy([g["fuzzy"]["Ke"], g["fuzzy"]["Kde"], g["fuzzy"]["Ku"]], grid, tab, p)
        else:
            pesos = json.loads((pathlib.Path(a.resultados) / "mlp_pesos.json").read_text())
            ctrl = MLPController(pesos, TS, -p.v_max, p.v_max)
        ts, ws, us = simular(ctrl, c, t_final=a.duracao)
        resultado["simulacao"] = metricas_degrau(ts, ws, us, ref, p.v_max)

    print(f"\nEnsaio: {a.csv}   referência = {ref:.4g} rad/s   amostras = {len(t)}")
    cols = list(resultado)
    print(f"{'métrica':24s}" + "".join(f"{c:>14s}" for c in cols))
    for k in resultado["bancada"]:
        print(f"{k:24s}" + "".join(f"{resultado[c][k]:14.4g}" for c in cols))
    print("\nTempo real do laço no microcontrolador:")
    for k, v in tempo.items():
        print(f"  {k:18s} {v:.4g}")
    if tempo["estouros_total"]:
        print("ATENÇÃO: houve estouro do período de amostragem – resultados não comparáveis à simulação.")


if __name__ == "__main__":
    main()
