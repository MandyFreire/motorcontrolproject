"""Fuzzy PI incremental: superfície de controle, verificação da base de regras e resposta
em malha fechada.

    python simulations/fuzzy/fuzzy_controller.py [--salvar pasta/]

Correções em relação à versão original:
  - base 5x5 COMPLETA (25 regras) em vez de 3 regras: a original deixava 25 % do espaço de
    entradas sem saída (scikit-fuzzy lança exceção);
  - convenção de sinal única no projeto, e = r - y: erro positivo -> aumentar tensão
    (a original fazia o contrário);
  - universos com np.linspace (inclui 0 e os extremos exatos); com np.arange, o 0 não
    existia e um ponto só tinha saída por um resíduo de ponto flutuante de 3,6e-15;
  - existe agora uma malha fechada com a planta (antes o controlador era avaliado num ponto).
"""
import argparse
import json
import pathlib

import _caminho  # noqa: F401
import matplotlib.pyplot as plt
import numpy as np

from motorlib.fuzzy import REGRAS, ROTULOS, surface_table
from motorlib.plant import MotorParams
from motorlib.simulate import cenarios, metricas_degrau, simular
from motorlib.tuning import make_fuzzy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--salvar", help="pasta onde salvar as figuras em vez de abrir janelas")
    a = ap.parse_args()

    print("Base de regras (linhas: e, colunas: de):")
    print("      " + "  ".join(f"{r:>3s}" for r in ROTULOS))
    for i, r in enumerate(ROTULOS):
        print(f"  {r:>3s} " + "  ".join(f"{ROTULOS[REGRAS[i][j]]:>3s}" for j in range(5)))

    grid, tab = surface_table()
    print(f"\nSuperfície {len(grid)}x{len(grid)}: todos os pontos definidos = {np.all(np.isfinite(tab))}")

    ganhos = _caminho.RAIZ / "results" / "ctms" / "ganhos.json"
    g = json.loads(ganhos.read_text())["fuzzy"] if ganhos.exists() else {"Ke": 1.34, "Kde": 37.6, "Ku": 0.64}
    p = MotorParams()
    c = cenarios(p)["nominal"]
    t, y, u = simular(make_fuzzy([g["Ke"], g["Kde"], g["Ku"]], grid, tab, p), c)
    print(f"\nMalha fechada (Ke={g['Ke']:.3g}, Kde={g['Kde']:.3g}, Ku={g['Ku']:.3g}):")
    for k, v in metricas_degrau(t, y, u, c.ref, p.v_max).items():
        print(f"  {k:22s} {v:.4g}")

    E, D = np.meshgrid(grid, grid, indexing="ij")
    fig = plt.figure(figsize=(11, 4.5))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.plot_surface(E, D, tab, cmap="coolwarm")
    ax.set_xlabel("e normalizado")
    ax.set_ylabel("Δe normalizado")
    ax.set_zlabel("Δu normalizado")
    ax.set_title("Superfície de controle")
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(t, y, label="velocidade")
    ax2.axhline(c.ref, color="k", ls=":", label="referência")
    ax2.set_xlabel("tempo (s)")
    ax2.set_ylabel("rad/s")
    ax2.set_title("Degrau – Fuzzy PI (CTMS)")
    ax2.grid(alpha=0.3)
    ax2.legend()
    fig.tight_layout()
    if a.salvar:
        pathlib.Path(a.salvar).mkdir(parents=True, exist_ok=True)
        fig.savefig(pathlib.Path(a.salvar) / "fuzzy.png", dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()
