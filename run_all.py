"""Pipeline completo e reprodutível: sintonia -> treino -> avaliação -> figuras -> firmware.

    python run_all.py                                  # modelo didático CTMS
    python run_all.py --params results/motor_identificado.json --arduino
                                                       # motor da bancada (após identificação)

Saídas em results/<nome>/: metricas.json, metricas.md, ganhos.json, mlp_pesos.json,
figuras PNG. Com --arduino, regenera arduino/motor_control/parametros_gerados.h.
Tudo determinístico (sementes fixas).
"""
import argparse
import json
import pathlib
import platform

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from motorlib.controllers import MLPController
from motorlib.export_c import gerar_header
from motorlib.fuzzy import surface_table
from motorlib.identificacao import carregar_params
from motorlib.ml import coletar, treinar
from motorlib.plant import MotorParams
from motorlib.simulate import TS, avaliar, cenarios, metricas_degrau, simular
from motorlib.tuning import make_fuzzy, make_pid, sintonizar_fuzzy, sintonizar_pid

RAIZ = pathlib.Path(__file__).resolve().parent
NOMES = {"pid": "PID", "fuzzy": "Fuzzy PI", "mlp": "MLP (clone do PID)"}
CORES = {"pid": "#1f77b4", "fuzzy": "#d62728", "mlp": "#2ca02c"}


def _fmt(v, nd=3):
    return "não atingido em 6 s" if not np.isfinite(v) else f"{v:.{nd}f}"


def tabela_md(res, ml_info, ganhos, ref, origem, original=None):
    L = [f"# Resultados – {origem}", "",
         f"Referência: {ref:.4g} rad/s · Ts = {TS * 1000:.0f} ms · saturação ±{ganhos['v_max']:.3g} V",
         "Gerado por `run_all.py`. Valores de SIMULAÇÃO, não de bancada.", ""]
    L += ["## Ganhos (mesma função custo e otimizador para PID e Fuzzy)", "",
          f"- PID: Kp = {ganhos['pid']['Kp']:.4g}, Ki = {ganhos['pid']['Ki']:.4g}, "
          f"Kd = {ganhos['pid']['Kd']:.3g}",
          f"- Fuzzy PI: Ke = {ganhos['fuzzy']['Ke']:.4g}, Kde = {ganhos['fuzzy']['Kde']:.4g}, "
          f"Ku = {ganhos['fuzzy']['Ku']:.4g}", ""]
    cab = "| Cenário | Métrica | " + " | ".join(NOMES[k] for k in res) + " |"
    L += ["## Métricas", "", cab, "|" + "---|" * (2 + len(res))]
    for cen in res["pid"]:
        for met in res["pid"][cen]:
            L.append(f"| {cen} | {met} | " + " | ".join(_fmt(res[k][cen][met]) for k in res) + " |")
    if original:
        L += ["", "## PID original do repositório (Kp=1, Ki=0,5, Kd=0,1) no cenário nominal", ""]
        L += [f"- {k}: {_fmt(v)}" for k, v in original.items()]
    L += ["", "## Rede neural", ""] + [f"- {k}: {v}" for k, v in ml_info.items()]
    return "\n".join(L) + "\n"


def figuras(series, ref, pasta, titulo):
    fig, axs = plt.subplots(2, 2, figsize=(11, 7), dpi=130, sharex=True)
    for ax, cen in zip(axs.flat, ["nominal", "carga", "inercia_x2", "ruido"]):
        for k, s in series.items():
            t, y, _ = s[cen]
            ax.plot(t, y, color=CORES[k], lw=1.3, label=NOMES[k],
                    ls="--" if k == "mlp" else "-")
        ax.axhline(ref, color="k", lw=0.6, ls=":")
        ax.set_title(cen)
        ax.grid(alpha=0.3)
    axs[1, 0].set_xlabel("tempo (s)")
    axs[1, 1].set_xlabel("tempo (s)")
    axs[0, 0].set_ylabel("velocidade (rad/s)")
    axs[1, 0].set_ylabel("velocidade (rad/s)")
    axs[0, 0].legend(fontsize=8)
    fig.suptitle(titulo + " – simulação")
    fig.tight_layout()
    fig.savefig(pasta / "resposta_cenarios.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 3.5), dpi=130)
    for k, s in series.items():
        t, _, u = s["nominal"]
        ax.step(t, u, where="post", color=CORES[k], lw=1.1, label=NOMES[k],
                ls="--" if k == "mlp" else "-")
    ax.set_xlim(0, 3)
    ax.set_xlabel("tempo (s)")
    ax.set_ylabel("tensão (V)")
    ax.set_title("Esforço de controle – cenário nominal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(pasta / "esforco_controle.png")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", help="JSON de parâmetros identificados (padrão: modelo CTMS)")
    ap.add_argument("--nome", help="subpasta de results/ (padrão: ctms ou identificado)")
    ap.add_argument("--arduino", action="store_true", help="regenera parametros_gerados.h")
    a = ap.parse_args()

    if a.params:
        p, origem = carregar_params(a.params), f"motor identificado ({a.params})"
    else:
        p, origem = MotorParams(), "modelo didático CTMS (U. Michigan) – não é um motor físico"
    pasta = RAIZ / "results" / (a.nome or ("identificado" if a.params else "ctms"))
    pasta.mkdir(parents=True, exist_ok=True)
    ref = cenarios(p)["nominal"].ref

    print("[1/5] sintonia do PID ...")
    g_pid = sintonizar_pid(p)
    print("      ", g_pid)
    print("[2/5] superfície fuzzy e sintonia do Fuzzy PI ...")
    grid, tab = surface_table()
    g_fz = sintonizar_fuzzy(grid, tab, p)
    print("      ", g_fz)

    print("[3/5] coleta de dados e treino da MLP ...")
    pid = make_pid([g_pid["Kp"], g_pid["Ki"], g_pid["Kd"]], p)
    X, y, ep = coletar(pid, params=p)
    pesos, ml_info = treinar(X, y, ep)
    # linha de base: regressão linear nos mesmos dados (mostra o quão linear é o alvo)
    val = np.isin(ep, ml_info["episodios_validacao"])
    A = np.c_[X, np.ones(len(X))]
    coef, *_ = np.linalg.lstsq(A[~val], y[~val], rcond=None)
    ml_info["rmse_validacao_regressao_linear_V"] = float(np.sqrt(np.mean((A[val] @ coef - y[val]) ** 2)))
    # variabilidade entre inicializações: reportar TODAS, não só a melhor, inclusive em malha
    # fechada. Mostra que "menos overshoot que o PID" pode ser só erro de aproximação.
    por_semente = []
    for s in range(5):
        w_s, i_s = treinar(X, y, ep, seed=s)
        r_s, _ = avaliar(MLPController(w_s, TS, -p.v_max, p.v_max), p)
        por_semente.append({
            "semente": s, "rmse_validacao_V": round(i_s["rmse_validacao_V"], 6),
            "overshoot_pct": round(r_s["nominal"]["overshoot_pct"], 4),
            "acomodacao_2pct_s": round(r_s["nominal"]["acomodacao_2pct_s"], 3),
            "IAE": round(r_s["nominal"]["IAE"], 4)})
    ml_info["malha_fechada_por_semente (semente 0 = modelo usado)"] = por_semente
    ml_info["entradas"] = "[erro, integral do erro, derivada filtrada da medição]"
    ml_info["arquitetura"] = "3-8-1, ReLU, sklearn MLPRegressor (lbfgs)"

    print("[4/5] avaliação nos 4 cenários ...")
    ctrls = {
        "pid": make_pid([g_pid["Kp"], g_pid["Ki"], g_pid["Kd"]], p),
        "fuzzy": make_fuzzy([g_fz["Ke"], g_fz["Kde"], g_fz["Ku"]], grid, tab, p),
        "mlp": MLPController(pesos, TS, -p.v_max, p.v_max),
    }
    res, series = {}, {}
    for k, c in ctrls.items():
        res[k], series[k] = avaliar(c, p)
    original = None
    if not a.params:
        c = cenarios(p)["nominal"]
        t, yy, uu = simular(make_pid([1.0, 0.5, 0.1], p), c)
        original = metricas_degrau(t, yy, uu, c.ref, p.v_max)

    print("[5/5] gravando resultados ...")
    ganhos = {"pid": g_pid, "fuzzy": g_fz, "v_max": p.v_max, "Ts": TS, "ref": ref,
              "params": {k: getattr(p, k) for k in ("J", "b", "K", "R", "L", "v_max")}}
    (pasta / "ganhos.json").write_text(json.dumps(ganhos, indent=2))
    (pasta / "mlp_pesos.json").write_text(json.dumps(pesos, indent=2))
    (pasta / "metricas.json").write_text(json.dumps(
        {"controladores": res, "pid_original_repositorio": original, "ml": ml_info,
         "ambiente": {"python": platform.python_version(), "numpy": np.__version__}},
        indent=2, default=float))
    (pasta / "metricas.md").write_text(tabela_md(res, ml_info, ganhos, ref, origem, original))
    figuras(series, ref, pasta, origem.split(" (")[0])
    if a.arduino:
        gerar_header(RAIZ / "arduino" / "motor_control" / "parametros_gerados.h",
                     p, ref, TS, g_pid, g_fz, grid, tab, pesos, origem)
        print("       parametros_gerados.h atualizado")
    print((pasta / "metricas.md").read_text())


if __name__ == "__main__":
    main()
