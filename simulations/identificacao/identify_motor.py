"""Identifica J e b do motor da bancada a partir do log do ensaio em malha aberta.

1. Grave o sketch arduino/motor_control com MODO_CONTROLE 0 e capture o CSV:
       python dashboard/realtime_plot.py --porta /dev/ttyACM0 --csv ensaio.csv
2. Meça R, K e v_max (ver docstring de motorlib/identificacao.py) e rode:
       python simulations/identificacao/identify_motor.py ensaio.csv --R 4.1 --K 0.29 --v_max 10.2
3. Use o JSON gerado para sintonizar e gerar o firmware:
       python run_all.py --params results/motor_identificado.json --arduino

Verificação: o script mostra o ajuste sobreposto aos dados e o R². Se R² < 0,98, não
use o modelo – investigue (saturação do driver, zona morta, folga de engrenagem, encoder).
"""
import argparse

import _caminho  # noqa: F401
import matplotlib.pyplot as plt

from motorlib.identificacao import identificar, ler_csv, salvar_params, vmax_do_log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--R", type=float, required=True, help="resistência de armadura [ohm]")
    ap.add_argument("--K", type=float, required=True, help="constante de FCEM [V.s/rad]")
    ap.add_argument("--L", type=float, default=1e-3, help="indutância [H] (padrão 1 mH)")
    ap.add_argument("--v_max", type=float, required=True, help="tensão no motor com PWM 100 %% [V]")
    ap.add_argument("--saida", default=str(_caminho.RAIZ / "results" / "motor_identificado.json"))
    a = ap.parse_args()

    t, w, u = ler_csv(a.csv)
    # O firmware registra u = PWM/255 * V_MAX do cabeçalho gerado (12 V no modelo didático).
    # A tensão real com PWM 100 % é a medida (--v_max): corrige a escala.
    vmax_fw = vmax_do_log(a.csv)
    if vmax_fw is None:
        print("ATENÇÃO: log sem a linha '# ... vmax=' – assumindo que u_V já está em volts reais")
    elif abs(vmax_fw - a.v_max) > 1e-6:
        print(f"tensões do log reescaladas: firmware usou V_MAX={vmax_fw:g} V, medido {a.v_max:g} V")
        u = u * a.v_max / vmax_fw
    p, info, w_fit = identificar(t, w, u, K=a.K, R=a.R, L=a.L, v_max=a.v_max)
    print(f"J = {p.J:.5g} kg.m²   b = {p.b:.5g} N.m.s   ganho DC = {p.dc_gain:.4g} (rad/s)/V")
    print(info)
    if info["R2"] < 0.98:
        print("ATENÇÃO: R² < 0,98 – o modelo linear não explica bem o ensaio. Não use sem investigar.")
    salvar_params(p, a.saida, info)
    print("salvo em", a.saida)

    plt.plot(t, w, ".", ms=2, label="medido")
    plt.plot(t, w_fit, label=f"modelo (R² = {info['R2']:.4f})")
    plt.xlabel("tempo (s)")
    plt.ylabel("velocidade (rad/s)")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()
