"""Gera arduino/motor_control/parametros_gerados.h a partir dos resultados da simulação.

Assim o firmware usa exatamente os mesmos ganhos, a mesma superfície fuzzy e os mesmos
pesos da rede avaliados em Python – não há transcrição manual de números.
No AVR, float e double têm 32 bits; a tabela fuzzy (21x21 = 1764 B) vai para a flash
(PROGMEM) porque o ATmega328P tem só 2 KB de SRAM.
"""
import datetime


def _f(x, nd=9):
    """Literal float C válido: 12 -> 12.0f, 1e-05 -> 1e-05f."""
    t = f"{float(x):.{nd}g}"
    if not any(c in t for c in ".en"):
        t += ".0"
    return t + "f"


def _arr(v):
    return ", ".join(_f(x) for x in v)


def gerar_header(caminho, params, ref, Ts, pid, fuzzy, grid, table, mlp, origem):
    n = len(grid)
    W1 = mlp["W1"]
    n_in, n_h = len(W1), len(W1[0])
    linhas = [
        "// ARQUIVO GERADO por run_all.py – NÃO EDITE À MÃO.",
        f"// Origem dos parâmetros do motor: {origem}",
        f"// Gerado em: {datetime.date.today().isoformat()}",
        "// Se o motor da bancada não foi identificado, estes ganhos são do MODELO DIDÁTICO",
        "// e NÃO servem para o motor real: rode a identificação e regenere.",
        "#pragma once",
        "#include <avr/pgmspace.h>",
        "",
        f"#define TS_S          {_f(Ts, 6)}",
        f"#define TS_US         {int(round(Ts * 1e6))}UL",
        f"#define V_MAX         {_f(params.v_max, 6)}   // tensão com PWM 100 % [V]",
        f"#define REF_PADRAO    {_f(ref, 6)}   // [rad/s]",
        "",
        f"#define PID_KP        {_f(pid['Kp'], 9)}",
        f"#define PID_KI        {_f(pid['Ki'], 9)}",
        f"#define PID_KD        {_f(pid['Kd'], 9)}",
        f"#define PID_TF        {_f(2 * Ts, 6)}   // filtro da derivada [s]",
        "",
        f"#define FZ_KE         {_f(fuzzy['Ke'], 9)}",
        f"#define FZ_KDE        {_f(fuzzy['Kde'], 9)}",
        f"#define FZ_KU         {_f(fuzzy['Ku'], 9)}",
        f"#define FZ_N          {n}",
        "// superfície F(e_n, de_n) em grade uniforme [-1,1]^2; índice [i_e][j_de]",
        "const float FZ_TABELA[FZ_N][FZ_N] PROGMEM = {",
    ]
    linhas += ["  {" + _arr(row) + "}," for row in table]
    linhas += [
        "};",
        "",
        f"#define MLP_NIN       {n_in}",
        f"#define MLP_NH        {n_h}",
        "const float MLP_W1[MLP_NIN][MLP_NH] = {",
    ]
    linhas += ["  {" + _arr(row) + "}," for row in W1]
    linhas += [
        "};",
        f"const float MLP_B1[MLP_NH] = {{{_arr(mlp['b1'])}}};",
        f"const float MLP_W2[MLP_NH] = {{{_arr(mlp['W2'])}}};",
        f"const float MLP_B2 = {_f(mlp['b2'], 9)};",
        f"const float MLP_XMEAN[MLP_NIN] = {{{_arr(mlp['x_mean'])}}};",
        f"const float MLP_XSTD_INV[MLP_NIN] = {{{_arr([1.0 / v for v in mlp['x_std']])}}};",
        f"const float MLP_YMEAN = {_f(mlp['y_mean'], 9)};",
        f"const float MLP_YSTD  = {_f(mlp['y_std'], 9)};",
        "",
    ]
    with open(caminho, "w") as f:
        f.write("\n".join(linhas))
