"""Paridade Python x C: compila arduino/motor_control/controladores.h no PC e verifica que,
para a mesma sequência de medições, as tensões calculadas coincidem com motorlib.

Garante que o firmware executa as MESMAS leis de controle que foram simuladas.
Diferenças esperadas: só arredondamento float32 (C/AVR) x float64 (Python).
Pulado se não houver compilador C no sistema.
"""
import json
import pathlib
import shutil
import subprocess

import numpy as np
import pytest

from motorlib.controllers import MLPController
from motorlib.export_c import gerar_header
from motorlib.fuzzy import surface_table
from motorlib.ml import coletar, treinar
from motorlib.plant import MotorParams
from motorlib.simulate import TS, cenarios, simular
from motorlib.tuning import make_fuzzy, make_pid

RAIZ = pathlib.Path(__file__).resolve().parents[1]
CC = shutil.which("gcc") or shutil.which("cc")

HARNESS = r"""
#include <stdio.h>
#include "controladores.h"
int main(void) {
  EstadoPID p; EstadoFuzzy f; EstadoMLP m;
  pid_reset(&p); fuzzy_reset(&f); mlp_reset(&m);
  float y;
  while (scanf("%f", &y) == 1)
    printf("%.7g %.7g %.7g\n", pid_update(&p, REF_PADRAO, y), fuzzy_update(&f, REF_PADRAO, y),
           mlp_update(&m, REF_PADRAO, y));
  return 0;
}
"""

PGMSPACE_STUB = "#pragma once\n#define PROGMEM\n#define pgm_read_float(p) (*(p))\n"


@pytest.mark.skipif(CC is None, reason="sem compilador C")
def test_controladores_c_iguais_ao_python(tmp_path):
    p = MotorParams()
    ref = cenarios(p)["nominal"].ref
    g_pid = {"Kp": 13.403871, "Ki": 27.4224742, "Kd": 0.05}   # Kd > 0 para exercitar a derivada
    g_fz = {"Ke": 1.34308551, "Kde": 37.59557, "Ku": 0.641574354}
    grid, tab = surface_table()
    pid_py = make_pid([g_pid["Kp"], g_pid["Ki"], g_pid["Kd"]], p)
    X, y, ep = coletar(make_pid([g_pid["Kp"], g_pid["Ki"], g_pid["Kd"]], p), n_episodios=6)
    pesos, _ = treinar(X, y, ep)

    (tmp_path / "avr").mkdir()
    (tmp_path / "avr" / "pgmspace.h").write_text(PGMSPACE_STUB)
    gerar_header(tmp_path / "parametros_gerados.h", p, ref, TS, g_pid, g_fz, grid, tab, pesos, "teste")
    shutil.copy(RAIZ / "arduino" / "motor_control" / "controladores.h", tmp_path)
    (tmp_path / "h.c").write_text(HARNESS)
    exe = tmp_path / "h"
    subprocess.run([CC, "-O2", "-Wall", "-Werror", f"-I{tmp_path}", str(tmp_path / "h.c"),
                    "-o", str(exe), "-lm"], check=True)

    # sequência de medições realista e com ruído: saída do PID em malha fechada com ruído
    _, w, _ = simular(make_pid([g_pid["Kp"], g_pid["Ki"], g_pid["Kd"]], p), cenarios(p)["ruido"])
    w = w + np.random.default_rng(3).normal(0, 0.01, len(w))
    out = subprocess.run([str(exe)], input="\n".join(f"{v:.9g}" for v in w),
                         capture_output=True, text=True, check=True).stdout
    u_c = np.array([[float(v) for v in l.split()] for l in out.strip().splitlines()])

    # Python com as MESMAS medições (malha aberta – compara só o controlador)
    fz_py = make_fuzzy([g_fz["Ke"], g_fz["Kde"], g_fz["Ku"]], grid, tab, p)
    mlp_py = MLPController(pesos, TS, -p.v_max, p.v_max)
    pid_py.reset(); fz_py.reset(); mlp_py.reset()
    w32 = w.astype(np.float32).astype(float)   # o C lê as medições em float32
    u_py = np.array([[pid_py.update(ref, v), fz_py.update(ref, v), mlp_py.update(ref, v)] for v in w32])

    assert u_c.shape == u_py.shape
    erro = np.max(np.abs(u_c - u_py), axis=0)
    # float32 tem ~7 dígitos; integradores acumulam arredondamento ao longo de 600 passos
    assert erro[0] < 1e-3, f"PID diverge: {erro[0]}"
    assert erro[1] < 1e-3, f"Fuzzy diverge: {erro[1]}"
    assert erro[2] < 1e-3, f"MLP diverge: {erro[2]}"
    (tmp_path / "erro.json").write_text(json.dumps(erro.tolist()))
