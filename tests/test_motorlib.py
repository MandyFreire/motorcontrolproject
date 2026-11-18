"""Testes do núcleo: planta, métricas, PID, fuzzy, ML e identificação.
Rodar da raiz:  python -m pytest -q
"""
import numpy as np
import pytest
import scipy.signal as ss

from motorlib.controllers import PID, MLPController
from motorlib.fuzzy import surface_table
from motorlib.identificacao import identificar, simular_aberta
from motorlib.ml import coletar, treinar
from motorlib.plant import DCMotor, MotorParams
from motorlib.simulate import TS, cenarios, metricas_degrau, simular
from motorlib.tuning import make_fuzzy, make_pid

# ganhos obtidos por run_all.py no modelo CTMS (fixados aqui para os testes serem rápidos)
PID_CTMS = [13.403871, 27.4224742, 3.07801377e-05]
FZ_CTMS = [1.34308551, 37.59557, 0.641574354]


@pytest.fixture(scope="module")
def tabela():
    return surface_table()


# ------------------------------------------------------------------ planta
def test_planta_igual_a_funcao_de_transferencia_do_repositorio_original():
    num, den = MotorParams().tf_coeffs()
    assert num == [0.01]
    assert np.allclose(den, [0.005, 0.06, 0.1001])


def test_discretizacao_zoh_exata_para_degrau():
    p = MotorParams()
    m = DCMotor(p, 0.01)
    y = np.array([m.step(1.0) for _ in range(300)])
    t = np.arange(301) * 0.01
    _, ys = ss.step(ss.TransferFunction(*p.tf_coeffs()), T=t)
    assert np.max(np.abs(y - ys[1:])) < 1e-12


def test_ganho_dc():
    assert MotorParams().dc_gain == pytest.approx(0.01 / 0.1001)


def test_cenario_inercia_alcancavel():
    """A referência precisa ser alcançável dentro da saturação em todos os cenários."""
    for c in cenarios().values():
        tensao_regime = c.ref / c.params.dc_gain
        assert tensao_regime < c.params.v_max


# ------------------------------------------------------------------ métricas
def test_metricas_primeira_ordem_conhecida():
    t = np.arange(0, 10, 0.001)
    tau = 0.5
    y = 1 - np.exp(-t / tau)
    m = metricas_degrau(t, y, np.zeros_like(t), r=1.0, v_max=12)
    assert m["overshoot_pct"] == 0
    assert m["subida_10_90_s"] == pytest.approx(tau * np.log(9), abs=2e-3)
    assert m["acomodacao_2pct_s"] == pytest.approx(-tau * np.log(0.02), abs=2e-3)


# ------------------------------------------------------------------ PID
def test_pid_original_do_repositorio_e_lento():
    """Reproduz a auditoria: Kp=1, Ki=0,5, Kd=0,1 não chega a 30 % da referência em 6 s."""
    c = cenarios()["nominal"]
    _, y, _ = simular(make_pid([1.0, 0.5, 0.1]), c)
    assert y[-1] / c.ref < 0.30


def test_pid_sintonizado_erro_nulo_e_overshoot_limitado():
    c = cenarios()["nominal"]
    t, y, u = simular(make_pid(PID_CTMS), c)
    m = metricas_degrau(t, y, u, c.ref, 12.0)
    assert m["erro_regime_pct"] < 0.01
    assert m["overshoot_pct"] <= 5.0
    assert np.max(np.abs(u)) <= 12.0


def test_antiwindup_congela_integrador_na_saturacao():
    pid = PID(100.0, 1000.0, 0.0, TS, -1.0, 1.0)
    for _ in range(100):
        pid.update(10.0, 0.0)  # erro enorme e constante: saída saturada o tempo todo
    assert pid.integ == pytest.approx(0.0)


# ------------------------------------------------------------------ fuzzy
def test_fuzzy_base_completa_sem_buracos(tabela):
    grid, tab = tabela
    assert np.all(np.isfinite(tab))
    assert len(grid) == 21 and grid[0] == -1 and grid[-1] == 1 and 0.0 in grid


def test_fuzzy_sinal_correto_e_simetrico(tabela):
    _, tab = tabela
    c = len(tab) // 2
    assert abs(tab[c, c]) < 1e-9                                 # e=0, de=0 -> du=0
    assert np.all(tab[c + 1:, c:] > 0)                           # e>0, de>=0 -> aumentar
    assert np.all(tab[:c, :c + 1] < 0)                           # e<0, de<=0 -> diminuir
    assert np.allclose(tab, -tab[::-1, ::-1], atol=1e-9)         # antissimetria


def test_fuzzy_sintonizado_erro_nulo(tabela):
    grid, tab = tabela
    c = cenarios()["nominal"]
    t, y, u = simular(make_fuzzy(FZ_CTMS, grid, tab), c)
    assert metricas_degrau(t, y, u, c.ref, 12.0)["erro_regime_pct"] < 0.01


# ------------------------------------------------------------------ ML
def test_ml_alvo_e_linear_e_rede_reproduz_pid():
    """Configuração de produção (20 episódios). Com menos dados a validação pode cair fora
    do envelope de treino e o erro sobe ~100x (ReLU extrapola mal) – ver docs/ml_theory.md."""
    pid = make_pid(PID_CTMS)
    X, y, ep = coletar(pid)
    A = np.c_[X, np.ones(len(X))]
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    assert np.max(np.abs(A @ coef - y)) < 1e-9          # alvo é exatamente linear
    pesos, info = treinar(X, y, ep)
    assert info["rmse_validacao_V"] < 0.05
    c = cenarios()["nominal"]
    _, y_pid, _ = simular(make_pid(PID_CTMS), c)
    _, y_mlp, _ = simular(MLPController(pesos, TS, -12, 12), c)
    assert np.max(np.abs(y_pid - y_mlp)) / c.ref < 0.01  # clone: difere < 1 % do PID


def test_dataset_divide_por_episodio():
    X, y, ep = coletar(make_pid(PID_CTMS), n_episodios=10)
    _, info = treinar(X, y, ep)
    assert info["amostras_treino"] + info["amostras_validacao"] == len(X)
    assert info["amostras_validacao"] % 300 == 0          # episódios inteiros


# ------------------------------------------------------------------ identificação
@pytest.mark.parametrize("real,L_usado", [
    (MotorParams(), 0.5),
    (MotorParams(J=0.002, b=0.002, K=0.3, R=4.0, L=0.002, v_max=10.0), 0.001),
])
def test_identificacao_recupera_J_e_b(real, L_usado):
    Ts = 0.01
    u = np.r_[np.zeros(20), np.full(200, 0.5 * real.v_max), np.full(200, 0.9 * real.v_max),
              np.full(200, 0.3 * real.v_max)]
    w = simular_aberta(real, u, Ts)
    w = w + np.random.default_rng(1).normal(0, 0.005 * w.max(), len(w))
    p, info, _ = identificar(np.arange(len(u)) * Ts, w, u, K=real.K, R=real.R, L=L_usado,
                             v_max=real.v_max)
    assert info["R2"] > 0.99
    assert p.J == pytest.approx(real.J, rel=0.05)
    assert p.b == pytest.approx(real.b, rel=0.05)


# ------------------------------------------------------------------ leitura de log de bancada
def test_log_descarta_execucao_anterior_e_le_vmax(tmp_path):
    from motorlib.identificacao import ler_csv, vmax_do_log
    log = tmp_path / "ensaio.csv"
    log.write_text(
        "t_ms,ref,w_rad_s,u_V,exec_us,estouros\n"
        "5350,6.0,9.9,6.0,40,0\n"                       # resto da execução anterior ao reset
        "# modo=0 Ts_ms=10.00 cpr=2400.00 vmax=12.00\n"
        "t_ms,ref,w_rad_s,u_V,exec_us,estouros\n"
        "0,0.0,0.0,0.0,40,0\n10,6.0,0.1,6.0,41,0\n")
    t, w, u = ler_csv(log)
    assert list(t) == [0.0, 0.01] and list(u) == [0.0, 6.0]
    assert vmax_do_log(log) == 12.0
