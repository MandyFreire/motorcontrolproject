"""Base de regras do Fuzzy PI e geração da superfície de controle (scikit-fuzzy).

Convenção de sinal (válida em TODO o projeto):  e = r - y
    e > 0  -> motor abaixo da referência -> aumentar tensão (du > 0)
    de > 0 -> erro crescendo              -> aumentar tensão

Base 5x5 COMPLETA (25 regras, tabela de MacVicar-Whelan): todo par (e, de) do universo
ativa ao menos uma regra, então a saída é sempre definida.
"""
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

ROTULOS = ["NG", "NP", "ZE", "PP", "PG"]  # negativo grande ... positivo grande
CENTROS = [-1.0, -0.5, 0.0, 0.5, 1.0]

# linhas: e = NG..PG ; colunas: de = NG..PG ; valor = índice do rótulo de saída
# du = sat(e_idx + de_idx) no espaço de índices -> diagonal anti-simétrica clássica
REGRAS = [[max(0, min(4, i + j - 2)) for j in range(5)] for i in range(5)]


def _particao(universe, var):
    """5 triangulares com 50 % de sobreposição; ombros nos extremos (pertinência 1 fora)."""
    var["NG"] = fuzz.trapmf(universe, [-1.01, -1.01, -1.0, -0.5])
    var["NP"] = fuzz.trimf(universe, [-1.0, -0.5, 0.0])
    var["ZE"] = fuzz.trimf(universe, [-0.5, 0.0, 0.5])
    var["PP"] = fuzz.trimf(universe, [0.0, 0.5, 1.0])
    var["PG"] = fuzz.trapmf(universe, [0.5, 1.0, 1.01, 1.01])


def build_system():
    # linspace inclui os extremos e o zero exatos (o arange original não incluía)
    uni = np.linspace(-1.0, 1.0, 201)
    e = ctrl.Antecedent(uni, "e")
    de = ctrl.Antecedent(uni, "de")
    du = ctrl.Consequent(uni, "du", defuzzify_method="centroid")
    for v in (e, de, du):
        _particao(uni, v)
    regras = [ctrl.Rule(e[ROTULOS[i]] & de[ROTULOS[j]], du[ROTULOS[REGRAS[i][j]]])
              for i in range(5) for j in range(5)]
    return ctrl.ControlSystem(regras)


def surface_table(n=21):
    """Avalia o sistema fuzzy numa grade n x n em [-1,1]^2. Retorna (grid, tabela[i_e][j_de]).
    Lança exceção se algum ponto ficar sem regra ativa (não deve ocorrer: base completa)."""
    sim = ctrl.ControlSystemSimulation(build_system())
    grid = np.linspace(-1.0, 1.0, n)
    tab = np.zeros((n, n))
    for i, ev in enumerate(grid):
        for j, dv in enumerate(grid):
            sim.input["e"], sim.input["de"] = ev, dv
            sim.compute()
            tab[i, j] = sim.output["du"]
    return grid, tab
