import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# Variáveis fuzzy
erro = ctrl.Antecedent(np.arange(-10, 10, 0.1), 'erro')
variacao_erro = ctrl.Antecedent(np.arange(-5, 5, 0.1), 'variacao_erro')
saida = ctrl.Consequent(np.arange(-1, 1, 0.01), 'saida')

# Funções de pertinência
erro['neg'] = fuzz.trimf(erro.universe, [-10, -10, 0])
erro['zero'] = fuzz.trimf(erro.universe, [-5, 0, 5])
erro['pos'] = fuzz.trimf(erro.universe, [0, 10, 10])

variacao_erro['neg'] = fuzz.trimf(variacao_erro.universe, [-5, -5, 0])
variacao_erro['zero'] = fuzz.trimf(variacao_erro.universe, [-1, 0, 1])
variacao_erro['pos'] = fuzz.trimf(variacao_erro.universe, [0, 5, 5])

saida['diminuir'] = fuzz.trimf(saida.universe, [-1, -1, 0])
saida['neutro'] = fuzz.trimf(saida.universe, [-0.5, 0, 0.5])
saida['aumentar'] = fuzz.trimf(saida.universe, [0, 1, 1])

# Regras
regras = [
    ctrl.Rule(erro['neg'] & variacao_erro['neg'], saida['aumentar']),
    ctrl.Rule(erro['zero'], saida['neutro']),
    ctrl.Rule(erro['pos'] & variacao_erro['pos'], saida['diminuir'])
]

# Sistema e simulação
sistema = ctrl.ControlSystem(regras)
simulador = ctrl.ControlSystemSimulation(sistema)

# Teste
simulador.input['erro'] = 3
simulador.input['variacao_erro'] = 0.5
simulador.compute()

print('Saída fuzzy:', simulador.output['saida'])
saida.view(sim=simulador)
plt.show()

