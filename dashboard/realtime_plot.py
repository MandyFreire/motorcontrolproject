"""Monitor serial do sketch arduino/motor_control: gráfico em tempo real + gravação em CSV.

    python dashboard/realtime_plot.py --porta COM3 --csv ensaio_pid.csv
    python dashboard/realtime_plot.py --porta /dev/ttyACM0 --csv ensaio.csv --sem-grafico

O CSV gravado é o DADO DE BANCADA: é ele que deve sustentar qualquer número publicado
(e é a entrada de simulations/identificacao/identify_motor.py e de analisar_ensaio.py).

Correções em relação à versão original:
  - lê o formato CSV que o sketch envia (a versão original fazia float() na linha
    "Setpoint: ... | RPM: ..." e quebrava na primeira leitura);
  - lê TODAS as linhas disponíveis a cada atualização (a original lia 1 linha a cada
    100 ms enquanto chegavam 31–137 linhas/s, e o atraso crescia sem limite);
  - leitura não bloqueante (a original travava a interface dentro do timer);
  - histórico limitado (janela deslizante) em vez de redesenhar tudo a cada quadro;
  - porta e baud configuráveis (a original fixava COM3 / 9600);
  - avisa se o firmware reportar estouro do período de amostragem.
"""
import argparse
import collections
import sys
import time

import serial

CAMPOS = ["t_ms", "ref", "w_rad_s", "u_V", "exec_us", "estouros"]


class Leitor:
    def __init__(self, porta, baud, caminho_csv):
        self.ser = serial.Serial(porta, baud, timeout=0)
        # Abrir a porta reinicia o Uno e o ensaio começa ~1 s depois. Descarta só o que
        # estava no buffer ANTES do reset; não espera (esperar perderia o início do ensaio).
        self.ser.reset_input_buffer()
        self.buf = b""
        self.csv = open(caminho_csv, "w") if caminho_csv else None
        if self.csv:  # cabeçalho garantido mesmo que a 1a linha chegue truncada
            self.csv.write(",".join(CAMPOS) + "\n")
        self.estouros = 0
        self.linhas = 0

    def ler(self):
        """Devolve a lista de amostras (dicts) completas que chegaram desde a última chamada."""
        self.buf += self.ser.read(self.ser.in_waiting or 1)
        *completas, self.buf = self.buf.split(b"\n")
        amostras = []
        for bruto in completas:
            linha = bruto.decode(errors="replace").strip()
            if not linha:
                continue
            if linha.startswith("t_ms"):
                continue                # cabeçalho já escrito na abertura do arquivo
            if linha.startswith("#"):
                if self.csv:
                    self.csv.write(linha + "\n")
                print(linha)
                continue
            partes = linha.split(",")
            if len(partes) != len(CAMPOS):
                continue                # linha truncada (ex.: logo após o reset)
            try:
                a = dict(zip(CAMPOS, map(float, partes)))
            except ValueError:
                continue
            if self.csv:
                self.csv.write(linha + "\n")
            if a["estouros"] > self.estouros:
                print(f"ATENÇÃO: {int(a['estouros'])} estouro(s) do período de amostragem "
                      f"em t = {a['t_ms'] / 1000:.2f} s – os dados não correspondem ao Ts simulado")
                self.estouros = a["estouros"]
            amostras.append(a)
            self.linhas += 1
        if self.csv:
            self.csv.flush()
        return amostras


def modo_texto(leitor):
    try:
        while True:
            for a in leitor.ler():
                if int(a["t_ms"]) % 1000 < 10:
                    print(f"t={a['t_ms'] / 1000:6.1f}s  ref={a['ref']:.3f}  w={a['w_rad_s']:.3f}  "
                          f"u={a['u_V']:.2f} V  exec={int(a['exec_us'])} us")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n{leitor.linhas} amostras gravadas")


def modo_grafico(leitor, janela_s):
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets

    n = int(janela_s * 100) + 1
    t, ref, w, u = (collections.deque(maxlen=n) for _ in range(4))

    app = QtWidgets.QApplication(sys.argv)
    win = pg.GraphicsLayoutWidget(title="Motor DC – tempo real")
    p1 = win.addPlot(row=0, col=0, title="Velocidade")
    p1.setLabel("left", "rad/s")
    p1.addLegend()
    c_ref = p1.plot(pen=pg.mkPen("w", style=QtCore.Qt.DashLine), name="referência")
    c_w = p1.plot(pen="y", name="medida")
    p2 = win.addPlot(row=1, col=0, title="Tensão de controle")
    p2.setLabel("left", "V")
    p2.setLabel("bottom", "tempo", "s")
    p2.setXLink(p1)
    c_u = p2.plot(pen="c")
    win.resize(900, 600)
    win.show()

    def atualizar():
        for a in leitor.ler():
            t.append(a["t_ms"] / 1000)
            ref.append(a["ref"])
            w.append(a["w_rad_s"])
            u.append(a["u_V"])
        if t:
            c_ref.setData(list(t), list(ref))
            c_w.setData(list(t), list(w))
            c_u.setData(list(t), list(u))

    timer = QtCore.QTimer()
    timer.timeout.connect(atualizar)
    timer.start(50)
    app.exec()
    print(f"{leitor.linhas} amostras gravadas")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--porta", required=True, help="ex.: COM3 ou /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", help="arquivo para gravar todas as amostras")
    ap.add_argument("--janela", type=float, default=10.0, help="segundos visíveis no gráfico")
    ap.add_argument("--sem-grafico", action="store_true")
    a = ap.parse_args()
    leitor = Leitor(a.porta, a.baud, a.csv)
    if a.sem_grafico:
        modo_texto(leitor)
    else:
        modo_grafico(leitor, a.janela)


if __name__ == "__main__":
    main()
