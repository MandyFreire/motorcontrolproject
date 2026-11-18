"""Permite rodar os scripts de qualquer pasta: adiciona a raiz do repositório ao sys.path."""
import pathlib
import sys

RAIZ = pathlib.Path(__file__).resolve().parents[1]
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))
