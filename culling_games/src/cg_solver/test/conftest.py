import sys
from pathlib import Path

# Permite rodar `pytest` direto na pasta do pacote, sem colcon build.
PACOTE = Path(__file__).resolve().parents[1]
TESTES = Path(__file__).resolve().parent
for caminho in (PACOTE, TESTES):
    if str(caminho) not in sys.path:
        sys.path.insert(0, str(caminho))
