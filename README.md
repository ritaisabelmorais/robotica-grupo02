# Robotica 2026/27 - Grupo 2 (PL3, bancada 2)

Projeto Python dos trabalhos laboratoriais com o UFACTORY Lite 6.

## 1. Ambiente (macOS, Python >= 3.10)

```bash
python3 -m venv ~/venvs/robotica
source ~/venvs/robotica/bin/activate
pip install -r requirements.txt
```

No VS Code: Cmd+Shift+P -> "Python: Select Interpreter" -> `~/venvs/robotica`.

## 2. Simulador de firmware da UFACTORY

```bash
docker run -it --platform linux/amd64 --name uf_software \
  -p 18333:18333 -p 502:502 -p 503:503 -p 504:504 \
  -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 \
  danielwang123321/uf-ubuntu-docker
# dentro do contentor, arrancar o firmware com o perfil Lite 6:
/xarm_scripts/xarm_start.sh 6 9
```

Das vezes seguintes: `docker start -ai uf_software` e voltar a correr o `xarm_start.sh`.
Interface web: http://127.0.0.1:18333

## 3. Configuracao

Todos os parametros estao em `config/robot.yaml`. Para mudar entre simulador e robo real
muda-se apenas `ip` (127.0.0.1 = simulador) e `check_joint_limit`.

## 4. Correr (sempre a partir desta pasta, com o venv ativo)

```bash
python -m pytest                      # testes unitarios (sem robo)
python -m scripts.check_connection    # liga-se e vai a home
python -m scripts.test8_fk            # LW0 parte 2, teste 8
python -m scripts.lifecycle_tests     # LW0 parte 2, testes T1-T9
```

Os registos ficam em `logs/session.log`.
