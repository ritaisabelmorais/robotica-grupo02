# Robótica 2026/27 — Grupo 2 (PL3, bancada 2)

Projeto Python dos trabalhos laboratoriais (LW0–LW4 e Desafio Final) com o UFACTORY Lite 6.
Testado em macOS (Apple M3) com Docker Desktop.

## 0. Obter o projeto

```bash
git clone https://github.com/ritaisabelmorais/robotica-grupo02.git
cd robotica-grupo02
```

## 1. Ambiente Python (Python ≥ 3.10)

```bash
python3 -m venv ~/venvs/robotica
source ~/venvs/robotica/bin/activate
pip install -r requirements.txt
```

Em cada Terminal novo: `source ~/venvs/robotica/bin/activate`.
No VS Code: Cmd+Shift+P → "Python: Select Interpreter" → `~/venvs/robotica`.

## 2. Contentor ROS 2 + Gazebo da UC (testes 1–7)

Usa o repositório da UC, numa pasta separada deste projeto (ver o README desse repositório
para o ramo do vosso sistema operativo):

```bash
git clone https://github.com/cazevedo/robotica.git && cd robotica
cp .env.example .env          # pôr ROS_DOMAIN_ID=2 (número do grupo)
./lab.sh build                # só da primeira vez
./lab.sh up
./lab.sh doctor
```

- Janelas (RViz, Gazebo): `./lab.sh vnc` ou http://localhost:6080
  (se aparecer "Failed to connect to server": `./lab.sh restart`).
- Cada Terminal novo dentro do contentor: `./lab.sh shell`. Os testes são `lab test 1` …
  `lab test 7` (o teste 1 precisa de `lab test 1b` noutro Terminal).
- Fechar: `lab kill` dentro do contentor e `./lab.sh down` no Mac.

## 3. Simulador de firmware da UFACTORY (teste 8 e testes do safe_session)

Desligar antes o contentor ROS (`./lab.sh down`) para libertar memória.

```bash
docker run -it --platform linux/amd64 --name uf_software \
  -p 18333:18333 -p 502:502 -p 503:503 -p 504:504 \
  -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 \
  danielwang123321/uf-ubuntu-docker
# dentro do contentor, arrancar o firmware com o perfil Lite 6:
/xarm_scripts/xarm_start.sh 6 9
```

Das vezes seguintes: `docker start -ai uf_software` e voltar a correr o `xarm_start.sh`.

Interface web (UFACTORY Studio): http://127.0.0.1:18333

- Em Live Control, escolher **No End Effector** (com "Cylinder" o simulador dá
  auto-colisão, erro C22, perto de home).
- O botão **Initial Position** tem de ser mantido premido até as juntas chegarem a 0.
- Se aparecer uma janela de erro: **Clear Error** e depois Initial Position.

## 4. Configuração

Todos os parâmetros estão em `config/robot.yaml`; nenhum IP está no código.

| Parâmetro | Simulador | Robô real |
| --- | --- | --- |
| `ip` | `127.0.0.1` | IP da bancada |
| `check_joint_limit` | `false` | `true` |
| `collision_tool_model` | `0` (sem ferramenta) | `7` (Lite 6 Gripper) |
| `cmd_timeout` | `10` (simulador emulado é lento) | `2` |
| `tcp_boundary` | valores de teste | valores combinados com o docente |

## 5. Correr (a partir desta pasta, com o venv ativo e o simulador ligado)

```bash
python -m pytest                             # testes unitários (não precisam do robô)
python -m scripts.check_connection           # abre a sessão segura e vai a home
python -m scripts.test8_fk                   # LW0 parte 2, teste 8
python -m scripts.lifecycle_tests            # LW0 parte 2, testes T1–T9
python -m scripts.lifecycle_tests --sem-t6   # o mesmo, sem o T6 (mais rápido)
```

Os registos de cada sessão ficam em `logs/session.log`.

## Problemas conhecidos

- `motion_enable` com código 3 no simulador: o firmware emulado não responde a tempo;
  o `safe_session` tenta até 3 vezes (`enable_retries`).
- Erro C23 (limite de junta): a junta 3 do Lite 6 só aceita até +11°.
- `connect socket failed`: o simulador não está ligado ou foi abaixo (repetir a secção 3).
