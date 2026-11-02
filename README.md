# Robótica 2026/27 — Grupo 2 (PL3, bancada 2)

Projeto Python dos trabalhos laboratoriais com o UFACTORY Lite 6. Por agora inclui o LW0 (parte 2), o LW1 e o LW2.
Testado em macOS (Apple M3) com Docker Desktop.

> **Como ler este README:** cada comando diz **onde** se escreve.
> - **[Mac]** — no Terminal do Mac (a linha começa por `utilizador@Mac ... %`)
> - **[Contentor]** — dentro do contentor ROS (a linha começa por `ros@lab0:~/dev_ws$`)
> - **[Simulador]** — dentro do contentor do simulador (a linha começa por `root@...`)


---

## Parte A — Preparar (só da primeira vez)

### A1. Obter este projeto — [Mac]

```bash
cd ~/Documents
git clone https://github.com/ritaisabelmorais/robotica-grupo02.git
```

### A2. Ambiente Python (Python ≥ 3.10) — [Mac]

```bash
python3 -m venv ~/venvs/robotica
source ~/venvs/robotica/bin/activate
cd ~/Documents/robotica-grupo02
pip install -r requirements.txt
```

### A3. Contentor ROS 2 da UC — [Mac]

Fica numa pasta à parte deste projeto.

```bash
cd ~/Documents
git clone https://github.com/cazevedo/robotica.git
cd robotica
cp .env.example .env
```

Abrir o ficheiro `.env` e pôr `ROS_DOMAIN_ID=2` (número do grupo). Depois:

```bash
./lab.sh build        
```

### A4. Simulador de firmware da UFACTORY — [Mac]

```bash
docker run -it --platform linux/amd64 --name uf_software \
  -p 18333:18333 -p 502:502 -p 503:503 -p 504:504 \
  -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 \
  danielwang123321/uf-ubuntu-docker
```

Fica-se dentro do simulador ([Simulador]). Sair com `exit`. Das vezes seguintes usa-se a Parte C.

---

## Parte B — Testes ROS 2 (testes 1 a 7)

Antes de começar: Docker Desktop aberto e simulador de firmware desligado (`docker stop uf_software`).

**Separador 1 — [Mac]:** ligar o contentor e entrar nele

```bash
cd ~/Documents/robotica
./lab.sh up
./lab.sh doctor
./lab.sh shell
```

**Browser:** abrir http://localhost:6080 (é aqui que aparecem o RViz e o Gazebo).
Se aparecer "Failed to connect to server": no Mac, `./lab.sh restart` e recarregar a página.

**Teste 1** (precisa de dois separadores):

- **Separador 1 — [Contentor]:** `lab test 1` (fica a publicar "Hello World")
- **Separador 2** (Cmd+T) **— [Mac]:** `cd ~/Documents/robotica` e `./lab.sh shell`
  depois **[Contentor]:** `lab test 1b` (deve mostrar `I heard: [Hello World: N]`)
- Parar com **Ctrl+C** nos dois.

**Testes 2 a 7 — Separador 1, [Contentor]:**

```bash
lab test 2
lab test 3      # RViz no browser; deixar aberto para os testes 4 e 5
```

Para os testes 4 e 5, abrir mais um separador e entrar no contentor
(`cd ~/Documents/robotica` e `./lab.sh shell`).

Entre testes que abrem janelas: `lab kill` no [Contentor].

**Fechar tudo:** `lab kill` no [Contentor] e `./lab.sh down` no [Mac].

---

## Parte C — Simulador de firmware e testes do projeto (teste 8 e T1–T9)

Antes de começar: contentor ROS desligado (`./lab.sh down`), para não faltar memória.

**Separador 1 — [Mac]:** ligar o simulador

```bash
docker start -ai uf_software
```

**[Simulador]:** arrancar o firmware do Lite 6 e deixar este separador aberto

```bash
/xarm_scripts/xarm_start.sh 6 9
```

**Browser:** abrir http://127.0.0.1:18333 (UFACTORY Studio)
- escolher **No End Effector** (com "Cylinder" dá auto-colisão, erro C22);
- manter **Initial Position** premido até as juntas chegarem a 0;
- se aparecer um erro: **Clear Error** e depois Initial Position.

**Separador 2** (Cmd+T) **— [Mac]:** correr o projeto

```bash
cd ~/Documents/robotica-grupo02
source ~/venvs/robotica/bin/activate
python -m pytest                             # testes sem robô (3 passed)
python -m scripts.check_connection           # liga-se e vai a home
python -m scripts.test8_fk                   # teste 8
python -m scripts.lifecycle_tests --sem-t6   # testes T1–T9 (sem o T6)
python -m scripts.lifecycle_tests            # todos, incluindo o T6 (mais lento)
```

Os registos ficam em `logs/session.log`.

**Fechar:** `docker stop uf_software` no [Mac].

---

## Configuração

Todos os parâmetros estão em `config/robot.yaml`; nenhum IP está no código.

| Parâmetro | Simulador | Robô real |
| --- | --- | --- |
| `ip` | `127.0.0.1` | IP da bancada |
| `check_joint_limit` | `false` | `true` |
| `collision_tool_model` | `0` (sem ferramenta) | `7` (Lite 6 Gripper) |
| `cmd_timeout` | `10` (simulador emulado é lento) | `2` |
| `tcp_boundary` | valores de teste | valores combinados com o docente |

## Problemas conhecidos

- `motion_enable` com código 3: o simulador emulado não respondeu a tempo; o `safe_session` tenta até 3 vezes.
- Erro C23: a junta 3 do Lite 6 só aceita até +11°.
- Erro C22 (auto-colisão): no Studio está escolhido "Cylinder" em vez de "No End Effector".
- `connect socket failed`: o simulador não está ligado ou foi abaixo (repetir a Parte C).
- `command not found: lab` ou `ros2`: estás no Mac; entra primeiro no contentor com `./lab.sh shell`.
