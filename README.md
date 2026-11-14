# Robótica 2026/27 — Grupo 2 (PL3, bancada 2)

Projeto Python dos trabalhos laboratoriais com o UFACTORY Lite 6. Inclui o LW0 (parte 1 em `lw0_parte1/`, parte 2), o LW1 e o LW2.
Testado em macOS (Apple M3) com Docker Desktop.

> **Como ler este README:** cada comando diz **onde** se escreve.
> - **[Mac]** — no Terminal do Mac (a linha começa por `utilizador@Mac ... %`)
> - **[Contentor]** — dentro do contentor ROS (a linha começa por `ros@lab0:~/dev_ws$`)
> - **[Simulador]** — dentro do contentor do simulador (a linha começa por `root@...`)

## Estrutura do repositório

| Trabalho | Onde está |
|---|---|
| LW0 parte 1 | `lw0_parte1/` — `safe_session.py`, `log_joints.py`, exemplo 2001 adaptado e registo das juntas |
| LW0 parte 2 | `lite6/` (sessão segura), `config/robot.yaml` (parâmetros), `scripts/` (testes do ciclo de vida), `tests/` (pytest), `logs/` |
| LW1 | `lw1/` — `forward_kinematics.py`, resultados do `fk_check` (sim e real) e vídeo |
| LW2 | `lw2/` — `inverse_kinematics.py`, resultados do `ik_check` e do estudo, e vídeo |


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

## Parte D — LW1 e LW2 (cinemática direta e inversa)

O código destes dois trabalhos corre como nós de ROS 2 dentro do contentor da UC (Parte B).

1. **[Mac]** copiar os ficheiros deste repositório para o workspace do contentor:
   - `lw1/forward_kinematics.py` → `robotica/src/lite6_kinematics/lite6_kinematics/`
   - `lw2/inverse_kinematics.py` → `robotica/src/lite6_ik/lite6_ik/`
2. **[Contentor]** compilar com `lab build` e abrir uma shell nova (`./lab.sh shell`).
3. **Separador 1 — [Contentor]:** `lab sim` (Gazebo; não mexer mais neste separador).

**LW1 — cinemática direta**

- **Separador 2 — [Contentor]:** `ros2 run lite6_kinematics fk_node`
- **Separador 3 — [Contentor]:** `ros2 run lite6_kinematics fk_check --ros-args -p robot:=sim`

**LW2 — cinemática inversa (algébrica)**

- **Separador 2 — [Contentor]:** `ros2 run lite6_ik ik_node`
- **Separador 3 — [Contentor]:** `ros2 run lite6_ik ik_check --ros-args -p robot:=sim`
- Estudo da Parte E: `ros2 run lite6_ik ik_study`

No robô real: em vez de `lab sim`, usar `lab gazebo off` e `lab real <IP>`, esperar por
"Configured and activated lite6_traj_controller" e correr os `*_check` com `robot:=real`.

Os resultados (CSV) ficam em `~/shared/` dentro do contentor e estão copiados em
`lw1/resultados/` e `lw2/resultados/`. Os vídeos estão em `lw1/` e `lw2/`.

---

## Windows

Testado em Windows 11 com PowerShell, Python 3.14.5 e Docker Desktop (motor WSL 2). Só está aqui o que foi corrido neste sistema: o `pytest` e os testes 1, 3 e 8.

> - **[Windows]** — no PowerShell (a linha começa por `PS C:\...>`)
> - **[Contentor]** e **[Simulador]** — como no resto deste README

Antes de começar: Docker Desktop aberto, com **Engine running**. Os scripts `.ps1` correram com a política de execução `RemoteSigned`.

### W1. Projeto e ambiente Python — [Windows]

```powershell
cd ~/Documents
git clone https://github.com/ritaisabelmorais/robotica-grupo02.git
python -m venv $HOME/venvs/robotica
& "$HOME\venvs\robotica\Scripts\Activate.ps1"
cd ~/Documents/robotica-grupo02
pip install -r requirements.txt
python -m pytest                             # testes sem robô (3 passed)
```

### W2. Contentor ROS 2 da UC — [Windows]

Usa o ramo `windows` e fica numa pasta à parte deste projeto.

```powershell
cd ~/Documents
git clone -b windows https://github.com/cazevedo/robotica.git
cd robotica
cp .env.example .env
notepad .env
```

No ficheiro `.env`, pôr `ROS_DOMAIN_ID=2` (número do grupo). Depois:

```powershell
.\lab.ps1 build
```

Demorou cerca de 30 minutos. **Atenção:** se já houver outra cópia deste contentor no mesmo PC (por exemplo, a das aulas), o build substitui a imagem `robotics-lab0:jazzy` que ambas usam, e a imagem anterior deixa de existir.

### W3. Testes ROS 2 (testes 1 e 3)

Antes de começar: simulador de firmware desligado (`docker stop uf_software`).

**Janela 1 — [Windows]:** ligar o contentor e entrar nele

```powershell
cd ~/Documents/robotica
.\lab.ps1 up
.\lab.ps1 doctor                             # All checks passed
.\lab.ps1 shell
```

O RViz abre como janela normal do Windows, através do WSLg (`display mode wslg` no `doctor`). Não é preciso abrir `localhost:6080`.

**Teste 1** (precisa de duas janelas):

- **Janela 1 — [Contentor]:** `lab test 1` (fica a publicar "Hello World")
- **Janela 2 — [Windows]:** `cd ~/Documents/robotica; .\lab.ps1 shell`
  depois **[Contentor]:** `lab test 1b` (deve mostrar `I heard: [Hello World: N]`)
- Parar com **Ctrl+C** nas duas.

**Teste 3 — [Contentor]:** `lab test 3` (abre o RViz com o Lite 6 e o painel MotionPlanning). Parar com **Ctrl+C**.

**Fechar tudo:** `exit` nas duas janelas do [Contentor] e `.\lab.ps1 down` no [Windows].

### W4. Simulador de firmware e teste 8

Antes de começar: contentor ROS desligado (`.\lab.ps1 down`).

**Janela 1 — [Windows]:** da primeira vez, criar o simulador (tudo numa só linha):

```powershell
docker run -it --platform linux/amd64 --name uf_software -p 18333:18333 -p 502:502 -p 503:503 -p 504:504 -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 danielwang123321/uf-ubuntu-docker
```

Das vezes seguintes:

```powershell
docker start -ai uf_software
```

**[Simulador]:** arrancar o firmware do Lite 6 e deixar esta janela aberta, sem fazer `exit`

```bash
/xarm_scripts/xarm_start.sh 6 9
```

Volta logo ao início da linha, com `No Sockets found` e `There is a screen on`. É normal: o firmware fica a correr em segundo plano.

**Browser:** como na Parte C. Abrir http://127.0.0.1:18333, escolher **No End Effector** e manter **Initial Position** premido até as juntas chegarem a 0.

**Janela 2 — [Windows]:** correr o projeto

```powershell
cd ~/Documents/robotica-grupo02
& "$HOME\venvs\robotica\Scripts\Activate.ps1"
python -m scripts.check_connection           # liga-se e vai a home (home code: 0)
python -m scripts.test8_fk                   # teste 8 (Pose A: x, y, z = 87,0 / 0 / 154,2 mm)
```

O `[set_state], xArm is not ready to move` no fim do `check_connection` é normal: o `safe_session` para o robô ao terminar.

**Fechar:** `docker stop uf_software` no [Windows].

---

## Configuração

Todos os parâmetros estão em `config/robot.yaml`; nenhum IP está no código do projeto (`lite6/`, `scripts/`, `lw1/`, `lw2/`). Os scripts do LW0 parte 1, em `lw0_parte1/`, foram corridos no IDE do Studio antes desta regra e têm o IP da bancada.

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
