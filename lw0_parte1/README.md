# LW0 — Parte 1: operar o UFACTORY Lite 6

Scripts corridos no Python IDE do UFACTORY Studio, no robô da bancada 2 (IP `192.168.1.181`).

| Ficheiro | O que faz |
|---|---|
| `safe_session.py` | Liga ao robô, limpa erros e avisos, ativa o movimento, aplica o modo reduzido (juntas a 54°/s, TCP a 150 mm/s) e a fronteira do TCP da bancada, define modo 0 e estado 0 e devolve o objeto `XArmAPI`. Não move o braço. |
| `log_joints.py` | Lê os seis ângulos durante 30 s, o mais depressa possível, com o braço guiado à mão em modo manual. Guarda `joints_log.csv` e, se houver matplotlib, `joints_plot.png`. |
| `plot_joints.py` | Desenha θ1..6(t) a partir de um CSV do `log_joints.py`: `python plot_joints.py joints_log_amostrado.csv`. |
| `move_joint_lite6.py` | Exemplo `xarm6/2001-move_joint` do SDK adaptado ao Lite 6 (J3 de −60° para +60°, velocidade 20°/s, sem a repetição em radianos). |
| `joints_log_amostrado.csv` | Registo do `log_joints.py` no robô: 61 145 amostras em 30 s (2038 Hz). Este ficheiro tem uma amostra a cada 0.5 s, copiada da consola do Studio. |
| `joints_plot.png` | Gráfico θ1..6(t) desse registo (Figura 1 do relatório). |

A fronteira do TCP em `safe_session.py` contém todas as poses usadas no LW0 e aplica-se à flange (offset do TCP nulo).
