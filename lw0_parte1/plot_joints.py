"""
plot_joints.py -- desenha theta_1..6(t) a partir de um CSV do log_joints.py.
Usar no portátil se o log_joints.py correu num sítio sem matplotlib.
    python plot_joints.py                          # lê joints_log.csv
    python plot_joints.py joints_log_amostrado.csv
"""

import csv
import sys

import matplotlib.pyplot as plt

fname = sys.argv[1] if len(sys.argv) > 1 else 'joints_log.csv'
with open(fname) as f:
    rows = [[float(x) for x in r] for r in list(csv.reader(f))[1:]]

t = [r[0] for r in rows]
duration = t[-1] - t[0]
rate = (len(rows) - 1) / duration if duration > 0 else 0

fig, ax = plt.subplots(figsize=(8, 4))
for j in range(6):
    ax.plot(t, [r[j + 1] for r in rows], label=f'$\\theta_{j + 1}$')
ax.set_xlabel('t (s)')
ax.set_ylabel('ângulo (graus)')
ax.set_title(f'Ângulos das juntas em modo manual ({rate:.1f} Hz)')
ax.legend(ncol=6, fontsize=8)
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig('joints_plot.png', dpi=200)
print(f'{len(rows)} amostras, cadência média {rate:.1f} Hz -> joints_plot.png')
