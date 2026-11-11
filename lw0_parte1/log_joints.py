"""
log_joints.py -- LW0 Parte 1, ponto 5 da secção 7

Lê os seis ângulos das juntas durante 30 s, o mais depressa possível,
enquanto um colega guia o braço à mão em modo manual.
Guarda os dados em joints_log.csv, calcula a cadência de amostragem
e (se o matplotlib existir) desenha theta_1..6(t) em joints_plot.png.

ANTES DE CORRER:
  - carga do TCP corretamente definida no Studio (senão o braço cai em modo manual);
  - docente presente; um colega com a mão na paragem de emergência.
"""

import csv
import time

from xarm.wrapper import XArmAPI

ROBOT_IP = '192.168.1.181'
DURATION = 30.0   # segundos


def check(code, what):
    if code != 0:
        raise RuntimeError(f'{what} falhou com código {code}')


def main():
    arm = XArmAPI(ROBOT_IP, is_radian=False)
    check(arm.clean_warn(), 'clean_warn')
    check(arm.clean_error(), 'clean_error')
    check(arm.motion_enable(enable=True), 'motion_enable')

    input('Carga do TCP definida? Carreguem Enter para passar a modo manual...')
    check(arm.set_mode(2), 'set_mode(2)')        # 2 = modo manual (ensino)
    check(arm.set_state(0), 'set_state')
    print(f'A gravar durante {DURATION:.0f} s. Guiem o braço à mão.')

    rows = []
    t0 = time.perf_counter()
    try:
        while True:
            t = time.perf_counter() - t0
            if t >= DURATION:
                break
            code, angles = arm.get_servo_angle()
            if code != 0:
                print(f'aviso: get_servo_angle devolveu código {code} em t={t:.3f}s')
                continue
            rows.append([t] + list(angles[:6]))
    finally:
        # voltar sempre a modo de posição, mesmo que algo falhe
        arm.set_mode(0)
        arm.set_state(0)
        arm.disconnect()

    n = len(rows)
    rate = n / DURATION
    print(f'{n} amostras em {DURATION:.0f} s -> cadência média {rate:.1f} Hz')

    with open('joints_log.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
        w.writerows(rows)
    print('Dados guardados em joints_log.csv')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib não disponível: usem plot_joints.py no portátil com o CSV.')
        return

    plot(rows, rate)


def plot(rows, rate):
    import matplotlib.pyplot as plt
    t = [r[0] for r in rows]
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
    print('Gráfico guardado em joints_plot.png')


if __name__ == '__main__':
    main()
