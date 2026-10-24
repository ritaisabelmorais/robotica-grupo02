"""Testes T1-T9 do exercicio de ciclo de vida (safe_session e home).

Correr contra o simulador de firmware (ip 127.0.0.1 no config/robot.yaml).
T7 e o pytest (tests/test_config.py).
"""
import logging
import math
import sys
import time

import yaml
from xarm.wrapper import XArmAPI

from lite6.session import safe_session, home, move_to

cfg = yaml.safe_load(open("config/robot.yaml"))
log = logging.getLogger("lite6")


# Poses de teste: so J1 (base) e J6 (punho) rodam. Com J3 = +20 graus o
# controlador deu C23 (limite de junta) e com J3 = -10 graus deu C22
# (auto-colisao): perto de home o braco esta dobrado e J2/J3/J5 tem pouca folga.
def dist(a, b):
    return math.dist(a[:3], b[:3])


def recover(arm, quem):
    """Depois de um erro do controlador: regista-o, limpa-o e volta a ativar."""
    print(f"   [{quem}] erro {arm.error_code} warn {arm.warn_code} -> a recuperar")
    log.info("%s: erro %s warn %s, a recuperar", quem, arm.error_code, arm.warn_code)
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)


# T6 - tempo de abertura do safe_session (10x)
# (python -m scripts.lifecycle_tests --sem-t6 salta esta parte)
if "--sem-t6" not in sys.argv:
    tempos = []
    for _ in range(10):
        t0 = time.time()
        with safe_session(cfg):
            tempos.append(time.time() - t0)
    print(f"T6: medio {sum(tempos) / len(tempos):.2f} s, max {max(tempos):.2f} s")

with safe_session(cfg) as arm:
    # T1 - estado depois de abrir
    print("T1: err", arm.error_code, "warn", arm.warn_code,
          "mode", arm.mode, "state", arm.state)

    # T2 - modo reduzido lido de volta do controlador
    print("T2:", arm.get_reduced_states())

    # T3 + T5 - home 10x a partir de poses diferentes
    piores, codigos = [], []
    for k in range(10):
        alvo = [10 * k - 45, 0, 0, 0, 0, 10 * k - 45]
        c = arm.set_servo_angle(angle=alvo, speed=cfg["home_speed"], wait=True)
        if c != 0 or arm.error_code != 0:
            print(f"   [T3 repeticao {k}] ida para {alvo}: code {c}")
            recover(arm, f"T3 ida {k}")
        code = home(arm, cfg)
        codigos.append(code)
        if code != 0 or arm.error_code != 0:
            recover(arm, f"T3 repeticao {k}")
        time.sleep(1.0)                      # deixar assentar
        _, ang = arm.get_servo_angle()       # leitura direta, nao a cache
        piores.append(max(abs(a - h) for a, h in zip(ang[:6], cfg["home"])))
    print(f"T3: codigos de home {codigos}")
    print(f"T5: pior erro por junta {max(piores):.4f} graus")

    # T8 - velocidade do TCP durante o home
    c = arm.set_servo_angle(angle=[60, 0, 0, 0, 0, 0],
                            speed=cfg["home_speed"], wait=True)
    print(f"   [T8] ida para J1 = 60 graus: code {c}")
    amostras = []
    arm.set_servo_angle(angle=cfg["home"], speed=cfg["home_speed"], wait=False)
    time.sleep(0.2)
    while arm.get_is_moving():
        amostras.append((time.time(), arm.position))
        time.sleep(0.05)
    passo = 10                               # 10 amostras = ~0,5 s
    v = [dist(amostras[i + passo][1], amostras[i][1]) /
         (amostras[i + passo][0] - amostras[i][0])
         for i in range(len(amostras) - passo)]
    v_inst = [dist(p2, p1) / (t2 - t1)
              for (t1, p1), (t2, p2) in zip(amostras, amostras[1:])]
    if v_inst:
        print(f"T8: pico entre amostras consecutivas {max(v_inst):.1f} mm/s "
              f"(sensivel ao ritmo a que a posicao e atualizada)")
    if v:
        print(f"T8: v_max em janelas de 0,5 s {max(v):.1f} mm/s (limite {cfg['reduced_tcp_speed']} mm/s), "
              f"{len(amostras)} amostras")
    else:
        print("T8: sem amostras (o movimento terminou antes de amostrar)")

    # T9 - pose fora da fronteira do TCP
    if arm.error_code != 0:
        recover(arm, "antes do T9")
    home(arm, cfg)
    antes = arm.position
    x_fora = cfg["tcp_boundary"][0] + 100   # 100 mm alem de x_max
    # T9a - pedido direto ao controlador (set_position)
    code = arm.set_position(x=x_fora, y=0, z=antes[2], roll=180, pitch=0, yaw=0,
                            speed=20, wait=True)
    log.info("T9a: set_position fora da fronteira -> code=%s", code)
    print(f"T9a (controlador): pedido x={x_fora} mm (x_max {cfg['tcp_boundary'][0]}) "
          f"-> code {code}, deslocamento {dist(arm.position, antes):.2f} mm, "
          f"posicao final {[round(v, 1) for v in arm.position[:3]]} mm, "
          f"err {arm.error_code} warn {arm.warn_code}")
    recover(arm, "depois do T9a")
    home(arm, cfg)
    # T9b - o mesmo pedido atraves de lite6.session.move_to
    antes = arm.position
    code = move_to(arm, cfg, x=x_fora, y=0, z=antes[2])
    print(f"T9b (move_to): code {code}, deslocamento {dist(arm.position, antes):.2f} mm")

# T4 - sair por excecao
try:
    with safe_session(cfg):
        raise RuntimeError("teste T4")
except RuntimeError:
    pass
chk = XArmAPI(cfg["ip"], check_joint_limit=False)
print("T4: estado depois da excecao:", chk.state, "(4 = parado)")
chk.disconnect()
