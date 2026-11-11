"""
move_joint_lite6.py -- exemplo xarm6/2001-move_joint adaptado ao Lite 6 (§6.2)

Alterações face ao original:
  1. J3: o original usa -60 graus, fora da gama do Lite 6 (-3.5 a +300 graus).
     Aqui usa-se +60 graus.
  2. Velocidade: o original usa 50 graus/s, muito perto do limite de 54 graus/s
     do modo reduzido. Aqui usa-se 20 graus/s.
  3. Removida a segunda metade do original (a mesma sequência em radianos).
  4. Verificação do código devolvido por cada chamada.
O original não usa a garra, por isso não há chamadas de garra a trocar.

MOSTRAR AO DOCENTE ANTES DE CORRER. Confirmar com ele que as poses
abaixo não batem na bancada nem saem da fronteira do TCP.
"""

from xarm.wrapper import XArmAPI

ROBOT_IP = '192.168.1.181'
SPEED = 20   # graus/s

# Sequência do original, com J3 trocado de -60 para +60
POSES = [
    [90,   0,  0, 0, 0, 0],
    [90,   0, 60, 0, 0, 0],
    [90, -30, 60, 0, 0, 0],
    [0,  -30, 60, 0, 0, 0],
    [0,    0, 60, 0, 0, 0],
    [0,    0,  0, 0, 0, 0],
]


def check(code, what):
    if code != 0:
        raise RuntimeError(f'{what} falhou com código {code}')


arm = XArmAPI(ROBOT_IP, is_radian=False)
try:
    check(arm.clean_warn(), 'clean_warn')
    check(arm.clean_error(), 'clean_error')
    check(arm.motion_enable(enable=True), 'motion_enable')
    check(arm.set_mode(0), 'set_mode')
    check(arm.set_state(0), 'set_state')

    check(arm.move_gohome(speed=SPEED, wait=True), 'move_gohome')
    for pose in POSES:
        check(arm.set_servo_angle(angle=pose, speed=SPEED, wait=True), f'set_servo_angle {pose}')
        code, angles = arm.get_servo_angle()
        print('pedido:', pose, '-> lido:', [round(a, 2) for a in angles[:6]])
    check(arm.move_gohome(speed=SPEED, wait=True), 'move_gohome')
finally:
    arm.disconnect()
