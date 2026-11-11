"""
safe_session.py -- LW0 Parte 1, ponto 6 da secção 7

Abre uma sessão segura com o Lite 6:
  liga-se, limpa erros/avisos, ativa o movimento, define modo 0 e estado 0,
  aplica o modo reduzido com os limites do §3 e uma fronteira do TCP,
  e devolve o objeto XArmAPI.

Uso noutros scripts:
    from safe_session import safe_session
    arm = safe_session()
    ...
    arm.disconnect()

ANTES DE USAR: preencher BOUNDARY com os valores da vossa bancada,
combinados com o docente. O script recusa-se a correr sem isso.
"""

from xarm.wrapper import XArmAPI

ROBOT_IP = '192.168.1.181'

# Limites do §3 do enunciado
MAX_JOINT_SPEED = 54      # graus/s (30% de 180 graus/s)
MAX_TCP_SPEED = 150       # mm/s

# Fronteira do TCP da bancada 2, em mm, no referencial da base:
# [x_max, x_min, y_max, y_min, z_max, z_min]
# Dimensionada para conter, com margem, todas as poses usadas no LW0
# (calculadas com a DH do fabricante):
#   - pose com tudo a zero ........... (87, 0, 154)
#   - Initial Position (home) ........ (275, 70, 545)
#   - poses do 2001 adaptado ......... y até 294, z até 504
#   - leituras no Live Control ....... x 204-275, z 246-545
# z_min = 80 mm mantém a flange acima da bancada. Como o offset do TCP
# é zero, a fronteira aplica-se à flange: a ponta da garra fica mais abaixo.
# A rever com o docente se a bancada tiver obstáculos dentro desta caixa.
BOUNDARY = [450, -150, 400, -400, 600, 80]


def check(code, what):
    """Todas as chamadas devolvem um código; diferente de 0 é erro."""
    if code != 0:
        raise RuntimeError(f'{what} falhou com código {code}')


def safe_session(ip=ROBOT_IP):
    if BOUNDARY is None:
        raise RuntimeError('Preencher BOUNDARY em safe_session.py antes de usar.')

    arm = XArmAPI(ip, is_radian=False)          # mm e graus
    if not arm.connected:
        raise RuntimeError(f'Não foi possível ligar a {ip}')

    # 1. limpar avisos e erros registados
    check(arm.clean_warn(), 'clean_warn')
    check(arm.clean_error(), 'clean_error')

    # 2. ativar o movimento
    check(arm.motion_enable(enable=True), 'motion_enable')

    # 3. modo reduzido: velocidades e fronteira do TCP
    check(arm.set_reduced_max_joint_speed(MAX_JOINT_SPEED), 'set_reduced_max_joint_speed')
    check(arm.set_reduced_max_tcp_speed(MAX_TCP_SPEED), 'set_reduced_max_tcp_speed')
    check(arm.set_reduced_tcp_boundary(BOUNDARY), 'set_reduced_tcp_boundary')
    check(arm.set_fence_mode(True), 'set_fence_mode')     # ativa a fronteira
    check(arm.set_reduced_mode(True), 'set_reduced_mode')

    # 4. modo 0 (controlo de posição) e estado 0 (pronto)
    check(arm.set_mode(0), 'set_mode')
    check(arm.set_state(0), 'set_state')

    print('Sessão segura pronta. Modo reduzido ativo:',
          f'juntas {MAX_JOINT_SPEED} graus/s, TCP {MAX_TCP_SPEED} mm/s, fronteira {BOUNDARY}')
    return arm


if __name__ == '__main__':
    # Teste: abre a sessão, mostra o estado reduzido e desliga. Não move o braço.
    arm = safe_session()
    print('estado reduzido:', arm.get_reduced_states())
    arm.disconnect()
