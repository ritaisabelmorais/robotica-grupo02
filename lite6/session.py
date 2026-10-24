"""Sessao segura e movimento para home do UFACTORY Lite 6.

Baseado no safe_session.py da LW0 parte 1: liga-se, limpa erros e avisos,
ativa o movimento, aplica o modo reduzido e a fronteira do TCP, e poe o
braco em modo 0 / estado 0. Diferencas em relacao a parte 1:
  - todos os parametros vem de config/robot.yaml (nenhum IP no codigo);
  - e um context manager: ao sair (normalmente ou por excecao) para o
    braco (estado 4) e desliga-se;
  - cada passo fica registado em logs/session.log.

Uso:
    cfg = yaml.safe_load(open("config/robot.yaml"))
    with safe_session(cfg) as arm:
        home(arm, cfg)
"""
import logging
from contextlib import contextmanager

from xarm.wrapper import XArmAPI

log = logging.getLogger("lite6")


def check(code, what):
    """Todas as chamadas devolvem um codigo; diferente de 0 e erro.

    Algumas funcoes do SDK (p. ex. set_fence_mode) devolvem o codigo
    dentro de uma lista, [0]; nesse caso usa-se o primeiro elemento.
    """
    if isinstance(code, (list, tuple)):
        code = code[0]
    log.info("%s -> code=%s", what, code)
    if code != 0:
        raise RuntimeError(f"{what} falhou com codigo {code}")


@contextmanager
def safe_session(cfg):
    logging.basicConfig(filename=cfg["log_file"], level=logging.INFO,
                        format="%(asctime)s %(message)s")
    arm = XArmAPI(cfg["ip"], is_radian=False,                 # mm e graus
                  check_joint_limit=cfg["check_joint_limit"])
    if not arm.connected:
        raise RuntimeError(f"Nao foi possivel ligar a {cfg['ip']}")
    # tempo maximo de espera pela resposta do controlador (s); o simulador
    # emulado responde mais devagar do que o braco real
    arm.set_timeout(cfg.get("cmd_timeout", 2))
    try:
        # 1. limpar avisos e erros registados
        check(arm.clean_warn(), "clean_warn")
        check(arm.clean_error(), "clean_error")
        # 2. ativar o movimento
        # o simulador emulado as vezes nao responde a tempo (codigo 3);
        # ativar o movimento e idempotente, por isso tenta-se ate N vezes
        for tentativa in range(1, cfg.get("enable_retries", 1) + 1):
            code = arm.motion_enable(enable=True)
            log.info("motion_enable tentativa %d -> code=%s", tentativa, code)
            if code == 0:
                break
        check(code, "motion_enable")
        # modelo geometrico da ferramenta usado na detecao de auto-colisao;
        # tem de corresponder a ferramenta montada (ver config/robot.yaml)
        check(arm.set_collision_tool_model(cfg["collision_tool_model"]),
              "set_collision_tool_model")
        # 3. modo reduzido: velocidades e fronteira do TCP
        if cfg["reduced_mode"]:
            check(arm.set_reduced_max_joint_speed(cfg["reduced_joint_speed"]),
                  "set_reduced_max_joint_speed")
            check(arm.set_reduced_max_tcp_speed(cfg["reduced_tcp_speed"]),
                  "set_reduced_max_tcp_speed")
            check(arm.set_reduced_tcp_boundary(cfg["tcp_boundary"]),
                  "set_reduced_tcp_boundary")
            check(arm.set_fence_mode(True), "set_fence_mode")   # ativa a fronteira
            check(arm.set_reduced_mode(True), "set_reduced_mode")
        # 4. modo 0 (controlo de posicao) e estado 0 (pronto)
        check(arm.set_mode(0), "set_mode")
        check(arm.set_state(0), "set_state")
        log.info("sessao aberta: err=%s warn=%s mode=%s state=%s",
                 arm.error_code, arm.warn_code, arm.mode, arm.state)
        yield arm
    finally:
        arm.set_state(4)  # parar
        log.info("sessao fechada")
        arm.disconnect()


def home(arm, cfg):
    """Move o braco para a configuracao home do config e devolve o codigo."""
    code = arm.set_servo_angle(angle=cfg["home"], speed=cfg["home_speed"], wait=True)
    log.info("home: code=%s angles=%s", code, arm.angles)
    return code


# Codigo devolvido quando o proprio software recusa um alvo (nao e do SDK).
FORA_DA_FRONTEIRA = -100


def dentro_da_fronteira(cfg, x, y, z):
    """True se (x, y, z) em mm estiver dentro de tcp_boundary do config."""
    x_max, x_min, y_max, y_min, z_max, z_min = cfg["tcp_boundary"]
    return x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max


def move_to(arm, cfg, x, y, z, roll=180, pitch=0, yaw=0, speed=20):
    """Movimento cartesiano que recusa, sem mexer o braco, alvos fora da fronteira.

    O controlador nao valida o alvo de set_position contra a fronteira antes de
    comecar o movimento (verificado no teste T9), por isso a verificacao e feita aqui.
    """
    if not dentro_da_fronteira(cfg, x, y, z):
        log.info("move_to recusado: alvo (%s, %s, %s) fora de %s",
                 x, y, z, cfg["tcp_boundary"])
        return FORA_DA_FRONTEIRA
    code = arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw,
                            speed=speed, wait=True)
    log.info("move_to (%s, %s, %s) -> code=%s", x, y, z, code)
    return code
