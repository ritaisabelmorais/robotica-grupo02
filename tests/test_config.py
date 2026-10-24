"""Testes unitarios que nao precisam do robo (T7 / requisito I1)."""
import pathlib

import yaml


def test_config_carrega():
    cfg = yaml.safe_load(open("config/robot.yaml"))
    for chave in ("ip", "home", "reduced_tcp_speed", "tcp_boundary"):
        assert chave in cfg


def test_ip_fora_do_codigo():
    for f in pathlib.Path(".").rglob("*.py"):
        assert "192." + "168" not in f.read_text(), f


def test_fronteira():
    from lite6.session import dentro_da_fronteira
    cfg = yaml.safe_load(open("config/robot.yaml"))
    assert dentro_da_fronteira(cfg, 87, 0, 154)          # home
    assert not dentro_da_fronteira(cfg, 500, 0, 154)     # alem de x_max
    assert not dentro_da_fronteira(cfg, 150, 0, 30)      # abaixo de z_min
