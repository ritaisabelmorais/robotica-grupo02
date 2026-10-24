"""Liga-se ao controlador indicado no config, imprime o estado e vai a home."""
import yaml

from lite6.session import safe_session, home

cfg = yaml.safe_load(open("config/robot.yaml"))
with safe_session(cfg) as arm:
    print("angulos (graus):", arm.angles)
    print("pose (mm, graus):", arm.position)
    print("home code:", home(arm, cfg))
