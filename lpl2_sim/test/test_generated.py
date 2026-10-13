# The world, map, masks and robot model are generated from config/corridor.yaml.
# Fail if someone edited the YAML (or a generated file) without regenerating.
import os
import subprocess
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(tool):
    return subprocess.run([sys.executable, os.path.join(PKG, 'tools', tool), '--check'],
                          capture_output=True, text=True)


def test_corridor_files_up_to_date():
    r = run('gen_corridor.py')
    assert r.returncode == 0, r.stdout + r.stderr


def test_robot_files_up_to_date():
    r = run('gen_robot.py')
    assert r.returncode == 0, r.stdout + r.stderr
