"""Launch and clean up only this isolated stationary perception test."""
from contextlib import ExitStack
import json
import os
from pathlib import Path
import re
import signal
import sys
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
planner_test = '--planner' in sys.argv
capture_solver = '--capture-solver' in sys.argv
longer_timing = '--longer-timing' in sys.argv
OUT = ROOT / ('diagnostics/solver_capture' if capture_solver else (('diagnostics/sando_lidar_longer_timing' if longer_timing else 'diagnostics/sando_lidar') if planner_test else 'diagnostics/ros_lidar'))
OUT.mkdir(exist_ok=True)
# Unique Gazebo partition plus dedicated ROS domain; no existing planner topics.
env = dict(os.environ, GZ_PARTITION=f'sando_ros_sensor_{os.getpid()}',
           ROS_DOMAIN_ID='88' if planner_test else '87', LIBGL_ALWAYS_SOFTWARE='1', ROS_LOG_DIR=str(OUT / 'ros_logs'))
# ros_gz_bridge installs a vendor gz wrapper/config path without Gazebo Sim.
# Use system Harmonic only for the simulator; retain ROS vendor paths for bridge.
server_env = dict(env, GZ_CONFIG_PATH='/usr/share/gz')
server_env['LD_LIBRARY_PATH'] = ':'.join(
    path for path in env.get('LD_LIBRARY_PATH', '').split(':')
    if path and not path.startswith('/opt/ros/'))
env['SANDO_TEST_OUTPUT'] = str(OUT)
(OUT / 'summary.json').write_text(json.dumps({'status': 'RUNNING: no result yet'}))
children = []
result = 1


def stop_child(child):
    try:
        os.killpg(child.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()


with ExitStack() as stack:
    try:
        preflight = subprocess.run(['/usr/bin/gz', 'sim', '--versions'], env=server_env,
                                   capture_output=True, text=True, timeout=8)
        (OUT / 'gazebo_preflight.txt').write_text(preflight.stdout + preflight.stderr)
        if preflight.returncode or not re.search(r'^8\.\d+\.\d+', preflight.stdout, re.MULTILINE):
            raise RuntimeError('System Gazebo Harmonic command unavailable; see gazebo_preflight.txt')
        commands = [
            ('server', ['/usr/bin/gz', 'sim', '-s', '-r', '-v', '4', '--headless-rendering',
                        str(ROOT / 'gazebo_worlds/hacktx_forest_lidar_test.sdf')], server_env),
            ('bridge', ['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge', '--ros-args',
                        '-p', f'config_file:={ROOT / "config/stationary_lidar_ros_test.yaml"}',
                        '-r', '__node:=stationary_lidar_test_bridge'], env),
        ]
        if planner_test:
            planner_env = dict(env)
            if capture_solver:
                snapshot = OUT / 'failed_model.lp'
                snapshot.unlink(missing_ok=True)
                planner_env['SANDO_SOLVER_SNAPSHOT'] = str(snapshot)
                planner_env['LD_PRELOAD'] = str(ROOT / 'diagnostics/capture_gurobi_model.so')
            commands.append(('planner', [str(ROOT / 'sando_harmonic_ws/install/sando/lib/sando/sando'),
                '--ros-args', '-r', '__ns:=/NX01', '--params-file',
                str(ROOT / 'sando_harmonic/config/sando_nx01.yaml'),
                '-p', 'sim_env:=fake_sim', '-p', 'use_global_pc:=false',
                '-p', 'use_state_update:=false', '-p', 'debug_verbose:=true',
                '-p', 'visual_level:=0'], planner_env))
        if planner_test and longer_timing:
            commands[-1][1].extend([
                '-p', 'use_dynamic_factor:=false',
                '-p', 'factor_initial:=2.5', '-p', 'factor_final:=4.0',
                '-p', 'factor_constant_step_size:=0.25'])
        for label, command, child_env in commands:
            log = stack.enter_context((OUT / f'{label}.log').open('w'))
            children.append((label, subprocess.Popen(command, env=child_env, stdout=log,
                                                    stderr=subprocess.STDOUT, start_new_session=True)))
        print(f'Testing stationary perception/planning on isolated domain {env["ROS_DOMAIN_ID"]}. No PX4 connection.', flush=True)
        validator = subprocess.Popen(['python3', str(ROOT / ('diagnostics/sando_lidar_fixture.py' if planner_test else 'diagnostics/validate_ros_lidar.py'))],
                                     env=env, start_new_session=True)
        monitored = list(children)
        children.append(('validator', validator))
        deadline = time.monotonic() + 60
        while validator.poll() is None:
            if capture_solver and (OUT / 'failed_model.lp').exists():
                (OUT / 'summary.json').write_text(json.dumps({'status': 'CAPTURED: failed Gurobi model; not a planning pass'}))
                result = 0
                break
            for label, child in monitored:
                if child.poll() is not None:
                    raise RuntimeError(f'{label} exited early with code {child.returncode}; see {label}.log')
            if time.monotonic() > deadline:
                raise RuntimeError('Validator exceeded 60 second deadline')
            try:
                validator.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                pass
        if not (capture_solver and result == 0):
            result = validator.returncode
    except (subprocess.TimeoutExpired, RuntimeError, OSError) as error:
        (OUT / 'runner_error.txt').write_text(str(error))
        (OUT / 'summary.json').write_text(json.dumps({'status': f'FAIL: {error}'}))
        print(str(error), flush=True)
    finally:
        for _, child in reversed(children):
            stop_child(child)
if result != 0:
    current = json.loads((OUT / 'summary.json').read_text())
    if current['status'].startswith('RUNNING'):
        (OUT / 'summary.json').write_text(json.dumps({'status': 'FAIL: validator did not produce a result',
                                                    'exit_code': result}))
print(f'Test exit code: {result}. Results: {OUT}', flush=True)
raise SystemExit(result)
