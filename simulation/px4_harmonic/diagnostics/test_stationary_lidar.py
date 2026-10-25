"""Run a sensor-only forest copy in an isolated Gazebo partition; never start PX4."""
import json
import os
from pathlib import Path
import signal
import subprocess
from analyze_stationary_lidar import analyze

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'diagnostics/stationary_lidar'
OUT.mkdir(exist_ok=True)
env = dict(os.environ, GZ_PARTITION=f'sando_sensor_test_{os.getpid()}',
           LIBGL_ALWAYS_SOFTWARE='1')
server = None
try:
    with (OUT / 'server.log').open('w') as log:
        server = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '-v', '4', '--headless-rendering',
             str(ROOT / 'gazebo_worlds/hacktx_forest_lidar_test.sdf')],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print('Running separate stationary sensor test; existing PX4 remains untouched.', flush=True)
        capture = subprocess.run(
            ['gz', 'topic', '-e', '--json-output', '-n', '1', '-t', '/sando/lidar/scan'],
            env=env, capture_output=True, text=True, timeout=45)
        (OUT / 'scan.json').write_text(capture.stdout)
        (OUT / 'capture_error.txt').write_text(capture.stderr)
        if capture.returncode:
            raise RuntimeError(f'Capture exited with code {capture.returncode}')
        # Protobuf JSON uses camelCase and strings for nonfinite ranges.
        report = analyze(json.loads(capture.stdout))
        (OUT / 'summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
except (subprocess.TimeoutExpired, ValueError, RuntimeError, TypeError, KeyError) as error:
    (OUT / 'failure.txt').write_text(str(error))
    print(f'Test did not pass: {error}. Logs saved in {OUT}')
finally:
    if server is not None:
        # Signal only the new test's process group, never existing Gazebo/PX4.
        try:
            os.killpg(server.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            server.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(server.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGKILL)
                server.wait()
