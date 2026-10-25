#!/usr/bin/env bash
# Read-only simulator queries; no control commands or model changes.
set -u
cd /home/harneksab/hacktx || exit 1
out=diagnostics/active_lidar
mkdir -p "$out"
for entry in 'front /world/default/model/x500_0/link/lidar_sensor_link/sensor/lidar/scan' 'planar /world/default/model/x500_0/link/link/sensor/lidar_2d_v2/scan'; do
  read -r label topic <<< "$entry"
  timeout 5 gz topic -i -t "$topic" > "$out/${label}_info.txt" 2>&1
  timeout 8 gz topic -e --json-output -n 1 -t "$topic" > "$out/${label}_sample.json" 2> "$out/${label}_error.txt"
  printf '%s scan capture exit code: %s (124 means timeout)\n' "$label" "$?"
done
# This service returns a snapshot; it does not alter the running world.
timeout 15 gz service -s /world/default/generate_world_sdf \
  --reqtype gz.msgs.SdfGeneratorConfig --reptype gz.msgs.StringMsg \
  --timeout 10000 \
  --req 'global_entity_gen_config { expand_include_tags { data: true } }' \
  > "$out/world_response.txt" 2> "$out/world_error.txt"
python3 - <<'PY'
import ast
from pathlib import Path
import xml.etree.ElementTree as ET
out = Path('diagnostics/active_lidar')
text = (out / 'world_response.txt').read_text()
try:
    data = ''.join(ast.literal_eval(line.strip()[6:]) for line in text.splitlines()
                   if line.strip().startswith('data: '))
    if not data:
        raise ValueError('No SDF returned; inspect world_response.txt and world_error.txt')
    root = ET.fromstring(data)
    (out / 'active_world.sdf').write_text(data)
    lines = []
    for model in root.findall('.//model'):
        if 'x500' not in model.get('name', ''):
            continue
        lines.append(f'MODEL {model.attrib}')
        for link in model.findall('.//link'):
            for sensor in link.findall('sensor'):
                lines.append(f'LINK {link.get("name")} SENSOR:\n{ET.tostring(sensor, encoding="unicode")}')
    (out / 'sensors.txt').write_text('\n'.join(lines))
    print('Active world and sensor definitions captured.')
except (ValueError, SyntaxError, ET.ParseError) as error:
    (out / 'snapshot_error.txt').write_text(str(error))
    print(f'Snapshot extraction failed: {error}')
PY
printf 'Capture complete: diagnostics/active_lidar/\n'
