"""Classify observed offsets without changing the running robotics system."""
import json
from pathlib import Path
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parent/'alignment'


def assess(observations,parameters):
    flags=[]
    params={}
    def collect(value):
        if isinstance(value,dict):
            if 'ros__parameters' in value:params.update(value['ros__parameters'])
            for item in value.values():collect(item)
    collect(parameters)
    sample=observations.get('last_alignment_observation')
    bridge=observations.get('state_bridge_comparison')
    if bridge and bridge['receipt_time_separation_s']<.1 and bridge['difference_m']>.1:
        flags.append('PX4-to-SANDO state position disagreement; verify conversion/origin before planner changes')
    if sample:
        internal=np.asarray(sample['planner_reported_state_enu_m'])
        start=np.asarray(sample['trajectory_initial_position_enu_m'])
        if (params.get('use_state_update') is False and sample['planner_state_difference_m']>.75
                and np.linalg.norm(start-internal)<.1):
            flags.append('Evidence supports frozen planner initialization: measured state differs, while trajectory starts at the frozen internal state')
        elif sample['planner_state_difference_m']<.25 and sample['position_difference_m']>.75:
            flags.append('Planner internal state aligns, but current trajectory reference does not; investigate trajectory age/plan timing')
        elif sample['planner_state_difference_m']>.75:
            flags.append('Planner internal state differs from measured state; inspect state subscription, initialization, and active parameters')
        if sample['header_age_s']>1:
            flags.append('Trajectory header is over one second old on arrival')
    if not flags:flags.append('Insufficient evidence for a specific cause; inspect captured samples')
    return {'findings':flags,'use_state_update':params.get('use_state_update'),
            'use_sim_time':params.get('use_sim_time'),
            'captured_trajectory_samples':len(observations.get('alignment_samples',[])),
            'limits':'Observational diagnosis only; no thresholds, parameters, flight modes, or commands changed.'}


if __name__=='__main__':
    observations=json.loads((ROOT/'observations.json').read_text())
    parameters=yaml.safe_load((ROOT/'planner_parameters.yaml').read_text()) or {}
    report=assess(observations,parameters)
    (ROOT/'assessment.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
