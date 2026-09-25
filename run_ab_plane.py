"""Plane-condition A/B test vs Jin 2024 (whose repo uses plane stress)."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asq import benchmarks as B
from run_bench import run_case, JIN_ANCHORS

# add anchors under the temp names
JIN_ANCHORS['jin_snt_pf_ps'] = JIN_ANCHORS['jin_snt_pf']
JIN_ANCHORS['jin_sns_pf_pe'] = JIN_ANCHORS['jin_sns_pf']
JIN_ANCHORS['jin_sns_pf_ps'] = JIN_ANCHORS['jin_sns_pf']

cases = [
    ('jin_snt_pf_ps', B.jin_snt_pf(plane_stress=True)),
    ('jin_sns_pf_pe', B.jin_sns_pf()),
    ('jin_sns_pf_ps', B.jin_sns_pf(plane_stress=True)),
]
for name, case in cases:
    try:
        run_case(name, case)
    except Exception:
        import traceback
        traceback.print_exc()
