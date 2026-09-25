"""Combined model-comparison figures for the report (SNT/SNS).

Reads results/*_rec.json and writes results/cmp_snt.png / cmp_sns.png
with all four constitutive models on one axes + Jin anchors (for PF).
"""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

R = 'results'


def load(name):
    try:
        r = json.load(open(f'{R}/{name}_rec.json'))
        return np.array(r['u']), np.array(r['F'])
    except FileNotFoundError:
        try:
            r = json.load(open(f'{R}/{name}_rec_partial.json'))
            return np.array(r['u']), np.array(r['F'])
        except FileNotFoundError:
            return None, None


for geom, cases in (
        ('snt', [('jin_snt_pf', 'AT2-PF (Jin mat.)', 'k-', 2.0),
                 ('jin_snt_mnld', 'CS-MNLD', 'b-', 1.6),
                 ('jin_snt_cgd', 'CS-CGD (UAL baseline)', 'g-', 1.6),
                 ('jin_snt_sc', 'CS-SC-MNLD', 'r-', 1.6)]),
        ('sns', [('jin_sns_pf', 'AT2-PF (Jin mat.)', 'k-', 2.0),
                 ('jin_sns_mnld', 'CS-MNLD', 'b-', 1.6),
                 ('jin_sns_cgd', 'CS-CGD (UAL baseline)', 'g-', 1.6),
                 ('jin_sns_sc', 'CS-SC-MNLD', 'r-', 1.6)])):
    fig, ax = plt.subplots(figsize=(6.0, 4.4))
    for name, lab, sty, lw in cases:
        u, F = load(name)
        if u is None:
            print('missing', name)
            continue
        ax.plot(u, F, sty, lw=lw, label=lab)
        i = int(np.argmax(F))
        ax.plot(u[i], F[i], 'o', ms=5, mfc='none', color=sty[0])
    # Jin digitised anchors (PF only)
    try:
        from run_bench import JIN_ANCHORS
        a = JIN_ANCHORS.get(f'jin_{geom}_pf')
        if a:
            ax.plot(a['u'], a['F'], 'kv', ms=5, label='Jin 2024 (digitised)')
    except Exception:
        pass
    ax.set_xlabel('u [mm]')
    ax.set_ylabel('F [kN]')
    ax.set_title({'snt': 'SNT', 'sns': 'SNS'}[geom]
                 + ' — model comparison (same $(\\sigma_c, G_f)$ calibration)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'{R}/cmp_{geom}.png', dpi=130)
    plt.close(fig)
    print('wrote', f'{R}/cmp_{geom}.png')

# summary numbers
print('\n== peaks ==')
for geom in ('snt', 'sns'):
    for name in (f'jin_{geom}_pf', f'jin_{geom}_mnld', f'jin_{geom}_cgd',
                 f'jin_{geom}_sc'):
        u, F = load(name)
        if u is None:
            continue
        i = int(np.argmax(F))
        print(f'{name:16s} peak {F[i]:.4f} @ {u[i]:.5f}  '
              f'end {F[-1]:.4f} @ {u[-1]:.5f}')
