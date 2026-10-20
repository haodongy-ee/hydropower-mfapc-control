"""Regenerate the README's dependency-free SVG charts from committed CSV data.

Run from any directory: python scripts/plot_readme_results.py
"""

import csv
from html import escape
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIGURES = ROOT / "figures"
NAVY = "#16324f"
MUTED = "#52657a"
GRID = "#dce5ee"
COLORS = {"MFAC_baseline": "#7c8da0", "MFAPC_final": "#11a6a6", "MPC_offsetfree": "#e49b36"}
OLD = "#7c8da0"
NEW = "#087e85"


def rows(name):
    with (ROOT / "results" / name).open(newline="", encoding="utf-8-sig") as file:
        return list(csv.DictReader(file))


def rect(x, y, width, height, fill, radius=0):
    return f'<rect x="{x:.1f}" y="{y:.1f}" width="{width:.1f}" height="{height:.1f}" rx="{radius}" fill="{fill}"/>'


def line(x1, y1, x2, y2, stroke=GRID, dash=""):
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{stroke}" stroke-width="1.2"{extra}/>'


def label(x, y, value, size=16, color=NAVY, anchor="start", weight=400):
    return f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" font-size="{size}" font-weight="{weight}" fill="{color}">{escape(str(value))}</text>'


def save(name, width, height, items, description):
    FIGURES.mkdir(exist_ok=True)
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-label="{escape(description)}">'
        f'<title>{escape(description)}</title>'
        f'<rect width="100%" height="100%" fill="#ffffff"/>'
        '<g font-family="Arial, Helvetica, sans-serif">'
        + "".join(items)
        + "</g></svg>\n"
    )
    (FIGURES / name).write_text(svg, encoding="utf-8")


def controller_chart():
    data = {r["Controller"]: r for r in rows("mpc_offsetfree_average_summary.csv")}
    names = [("MFAC_baseline", "MFAC"), ("MFAPC_final", "MFAPC"), ("MPC_offsetfree", "Offset-free MPC")]
    metrics = [
        ("IAE (pu·s)", "IAE", 0.9, 3, 1),
        ("35 s overshoot (%)", "DirectionalOvershoot35", 4.2, 2, 1),
        ("Settling after 35 s (s)", "SettlingTime35", 2.6, 2, 1),
    ]
    out = [label(44, 49, "Tracking performance across six scenarios", 27, weight=700),
           label(44, 78, "Mean of the same six deterministic reference profiles · lower is better", 15, MUTED)]
    for index, (title, key, maximum, decimals, _) in enumerate(metrics):
        x = 44 + index * 322
        out += [rect(x, 109, 302, 365, "#f5f8fb", 10), label(x + 16, 144, title, 18, weight=700)]
        bar_x, bar_width = x + 119, 158
        for tick in range(3):
            gx = bar_x + tick * bar_width / 2
            out += [line(gx, 171, gx, 412), label(gx, 433, f"{tick * maximum / 2:.1f}", 12, MUTED, "middle")]
        for row, (code, display) in enumerate(names):
            y = 196 + row * 72
            value = float(data[code][key])
            out += [label(x + 16, y + 19, display, 14),
                    rect(bar_x, y, bar_width * value / maximum, 27, COLORS[code], 4),
                    label(bar_x, y - 6, f"{value:.{decimals}f}", 13, NAVY, weight=600)]
    out += [label(44, 511, "MPC has the lowest mean IAE and settling time; MFAPC has the lowest 35 s overshoot.", 16, MUTED)]
    save("controller_tradeoffs.svg", 1020, 545, out, "Three-controller comparison of mean IAE, 35-second overshoot, and settling time")


def noise_chart():
    before = {r["CaseID"]: r for r in rows("mpc_robustness_screening_results.csv")}
    after = {r["CaseID"]: r for r in rows("mpc_23_screening_results.csv")}
    cases = [("noise_001", "0.001"), ("noise_003", "0.003"), ("noise_005", "0.005")]
    out = [label(44, 49, "Noise robustness: phase 2 vs upgraded MPC", 27, weight=700),
           label(44, 78, "Matched deterministic cases, same noise seed and evaluation window", 15, MUTED),
           rect(670, 45, 18, 18, OLD, 3), label(697, 60, "Phase 2", 14),
           rect(790, 45, 18, 18, NEW, 3), label(817, 60, "Phase 2.3", 14)]
    panels = [
        ("Steady output fluctuation (pu)", "SteadyFluctuation", 0.014, ".3f", 0.008),
        ("Control total variation", "ControlTotalVariation", 4.0, ".2f", None),
    ]
    for index, (title, key, maximum, fmt, threshold) in enumerate(panels):
        x = 44 + index * 485
        out += [rect(x, 111, 463, 421, "#f5f8fb", 10), label(x + 16, 145, title, 18, weight=700)]
        top, bottom = 182, 446
        for tick in range(5):
            gy = bottom - tick * (bottom - top) / 4
            out += [line(x + 56, gy, x + 437, gy),
                    label(x + 48, gy + 5, format(tick * maximum / 4, fmt), 12, MUTED, "end")]
        if threshold is not None:
            gy = bottom - threshold / maximum * (bottom - top)
            out += [line(x + 56, gy, x + 437, gy, "#cc5149", "5 4"),
                    label(x + 436, gy - 7, "acceptance limit", 12, "#ae4039", "end", 600)]
        for pos, (case, noise) in enumerate(cases):
            cx = x + 122 + pos * 119
            for offset, source, color in [(-19, before, OLD), (10, after, NEW)]:
                value = float(source[case][key])
                height = value / maximum * (bottom - top)
                out += [rect(cx + offset, bottom - height, 27, height, color, 3),
                        label(cx + offset + 13.5, bottom - height - 7, format(value, fmt), 12, NAVY, "middle", 600)]
            out += [label(cx + 5, 479, noise, 15, NAVY, "middle")]
        out += [label(x + 244, 507, "Measurement noise σ (pu)", 14, MUTED, "middle")]
    out += [label(44, 568, "At σ = 0.003 and 0.005 pu, upgraded MPC moves below the 0.008 pu fluctuation limit.", 16, MUTED)]
    save("noise_robustness.svg", 1020, 595, out, "Noise robustness comparison showing steady fluctuation and control total variation before and after the MPC upgrade")


if __name__ == "__main__":
    controller_chart()
    noise_chart()
    print("Updated figures/controller_tradeoffs.svg and figures/noise_robustness.svg")
