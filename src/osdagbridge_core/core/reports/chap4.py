from typing import TYPE_CHECKING
from osdagbridge_core.core.utils.common import (
    KEY_SPAN,
    KEY_TS_NO_OF_GIRDERS,
    KEY_SD_DEFL_LIVE_RAW,
    KEY_SD_DEFL_TOTAL_RAW,
)
from osdagbridge_core.core.reports.report_utils import (
    _tex, _fig_embed, render_analysis_demands_table, render_report_table,
)

if TYPE_CHECKING:
    from .report_generator import ReportDataBridge


def build_vertical_support_reactions(result_handler):
    """Build individual girder support reactions for report Table 4.2."""
    ds = result_handler.ds
    if ds is None:
        return {}

    g_map, _ = result_handler.build_girders(verbose=False)
    classified = result_handler.classify_loadcases()

    # Exclude vehicle load cases to retain the applicable static load cases.
    exclude = set(
        classified.get("vehicle_static", [])
        + classified.get("vehicle_moving", [])
    )
    load_cases = [
        lc for lc in classified.get("all", [])
        if lc not in exclude and "moving" not in str(lc).lower()
    ]

    # Map interior girders to report labels; edge girders are not included.
    girder_map = {}
    gi = 1
    for gmap_key in g_map:
        if gmap_key.startswith("EB"):
            continue
        girder_map[f"G{gi}"] = gmap_key
        gi += 1

    reaction_data = {}
    # Use the existing reaction query to retain its support-orientation handling.
    reaction_query = getattr(result_handler, "_get_reactions_df", None)

    if not callable(reaction_query):
        return reaction_data

    for lc in load_cases:
        reaction_rows = reaction_query(lc)
        reaction_by_girder = {}

        if reaction_rows is not None and not reaction_rows.empty:
            reaction_by_girder = {
                str(row["Girder"]): row
                for _, row in reaction_rows.iterrows()
            }

        reaction_data[str(lc)] = {}
        for girder_label, gmap_key in girder_map.items():
            row = reaction_by_girder.get(gmap_key)
            # Preserve missing values rather than treating them as zero reactions.
            reaction_data[str(lc)][girder_label] = {
                "left_kN": float(row["Ra (kN)"]) if row is not None else None,
                "right_kN": float(row["Rb (kN)"]) if row is not None else None,
            }

    return reaction_data


def ch4_analysis(asum, fig_paths, bridge: "ReportDataBridge"):
    lc_summary  = (asum or {}).get('load_cases', {})
    rxn_summary = (asum or {}).get('reactions',  {})

    def _is_moving(lc_name: str) -> bool:
        n = lc_name.lower()
        return "moving" in n or " pos_" in n

    lc_summary  = {k: v for k, v in lc_summary.items()  if not _is_moving(k)}
    rxn_summary = {k: v for k, v in rxn_summary.items() if not _is_moving(k)}

    def _fmt(val, nd=3):
        try:
            return f"{float(val):.{nd}f}"
        except (TypeError, ValueError):
            return r"---"

    def _merged_row(lc, bm_d, rxn_d):
        bm_d  = bm_d  or {}
        rxn_d = rxn_d or {}
        return (
            _tex(lc)                              + r" & "
            + _fmt(bm_d.get('max_bm'))           + r" & "
            + _tex(bm_d.get('bm_girder', '---')) + r" & "
            + _fmt(bm_d.get('bm_location'))      + r" & "
            + _fmt(bm_d.get('max_sf'))           + r" & "
            + _tex(bm_d.get('sf_girder', '---')) + r" & "
            + _fmt(bm_d.get('sf_location'))      + r" & "
            + _fmt(rxn_d.get('left_kN'))         + r" & "
            + _fmt(rxn_d.get('right_kN'))        + r" \\[6pt]"
        )

    all_lcs = list(lc_summary.keys()) + [k for k in rxn_summary if k not in lc_summary]

    merged_body = ("\n\\hline\n").join(
        _merged_row(lc, lc_summary.get(lc), rxn_summary.get(lc))
        for lc in all_lcs
    ) if all_lcs else r"--- & --- & --- & --- & --- & --- & --- & --- & --- \\[6pt]"

    _span_m         = float(bridge.input_dict.get(KEY_SPAN, 0) or 0)
    _allow_live_mm  = _span_m * 1000.0 / 800.0
    _allow_total_mm = _span_m * 1000.0 / 600.0

    # Find governing girder — worst (max) deflection across all girders
    try:
        n = int(bridge.input_dict.get(KEY_TS_NO_OF_GIRDERS, 1) or 1)
    except (TypeError, ValueError):
        n = 1

    _live_mm  = None
    _total_mm = None
    for _gi in range(1, n + 1):
        # RAW (pre-camber) — this is the Analysis Results chapter, so it must report what
        # the analysis produced and match the deflection plots below.
        _l = bridge.output_dict.get(f"{KEY_SD_DEFL_LIVE_RAW}.G{_gi}")
        _t = bridge.output_dict.get(f"{KEY_SD_DEFL_TOTAL_RAW}.G{_gi}")
        if _l is not None:
            _live_mm  = max(_live_mm, float(_l))  if _live_mm  is not None else float(_l)
        if _t is not None:
            _total_mm = max(_total_mm, float(_t)) if _total_mm is not None else float(_t)

    _live_str        = f"{_live_mm:.3f} mm"  if _live_mm  is not None else "---"
    _total_str       = f"{_total_mm:.3f} mm" if _total_mm is not None else "---"
    _allow_live_str  = f"L/800 = {_allow_live_mm:.1f} mm"
    _allow_total_str = f"L/600 = {_allow_total_mm:.1f} mm"
    _live_status     = ("PASS" if _live_mm  <= _allow_live_mm  else r"\textcolor{red}{FAIL}") if _live_mm  is not None else "---"
    _total_status    = ("PASS" if _total_mm <= _allow_total_mm else r"\textcolor{red}{FAIL}") if _total_mm is not None else "---"
    defl_rows = [
        [r"Deflection due to Live Load, $\delta_{LL}$", _live_str],
        [r"Allowable Live Load Deflection ($\Delta_{allow}$)", _allow_live_str],
        ["Live Load Deflection Check Status", _live_status],
        [r"Deflection due to Total Load, $\delta_{total}$", _total_str],
        [r"Allowable Total Deflection ($\Delta_{allow}$)", _allow_total_str],
        ["Total Load Deflection Check Status", _total_status],
    ]
    # Embed the Plan View with A/B support markings before Table 4.2.
    """ + _fig_embed(
    fig_paths.get('girder_top_support_marked'),
    'Plan View showing support locations A and B.',
    width=r'0.85\textwidth'
    ) + r"""
    # Table 4.2 rows — individual girder reactions, taken straight from the
    # per-girder summary.  The summary's aggregate fields remain available for
    # the existing Table 4.1 and other consumers.
    vertical_support_reactions = bridge.output_dict.get(
        "build_vertical_support_reactions", {}
    )

    girder_labels = []
    for reaction_data in vertical_support_reactions.values():
        for label in reaction_data:
            if label not in girder_labels:
                girder_labels.append(label)

    def _reaction_cell(reaction_data, girder_label, side):
        """Get and format one girder's left or right support reaction."""
        value = (reaction_data or {}).get(girder_label, {}).get(side)
        return _fmt(value)

    rxn_body_parts = []
    for lc, reaction_data in vertical_support_reactions.items():
        girder_rows = []
        for girder_label in girder_labels:
            girder_rows.append(
                " & ".join([
                    _reaction_cell(reaction_data, girder_label, "left_kN"),
                    _reaction_cell(reaction_data, girder_label, "right_kN"),
                ])
            )

        if girder_rows:
            load_case_cell = (
                r"\multirow[c]{" + str(len(girder_rows)) + r"}{=}{"
                r"\parbox[c]{\linewidth}{\centering " + _tex(lc) + "}}"
            )
            rxn_body_parts.extend(
                [
                    (load_case_cell if row_idx == 0 else "") + " & " + girder_label + " & " + row + (
                        r" \\[6pt]" if row_idx == len(girder_rows) - 1 else r" \\*"
                    )
                    for row_idx, (girder_label, row) in enumerate(
                        zip(girder_labels, girder_rows)
                    )
                ]
            )
        else:
            rxn_body_parts.append(
                r"\multirow[c]{1}{=}{\parbox[c]{\linewidth}{\centering "
                + _tex(lc) + r"}} & --- & --- & --- \\[6pt]"
            )

        rxn_body_parts.append(r"\hline")

    rxn_body = "\n".join(rxn_body_parts)
    reaction_header_rows = [[
        "Load Case",
        "Girder",
        "Ra (kN)",
        "Rb (kN)",
    ]]
    reaction_widths = [4.5, 2.5, 4.5, 4.5]
    reaction_align = ["C", "C", "C", "C"]

    return r"""
\chapter{Analysis Results}

A grillage model was used for structural analysis. The deck is idealized as a grid of elastic beam elements --- longitudinal members represent the composite steel girders with effective slab, and transverse members represent the slab or cross frames. This section summarizes the critical output from that analysis.

""" + render_analysis_demands_table(merged_body) + r"""

\vspace{1em}
\noindent
""" + _fig_embed(
    fig_paths.get('girder_top_support_marked'),
    'Plan View showing support locations A and B.',
    width=r'0.85\textwidth'
) + r"""

""" + render_report_table(
    "Vertical Support Reactions", [],
    headers=[""] * 4,
    widths=reaction_widths, align=reaction_align,
    longtable=True, escape=False, body_latex=rxn_body + "\n\\hline",
    header_rows=reaction_header_rows) + r"""

""" + render_report_table(
    "Deflection Summary (Live Load & Total Load)", defl_rows,
    headers=["parameter", "value"], widths=[7.0, 8.5],
    align=["L", "L"], longtable=True, escape=False) + r"""

\vspace{1em}
\noindent
""" + _fig_embed(fig_paths.get('bm_envelope'), 'Bending Moment Envelope (Envelope ULS): Max/min BM along span. X-axis: distance from left support (m). Y-axis: Bending Moment (kN-m).', width=r'0.75\textwidth') + r"""
""" + _fig_embed(fig_paths.get('sf_envelope'), 'Shear Force Envelope (Envelope ULS): Max/min SF along span. X-axis: distance from left support (m). Y-axis: Shear Force (kN).', width=r'0.75\textwidth') + r"""
""" + _fig_embed(fig_paths.get('defl_ll'), 'Vertical Deflection D$_y$ (1.0 LL): Maximum deflection along span. Load Case: 1.0 LL, Combination: $D_y$. Nodes shown. Isometric view.', width=r'0.75\textwidth') + r"""
"""
