import os
import re
from copy import copy

import matplotlib
matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt

from osdagbridge_core.core.bridge_types.plate_girder.designer import BridgeConfig
from osdagbridge_core.core.utils.common import (
    KEY_BOQ_END_DIAPHRAGM_WEIGHT_T,
    KEY_MP_GD_MEMBER_ID,
    KEY_MP_GD_SELECT_GIRDER,
    KEY_SD_MU_APPLIED,
    KEY_SD_MD_CAPACITY,
    KEY_UTIL_FLEXURE,
    KEY_SD_SHEAR_VU,
    KEY_SD_SHEAR_VCR,
    KEY_UTIL_SHEAR,
    KEY_SD_MDV,
    KEY_UTIL_INTERACTION,
    KEY_SD_MN_RATIO,
    KEY_SD_LTB_MB,
    KEY_UTIL_LTB,
    KEY_SD_IS_IYS_MIN,
    KEY_SD_IS_IYS_PROV,
    KEY_SD_IS_FQ,
    KEY_SD_IS_FQD,
    KEY_SD_BS_R,
    KEY_SD_BS_FCDW_WB,
    KEY_SD_BS_FCDW_LC,
    KEY_SD_BS_FPSD,
    KEY_SD_BS_FCD,
    KEY_SD_STRESS_STEEL,
    KEY_SD_STRESS_STEEL_ALLOWABLE,
    KEY_DESIGN_MODE,
    KEY_SD_SECTION_PROP_IV,
    KEY_SD_SECTION_PROP_IT,
    KEY_SD_SECTION_PROP_IW,
    KEY_SD_FLANGE_SLENDERNESS,
    KEY_SD_SHEAR_AV,
    KEY_SD_HIGH_SHEAR,
    KEY_SD_MN_AXIAL,
    KEY_SD_MN_MOMENT,
    KEY_SD_STIFF_METHOD,
    KEY_SD_STIFF_INT_THICK,
    KEY_SD_STIFF_INT_SPACING,
    KEY_SD_STIFF_END_THICK,
    KEY_SD_STIFF_END_COUNT,
    KEY_MP_GIRDER_SECTIONAL_IY,
    KEY_MP_GIRDER_TORSION_CONSTANT_IT,
    KEY_MP_GIRDER_WARPING_CONSTANT_IW,
    KEY_MP_STIFFENER_LONGITUDINAL,
    KEY_TS_NO_OF_GIRDERS,
)

_GROUPED_TABLE_PROBE = False
_GROUPED_TABLE_BREAKS = set()

_GIRDER_LTB_PROPERTIES = (
    (KEY_MP_GIRDER_SECTIONAL_IY, KEY_SD_SECTION_PROP_IV, "Iy_mm4", 1e-12),
    (KEY_MP_GIRDER_TORSION_CONSTANT_IT, KEY_SD_SECTION_PROP_IT, "It_mm4", 1e-12),
    (KEY_MP_GIRDER_WARPING_CONSTANT_IW, KEY_SD_SECTION_PROP_IW, "Iw_mm6", 1e-18),
)
_MN_REPORT_KEYS = (KEY_SD_MN_AXIAL, KEY_SD_MN_MOMENT, KEY_SD_MN_RATIO)

# Check ID, name, ratio key, owned key prefixes, individually owned keys.
_GIRDER_REPORT_SECTIONS = (
    (1, None, KEY_UTIL_FLEXURE, (KEY_SD_MU_APPLIED, KEY_SD_FLANGE_SLENDERNESS, KEY_SD_SECTION_PROP_IV), ()),
    (2, "ULS Shear Buckling", KEY_UTIL_SHEAR, (KEY_SD_SHEAR_AV, KEY_SD_STIFF_METHOD, KEY_SD_IS_IYS_MIN), ()),
    (3, None, KEY_UTIL_INTERACTION, (), (KEY_SD_HIGH_SHEAR, KEY_SD_MDV)),
    (4, None, KEY_SD_MN_RATIO, (), _MN_REPORT_KEYS),
    (5, "LTB (Construction Stage)", KEY_UTIL_LTB, (KEY_SD_LTB_MB,), tuple(row[1] for row in _GIRDER_LTB_PROPERTIES)),
)
_CHECK_FALLBACK_FIELDS = {
    1: (KEY_SD_MU_APPLIED, KEY_SD_MD_CAPACITY),
    2: (KEY_SD_SHEAR_VU, KEY_SD_SHEAR_VCR),
    3: (None, KEY_SD_MDV),
    5: (None, KEY_SD_LTB_MB),
}
# Keep check-row precision and availability for the existing stiffener tables.
_STIFFENER_CHECK_FIELDS = (
    (20, "Int.Stiff: Iys \u2265 Iys_min", KEY_SD_IS_IYS_MIN, KEY_SD_IS_IYS_PROV),
    (20, "Int.Stiff: Buckling Fqd\u2265Fq", KEY_SD_IS_FQ, KEY_SD_IS_FQD),
    (21, "Brg.Stiff: Web Buckling", KEY_SD_BS_R, KEY_SD_BS_FCDW_WB),
    (21, "Brg.Stiff: Local Crushing", KEY_SD_BS_R, KEY_SD_BS_FCDW_LC),
    (21, "Brg.Stiff: Bearing Contact", KEY_SD_BS_R, KEY_SD_BS_FPSD),
    (21, "Brg.Stiff: Column Buckling", KEY_SD_BS_R, KEY_SD_BS_FCD),
)


def _governing_check(record, check_id, label=None):
    """Return the governing (load case, check), retaining older girder-summary results."""
    labels = (label, "ULS Shear") if check_id == 2 and label == "ULS Shear Buckling" else (label,)
    for name in labels:
        candidates = [(case, check) for case, data in (record.get("per_lc") or {}).items()
                      for check in data.get("checks", [])]
        candidates += [(None, check) for check in record.get("checks") or []]
        matches = [(case, check) for case, check in candidates
                   if check.get("check_id", check.get("id")) == check_id
                   and (name is None or check.get("name", check.get("label")) == name)]
        for source in ([item for item in matches if item[0] is not None],
                       [item for item in matches if item[0] is None]):
            if source:
                return max(source, key=lambda item: item[1].get("dcr") or 0)
    return None, None


def _check_values(check, fields):
    return {key: check.get(field) for key, field in zip(fields, ("demand", "capacity")) if key is not None}


def _girder_publisher_results(backend, config, record, engine):
    """Supply the existing publisher with this case's final section and calculated fields."""
    section, capacity = config.section, engine.capacity
    details = capacity.details
    classification, buckling = details["section_class"], details["buckling_resistance"]
    intermediate, bearing = details.get("intermediate_stiffener") or {}, details.get("bearing_stiffener") or {}
    results = {**backend.design_results, **vars(capacity), **vars(engine.demand), **record["section"]}
    results.update(classification, per_girder={}, capacity_details=details, category_urs=engine.category_urs(),
                   A_steel_mm2=section.A_steel, Iz_steel_mm4=section.Iz_steel,
                   Ze_steel_mm3=section.Ze_steel, Zp_steel_mm3=section.Zp_steel,
                   section_class_governing=classification["governing_class"],
                   ltb_LLT_mm=buckling["LLT_mm"], ltb_Mpl_kNm=buckling["Mpl_kNm"],
                   stiff_method=capacity.shear_method,
                   stiff_int_thick_req=intermediate.get("tq_req_1sided_mm"),
                   stiff_int_space_req=intermediate.get("c_req_min_mm"),
                   stiff_end_thick_req=bearing.get("tq_req_bearing_mm"))
    for side in ("web", "flange"):
        results[f"{side}_class_limit"] = classification[f"{side}_limit"]
        results[f"section_class_{side}"] = classification[f"{side}_class"]
    for target, source in (("is_tq_mm", "tq_mm"), ("is_c_mm", "c_mm"),
                           ("bs_tq_mm", "bs_tq_mm"), ("bs_n_plates", "bs_n_plates")):
        results[target] = getattr(config.stiffener, source, 0)
    panel = details["shear_buckling"]["c_mm"]
    results["panel_cd_ratio"] = round(panel / section.dw, 3) if panel > 0 else None
    return results


def _girder_publisher_inputs(backend, index, buckling):
    """Give the existing publisher this member's inputs, without changing bridge state."""
    inputs = dict(backend.input_dict)
    suffix, member = f".G{index + 1}.M1", f"G{index + 1}M1"
    inputs.update((key[:-len(suffix)], value) for key, value in backend.input_dict.items()
                  if isinstance(key, str) and key.endswith(suffix))
    stiffeners = (backend.input_dict.get("stiffener_by_member") or {}).get(member) or {}
    inputs["stiffener_by_member"] = {member: dict(stiffeners, longitudinal_stiffener=
        inputs.get(KEY_MP_STIFFENER_LONGITUDINAL, stiffeners.get("longitudinal_stiffener", "No")))}
    inputs.update((source, buckling[field] * scale) for source, _, field, scale in _GIRDER_LTB_PROPERTIES)
    return inputs


def _publish_girder_case(backend, index, config, record, engine):
    """Reuse the publisher, retaining the existing report's Custom-size precision."""
    publisher = copy(backend)
    publisher.input_dict = _girder_publisher_inputs(backend, index, engine.capacity.details["buckling_resistance"])
    publisher.output_dict = {}
    results = _girder_publisher_results(backend, config, record, engine)
    publisher.store_design_results(results)
    if str(publisher.input_dict.get(KEY_DESIGN_MODE, "")).strip().lower() in {"custom", "customized"}:
        for key, field in ((KEY_SD_STIFF_INT_THICK, "is_tq_mm"), (KEY_SD_STIFF_INT_SPACING, "is_c_mm"),
                           (KEY_SD_STIFF_END_THICK, "bs_tq_mm"), (KEY_SD_STIFF_END_COUNT, "bs_n_plates")):
            publisher.output_dict[key] = results[field]
    intermediate = engine.capacity.details.get("intermediate_stiffener") or {}
    if not intermediate or intermediate.get("skipped") or intermediate.get("design_guidance"):
        for key in (KEY_SD_IS_IYS_MIN, KEY_SD_IS_IYS_PROV, KEY_SD_IS_FQ, KEY_SD_IS_FQD):
            publisher.output_dict.pop(key, None)
    return publisher.output_dict


def _report_group_values(published, anchors, exact_keys):
    """Copy only fields owned by this check, keeping M-V and M-N separate."""
    prefixes = tuple(key.rsplit(".", 1)[0] + "." for key in anchors)
    return {key: value for key, value in published.items() if key in exact_keys or key.startswith(prefixes)}


def _stored_stiffener_values(record):
    """Preserve stored stiffener precision and the existing stress-check fallback."""
    values = {}
    for check_id, label, demand_key, capacity_key in _STIFFENER_CHECK_FIELDS:
        _, check = _governing_check(record, check_id, label)
        if check is not None:
            values.update(_check_values(check, (demand_key, capacity_key)))
    _, stress = _governing_check(record, 11)
    values["design_results"] = dict(zip((KEY_SD_STRESS_STEEL, KEY_SD_STRESS_STEEL_ALLOWABLE),
                                       ((stress or {}).get("demand"), (stress or {}).get("capacity"))))
    return values


def publish_girder_outputs(backend):
    """Publish one authoritative report dictionary per girder, using cached selected cases."""
    outputs = {}
    for index, (name, record) in enumerate(backend.design_results.get("per_girder", {}).items()):
        config = BridgeConfig.from_plate_girder_bridge(backend, girder_index=index)
        values, published_cases = {}, {}
        for check_id, label, ratio_key, anchors, exact_keys in _GIRDER_REPORT_SECTIONS:
            case, check = _governing_check(record, check_id, label)
            if check is None:
                if check_id == 4 and record.get("per_lc") and all(
                        data.get("Nu_kN") == 0 for data in record["per_lc"].values()):
                    values.update(dict.fromkeys(_MN_REPORT_KEYS))
                continue
            if case is not None:
                if case not in published_cases:
                    engine = backend.get_dcr_engine_for_selection(name, case)
                    if engine is None or engine.capacity is None:
                        raise ValueError(f"Cannot publish report details for {name}, {case}.")
                    published_cases[case] = _publish_girder_case(backend, index, config, record, engine)
                values.update(_report_group_values(published_cases[case], anchors, exact_keys))
            else:
                fields = _CHECK_FALLBACK_FIELDS.get(check_id, (None, None))
                if check_id == 2 and check.get("name", check.get("label")) != "ULS Shear Buckling":
                    fields = (fields[0], None)  # Plastic shear resistance is not Vcr.
                values.update(_check_values(check, fields))
            ratio = check.get("dcr")
            values[ratio_key] = ratio * (1 if check_id == 4 else 100) if ratio is not None else None
            status = check.get("status")
            if status:
                values[ratio_key + ".status"] = r"\textcolor{red}{FAIL}" if status == "FAIL" else status
        values.update(_stored_stiffener_values(record))
        outputs[name] = values
    backend.output_dict["girder_outputs"] = outputs


def iter_girder_outputs(output_dict, girder_entries):
    """Read published girder outputs; do not select or recompute report values here."""
    for index, (label, _) in enumerate(girder_entries, start=1):
        name, suffix = f"G{index}", f".G{index}.M1"
        values = {key[:-len(suffix)]: value for key, value in output_dict.items()
                  if isinstance(key, str) and key.endswith(suffix)}
        values.update(output_dict.get("girder_outputs", {}).get(name) or {})
        yield label, dict(values, _report_girder=name)


def configure_grouped_table_breaks(probe=False, breaks=None):
    global _GROUPED_TABLE_PROBE, _GROUPED_TABLE_BREAKS
    _GROUPED_TABLE_PROBE = probe
    _GROUPED_TABLE_BREAKS = set(breaks or [])


def grouped_table_key(caption):
    return "".join(ch.lower() if ch.isalnum() else "_" for ch in str(caption)).strip("_")[:80]


def _tex(value):
    """Escape a Python value for safe LaTeX embedding."""
    s = str(value) if value is not None else ''
    if not s:
        return r''
    # Normalise non-ASCII glyphs from section designations (e.g. "∠ 100 ⅹ 100ⅹ 10")
    # that pdflatex cannot render.
    for uni, ascii_ in [('∠', 'L'), ('ⅹ', 'x'), ('×', 'x')]:
        s = s.replace(uni, ascii_)
    s = s.replace('\\', r'\textbackslash{}')
    for ch, esc in [('&', r'\&'), ('%', r'\%'), ('$', r'\$'), ('#', r'\#'),
                    ('{', r'\{'), ('}', r'\}'),          
                    ('_', r'\_\allowbreak{}'),            
                    ('~', r'\textasciitilde{}'), ('^', r'\^{}')]:
        s = s.replace(ch, esc)
    s = s.replace(':', r':\allowbreak{}')
    return s


def _render_value(source_dict, key, unit=""):
    val = source_dict.get(key)
    if val in ("", None):
        return "Not available" if source_dict.get("_report_girder") else ""
    if isinstance(val, float):
        val = f"{val:.2f}".rstrip("0").rstrip(".")
    return _tex(val) + unit


def _render_mm(source_dict, key):
    val = source_dict.get(key)
    if val in ("", None):
        return ""
    return _render_value({key: float(val) * 1000}, key)


def _table_layout(headers, rows, widths=None, align=None):
    """Column count, scaled column widths (cm) and alignment shared by the table renderers."""
    headers = headers or []
    ncols = max(1, len(headers) or max((len(row) for row in rows), default=1))
    max_content_width = 15.5 - (0.43 * ncols) - (0.02 * (ncols + 1))
    if widths is None:
        def _score(cell):
            text = str(cell or "").replace("\\allowbreak{}", "")
            for ch in "\\{}$_^":
                text = text.replace(ch, "")
            longest = max((len(part) for part in text.split()), default=0)
            return max(1.0, min(18.0, longest * 0.65 + len(text) * 0.12))
        widths = [
            max(_score(headers[i] if i < len(headers) else ""),
                max((_score(row[i]) for row in rows if i < len(row)), default=1.0))
            for i in range(ncols)
        ]
        min_width = min(2.0, max(1.2, max_content_width / ncols * 0.65))
        remaining = max_content_width - (min_width * ncols)
        widths = ([min_width + remaining * w / sum(widths) for w in widths]
                  if remaining > 0 and sum(widths) > 0 else [max_content_width / ncols] * ncols)
    else:
        widths = list(widths)
        if len(widths) < ncols:
            widths.extend([widths[-1] if widths else round(14.0 / ncols, 2)] * (ncols - len(widths)))
        widths = widths[:ncols]
    scale = max_content_width / sum(widths) if sum(widths) > 0 else 1
    widths = [round(w * scale, 2) for w in widths]
    
    if align is None:
        align = ["L"] * ncols
    else:
        align = list(align)
        if len(align) < ncols:
            align.extend([align[-1] if align else "L"] * (ncols - len(align)))
        align = align[:ncols]

    return ncols, widths, align


def render_report_table(caption, rows, headers=None, widths=None, align=None, longtable=False, escape=True, header_rows=None, header_clines=None, body_latex=None):
    headers = headers or []
    ncols, widths, align = _table_layout(headers, rows, widths, align)

    def _header(cell):
        text = str(cell or "")
        text = text[:1].upper() + text[1:] if text else text
        return r"\textbf{" + (_tex(text) if escape else text) + "}"
    colspec = "|" + "|".join(f"{a}{{{w}cm}}" for a, w in zip(align, widths)) + "|"
    header = ""
    if header_rows:
        rendered = []
        header_clines = header_clines or {}
        for row_idx, row in enumerate(header_rows):
            cells = []
            for cell in row:
                if isinstance(cell, tuple):
                    text, span = cell
                    cells.append(r"\multicolumn{" + str(span) + r"}{c|}{\textbf{" + (_tex(text) if escape else text) + "}}")
                else:
                    cells.append(_header(cell) if cell else "")
            rendered.append(" & ".join(cells) + r" \\")
            if row_idx in header_clines:
                rendered.append(r"\cline{" + header_clines[row_idx] + "}")
        header = "\\hline\n" + "\n".join(rendered) + "\n\\hline\n"
    elif headers:
        header_row = " & ".join(_header(h) for h in headers) + r" \\"
        header = "\\hline\n" + header_row + "\n\\hline\n"
    fmt = _tex if escape else str
    body = body_latex if body_latex is not None else "\n".join(" & ".join(fmt(cell) for cell in row) + r" \\" + "\n\\hline" for row in rows)

    if longtable:
        repeat = (header + "\\endfirsthead\n" + header + "\\endhead\n") if header else "\\hline\n"
        return ("\\begin{longtable}{" + colspec + "}\n"
                + "\\captionsetup{justification=centering,font={small,it}}\n\\caption{"
                + _tex(caption) + "}\\\\\n" + repeat + body
                + "\n\\end{longtable}")

    return ("\\begin{table}[H]\n\\centering\n\\captionsetup{justification=centering,font={small,it}}\n\\caption{"
            + _tex(caption) + "}\n\\begin{tabular}{" + colspec
            + "}\n" + (header or "\\hline\n") + body
            + "\n\\end{tabular}\n\\end{table}")


def render_vehicle_live_load_table(rows):
    return render_report_table(
        "Vehicle Live Loads (LL)", rows,
        header_rows=[
            ["Vehicle", "Total Load (kN)", "Impact Factor", ("Braking Load", 3)],
            ["", "", "", "Considered?", "Value", "Eccentricity"],
        ],
        header_clines={0: "4-6"},
        widths=[2.7, 2.0, 1.9, 2.3, 1.7, 2.2],
        align=["L", "C", "C", "C", "C", "C"], longtable=True, escape=False)


def render_analysis_demands_table(body_latex):
    header = r"""\hline
\multirow{2}{*}{\makecell{\textbf{Load}\\\textbf{Case/}\\\textbf{Comb.}}}
& \multicolumn{3}{c|}{\textbf{Bending Moment}}
& \multicolumn{3}{c|}{\textbf{Shear Force}}
& \multicolumn{2}{c|}{\textbf{Reaction at Supports}}\\
\cline{2-9}
& \makecell{\textbf{Max}\\\textbf{(kNm)}}
& \textbf{Girder}
& \makecell{\textbf{Loc.}\\\textbf{(m)}}
& \makecell{\textbf{Max}\\\textbf{(kN)}}
& \textbf{Girder}
& \makecell{\textbf{Loc.}\\\textbf{(m)}}
& \makecell{\textbf{Left}}
& \makecell{\textbf{Right}}\\
\hline
"""
    return (r"""\vspace{1em}
\begingroup
\footnotesize
\setlength{\tabcolsep}{3pt}
\renewcommand{\arraystretch}{1.25}
\setlength{\LTleft}{\dimexpr(\textwidth-17.8cm)/2\relax}
\setlength{\LTright}{\dimexpr(\textwidth-17.8cm)/2\relax}
\begin{longtable}{|
>{\centering\arraybackslash}p{3.1cm}|
>{\centering\arraybackslash}p{1.7cm}|
>{\centering\arraybackslash}p{1.2cm}|
>{\centering\arraybackslash}p{1.3cm}|
>{\centering\arraybackslash}p{1.7cm}|
>{\centering\arraybackslash}p{1.2cm}|
>{\centering\arraybackslash}p{1.3cm}|
>{\centering\arraybackslash}p{1.6cm}|
>{\centering\arraybackslash}p{1.6cm}|}
\captionsetup{justification=centering,font={small,it}}
\caption{Summary of Maximum Demands}\\
""" + header + r"""\endfirsthead
""" + header + r"""\endhead
""" + body_latex + r"""
\hline
\end{longtable}
\endgroup
""")


def render_parameter_value_table(caption, rows, longtable=False):
    return render_report_table(
        caption, rows, headers=["parameter", "value"], widths=[6.4, 8.6],
        align=["L", "L"], longtable=longtable, escape=False)


def render_grouped_report_table(caption, groups, headers=None, widths=None, align=None, escape=True,
                                longtable_min_rows=19):
    headers = headers or []
    groups = [(label, [list(row) for row in group_rows]) for label, group_rows in groups]
    rows = [[label] + row for label, group_rows in groups for row in group_rows]
    ncols, widths, align = _table_layout(headers, rows, widths, align)

    def _header(cell, width):
        text = str(cell or "")
        text = text[:1].upper() + text[1:] if text else text
        text = _tex(text) if escape else text
        return (r"\parbox[c][2.8\baselineskip][c]{" + str(width)
                + r"cm}{\centering\textbf{" + text + "}}")
    fmt = _tex if escape else str
    colspec = "|" + "|".join(f"{a}{{{w}cm}}" for a, w in zip(align, widths)) + "|"
    header = "\\hline\n" + " & ".join(_header(h, w) for h, w in zip(headers, widths)) + r" \\" + "\n\\hline\n"

    table_key = grouped_table_key(caption)

    def _row_marker(group_idx, row_idx, edge):
        return r"\noalign{\label{osdaggrp:" + table_key + ":" + str(group_idx) + ":" + str(row_idx) + ":" + edge + r"}}"

    def _group_tex(label, group_rows, group_idx=0, row_offset=0):
        out = []
        for idx, row in enumerate(group_rows):
            if _GROUPED_TABLE_PROBE:
                out.append(_row_marker(group_idx, row_offset + idx, "s"))
                first = fmt(label)
                line_end = r" \\"
            else:
                first = (r"\multirow{" + str(len(group_rows)) + "}{" + str(widths[0])
                         + r"cm}{\centering " + fmt(label) + "}") if idx == 0 else ""
                line_end = r" \\" if idx == len(group_rows) - 1 else r" \\*"
            line = " & ".join([first] + [fmt(cell) for cell in row]) + line_end
            out.append(line)
            if _GROUPED_TABLE_PROBE:
                out.append(_row_marker(group_idx, row_offset + idx, "e"))
            out.append(r"\hline" if _GROUPED_TABLE_PROBE or idx == len(group_rows) - 1 else f"\\cline{{2-{ncols}}}")
        return "\n".join(out)

    def _break_indexes(table_key, group_idx, group_rows):
        prefix = table_key + ":" + str(group_idx) + ":"
        indexes = sorted(
            int(marker[len(prefix):]) for marker in _GROUPED_TABLE_BREAKS
            if marker.startswith(prefix) and marker[len(prefix):].isdigit()
        )
        return [idx for idx in indexes if 0 < idx < len(group_rows)]

    def _visual_rows(cells):
        row_height = 1
        for cell, width in zip(cells, widths):
            text = str(cell or "").replace("\\allowbreak{}", "")
            for ch in "\\{}$_^":
                text = text.replace(ch, "")
            chars_per_line = max(8, int(width * 4.5))
            row_height = max(row_height, (len(text) + chars_per_line - 1) // chars_per_line)
        return row_height

    def _group_cost(label, group_rows):
        return sum(_visual_rows([label] + row) for row in group_rows)

    estimated_rows = sum(
        _visual_rows([label] + row) for label, group_rows in groups for row in group_rows
    )
    body_parts = []
    for group_idx, (label, group_rows) in enumerate(groups):
        breaks = _break_indexes(table_key, group_idx, group_rows)
        starts = [0] + breaks
        ends = breaks + [len(group_rows)]
        for chunk_idx, (start, end) in enumerate(zip(starts, ends)):
            chunk_rows = group_rows[start:end]
            if not _GROUPED_TABLE_PROBE:
                if chunk_idx > 0:
                    body_parts.append(r"\noalign{\penalty -10000}")
                else:
                    if group_idx == 0 and chunk_rows:
                        need = max(4, min(8, _visual_rows([label] + chunk_rows[0]) + 3))
                    else:
                        need = max(4, min(30, _group_cost(label, chunk_rows) + 2))
                    body_parts.append(r"\noalign{\needspace{" + str(need) + r"\baselineskip}}")
            body_parts.append(_group_tex(label, chunk_rows, group_idx, start))
    body = "\n".join(body_parts)
    caption_setup = r"\captionsetup{justification=centering,font={small,it}}"
    total_rows = sum(len(group_rows) for _, group_rows in groups)
    if total_rows < longtable_min_rows and estimated_rows < longtable_min_rows:
        need = max(6, min(34, estimated_rows + 5))
        return (r"\Needspace{" + str(need) + r"\baselineskip}" + "\n"
                + r"\begin{table}[H]" + "\n"
                + r"\centering" + "\n"
                + caption_setup + "\n" + r"\caption{" + _tex(caption) + "}\n"
                + r"\begin{tabular}{" + colspec + "}\n"
                + header + body + "\n"
                + r"\end{tabular}" + "\n"
                + r"\end{table}")

    start_label = (r"\label{osdaggrpstart:" + table_key + r"}" if _GROUPED_TABLE_PROBE else "")
    first_head = caption_setup + "\n" + r"\caption{" + _tex(caption) + r"}" + start_label + r"\\" + "\n" + header
    cont_head = (r"\multicolumn{" + str(ncols) + r"}{l}{\small\itshape Continued from previous page}\\"
                 + "\n" + header)
    cont_foot = (r"\multicolumn{" + str(ncols)
                 + r"}{r}{\small\itshape Continued on next page...}\\" + "\n")
    first_row_need = (_visual_rows([groups[0][0]] + groups[0][1][0]) if groups and groups[0][1] else 1)
    first_chunk_need = max(4, min(8, first_row_need + 3))
    start_guard = (r"\newpage" if (table_key + ":__start__") in _GROUPED_TABLE_BREAKS
                   else r"\Needspace{" + str(first_chunk_need) + r"\baselineskip}")
    return (start_guard + "\n"
            + r"\begin{longtable}{" + colspec + "}\n"
            + first_head + r"\endfirsthead" + "\n"
            + cont_head + r"\endhead" + "\n"
            + cont_foot + r"\endfoot" + "\n"
            + r"\hline" + "\n"
            + r"\endlastfoot" + "\n"
            + body + "\n"
            + r"\end{longtable}")


def get_girder_entries(input_dict):
    """
    Retrieve all girder labels and member IDs from backend keys.

    Usage Example:
    --------------------------
    girder_entries = get_girder_entries(bridge.input_dict)
    
    # 1. Fallback handling (if backend hasn't populated keys yet)
    if not girder_entries:
        n = int(bridge.input_dict.get(KEY_TS_NO_OF_GIRDERS, 1))
        girder_entries = [(f"Girder {i}", f"M1") for i in range(1, n + 1)]
        
    # 2. Get total number of girders safely
    n_girders = len(girder_entries)
    
    # 3. Iterate over the girders to build table rows
    for lbl, mid in girder_entries:
        # lbl will be e.g., "G1", mid will be e.g., "G1M1"
        # Access girder specific keys dynamically:
        # val = input_dict.get(f"{KEY_MP_GIRDER_DEPTH}.{lbl}.{mid}")
        pass

    Returns:
        List[Tuple[str, str]]
    """
    n = int(input_dict.get(KEY_TS_NO_OF_GIRDERS, 0))

    entries = []

    for i in range(1, n + 1):
        label = input_dict.get(f"{KEY_MP_GD_SELECT_GIRDER}.G{i}") or f"G{i}"
        entries.append(
            (
                label,
                input_dict.get(f"{KEY_MP_GD_MEMBER_ID}.G{i}.M1", ""),
            )
        )

    return entries


def _fig_or_placeholder(path, caption, width=r'0.9\textwidth'):
    """Embed figure if path is provided (file already copied to assets), else show placeholder box.
    path is the relative path as pdflatex will see it (e.g. 'assets/plan.png').
    """
    if path:
        p = path.replace('\\', '/')
        return (r'\begin{figure}[H]' + '\n'
                r'\centering' + '\n'
                r'\includegraphics[width=' + width + ']{' + p + '}\n'
                r'\caption*{' + caption + '}\n'
                r'\end{figure}')
    return (r'\begin{figure}[H]' + '\n'
            r'\centering' + '\n'
            r'\fbox{\parbox{0.97\textwidth}{' + '\n'
            r'\textit{[ PLACEHOLDER: ' + caption + r' ]}' + '\n'
            r'}}' + '\n'
            r'\caption*{' + caption + '}\n'
            r'\end{figure}')

def _fig_embed(path, caption, width=r'\textwidth', height=None, numbered=False):
    """Embed a real figure when path is provided (already copied); otherwise use an fbox placeholder."""
    if path:
        p = path.replace('\\', '/')
        opts = 'width=' + width
        if height:
            opts += ',height=' + height + ',keepaspectratio'
        cap_cmd = r'\caption' if numbered else r'\caption*'
        return (r'\begin{figure}[H]' + '\n'
                r'\vspace{-0.5em}' + '\n'
                r'\centering' + '\n'
                r'\includegraphics[' + opts + ']{' + p + '}\n'
                r'\vspace{0.5em}' + '\n'
                + cap_cmd + r'{\small ' + caption + '}\n'
                r'\vspace{-0.5em}' + '\n'
                r'\end{figure}')
    # fbox placeholder — matches template exactly
    return (r'\noindent\fbox{\parbox{0.97\textwidth}{' + '\n'
            r'\textit{[ PLACEHOLDER: ' + caption + r' ]}' + '\n'
            r'}}')


class ReportChartGenerator:
    """Generate temporary vector charts for report figures."""

    def __init__(self, output_dir, latex_dir="assets"):
        self.output_dir = output_dir
        self.latex_dir = latex_dir

    def _number(self, value):
        
        text = str(value or "").strip()
        if not text or text.upper() in {"N.A.", "NA", "N/A", "---"}:
            return 0.0
        text = text.replace(",", "")
        # Read LaTeX scientific notation correctly for very small BOQ values.
        sci = re.search(r"(-?\d+(?:\.\d+)?)\s*\\times\s*10\^\{(-?\d+)\}", text)
        if sci:
            return float(sci.group(1)) * (10.0 ** int(sci.group(2)))
        match = re.search(r"-?\d+(?:\.\d+)?", text)
        return float(match.group(0)) if match else 0.0

    def _first_number(self, quantities, keys):
        for key in keys:
            value = self._number(quantities.get(key))
            if value:
                return value
        return 0.0

    def _save_bar_chart(self, filename, labels, values, xlabel, ylabel, title, colors):
       
        os.makedirs(self.output_dir, exist_ok=True)
        os.environ.setdefault("MPLCONFIGDIR", os.path.join(self.output_dir, "mplconfig"))
        os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)
        
        matplotlib.use("Agg", force=True)
        matplotlib.rcParams["pdf.fonttype"] = 42
        matplotlib.rcParams["ps.fonttype"] = 42
        

        fig, ax = plt.subplots(figsize=(7.0, 3.8), dpi=220)
        bars = ax.bar(labels, values, color=colors, width=0.55)
        ax.set_title(title, fontsize=12, fontweight="bold", pad=10)
        ax.set_xlabel(xlabel, fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.35)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ymax = max(values) if values else 0.0
        ax.set_ylim(0, ymax * 1.18 if ymax > 0 else 1)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    f"{value:.2f}", ha="center", va="bottom", fontsize=9)
        fig.tight_layout()
        path = os.path.join(self.output_dir, filename)
        fig.savefig(path, format="pdf", bbox_inches="tight")
        plt.close(fig)
        return (self.latex_dir.rstrip("/\\") + "/" + filename).replace("\\", "/")

    def _save_ur_chart(self, filename, labels, values):
        
        os.makedirs(self.output_dir, exist_ok=True)
        os.environ.setdefault("MPLCONFIGDIR", os.path.join(self.output_dir, "mplconfig"))
        os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)
        
        matplotlib.use("Agg", force=True)
        matplotlib.rcParams["pdf.fonttype"] = 42
        matplotlib.rcParams["ps.fonttype"] = 42
        

        plot_values = [v if v is not None else 0.0 for v in values]
        colors = ["#b55353" if (v is not None and v > 1.0) else "#6aa84f" if v is not None else "#9e9e9e" for v in values]
        fig, ax = plt.subplots(figsize=(7.2, 3.8), dpi=220)
        bars = ax.bar(labels, plot_values, color=colors, width=0.55)
        ax.axhline(1.0, color="#c00000", linestyle="--", linewidth=1.3, label="UR = 1.0")
        ax.set_title("Utilization Ratio Summary", fontsize=12, fontweight="bold", pad=10)
        ax.set_xlabel("Primary Element", fontsize=10)
        ax.set_ylabel("Utilization Ratio (Demand / Capacity)", fontsize=10)
        ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.35)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.legend(loc="upper right", frameon=False, fontsize=9)
        ax.set_ylim(0, max(plot_values + [1.0]) * 1.22)
        for bar, value in zip(bars, values):
            label = "N/A" if value is None else f"{value:.2f}"
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    label, ha="center", va="bottom", fontsize=9)
        fig.tight_layout()
        path = os.path.join(self.output_dir, filename)
        fig.savefig(path, format="pdf", bbox_inches="tight")
        plt.close(fig)
        return (self.latex_dir.rstrip("/\\") + "/" + filename).replace("\\", "/")

    def _max_dcr(self, data):
        best = None
        if isinstance(data, dict):
            for key, value in data.items():
                if str(key).lower() in {"dcr", "ur", "efficiency", "utilization", "utilization_ratio"}:
                    n = self._number(value)
                    if n > 0:
                        best = n if best is None else max(best, n)
                child = self._max_dcr(value)
                if child is not None:
                    best = child if best is None else max(best, child)
        elif isinstance(data, (list, tuple)):
            for item in data:
                child = self._max_dcr(item)
                if child is not None:
                    best = child if best is None else max(best, child)
        return best

    def generate_utilization_ratio_charts(self, bridge):
        try:
            from osdagbridge_core.core.reports.executive_summary import _governing_member

            output = bridge.output_dict or {}
            steel = self._max_dcr((output.get("design_results", {}) or {}).get("per_girder", {}))
            deck_rpt = output.get("deck_report_values", {}) or {}
            deck_values = []
            for demand_key, capacity_key in (
                ("deck.report.m_uls_sag", "deck.report.mu_bot"),
                ("deck.report.m_uls_hog", "deck.report.mu_top"),
                ("deck.report.punch_ved", "deck.report.vrd_c_mpa"),
                ("deck.report.shear_ved", "deck.report.shear_vrdc"),
                ("deck.report.wk_bot", "deck.report.wk_limit"),
                ("deck.report.wk_top", "deck.report.wk_limit"),
            ):
                demand = self._number(deck_rpt.get(demand_key))
                capacity = self._number(deck_rpt.get(capacity_key))
                if capacity > 0:
                    deck_values.append(demand / capacity)
            deck = max(deck_values) if deck_values else self._max_dcr(output.get("deck_design_results", {}))
            cb_values = []
            for pair in bridge.get_cb_pairs():
                for member in ("diagonal", "chord"):
                    for force_type in ("compression", "tension"):
                        n = self._number(bridge.get_cb_efficiency(pair, member, force_type))
                        if n > 0:
                            cb_values.append(n)
            cross_bracing = max(cb_values) if cb_values else self._max_dcr(output.get("crossbracing_design_results", {}))
            end_diaphragm, _ = _governing_member(
                output.get("end_diaphragm_design_results", {})
            )
            return {"ur_summary": self._save_ur_chart(
                "utilization_ratio_summary.pdf",
                ["Steel Plate\nGirders", "Concrete\nDeck Slab", "Cross\nBracing", "End\nDiaphragms"],
                [steel, deck, cross_bracing, end_diaphragm])}
        except Exception:
            return {}

    def generate_material_quantity_charts(self, quantities):
        try:
            bracing = sum(self._number(quantities.get(key)) for key in (
                "bracing_top_wt_total", "bracing_bot_wt_total", "bracing_diag_wt_total"
            ))
            diaphragm = self._number(quantities.get(KEY_BOQ_END_DIAPHRAGM_WEIGHT_T))
            return {
                "steel": self._save_bar_chart(
                    "material_steel_quantities.pdf",
                    ["Girders", "Cross Bracing", "End Diaphragms"],
                    [self._number(quantities.get("steel_girders_wt_total")), bracing, diaphragm],
                    "Structural Steel Component", "Weight (t)", "Structural Steel Tonnage",
                    ["#3b6ea8", "#6aa84f", "#c27c3a"]),
                "concrete_rebar": self._save_bar_chart(
                    "material_concrete_rebar_quantities.pdf",
                    ["Concrete Volume", "Reinforcement Steel"],
                    [self._number(quantities.get("concrete_deck_vol_total")),
                     self._number(quantities.get("rebar_deck_wt_total"))],
                    "Material Quantity Type", r"Quantity (m$^3$ / t)", "Concrete Volume and Reinforcement Steel",
                    ["#7a9cc6", "#b55353"]),
            }
        except Exception:
            return {}


