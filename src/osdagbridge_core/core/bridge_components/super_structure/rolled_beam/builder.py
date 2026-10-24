import math
import numpy as np
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Trsf, gp_Ax3, gp_Dir, gp_Ax2, gp_Ax1
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeCylinder
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_Transform
)
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from osdagbridge_core.core.utils.common import (
    KEY_MP_GIRDER_DEPTH,
    KEY_MP_GIRDER_TOP_FLANGE_WIDTH,
    KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH,
    KEY_MP_GIRDER_TOP_FLANGE_THICKNESS,
    KEY_MP_GIRDER_BOTTOM_FLANGE_THICKNESS,
    KEY_MP_GIRDER_WEB_THICKNESS,
)

ROLLED_END_STIFFENER_SPACING = 50.0


def _make_rolled_edge(p1, p2):
    return BRepBuilderAPI_MakeEdge(p1, p2).Edge()


def _make_rolled_wire_from_points(points):
    wire_maker = BRepBuilderAPI_MakeWire()
    for i in range(len(points)):
        wire_maker.Add(_make_rolled_edge(points[i], points[(i + 1) % len(points)]))
    return wire_maker.Wire()


def _make_rolled_plate(origin, length, width, thickness, u_dir, w_dir):
    v_dir = np.cross(w_dir, u_dir)
    a1 = origin + (thickness / 2.0) * u_dir + (length / 2.0) * v_dir
    a2 = origin - (thickness / 2.0) * u_dir + (length / 2.0) * v_dir
    a3 = origin - (thickness / 2.0) * u_dir - (length / 2.0) * v_dir
    a4 = origin + (thickness / 2.0) * u_dir - (length / 2.0) * v_dir
    pts = [gp_Pnt(*a1), gp_Pnt(*a2), gp_Pnt(*a3), gp_Pnt(*a4)]
    wire = _make_rolled_wire_from_points(pts)
    face = BRepBuilderAPI_MakeFace(wire).Face()
    extrude_vec = gp_Vec(*(width * w_dir))
    return BRepPrimAPI_MakePrism(face, extrude_vec).Shape()


def _rotate_rolled_about_z(shape, angle_deg):
    trsf = gp_Trsf()
    trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.radians(angle_deg))
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def _create_rolled_stiffener_plate(position, width, height, thickness, chamfer, side):
    x, y, z = map(float, position)
    c = chamfer
    if side == "right":
        pts = [
            gp_Pnt(0, 0, height / 2.0 - c),
            gp_Pnt(c, 0, height / 2.0),
            gp_Pnt(width, 0, height / 2.0),
            gp_Pnt(width, 0, -height / 2.0),
            gp_Pnt(c, 0, -height / 2.0),
            gp_Pnt(0, 0, -height / 2.0 + c),
        ]
    else:
        pts = [
            gp_Pnt(0, 0, height / 2.0 - c),
            gp_Pnt(0, 0, -height / 2.0 + c),
            gp_Pnt(-c, 0, -height / 2.0),
            gp_Pnt(-width, 0, -height / 2.0),
            gp_Pnt(-width, 0, height / 2.0),
            gp_Pnt(-c, 0, height / 2.0),
        ]
    wire = BRepBuilderAPI_MakeWire()
    for i in range(len(pts)):
        wire.Add(BRepBuilderAPI_MakeEdge(pts[i], pts[(i + 1) % len(pts)]).Edge())
    face = BRepBuilderAPI_MakeFace(wire.Wire()).Face()
    solid = BRepPrimAPI_MakePrism(face, gp_Vec(0, thickness, 0)).Shape()
    local_ax = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(0, 1, 0))
    global_ax = gp_Ax3(gp_Pnt(x, y - thickness / 2.0, z), gp_Dir(0, 0, 1), gp_Dir(0, 1, 0))
    trsf = gp_Trsf()
    trsf.SetDisplacement(local_ax, global_ax)
    return BRepBuilderAPI_Transform(solid, trsf, True).Shape()


def create_rolled_shear_stud(base_dia, base_height, top_dia, top_height):
    base_radius = base_dia / 2.0
    top_radius = top_dia / 2.0
    base_axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    base_cyl = BRepPrimAPI_MakeCylinder(base_axis, base_radius, base_height).Shape()
    top_axis = gp_Ax2(gp_Pnt(0, 0, base_height), gp_Dir(0, 0, 1))
    top_cyl = BRepPrimAPI_MakeCylinder(top_axis, top_radius, top_height).Shape()
    return BRepAlgoAPI_Fuse(base_cyl, top_cyl).Shape()


def _make_rolled_i_section_solid(bf, bf_b, tf, tf_b, tw, depth, z_top, length):
    b_t = bf / 2.0
    b_b = bf_b / 2.0
    w = tw / 2.0
    z_top_in = z_top - tf
    z_bot = z_top - depth
    z_bot_in = z_bot + tf_b

    pts = [
        gp_Pnt(-b_t, 0.0, z_top),
        gp_Pnt(b_t, 0.0, z_top),
        gp_Pnt(b_t, 0.0, z_top_in),
        gp_Pnt(w, 0.0, z_top_in),
        gp_Pnt(w, 0.0, z_bot_in),
        gp_Pnt(b_b, 0.0, z_bot_in),
        gp_Pnt(b_b, 0.0, z_bot),
        gp_Pnt(-b_b, 0.0, z_bot),
        gp_Pnt(-b_b, 0.0, z_bot_in),
        gp_Pnt(-w, 0.0, z_bot_in),
        gp_Pnt(-w, 0.0, z_top_in),
        gp_Pnt(-b_t, 0.0, z_top_in),
    ]
    wire = _make_rolled_wire_from_points(pts)
    face = BRepBuilderAPI_MakeFace(wire).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0, length, 0)).Shape()


def build_rolled_girder_geometry(
    *,
    section_dims=None,
    length,
    reference_top_z=None,
    include_intermediate_stiffeners=True,
    intermediate_stiffener_spacing=750,
    intermediate_stiffener_thickness=20,
    chamfer_length=40,
    num_end_stiffener_pairs=2,
    T_es=25,
    intermediate_stiffener_outstand=None,
    end_stiffener_outstand=None,
    include_longitudinal_stiffeners=False,
    num_longitudinal_stiffeners=1,
    longitudinal_stiffener_thickness=20,
    longitudinal_stiffener_outstand=None,
    shear_stud_base_diameter=16,
    shear_stud_top_diameter=25,
    shear_stud_base_height=135,
    shear_stud_top_height=15,
    num_shear_studs_per_section=2,
    shear_stud_transverse_spacing=100,
    shear_stud_pitch=200,
    right_guided=False,
    **kwargs
):
    dims = section_dims or {}
    depth = float(dims.get(KEY_MP_GIRDER_DEPTH, kwargs.get("depth", kwargs.get("D", 0.0))))
    tf = float(dims.get(KEY_MP_GIRDER_TOP_FLANGE_THICKNESS, kwargs.get("tf", kwargs.get("T_ft", 0.0))))
    tf_b = float(dims.get(KEY_MP_GIRDER_BOTTOM_FLANGE_THICKNESS, kwargs.get("tf_b", kwargs.get("T_fb", tf))))
    bf = float(dims.get(KEY_MP_GIRDER_TOP_FLANGE_WIDTH, kwargs.get("bf", kwargs.get("B_ft", 0.0))))
    bf_b = float(dims.get(KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH, kwargs.get("bf_b", kwargs.get("B_fb", bf))))
    tw = float(dims.get(KEY_MP_GIRDER_WEB_THICKNESS, kwargs.get("tw", 0.0)))

    Z_top = reference_top_z if reference_top_z is not None else (depth / 2.0 + tf)
    web_depth = depth - tf - tf_b
    z_web_center = Z_top - tf - web_depth / 2.0

    web_shapes = [_make_rolled_i_section_solid(bf, bf_b, tf, tf_b, tw, depth, Z_top, length)]

    intermediate_stiffeners = []
    bearing_stiffeners = []
    longitudinal_stiffeners = []

    default_stiff_width = (min(bf, bf_b) - tw) / 2.0
    int_stiff_width = intermediate_stiffener_outstand if intermediate_stiffener_outstand is not None else default_stiff_width
    end_stiff_width = end_stiffener_outstand if end_stiffener_outstand is not None else default_stiff_width
    chamfer = min(chamfer_length, web_depth / 4.0)

    if include_intermediate_stiffeners:
        num_panels = max(1, int(length // intermediate_stiffener_spacing))
        end_stiffener_gap = T_es / 2.0
        end_stiffener_zone = end_stiffener_gap + (num_end_stiffener_pairs - 1) * ROLLED_END_STIFFENER_SPACING + ROLLED_END_STIFFENER_SPACING
        start_y = end_stiffener_zone
        end_y = length - end_stiffener_zone

        for i in range(1, num_panels):
            y = i * intermediate_stiffener_spacing
            if y <= start_y or y >= end_y:
                continue
            intermediate_stiffeners.append(
                _create_rolled_stiffener_plate([tw / 2.0, y, z_web_center], int_stiff_width, web_depth, intermediate_stiffener_thickness, chamfer, "right")
            )
            intermediate_stiffeners.append(
                _create_rolled_stiffener_plate([-tw / 2.0, y, z_web_center], int_stiff_width, web_depth, intermediate_stiffener_thickness, chamfer, "left")
            )

    end_stiffener_gap = T_es / 2.0
    end_positions = []
    for i in range(num_end_stiffener_pairs):
        end_positions.append(end_stiffener_gap + i * ROLLED_END_STIFFENER_SPACING)
    for i in range(num_end_stiffener_pairs):
        end_positions.append(length - end_stiffener_gap - i * ROLLED_END_STIFFENER_SPACING)

    for y in end_positions:
        bearing_stiffeners.append(
            _create_rolled_stiffener_plate([tw / 2.0, y, z_web_center], end_stiff_width, web_depth, T_es, chamfer, "right")
        )
        bearing_stiffeners.append(
            _create_rolled_stiffener_plate([-tw / 2.0, y, z_web_center], end_stiff_width, web_depth, T_es, chamfer, "left")
        )

    if include_longitudinal_stiffeners:
        long_stiff_width = longitudinal_stiffener_outstand if longitudinal_stiffener_outstand is not None else default_stiff_width
        long_stiff_start = T_es
        seg_len = (length - T_es) - long_stiff_start
        if seg_len > 0:
            heights = []
            if num_longitudinal_stiffeners == 1:
                heights.append(z_web_center + web_depth / 2.0 - web_depth / 3.0)
            elif num_longitudinal_stiffeners == 2:
                heights.append(z_web_center + web_depth / 2.0 - web_depth / 3.0)
                heights.append(z_web_center + web_depth / 2.0 - 2.0 * web_depth / 3.0)
            for h in heights:
                long_stiff_r = _make_rolled_plate(
                    origin=np.array([tw / 2.0 + long_stiff_width / 2.0, long_stiff_start, h]),
                    length=long_stiff_width,
                    width=seg_len,
                    thickness=longitudinal_stiffener_thickness,
                    u_dir=np.array([0., 0., 1.]),
                    w_dir=np.array([0., 1., 0.])
                )
                longitudinal_stiffeners.append(long_stiff_r)
                long_stiff_l = _make_rolled_plate(
                    origin=np.array([-tw / 2.0 - long_stiff_width / 2.0, long_stiff_start, h]),
                    length=long_stiff_width,
                    width=seg_len,
                    thickness=longitudinal_stiffener_thickness,
                    u_dir=np.array([0., 0., 1.]),
                    w_dir=np.array([0., 1., 0.])
                )
                longitudinal_stiffeners.append(long_stiff_l)

    shear_studs = []
    if num_shear_studs_per_section > 0 and shear_stud_pitch > 0:
        base_stud = create_rolled_shear_stud(shear_stud_base_diameter, shear_stud_base_height, shear_stud_top_diameter, shear_stud_top_height)
        z_stud = Z_top
        min_edge = 50.0
        if length > 2 * min_edge:
            max_intervals = int((length - 2 * min_edge) // shear_stud_pitch)
            edge = (length - max_intervals * shear_stud_pitch) / 2.0
            start_y = edge
            num_rows = max_intervals + 1
        else:
            start_y = length / 2.0
            num_rows = 1

        transverse_positions = []
        if num_shear_studs_per_section == 1:
            transverse_positions = [0.0]
        else:
            min_edge_margin = max(50.0, shear_stud_base_diameter)
            max_total_transverse_width = max(0.0, bf - 2 * min_edge_margin)
            desired_total_width = (num_shear_studs_per_section - 1) * shear_stud_transverse_spacing
            actual_total_width = min(desired_total_width, max_total_transverse_width)
            actual_spacing = actual_total_width / (num_shear_studs_per_section - 1) if num_shear_studs_per_section > 1 else 0.0
            start_x = -actual_total_width / 2.0
            for k in range(num_shear_studs_per_section):
                transverse_positions.append(start_x + k * actual_spacing)

        for i in range(num_rows):
            y_pos = start_y + i * shear_stud_pitch
            for x_pos in transverse_positions:
                trsf = gp_Trsf()
                trsf.SetTranslation(gp_Vec(x_pos, y_pos, z_stud))
                stud = BRepBuilderAPI_Transform(base_stud, trsf, True).Shape()
                shear_studs.append(stud)

    supports_vertical = []
    supports_wide_horiz = []
    supports_long_horiz = []
    support_groups = {}

    z_contact_left = Z_top - depth
    z_contact_right = Z_top - depth
    support_width_left = max(bf, bf_b)
    support_width_right = max(bf, bf_b)

    base_dim = min(0.10 * length, 0.75 * depth)
    h_supp = base_dim / 1.5
    w_supp = base_dim / 2.0
    h_tall = h_supp
    h_short = h_supp * 0.35
    w_narrow = h_supp * 0.25

    def _rect_wire(p0, p1, p2, p3):
        return BRepBuilderAPI_MakeWire(
            BRepBuilderAPI_MakeEdge(p0, p1).Edge(),
            BRepBuilderAPI_MakeEdge(p1, p2).Edge(),
            BRepBuilderAPI_MakeEdge(p2, p3).Edge(),
            BRepBuilderAPI_MakeEdge(p3, p0).Edge(),
        ).Wire()

    z_L = z_contact_left
    sw_L = support_width_left

    supports_vertical.append(_rect_wire(
        gp_Pnt(-w_narrow / 2.0, 0, z_L),
        gp_Pnt(w_narrow / 2.0, 0, z_L),
        gp_Pnt(w_narrow / 2.0, 0, z_L - h_tall),
        gp_Pnt(-w_narrow / 2.0, 0, z_L - h_tall),
    ))
    support_groups.setdefault(("Support Vertical", "Left"), []).append(supports_vertical[-1])

    if right_guided:
        supports_wide_horiz.append(_rect_wire(
            gp_Pnt(-sw_L / 2.0, 0, z_L),
            gp_Pnt(sw_L / 2.0, 0, z_L),
            gp_Pnt(sw_L / 2.0, 0, z_L - h_short),
            gp_Pnt(-sw_L / 2.0, 0, z_L - h_short),
        ))
        support_groups.setdefault(("Support Transverse", "Left"), []).append(supports_wide_horiz[-1])

    supports_long_horiz.append(_rect_wire(
        gp_Pnt(0, 0, z_L),
        gp_Pnt(0, w_supp, z_L),
        gp_Pnt(0, w_supp, z_L - h_short),
        gp_Pnt(0, 0, z_L - h_short),
    ))
    support_groups.setdefault(("Support Longitudinal", "Left"), []).append(supports_long_horiz[-1])

    z_R = z_contact_right
    sw_R = support_width_right

    supports_vertical.append(_rect_wire(
        gp_Pnt(-w_narrow / 2.0, length, z_R),
        gp_Pnt(w_narrow / 2.0, length, z_R),
        gp_Pnt(w_narrow / 2.0, length, z_R - h_tall),
        gp_Pnt(-w_narrow / 2.0, length, z_R - h_tall),
    ))
    support_groups.setdefault(("Support Vertical", "Right"), []).append(supports_vertical[-1])

    if right_guided:
        supports_wide_horiz.append(_rect_wire(
            gp_Pnt(-sw_R / 2.0, length, z_R),
            gp_Pnt(sw_R / 2.0, length, z_R),
            gp_Pnt(sw_R / 2.0, length, z_R - h_short),
            gp_Pnt(-sw_R / 2.0, length, z_R - h_short),
        ))
        support_groups.setdefault(("Support Transverse", "Right"), []).append(supports_wide_horiz[-1])

    web_shapes = [_rotate_rolled_about_z(w, -90) for w in web_shapes]
    intermediate_stiffeners = [_rotate_rolled_about_z(s, -90) for s in intermediate_stiffeners]
    bearing_stiffeners = [_rotate_rolled_about_z(s, -90) for s in bearing_stiffeners]
    longitudinal_stiffeners = [_rotate_rolled_about_z(s, -90) for s in longitudinal_stiffeners]
    stiffeners = intermediate_stiffeners + bearing_stiffeners + longitudinal_stiffeners
    shear_studs = [_rotate_rolled_about_z(s, -90) for s in shear_studs]

    supports_vertical = [_rotate_rolled_about_z(s, -90) for s in supports_vertical]
    supports_wide_horiz = [_rotate_rolled_about_z(s, -90) for s in supports_wide_horiz]
    supports_long_horiz = [_rotate_rolled_about_z(s, -90) for s in supports_long_horiz]
    support_groups = {k: [_rotate_rolled_about_z(s, -90) for s in v] for k, v in support_groups.items()}

    return {
        "web": web_shapes,
        "top_flange": [],
        "bottom_flange": [],
        "stiffeners": stiffeners,
        "intermediate_stiffeners": intermediate_stiffeners,
        "bearing_stiffeners": bearing_stiffeners,
        "longitudinal_stiffeners": longitudinal_stiffeners,
        "supports_tri": supports_vertical + supports_wide_horiz + supports_long_horiz,
        "supports_vertical": supports_vertical,
        "supports_wide_horiz": supports_wide_horiz,
        "supports_long_horiz": supports_long_horiz,
        "support_groups": support_groups,
        "supports_cyl": [],
        "shear_studs": shear_studs,
    }
