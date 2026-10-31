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
    KEY_MP_GIRDER_WEB_DEPTH,
)


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


def _make_rolled_i_section_solid(section_dims, z_top, length):
    pts = [
        gp_Pnt(-section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH] / 2.0, 0.0, z_top),
        gp_Pnt(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH] / 2.0, 0.0, z_top),
        gp_Pnt(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_TOP_FLANGE_THICKNESS]),
        gp_Pnt(section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_TOP_FLANGE_THICKNESS]),
        gp_Pnt(section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_DEPTH] + section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_THICKNESS]),
        gp_Pnt(section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_DEPTH] + section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_THICKNESS]),
        gp_Pnt(section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_DEPTH]),
        gp_Pnt(-section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_DEPTH]),
        gp_Pnt(-section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_DEPTH] + section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_THICKNESS]),
        gp_Pnt(-section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_DEPTH] + section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_THICKNESS]),
        gp_Pnt(-section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_TOP_FLANGE_THICKNESS]),
        gp_Pnt(-section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH] / 2.0, 0.0, z_top - section_dims[KEY_MP_GIRDER_TOP_FLANGE_THICKNESS]),
    ]
    wire = _make_rolled_wire_from_points(pts)
    face = BRepBuilderAPI_MakeFace(wire).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0, length, 0)).Shape()


def build_rolled_girder_geometry(
    *,
    section_dims,
    length,
    reference_top_z,
    include_intermediate_stiffeners,
    intermediate_stiffener_spacing,
    intermediate_stiffener_thickness,
    intermediate_stiffener_outstand,
    num_end_stiffener_pairs,
    end_stiffener_spacing,
    T_es,
    end_stiffener_outstand,
    num_longitudinal_stiffeners,
    longitudinal_stiffener_thickness,
    longitudinal_stiffener_outstand,
    shear_stud_base_diameter,
    shear_stud_top_diameter,
    shear_stud_base_height,
    shear_stud_top_height,
    num_shear_studs_per_section,
    shear_stud_transverse_spacing,
    shear_stud_pitch,
    right_guided,
    chamfer_length=40,
):
    web_shapes = [_make_rolled_i_section_solid(section_dims, reference_top_z, length)]
    intermediate_stiffeners = []
    bearing_stiffeners = []
    longitudinal_stiffeners = []
    z_web_center = reference_top_z - section_dims[KEY_MP_GIRDER_TOP_FLANGE_THICKNESS] - section_dims[KEY_MP_GIRDER_WEB_DEPTH] / 2.0
    chamfer = min(chamfer_length, section_dims[KEY_MP_GIRDER_WEB_DEPTH] / 4.0)

    if include_intermediate_stiffeners:
        num_panels = max(1, int(length // intermediate_stiffener_spacing))
        end_stiffener_zone = T_es / 2.0 + (num_end_stiffener_pairs - 1) * end_stiffener_spacing + end_stiffener_spacing

        for i in range(1, num_panels):
            y = i * intermediate_stiffener_spacing
            if y <= end_stiffener_zone or y >= length - end_stiffener_zone:
                continue
            intermediate_stiffeners.append(
                _create_rolled_stiffener_plate([section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0, y, z_web_center], intermediate_stiffener_outstand, section_dims[KEY_MP_GIRDER_WEB_DEPTH], intermediate_stiffener_thickness, chamfer, "right")
            )
            intermediate_stiffeners.append(
                _create_rolled_stiffener_plate([-section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0, y, z_web_center], intermediate_stiffener_outstand, section_dims[KEY_MP_GIRDER_WEB_DEPTH], intermediate_stiffener_thickness, chamfer, "left")
            )

    end_positions = []
    for i in range(num_end_stiffener_pairs):
        end_positions.append(T_es / 2.0 + i * end_stiffener_spacing)
    for i in range(num_end_stiffener_pairs):
        end_positions.append(length - T_es / 2.0 - i * end_stiffener_spacing)

    for y in end_positions:
        bearing_stiffeners.append(
            _create_rolled_stiffener_plate([section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0, y, z_web_center], end_stiffener_outstand, section_dims[KEY_MP_GIRDER_WEB_DEPTH], T_es, chamfer, "right")
        )
        bearing_stiffeners.append(
            _create_rolled_stiffener_plate([-section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0, y, z_web_center], end_stiffener_outstand, section_dims[KEY_MP_GIRDER_WEB_DEPTH], T_es, chamfer, "left")
        )

    if num_longitudinal_stiffeners:
        seg_len = (length - T_es) - T_es
        if seg_len > 0:
            heights = []
            if num_longitudinal_stiffeners == 1:
                heights.append(z_web_center + section_dims[KEY_MP_GIRDER_WEB_DEPTH] / 2.0 - section_dims[KEY_MP_GIRDER_WEB_DEPTH] / 3.0)
            elif num_longitudinal_stiffeners == 2:
                heights.append(z_web_center + section_dims[KEY_MP_GIRDER_WEB_DEPTH] / 2.0 - section_dims[KEY_MP_GIRDER_WEB_DEPTH] / 3.0)
                heights.append(z_web_center + section_dims[KEY_MP_GIRDER_WEB_DEPTH] / 2.0 - 2.0 * section_dims[KEY_MP_GIRDER_WEB_DEPTH] / 3.0)
            for h in heights:
                longitudinal_stiffeners.append(_make_rolled_plate(
                    origin=np.array([section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0 + longitudinal_stiffener_outstand / 2.0, T_es, h]),
                    length=longitudinal_stiffener_outstand,
                    width=seg_len,
                    thickness=longitudinal_stiffener_thickness,
                    u_dir=np.array([0., 0., 1.]),
                    w_dir=np.array([0., 1., 0.])
                ))
                longitudinal_stiffeners.append(_make_rolled_plate(
                    origin=np.array([-section_dims[KEY_MP_GIRDER_WEB_THICKNESS] / 2.0 - longitudinal_stiffener_outstand / 2.0, T_es, h]),
                    length=longitudinal_stiffener_outstand,
                    width=seg_len,
                    thickness=longitudinal_stiffener_thickness,
                    u_dir=np.array([0., 0., 1.]),
                    w_dir=np.array([0., 1., 0.])
                ))

    shear_studs = []
    if num_shear_studs_per_section > 0 and shear_stud_pitch > 0:
        base_stud = create_rolled_shear_stud(shear_stud_base_diameter, shear_stud_base_height, shear_stud_top_diameter, shear_stud_top_height)
        min_edge = 50.0
        if length > 2 * min_edge:
            max_intervals = int((length - 2 * min_edge) // shear_stud_pitch)
            start_y = (length - max_intervals * shear_stud_pitch) / 2.0
            num_rows = max_intervals + 1
        else:
            start_y = length / 2.0
            num_rows = 1

        transverse_positions = []
        if num_shear_studs_per_section == 1:
            transverse_positions = [0.0]
        else:
            min_edge_margin = max(50.0, shear_stud_base_diameter)
            actual_total_width = min((num_shear_studs_per_section - 1) * shear_stud_transverse_spacing, max(0.0, section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH] - 2 * min_edge_margin))
            actual_spacing = actual_total_width / (num_shear_studs_per_section - 1)
            for k in range(num_shear_studs_per_section):
                transverse_positions.append(-actual_total_width / 2.0 + k * actual_spacing)

        for i in range(num_rows):
            y_pos = start_y + i * shear_stud_pitch
            for x_pos in transverse_positions:
                trsf = gp_Trsf()
                trsf.SetTranslation(gp_Vec(x_pos, y_pos, reference_top_z))
                shear_studs.append(BRepBuilderAPI_Transform(base_stud, trsf, True).Shape())

    supports_vertical = []
    supports_wide_horiz = []
    supports_long_horiz = []
    support_groups = {}

    base_dim = min(0.10 * length, 0.75 * section_dims[KEY_MP_GIRDER_DEPTH])
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

    supports_vertical.append(_rect_wire(
        gp_Pnt(-w_narrow / 2.0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
        gp_Pnt(w_narrow / 2.0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
        gp_Pnt(w_narrow / 2.0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_tall),
        gp_Pnt(-w_narrow / 2.0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_tall),
    ))
    support_groups.setdefault(("Support Vertical", "Left"), []).append(supports_vertical[-1])

    if right_guided:
        supports_wide_horiz.append(_rect_wire(
            gp_Pnt(-max(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH], section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH]) / 2.0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
            gp_Pnt(max(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH], section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH]) / 2.0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
            gp_Pnt(max(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH], section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH]) / 2.0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_short),
            gp_Pnt(-max(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH], section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH]) / 2.0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_short),
        ))
        support_groups.setdefault(("Support Transverse", "Left"), []).append(supports_wide_horiz[-1])

    supports_long_horiz.append(_rect_wire(
        gp_Pnt(0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
        gp_Pnt(0, w_supp, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
        gp_Pnt(0, w_supp, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_short),
        gp_Pnt(0, 0, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_short),
    ))
    support_groups.setdefault(("Support Longitudinal", "Left"), []).append(supports_long_horiz[-1])

    supports_vertical.append(_rect_wire(
        gp_Pnt(-w_narrow / 2.0, length, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
        gp_Pnt(w_narrow / 2.0, length, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
        gp_Pnt(w_narrow / 2.0, length, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_tall),
        gp_Pnt(-w_narrow / 2.0, length, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_tall),
    ))
    support_groups.setdefault(("Support Vertical", "Right"), []).append(supports_vertical[-1])

    if right_guided:
        supports_wide_horiz.append(_rect_wire(
            gp_Pnt(-max(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH], section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH]) / 2.0, length, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
            gp_Pnt(max(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH], section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH]) / 2.0, length, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH]),
            gp_Pnt(max(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH], section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH]) / 2.0, length, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_short),
            gp_Pnt(-max(section_dims[KEY_MP_GIRDER_TOP_FLANGE_WIDTH], section_dims[KEY_MP_GIRDER_BOTTOM_FLANGE_WIDTH]) / 2.0, length, reference_top_z - section_dims[KEY_MP_GIRDER_DEPTH] - h_short),
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
