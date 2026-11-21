"""
Cross Bracing Builder for Plate Girder Bridges
===============================================

This module provides functionality to create cross bracing systems for steel plate girder bridges.
It supports multiple section types and bracing configurations with skew angle capability.

Supported Section Types:
    - ANGLE: Single angle section (L-shape)
    - CHANNEL: Channel section (C-shape)
    - DOUBLE_ANGLE: Back-to-back double angle
    - DOUBLE_CHANNEL: Back-to-back double channel
    - I_SECTION: I-beam or H-beam section

Supported Bracing Patterns:
    - X-bracing: Diagonal cross pattern
    - K-bracing: K-pattern with central node
    - Horizontal Diaphragm: Rolled or welded beam diaphragms

Features:
    - Skew angle support for skewed bridges
    - Configurable bracket options
    - End diaphragm customization


"""

import math
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Trsf, gp_Ax1, gp_Ax2, gp_Dir
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_Transform,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeFace,
)
from OCC.Core.Precision import precision

from osdagbridge_core.core.utils.common import (
    KEY_MP_CB_TYPE, KEY_MP_CB_TOP_CHORD, KEY_MP_CB_BOTTOM_CHORD,
    KEY_MP_CB_BRACING_SECTION_TYPE,
    KEY_MP_CB_TOP_CHORD_SECTION_TYPE,
    KEY_MP_CB_BOTTOM_CHORD_SECTION_TYPE,
    KEY_MP_CB_DIAGONAL_LEG_H, KEY_MP_CB_DIAGONAL_LEG_W, KEY_MP_CB_DIAGONAL_THICKNESS,
    KEY_MP_CB_TOP_CHORD_LEG_H, KEY_MP_CB_TOP_CHORD_LEG_W, KEY_MP_CB_TOP_CHORD_THICKNESS,
    KEY_MP_CB_BOTTOM_CHORD_LEG_H, KEY_MP_CB_BOTTOM_CHORD_LEG_W, KEY_MP_CB_BOTTOM_CHORD_THICKNESS,
    KEY_MP_ED_BRACING_TYPE, KEY_MP_ED_TOP_CHORD, KEY_MP_ED_BOTTOM_CHORD,
    KEY_MP_ED_BRACING_SECTION,
    KEY_MP_ED_TOP_CHORD_SECTION_TYPE,
    KEY_MP_ED_BOTTOM_CHORD_SECTION_TYPE,
    KEY_MP_ED_DIAGONAL_LEG_H, KEY_MP_ED_DIAGONAL_LEG_W, KEY_MP_ED_DIAGONAL_THICKNESS,
    KEY_MP_ED_TOP_CHORD_LEG_H, KEY_MP_ED_TOP_CHORD_LEG_W, KEY_MP_ED_TOP_CHORD_THICKNESS,
    KEY_MP_ED_BOTTOM_CHORD_LEG_H, KEY_MP_ED_BOTTOM_CHORD_LEG_W, KEY_MP_ED_BOTTOM_CHORD_THICKNESS,
)


def _parse_sec_type(raw):
    """Convert a UI section-type string to an OCC builder type identifier."""
    if raw is None:
        return None
    s = str(raw).strip().upper()
    if "DOUBLE" in s:
        return "DOUBLE_CHANNEL" if "CHANNEL" in s else "DOUBLE_ANGLE"
    if "CHANNEL" in s:
        return "CHANNEL"
    if "ANGLE" in s:
        return "ANGLE"
    if "I_SECTION" in s or "I-SECTION" in s or "ISECTION" in s:
        return "I_SECTION"
    return None


def _is_enabled(val):
    """Return True unless val explicitly means disabled."""
    return str(val).strip().lower() not in ("no", "false", "0", "none")


# MEMBER ROLES
# A bracing frame is built from three members — the diagonals, the top chord and the
# bottom chord — and each is designed from its own section. They therefore need their
# own hover labels, so the frame builders below return (role, shape) pairs rather than
# bare shapes, and build_cross_bracings() groups them by (component, girder pair, role).
#
# These are plain in-memory tags used while the model is being built. They are not
# KEY_* constants: they never go into input_dict or output_dict, and are not saved.
ROLE_DIAGONAL     = "diagonal"
ROLE_TOP_CHORD    = "top_chord"
ROLE_BOTTOM_CHORD = "bottom_chord"
ROLE_DIAPHRAGM    = "diaphragm"      # solid-beam end diaphragm (no chords/diagonals)

ROLE_NAMES = {
    ROLE_DIAGONAL:     "Diagonal",
    ROLE_TOP_CHORD:    "Top Chord",
    ROLE_BOTTOM_CHORD: "Bottom Chord",
    ROLE_DIAPHRAGM:    "Diaphragm",
}

# The outer half of the grouping key, and also the key each component is registered
# under in the CAD. Keep it in step with the checkbox -> AIS-key map in
# cad_3d.update_component_visibility(), or the visibility checkbox stops matching.
COMPONENT_CROSS_BRACING = "Cross Bracing"
COMPONENT_END_DIAPHRAGM = "End Diaphragm"


def _pk(key, pair_key):
    """Append the girder pair to a KEY constant.

    e.g. _pk(KEY_MP_CB_DIAGONAL_LEG_H, 'G1G2')
         -> 'member_properties.cross_bracing_details.diagonal.leg_h.G1G2'

    This must match how the designer *writes* these values: populate_section_properties()
    in crossbracingforces.py stores them via make_pair_key(key, pair_id), i.e.
    f"{key}.{pair_id}".  An earlier version inserted the pair into the middle of the
    path instead, so every lookup below missed and the CAD silently fell back to the
    caller's default section dimensions — wrong geometry, not just a wrong tooltip.
    """
    return f"{key}.{pair_key}"


# SECTION GEOMETRY CREATORS

def _translate_section(shape, x, y, z):
    transform = gp_Trsf()
    transform.SetTranslation(gp_Vec(x, y, z))
    return BRepBuilderAPI_Transform(shape, transform, False).Shape()


def _extrude_section(poly, length):
    poly.Close()
    if not poly.IsDone():
        raise ValueError("Could not build the bracing profile wire")
    face = BRepBuilderAPI_MakeFace(poly.Wire())
    if not face.IsDone():
        raise ValueError("Could not build the bracing profile face")
    prism = BRepPrimAPI_MakePrism(face.Face(), gp_Vec(length, 0, 0))
    if not prism.IsDone():
        raise ValueError("Could not extrude the bracing profile")
    return prism.Shape()


def _has_open_flange_recess(depth, width, web_thickness, flange_thickness):
    tolerance = precision.Confusion()
    return width - web_thickness > tolerance and depth - 2 * flange_thickness > tolerance


def _fuse_web_and_flanges(length, depth, width, web_thickness, flange_thickness, flange_y):
    web = BRepPrimAPI_MakeBox(length, web_thickness, depth).Shape()
    flange = BRepPrimAPI_MakeBox(length, width, flange_thickness).Shape()
    bottom = _translate_section(flange, 0, flange_y, 0)
    top = _translate_section(flange, 0, flange_y, depth - flange_thickness)
    return BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(web, bottom).Shape(), top).Shape()


def _create_angle_section(length, leg_h, leg_w, thickness):
    """
    Create a single angle section (L-shape).
    
    Args:
        length: Length of the section along X-axis
        leg_h: Height of the vertical leg (longer leg)
        leg_w: Width of the horizontal leg (shorter leg)
        thickness: Thickness of the angle
        
    Returns:
        TopoDS_Shape: The angle section centered at origin
    """
    if min(leg_h, leg_w) - thickness <= precision.Confusion():
        vertical = BRepPrimAPI_MakeBox(length, thickness, leg_h).Shape()
        horizontal = BRepPrimAPI_MakeBox(length, leg_w, thickness).Shape()
        section = BRepAlgoAPI_Fuse(vertical, horizontal).Shape()
        return _translate_section(section, -length / 2, -leg_w / 2, -leg_h / 2)

    # Construct 6-point closed 2D polyline in the Y-Z plane at x = -length / 2
    # Profile coordinates are pre-centered around origin
    y0 = -leg_w / 2.0
    z0 = -leg_h / 2.0
    x0 = -length / 2.0

    poly = BRepBuilderAPI_MakePolygon()
    poly.Add(gp_Pnt(x0, y0, z0))
    poly.Add(gp_Pnt(x0, y0 + leg_w, z0))
    poly.Add(gp_Pnt(x0, y0 + leg_w, z0 + thickness))
    poly.Add(gp_Pnt(x0, y0 + thickness, z0 + thickness))
    poly.Add(gp_Pnt(x0, y0 + thickness, z0 + leg_h))
    poly.Add(gp_Pnt(x0, y0, z0 + leg_h))
    return _extrude_section(poly, length)


def _create_channel_section(length, depth, flange_width, web_thickness, flange_thickness):
    """
    Create a channel section (C-shape).
    
    Args:
        length: Length of the section along X-axis
        depth: Overall depth of the channel
        flange_width: Width of top and bottom flanges
        web_thickness: Thickness of the web
        flange_thickness: Thickness of the flanges
        
    Returns:
        TopoDS_Shape: The channel section centered at origin
    """
    if not _has_open_flange_recess(depth, flange_width, web_thickness, flange_thickness):
        section = _fuse_web_and_flanges(
            length, depth, flange_width, web_thickness, flange_thickness, 0
        )
        return _translate_section(section, -length / 2, -flange_width / 2, -depth / 2)

    # Construct 8-point closed 2D polyline in the Y-Z plane at x = -length / 2
    # Profile coordinates are pre-centered around origin
    y0 = -flange_width / 2.0
    z0 = -depth / 2.0
    x0 = -length / 2.0

    poly = BRepBuilderAPI_MakePolygon()
    poly.Add(gp_Pnt(x0, y0, z0))
    poly.Add(gp_Pnt(x0, y0 + flange_width, z0))
    poly.Add(gp_Pnt(x0, y0 + flange_width, z0 + flange_thickness))
    poly.Add(gp_Pnt(x0, y0 + web_thickness, z0 + flange_thickness))
    poly.Add(gp_Pnt(x0, y0 + web_thickness, z0 + depth - flange_thickness))
    poly.Add(gp_Pnt(x0, y0 + flange_width, z0 + depth - flange_thickness))
    poly.Add(gp_Pnt(x0, y0 + flange_width, z0 + depth))
    poly.Add(gp_Pnt(x0, y0, z0 + depth))
    return _extrude_section(poly, length)

def _create_double_angle_section(length, leg_h, leg_w, thickness, connection_type="SHORTER_LEG"):
    """
    Create a double angle section (back-to-back angles).
    
    Args:
        length: Length of the section along X-axis
        leg_h: Height of the longer leg
        leg_w: Width of the shorter leg
        thickness: Thickness of each angle
        connection_type: "LONGER_LEG" or "SHORTER_LEG"
        
    Returns:
        TopoDS_Shape: The double angle section centered at origin
    """
    x0 = -length / 2.0
    t = thickness
    t_half = t / 2.0

    if connection_type == "SHORTER_LEG":
        h_vert = leg_w
        w_horiz = leg_h
    else:  # LONGER_LEG
        h_vert = leg_h
        w_horiz = leg_w

    if h_vert - t <= precision.Confusion():
        # With a collapsed or reversed web recess, retain the legacy box union.
        web = BRepPrimAPI_MakeBox(
            gp_Pnt(x0, -t_half, -h_vert / 2),
            gp_Pnt(-x0, t_half, h_vert / 2),
        ).Shape()
        left = BRepPrimAPI_MakeBox(
            gp_Pnt(x0, -t_half - w_horiz, h_vert / 2 - t),
            gp_Pnt(-x0, -t_half, h_vert / 2),
        ).Shape()
        right = _translate_section(left, 0, t + w_horiz, 0)
        return BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(web, left).Shape(), right).Shape()

    z_bot = -h_vert / 2.0
    z_flange_bot = h_vert / 2.0 - t
    z_top = h_vert / 2.0
    y_outer_pos = t_half + w_horiz
    y_outer_neg = -(t_half + w_horiz)

    poly = BRepBuilderAPI_MakePolygon()
    poly.Add(gp_Pnt(x0, -t_half, z_bot))
    poly.Add(gp_Pnt(x0, t_half, z_bot))
    poly.Add(gp_Pnt(x0, t_half, z_flange_bot))
    poly.Add(gp_Pnt(x0, y_outer_pos, z_flange_bot))
    poly.Add(gp_Pnt(x0, y_outer_pos, z_top))
    poly.Add(gp_Pnt(x0, y_outer_neg, z_top))
    poly.Add(gp_Pnt(x0, y_outer_neg, z_flange_bot))
    poly.Add(gp_Pnt(x0, -t_half, z_flange_bot))
    return _extrude_section(poly, length)


def _create_double_channel_section(length, depth, flange_width, web_thickness, flange_thickness):
    """
    Create a double channel section (back-to-back channels).
    
    Args:
        length: Length of the section along X-axis
        depth: Overall depth of each channel
        flange_width: Width of flanges
        web_thickness: Thickness of the web
        flange_thickness: Thickness of the flanges
        
    Returns:
        TopoDS_Shape: The double channel section centered at origin
    """
    # The two channel webs share a face. Their union is a wider I profile.
    if _has_open_flange_recess(depth, flange_width, web_thickness, flange_thickness):
        return _create_i_section(
            length, depth, 2 * flange_width, 2 * web_thickness, flange_thickness
        )

    base = _create_channel_section(length, depth, flange_width, web_thickness, flange_thickness)
    mirror = gp_Trsf()
    mirror.SetMirror(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)))
    mirrored = BRepBuilderAPI_Transform(base, mirror, True).Shape()
    offset = flange_width / 2.0
    return BRepAlgoAPI_Fuse(
        _translate_section(base, 0, offset, 0),
        _translate_section(mirrored, 0, -offset, 0),
    ).Shape()


def _create_i_section(length, depth, flange_width, web_thickness, flange_thickness):
    """
    Create an I-section (I-beam or H-beam).
    
    Args:
        length: Length of the section along X-axis
        depth: Overall depth of the I-section
        flange_width: Width of top and bottom flanges
        web_thickness: Thickness of the web
        flange_thickness: Thickness of the flanges
        
    Returns:
        TopoDS_Shape: The I-section centered at origin
    """
    if not _has_open_flange_recess(depth, flange_width, web_thickness, flange_thickness):
        section = _fuse_web_and_flanges(
            length, depth, flange_width, web_thickness, flange_thickness,
            (web_thickness - flange_width) / 2,
        )
        return _translate_section(section, -length / 2, -web_thickness / 2, -depth / 2)

    bf = flange_width / 2.0
    d = depth / 2.0
    tw = web_thickness / 2.0
    tf = flange_thickness
    x0 = -length / 2.0

    poly = BRepBuilderAPI_MakePolygon()
    poly.Add(gp_Pnt(x0, -bf, -d))
    poly.Add(gp_Pnt(x0, bf, -d))
    poly.Add(gp_Pnt(x0, bf, -d + tf))
    poly.Add(gp_Pnt(x0, tw, -d + tf))
    poly.Add(gp_Pnt(x0, tw, d - tf))
    poly.Add(gp_Pnt(x0, bf, d - tf))
    poly.Add(gp_Pnt(x0, bf, d))
    poly.Add(gp_Pnt(x0, -bf, d))
    poly.Add(gp_Pnt(x0, -bf, d - tf))
    poly.Add(gp_Pnt(x0, -tw, d - tf))
    poly.Add(gp_Pnt(x0, -tw, -d + tf))
    poly.Add(gp_Pnt(x0, -bf, -d + tf))
    return _extrude_section(poly, length)


# UTILITY FUNCTIONS

def _get_roll_angle(section_type, roll_sign):
    """
    Determine the roll angle for section orientation.
    
    Args:
        section_type: Type of section ("ANGLE", "CHANNEL", etc.)
        roll_sign: Sign indicating roll direction (+1 or -1)
        
    Returns:
        float: Roll angle in radians
    """
    if section_type in ("ANGLE", "CHANNEL"):
        return roll_sign * (math.pi / 2)
    return 0.0


def _create_section_solid(section_type, length, thickness, dims):
    """
    Create a section solid based on type and dimensions.
    
    Args:
        section_type: Type of section to create
        length: Length of the section
        thickness: Thickness parameter (used for angles)
        dims: Dictionary containing section dimensions
        
    Returns:
        TopoDS_Shape: The created section
        
    Raises:
        ValueError: If section type is not supported
    """
    dims = dict(dims)  # shallow copy — lines below mutate dims to fill missing keys

    if "depth" not in dims and "leg_h" in dims:
        dims["depth"] = dims["leg_h"]
    if "flange_width" not in dims and "leg_w" in dims:
        dims["flange_width"] = dims["leg_w"]
    if "leg_h" not in dims and "depth" in dims:
        dims["leg_h"] = dims["depth"]
    if "leg_w" not in dims and "flange_width" in dims:
        dims["leg_w"] = dims["flange_width"]

    if "web_thickness" not in dims:
        dims["web_thickness"] = thickness
    if "flange_thickness" not in dims:
        dims["flange_thickness"] = thickness

    if section_type == "ANGLE":
        # Support both angle-specific and generic dimension keys
        h = dims.get("leg_h", dims.get("depth", 100))
        w = dims.get("leg_w", dims.get("flange_width", 50))
        return _create_angle_section(length, h, w, thickness)
    
    if section_type == "CHANNEL":
        d = dims.get("depth", 100)
        wf = dims.get("flange_width", 50)
        tw = dims.get("web_thickness", 5)
        tf = dims.get("flange_thickness", 7)
        return _create_channel_section(length, d, wf, tw, tf)
    
    if section_type == "DOUBLE_ANGLE":
        h = dims.get("leg_h", dims.get("depth", 100))
        w = dims.get("leg_w", dims.get("flange_width", 50))
        return _create_double_angle_section(
            length, h, w, thickness,
            dims.get("connection_type", "LONGER_LEG")
        )
    
    if section_type == "DOUBLE_CHANNEL":
        d = dims.get("depth", 100)
        wf = dims.get("flange_width", 50)
        tw = dims.get("web_thickness", 5)
        tf = dims.get("flange_thickness", 7)
        return _create_double_channel_section(length, d, wf, tw, tf)
    
    if section_type == "I_SECTION":
        d = dims.get("depth", 100)
        wf = dims.get("flange_width", 50)
        tw = dims.get("web_thickness", 10)
        tf = dims.get("flange_thickness", 15)
        return _create_i_section(length, d, wf, tw, tf)
    
    raise ValueError(f"Unsupported section type: {section_type}")


# MEMBER CREATION FUNCTIONS

def _create_diagonal_member(p1, p2, thickness, section_type, dims, roll_sign):
    """
    Create a diagonal bracing member between two points.
    
    
    Args:
        p1: Starting point (gp_Pnt)
        p2: Ending point (gp_Pnt)
        thickness: Section thickness
        section_type: Type of section
        dims: Section dimensions dictionary
        roll_sign: Roll direction indicator
        
    Returns:
        TopoDS_Shape: The positioned bracing member
    """
    vec = gp_Vec(p1, p2)
    length = vec.Magnitude()
    if length <= precision.Confusion():
        raise ValueError("Bracing member endpoints must be separated")

    solid = _create_section_solid(section_type, length, thickness, dims)
    x_dir = gp_Dir(1, 0, 0)
    target = gp_Dir(vec)
    axis = gp_Vec(x_dir).Crossed(gp_Vec(target))
    alignment = gp_Trsf()
    if axis.Magnitude() > 1e-12:
        alignment.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(axis)), x_dir.Angle(target))
    elif x_dir.Dot(target) < 0:
        alignment.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(0, 0, 1)), math.pi)

    placement = gp_Trsf()
    roll = _get_roll_angle(section_type, roll_sign)
    if abs(roll) > 1e-6:
        placement.SetRotation(gp_Ax1(gp_Pnt(), target), roll)
    placement.Multiply(alignment)
    placement.SetTranslationPart(gp_Vec(
        (p1.X() + p2.X()) / 2,
        (p1.Y() + p2.Y()) / 2,
        (p1.Z() + p2.Z()) / 2,
    ))
    return BRepBuilderAPI_Transform(solid, placement, False).Shape()


# BRACING PATTERN FUNCTIONS

def _x_bracing(x, yL, yR, depth, tf, flange_w,
               diagonal_thickness, diagonal_section_type, diagonal_dims,
               top_chord_thickness, top_chord_section_type, top_chord_dims,
               bottom_chord_thickness, bottom_chord_section_type, bottom_chord_dims,
               bracket, skew_angle=0):
    """
    Create X-bracing pattern between two girders.
    
    X-bracing consists of two diagonal members crossing each other,
    with optional horizontal brackets at top and/or bottom.
    
    Args:
        x: Longitudinal position
        yL: Left girder lateral position
        yR: Right girder lateral position
        depth: Girder depth
        tf: Flange thickness
        flange_w: Flange width
        
        diagonal_thickness: Thickness of diagonal members
        diagonal_section_type: Section type for diagonal members
        diagonal_dims: Section dimensions for diagonal members
        
        top_chord_thickness: Thickness of top chord/bracket
        top_chord_section_type: Section type for top chord/bracket
        top_chord_dims: Section dimensions for top chord/bracket
        
        bottom_chord_thickness: Thickness of bottom chord/bracket
        bottom_chord_section_type: Section type for bottom chord/bracket
        bottom_chord_dims: Section dimensions for bottom chord/bracket
        
        bracket: Bracket option ("NONE", "UPPER", "LOWER", "BOTH")
        skew_angle: Bridge skew angle in degrees
        
    Returns:
        list[tuple[str, TopoDS_Shape]]: (role, shape) pairs — see the ROLE_* constants.
    """
    # Calculate vertical positions (top and bottom of web)
    z_bot = -depth / 2
    z_top = +depth / 2
    
    def get_skew_x(y):
        """Calculate longitudinal offset due to skew."""
        if skew_angle == 0:
            return 0.0
        return y * math.tan(math.radians(skew_angle))
    
    # Apply skew offsets
    x_l = x + get_skew_x(yL)
    x_r = x + get_skew_x(yR)
    
    # Create main X-diagonals using diagonal section
    braces = [
        # Left TOP → Right BOTTOM diagonal
        (ROLE_DIAGONAL, _create_diagonal_member(
            gp_Pnt(x_l, yL, z_top), 
            gp_Pnt(x_r, yR, z_bot),
            diagonal_thickness, diagonal_section_type, diagonal_dims, +1
        )),
        # Left BOTTOM → Right TOP diagonal
        (ROLE_DIAGONAL, _create_diagonal_member(
            gp_Pnt(x_l, yL, z_bot), 
            gp_Pnt(x_r, yR, z_top),
            diagonal_thickness, diagonal_section_type, diagonal_dims, -1
        ))
    ]
    
    # Add optional bottom bracket using bottom chord section
    if bracket in ("LOWER", "BOTH"):
        braces.append(
            (ROLE_BOTTOM_CHORD, _create_diagonal_member(
                gp_Pnt(x_l, yL, z_bot), 
                gp_Pnt(x_r, yR, z_bot),
                bottom_chord_thickness, bottom_chord_section_type, bottom_chord_dims, +1
            ))
        )
    
    # Add optional top bracket using top chord section
    if bracket in ("UPPER", "BOTH"):
        braces.append(
            (ROLE_TOP_CHORD, _create_diagonal_member(
                gp_Pnt(x_l, yL, z_top), 
                gp_Pnt(x_r, yR, z_top),
                top_chord_thickness, top_chord_section_type, top_chord_dims, +1
            ))
        )
    
    return braces


def _k_bracing(x, yL, yR, depth, tf, flange_w,
               diagonal_thickness, diagonal_section_type, diagonal_dims,
               top_chord_thickness, top_chord_section_type, top_chord_dims,
               bottom_chord_thickness, bottom_chord_section_type, bottom_chord_dims,
               top_bracket, skew_angle=0):
    """
    Create K-bracing pattern between two girders.
    
    K-bracing consists of two diagonal members meeting at a central
    bottom node, with a mandatory bottom horizontal and optional top horizontal.
    
    Args:
        x: Longitudinal position
        yL: Left girder lateral position
        yR: Right girder lateral position
        depth: Girder depth
        tf: Flange thickness
        flange_w: Flange width
        
        diagonal_thickness: Thickness of diagonal members
        diagonal_section_type: Section type for diagonal members
        diagonal_dims: Section dimensions for diagonal members
        
        top_chord_thickness: Thickness of top chord/bracket
        top_chord_section_type: Section type for top chord/bracket
        top_chord_dims: Section dimensions for top chord/bracket
        
        bottom_chord_thickness: Thickness of bottom chord/bracket
        bottom_chord_section_type: Section type for bottom chord/bracket
        bottom_chord_dims: Section dimensions for bottom chord/bracket
        
        top_bracket: Whether to include top horizontal bracket
        skew_angle: Bridge skew angle in degrees
        
    Returns:
        list[tuple[str, TopoDS_Shape]]: (role, shape) pairs — see the ROLE_* constants.
    """
    # Calculate vertical positions
    z_bot = -depth / 2
    z_top = +depth / 2
    
    # Calculate midpoint between girders
    ym = (yL + yR) / 2
    
    def get_skew_x(y):
        """Calculate longitudinal offset due to skew."""
        if skew_angle == 0:
            return 0.0
        return y * math.tan(math.radians(skew_angle))
    
    # Apply skew offsets
    x_l = x + get_skew_x(yL)
    x_r = x + get_skew_x(yR)
    x_m = x + get_skew_x(ym)
    
    # Create K-pattern members
    braces = [
        # Left TOP → Middle BOTTOM diagonal (using diagonal section)
        (ROLE_DIAGONAL, _create_diagonal_member(
            gp_Pnt(x_l, yL, z_top), 
            gp_Pnt(x_m, ym, z_bot),
            diagonal_thickness, diagonal_section_type, diagonal_dims, +1
        )),
        # Right TOP → Middle BOTTOM diagonal (using diagonal section)
        (ROLE_DIAGONAL, _create_diagonal_member(
            gp_Pnt(x_r, yR, z_top), 
            gp_Pnt(x_m, ym, z_bot),
            diagonal_thickness, diagonal_section_type, diagonal_dims, -1
        )),
        # Bottom horizontal (mandatory for K-bracing, using bottom chord section)
        (ROLE_BOTTOM_CHORD, _create_diagonal_member(
            gp_Pnt(x_l, yL, z_bot), 
            gp_Pnt(x_r, yR, z_bot),
            bottom_chord_thickness, bottom_chord_section_type, bottom_chord_dims, +1
        ))
    ]
    
    # Add optional top bracket using top chord section
    if top_bracket:
        braces.append(
            (ROLE_TOP_CHORD, _create_diagonal_member(
                gp_Pnt(x_l, yL, z_top), 
                gp_Pnt(x_r, yR, z_top),
                top_chord_thickness, top_chord_section_type, top_chord_dims, +1
            ))
        )
    
    return braces


def _diaphragm_bracing(x, yL, yR, depth, tf, thickness, section_type, dims, skew_angle=0):
    """
    Create a horizontal diaphragm member (rolled or welded beam).
    
    Diaphragm is placed at the top of the girder depth, typically used
    at bridge ends for load distribution and stability.
    
    Args:
        x: Longitudinal position
        yL: Left girder lateral position
        yR: Right girder lateral position
        depth: Girder depth
        tf: Flange thickness
        thickness: Diaphragm thickness
        section_type: Section type (typically I_SECTION)
        dims: Section dimensions
        skew_angle: Bridge skew angle in degrees
        
    Returns:
        list: List containing the diaphragm member shape
    """
    dims = dict(dims)  # shallow copy — avoid mutating caller's dict

    # Place exactly at the bottom of the girder top flange
    # top edge of web is at +depth / 2
    # diaphragm is centered at gp_Pnt, so center = (top_edge) - (member_depth / 2)
    member_depth = dims.get("depth", 100)
    z_center = (depth / 2) - (member_depth / 2)
    
    def get_skew_x(y):
        """Calculate longitudinal offset due to skew."""
        if skew_angle == 0:
            return 0.0
        return y * math.tan(math.radians(skew_angle))
    
    # Apply skew offsets
    x_l = x + get_skew_x(yL)
    x_r = x + get_skew_x(yR)
    
    # Create horizontal diaphragm member
    return [
        (ROLE_DIAPHRAGM, _create_diagonal_member(
            gp_Pnt(x_l, yL, z_center), 
            gp_Pnt(x_r, yR, z_center),
            thickness, 
            section_type, 
            dims, 
            +1
        ))
    ]




def build_cross_bracings(
    *,
    # Bridge geometry parameters
    span_length_L,
    num_girders,
    girder_spacing,
    girder_depths,          # Can be float (uniform) or List[List[float]] [panel_idx][girder_idx]
    reference_depth,        # Reference web depth for vertical alignment
    flange_thickness,
    flange_width,
    
    # Internal bracing configuration
    bracing_type,          # "X" or "K"
    
    # Diagonal member configuration
    diagonal_section_type,
    diagonal_section_dims,
    diagonal_thickness,
    
    # Top chord/bracket configuration
    top_chord_section_type,
    top_chord_section_dims,
    top_chord_thickness,
    
    # Bottom chord/bracket configuration
    bottom_chord_section_type,
    bottom_chord_section_dims,
    bottom_chord_thickness,
    
    # Spacing and options
    panel_spacing,         # Spacing between bracing frames
    bracket_option="BOTH", # For X-bracing: "NONE", "UPPER", "LOWER", "BOTH"
    top_bracket=False,     # For K-bracing: include top horizontal
    skew_angle=0,          # Bridge skew angle in degrees
    
    # End diaphragm configuration
    end_diaphragm_type="Cross Bracing",  # "Cross Bracing", "Rolled Beam", "Welded Beam"
    end_diaphragm_bracing_type=None,     # If None, use bracing_type
    
    # End diaphragm diagonal configuration
    end_diaphragm_diagonal_section_type=None,  # If None, error
    end_diaphragm_diagonal_section_dims=None,
    end_diaphragm_diagonal_thickness=None,
    
    # End diaphragm top chord configuration
    end_diaphragm_top_chord_section_type=None,
    end_diaphragm_top_chord_section_dims=None,
    end_diaphragm_top_chord_thickness=None,
    
    # End diaphragm bottom chord configuration
    end_diaphragm_bottom_chord_section_type=None,
    end_diaphragm_bottom_chord_section_dims=None,
    end_diaphragm_bottom_chord_thickness=None,
    
    # For Rolled/Welded beam diaphragms
    end_diaphragm_section="I_SECTION",
    end_diaphragm_dims=None,
    end_diaphragm_spacing=0,
    output_dict=None,
):

    bracings = []
    # Parallel index of the same shapes, grouped as (component, pair_id, role) -> [shapes].
    # The CAD layer uses it to give each member its own hover label; the flat list above
    # is unchanged and is what the IFC export and the legacy display path still read.
    bracing_groups = {}

    # Calculate bracing frame positions along the span
    # Number of internal panels
    n_internal = int(span_length_L / panel_spacing) - 1
    # Total number of frames (internal + 2 end frames)
    n_total = n_internal + 2
    # Actual spacing (may differ slightly from panel_spacing)
    spacing = span_length_L / (n_total - 1)
    # Generate longitudinal positions
    x_positions = [i * spacing for i in range(n_total)]
    
    # Calculate total transverse width
    total_width = (num_girders - 1) * girder_spacing
    
    # Loop through all bracing frames
    for idx_x, x in enumerate(x_positions):
        # Loop through all bays between girders
        for i in range(num_girders - 1):
            # Girder pair this bay sits between, e.g. "G1G2".  Defined here rather than
            # inside the `if output_dict:` blocks below so _collect() can always group
            # by it, even when no design snapshot was passed in.
            bay_pair_id = f"G{i+1}G{i+2}"

            # Calculate lateral positions of left and right girders
            yL = (i * girder_spacing) - total_width / 2
            yR = yL + girder_spacing
            
            # Get girder depths for this bay
            if isinstance(girder_depths, (int, float)):
                d_L = girder_depths
                d_R = girder_depths
            else:
                # depths[panel_idx][girder_idx]
                d_L = girder_depths[idx_x][i]
                d_R = girder_depths[idx_x][i+1]
            
            # The smaller girder depth controls the bracing height
            eff_depth = min(d_L, d_R)
            
            # Calculate vertical offset to keep top flush with reference web top
            # reference_depth/2 is the Z-coord of the top of the reference web
            # eff_depth/2 is the Z-coord of the top of the bracing frame (at center)
            z_offset = (reference_depth - eff_depth) / 2.0
            
            def _apply_z_offset(tagged, dz):
                """Shift a frame's (role, shape) pairs vertically, keeping the roles."""
                if not dz: return tagged
                trsf = gp_Trsf()
                trsf.SetTranslation(gp_Vec(0, 0, dz))
                return [(role, BRepBuilderAPI_Transform(s, trsf, False).Shape())
                        for role, s in tagged]

            def _collect(tagged, dz, component, pair_id):
                """Offset a frame, then record its shapes both ways.

                Every shape still lands in the flat `bracings` list exactly as before.
                The addition is `bracing_groups`, which keeps the same shapes filed
                under (component, girder pair, role) so each member can be labelled
                with its own designed section instead of one string for all of them.
                """
                for role, shape in _apply_z_offset(tagged, dz):
                    bracings.append(shape)
                    bracing_groups.setdefault((component, pair_id, role), []).append(shape)

            # Check if this is an end position (first or last frame)
            is_end = (x == x_positions[0] or x == x_positions[-1])
            is_first = (x == x_positions[0])
            is_last = (x == x_positions[-1])
            
            # END DIAPHRAGM HANDLING
            if is_end:
                # Apply longitudinal offset for end diaphragms
                # Base offset from parameters or default
                base_offset = end_diaphragm_spacing if end_diaphragm_spacing > 0 else 200.0
                
                # Skew-aware adjustment:
                # Stiffeners are perpendicular to the girder axis (global Y in girder local coords).
                # Diaphragms are skewed (parallel to skew).
                # To clear the stiffener tip (which extends ~200mm from web), 
                # we need an extra shift equal to (stiffener_width * tan(skew)).
                
                # We use a hardcoded 200mm for stiffener width for now to match cad_generator defaults
                stiffener_width = 300.0
                skew_rad = math.radians(abs(skew_angle))
                extra_offset = stiffener_width * math.tan(skew_rad)
                
                offset = base_offset + extra_offset
                
                if is_first:
                    x_eff = x + offset
                else: # is_last
                    x_eff = x - offset

                if end_diaphragm_type == "Cross Bracing":
                    # Use end diaphragm section configurations
                    p_ed_bracing_type = end_diaphragm_bracing_type
                    p_ed_diag_thick = end_diaphragm_diagonal_thickness
                    p_ed_diag_sec_type = end_diaphragm_diagonal_section_type
                    p_ed_diag_dims = end_diaphragm_diagonal_section_dims
                    
                    p_ed_tc_thick = end_diaphragm_top_chord_thickness
                    p_ed_tc_sec_type = end_diaphragm_top_chord_section_type
                    p_ed_tc_dims = end_diaphragm_top_chord_section_dims
                    
                    p_ed_bc_thick = end_diaphragm_bottom_chord_thickness
                    p_ed_bc_sec_type = end_diaphragm_bottom_chord_section_type
                    p_ed_bc_dims = end_diaphragm_bottom_chord_section_dims
                    
                    p_ed_bracket_option = "BOTH"
                    p_ed_top_bracket = True

                    if output_dict:
                        pair_key = f"G{i+1}G{i+2}"
                        ed_suffix = f".{pair_key}.E{i+1}M1"

                        # bracing type
                        _bt = output_dict.get(KEY_MP_ED_BRACING_TYPE + ed_suffix)
                        if _bt is not None:
                            p_ed_bracing_type = "K" if "K" in str(_bt).upper() else "X"

                        # top / bottom chord enabled
                        _tc_en = _is_enabled(output_dict.get(KEY_MP_ED_TOP_CHORD + ed_suffix))
                        _bc_en = _is_enabled(output_dict.get(KEY_MP_ED_BOTTOM_CHORD + ed_suffix))

                        # section types (UI keys, then designed output keys)
                        _diag_st = (
                            _parse_sec_type(output_dict.get(KEY_MP_ED_BRACING_SECTION + ed_suffix))
                        )
                        if _diag_st:
                            p_ed_diag_sec_type = _diag_st

                        _tc_st = (
                            _parse_sec_type(output_dict.get(KEY_MP_ED_TOP_CHORD_SECTION_TYPE + ed_suffix))
                        )
                        if _tc_st:
                            p_ed_tc_sec_type = _tc_st

                        _bc_st = (
                            _parse_sec_type(output_dict.get(KEY_MP_ED_BOTTOM_CHORD_SECTION_TYPE + ed_suffix))
                        )
                        if _bc_st:
                            p_ed_bc_sec_type = _bc_st

                        # section dimensions from designed output keys
                        _d_lh = output_dict.get(_pk(KEY_MP_ED_DIAGONAL_LEG_H, pair_key))
                        _d_lw = output_dict.get(_pk(KEY_MP_ED_DIAGONAL_LEG_W, pair_key))
                        _d_tw = output_dict.get(_pk(KEY_MP_ED_DIAGONAL_THICKNESS, pair_key))
                        if _d_lh is not None and _d_lw is not None:
                            p_ed_diag_dims = {"leg_h": float(_d_lh), "leg_w": float(_d_lw), "connection_type": "LONGER_LEG"}
                        if _d_tw is not None:
                            p_ed_diag_thick = float(_d_tw)

                        _tc_lh = output_dict.get(_pk(KEY_MP_ED_TOP_CHORD_LEG_H, pair_key))
                        _tc_lw = output_dict.get(_pk(KEY_MP_ED_TOP_CHORD_LEG_W, pair_key))
                        _tc_tw = output_dict.get(_pk(KEY_MP_ED_TOP_CHORD_THICKNESS, pair_key))
                        if _tc_lh is not None and _tc_lw is not None:
                            p_ed_tc_dims = {"leg_h": float(_tc_lh), "leg_w": float(_tc_lw), "connection_type": "LONGER_LEG"}
                        if _tc_tw is not None:
                            p_ed_tc_thick = float(_tc_tw)

                        _bc_lh = output_dict.get(_pk(KEY_MP_ED_BOTTOM_CHORD_LEG_H, pair_key))
                        _bc_lw = output_dict.get(_pk(KEY_MP_ED_BOTTOM_CHORD_LEG_W, pair_key))
                        _bc_tw = output_dict.get(_pk(KEY_MP_ED_BOTTOM_CHORD_THICKNESS, pair_key))
                        if _bc_lh is not None and _bc_lw is not None:
                            p_ed_bc_dims = {"leg_h": float(_bc_lh), "leg_w": float(_bc_lw), "connection_type": "LONGER_LEG"}
                        if _bc_tw is not None:
                            p_ed_bc_thick = float(_bc_tw)

                        # bracket configuration
                        if _tc_en and _bc_en:
                            p_ed_bracket_option = "BOTH"
                        elif _tc_en:
                            p_ed_bracket_option = "UPPER"
                        elif _bc_en:
                            p_ed_bracket_option = "LOWER"
                        else:
                            p_ed_bracket_option = "NONE"
                        p_ed_top_bracket = _tc_en

                    if p_ed_bracing_type == "X":
                        frame = _x_bracing(
                                x_eff, yL, yR,
                                eff_depth, flange_thickness, flange_width,
                                p_ed_diag_thick,
                                p_ed_diag_sec_type,
                                p_ed_diag_dims,
                                p_ed_tc_thick,
                                p_ed_tc_sec_type,
                                p_ed_tc_dims,
                                p_ed_bc_thick,
                                p_ed_bc_sec_type,
                                p_ed_bc_dims,
                                p_ed_bracket_option,
                                skew_angle=skew_angle
                        )
                        _collect(frame, z_offset, COMPONENT_END_DIAPHRAGM, bay_pair_id)
                    elif p_ed_bracing_type == "K":
                        frame = _k_bracing(
                                x_eff, yL, yR,
                                eff_depth, flange_thickness, flange_width,
                                p_ed_diag_thick,
                                p_ed_diag_sec_type,
                                p_ed_diag_dims,
                                p_ed_tc_thick,
                                p_ed_tc_sec_type,
                                p_ed_tc_dims,
                                p_ed_bc_thick,
                                p_ed_bc_sec_type,
                                p_ed_bc_dims,
                                p_ed_top_bracket,
                                skew_angle=skew_angle
                        )
                        _collect(frame, z_offset, COMPONENT_END_DIAPHRAGM, bay_pair_id)
                
                elif end_diaphragm_type == "Rolled Beam" or end_diaphragm_type == "Welded Beam":
                    diaphragm_dims = end_diaphragm_dims if end_diaphragm_dims is not None else {
                        "depth": eff_depth * 0.8,
                        "flange_width": 200,
                        "web_thickness": 10,
                        "flange_thickness": 15
                    }
                    # Validate dimensions or use defaults (slightly larger than rolled)
                    if "depth" not in diaphragm_dims or "flange_width" not in diaphragm_dims:
                        diaphragm_dims = {
                            "depth": eff_depth * 0.85,
                            "flange_width": 250,
                            "web_thickness": 12,
                            "flange_thickness": 20
                        }
                    frame = _diaphragm_bracing(
                                x_eff, yL, yR,
                                eff_depth, flange_thickness,
                                diaphragm_dims.get("web_thickness", 10),
                                end_diaphragm_section,
                                diaphragm_dims,
                                skew_angle=skew_angle
                    )
                    _collect(frame, z_offset, COMPONENT_END_DIAPHRAGM, bay_pair_id)
                
                continue
            
            # INTERNAL BRACING HANDLING
            # Build normal X or K bracing for internal panels using separate sections
            p_bracing_type = bracing_type
            p_diag_thick = diagonal_thickness
            p_diag_sec_type = diagonal_section_type
            p_diag_dims = diagonal_section_dims
            p_tc_thick = top_chord_thickness
            p_tc_sec_type = top_chord_section_type
            p_tc_dims = top_chord_section_dims
            p_bc_thick = bottom_chord_thickness
            p_bc_sec_type = bottom_chord_section_type
            p_bc_dims = bottom_chord_section_dims
            p_bracket_option = bracket_option
            p_top_bracket = top_bracket

            pair_key = f"G{i+1}G{i+2}"
            cb_suffix = f".{pair_key}.B{i+1}M1"

            if output_dict:
                # --- bracing type ---
                _bt = output_dict.get(KEY_MP_CB_TYPE + cb_suffix)
                if _bt is not None:
                    p_bracing_type = "K" if "K" in str(_bt).upper() else "X"

                # --- top / bottom chord enabled ---
                _tc_en = _is_enabled(output_dict.get(KEY_MP_CB_TOP_CHORD + cb_suffix))
                _bc_en = _is_enabled(output_dict.get(KEY_MP_CB_BOTTOM_CHORD + cb_suffix))

                # --- section types (UI keys, then designed output keys) ---
                _diag_st = (
                    _parse_sec_type(output_dict.get(KEY_MP_CB_BRACING_SECTION_TYPE + cb_suffix))
                )
                if _diag_st:
                    p_diag_sec_type = _diag_st

                _tc_st = (
                    _parse_sec_type(output_dict.get(KEY_MP_CB_TOP_CHORD_SECTION_TYPE + cb_suffix))
                )
                if _tc_st:
                    p_tc_sec_type = _tc_st

                _bc_st = (
                    _parse_sec_type(output_dict.get(KEY_MP_CB_BOTTOM_CHORD_SECTION_TYPE + cb_suffix))
                )
                if _bc_st:
                    p_bc_sec_type = _bc_st

                # --- section dimensions from designed output keys ---
                _d_lh = output_dict.get(_pk(KEY_MP_CB_DIAGONAL_LEG_H, pair_key))
                _d_lw = output_dict.get(_pk(KEY_MP_CB_DIAGONAL_LEG_W, pair_key))
                _d_tw = output_dict.get(_pk(KEY_MP_CB_DIAGONAL_THICKNESS, pair_key))
                if _d_lh is not None and _d_lw is not None:
                    p_diag_dims = {"leg_h": float(_d_lh), "leg_w": float(_d_lw), "connection_type": "LONGER_LEG"}
                if _d_tw is not None:
                    p_diag_thick = float(_d_tw)

                _tc_lh = output_dict.get(_pk(KEY_MP_CB_TOP_CHORD_LEG_H, pair_key))
                _tc_lw = output_dict.get(_pk(KEY_MP_CB_TOP_CHORD_LEG_W, pair_key))
                _tc_tw = output_dict.get(_pk(KEY_MP_CB_TOP_CHORD_THICKNESS, pair_key))
                if _tc_lh is not None and _tc_lw is not None:
                    p_tc_dims = {"leg_h": float(_tc_lh), "leg_w": float(_tc_lw), "connection_type": "LONGER_LEG"}
                if _tc_tw is not None:
                    p_tc_thick = float(_tc_tw)

                _bc_lh = output_dict.get(_pk(KEY_MP_CB_BOTTOM_CHORD_LEG_H, pair_key))
                _bc_lw = output_dict.get(_pk(KEY_MP_CB_BOTTOM_CHORD_LEG_W, pair_key))
                _bc_tw = output_dict.get(_pk(KEY_MP_CB_BOTTOM_CHORD_THICKNESS, pair_key))
                if _bc_lh is not None and _bc_lw is not None:
                    p_bc_dims = {"leg_h": float(_bc_lh), "leg_w": float(_bc_lw), "connection_type": "LONGER_LEG"}
                if _bc_tw is not None:
                    p_bc_thick = float(_bc_tw)

                # --- bracket configuration ---
                if _tc_en and _bc_en:
                    p_bracket_option = "BOTH"
                elif _tc_en:
                    p_bracket_option = "UPPER"
                elif _bc_en:
                    p_bracket_option = "LOWER"
                else:
                    p_bracket_option = "NONE"
                p_top_bracket = _tc_en

            if p_bracing_type == "X":
                frame = _x_bracing(
                    x, yL, yR,
                    eff_depth, flange_thickness, flange_width,
                    p_diag_thick, p_diag_sec_type, p_diag_dims,
                    p_tc_thick, p_tc_sec_type, p_tc_dims,
                    p_bc_thick, p_bc_sec_type, p_bc_dims,
                    p_bracket_option,
                    skew_angle=skew_angle
                )
                _collect(frame, z_offset, COMPONENT_CROSS_BRACING, bay_pair_id)
            
            elif p_bracing_type == "K":
                frame = _k_bracing(
                    x, yL, yR,
                    eff_depth, flange_thickness, flange_width,
                    p_diag_thick, p_diag_sec_type, p_diag_dims,
                    p_tc_thick, p_tc_sec_type, p_tc_dims,
                    p_bc_thick, p_bc_sec_type, p_bc_dims,
                    p_top_bracket,
                    skew_angle=skew_angle
                )
                _collect(frame, z_offset, COMPONENT_CROSS_BRACING, bay_pair_id)
    
    return bracings, bracing_groups