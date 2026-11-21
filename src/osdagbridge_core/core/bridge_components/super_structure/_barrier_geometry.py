"""Shared OCCT construction for metallic barriers and railings."""

import math

from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeWire,
)
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCC.Core.Geom import Geom_BSplineCurve
from OCC.Core.GeomAPI import GeomAPI_PointsToBSpline
from OCC.Core.TColgp import TColgp_Array1OfPnt
from OCC.Core.TopTools import TopTools_ListOfShape
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.gp import gp_Dir, gp_Pln, gp_Pnt, gp_Vec


def _assemble_parts(parts, *, separated):
    if not parts:
        return None
    if len(parts) == 1:
        return parts[0]
    if separated:
        builder = BRep_Builder()
        compound = TopoDS_Compound()
        builder.MakeCompound(compound)
        for part in parts:
            builder.Add(compound, part)
        return compound

    arguments = TopTools_ListOfShape()
    arguments.Append(parts[0])
    tools = TopTools_ListOfShape()
    for part in parts[1:]:
        tools.Append(part)
    fuse = BRepAlgoAPI_Fuse()
    fuse.SetArguments(arguments)
    fuse.SetTools(tools)
    fuse.SetNonDestructive(True)
    fuse.Build()
    if not fuse.IsDone() or fuse.Shape().IsNull():
        raise ValueError("Could not join the barrier or railing parts")
    return fuse.Shape()


def _create_w_beam(length, height, thickness, amplitude, skew):
    """Extrude a W-profile with its original transverse thickness."""
    point_count = 40
    points = TColgp_Array1OfPnt(1, point_count)
    sigma = height / 10.0
    for i in range(point_count):
        z = height * i / (point_count - 1)
        y = amplitude * (
            math.exp(-((z - height * 0.25) ** 2) / (2 * sigma ** 2))
            + math.exp(-((z - height * 0.75) ** 2) / (2 * sigma ** 2))
        )
        points.SetValue(i + 1, gp_Pnt(0, y, z))

    fit = GeomAPI_PointsToBSpline(points)
    if not fit.IsDone():
        raise ValueError("Could not fit the W-beam profile")
    outer = fit.Curve()
    # Reuse the fitted knots and translate the poles to form the inner curve.
    inner = Geom_BSplineCurve(outer.Poles(), outer.Knots(), outer.Multiplicities(), outer.Degree())
    for curve, offset in ((outer, 0), (inner, -thickness)):
        for i in range(1, curve.NbPoles() + 1):
            pole = curve.Pole(i)
            y = pole.Y() + offset
            curve.SetPole(i, gp_Pnt(skew * y, y, pole.Z()))

    outer_start, outer_end = outer.StartPoint(), outer.EndPoint()
    inner_start, inner_end = inner.StartPoint(), inner.EndPoint()
    wire = BRepBuilderAPI_MakeWire()
    wire.Add(BRepBuilderAPI_MakeEdge(outer).Edge())
    wire.Add(BRepBuilderAPI_MakeEdge(outer_end, inner_end).Edge())
    wire.Add(BRepBuilderAPI_MakeEdge(inner).Edge())
    wire.Add(BRepBuilderAPI_MakeEdge(inner_start, outer_start).Edge())
    if not wire.IsDone():
        raise ValueError("Could not close the W-beam profile")
    plane = gp_Pln(gp_Pnt(), gp_Dir(1, -skew, 0))
    face = BRepBuilderAPI_MakeFace(plane, wire.Wire(), True)
    if not face.IsDone():
        raise ValueError("Could not build the W-beam face")
    prism = BRepPrimAPI_MakePrism(face.Face(), gp_Vec(length, 0, 0))
    if not prism.IsDone():
        raise ValueError("Could not extrude the W-beam profile")
    return prism.Shape()
