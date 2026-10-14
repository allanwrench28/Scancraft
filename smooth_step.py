import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile

import numpy as np
import trimesh
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Builder, BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_Copy, BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing,
)
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCC.Core.BRepTools import BRepTools_ReShape
from OCC.Core.Geom import Geom_BSplineSurface, Geom_Plane
from OCC.Core.GeomAbs import (
    GeomAbs_BSplineSurface, GeomAbs_Cylinder, GeomAbs_Plane,
)
from OCC.Core.GProp import GProp_GProps
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.Interface import Interface_Static
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.Core.ShapeFix import ShapeFix_Face, ShapeFix_Solid
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.STEPControl import (
    STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer,
)
from OCC.Core.TColgp import TColgp_Array2OfPnt
from OCC.Core.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal
from OCC.Core.TopAbs import (
    TopAbs_EDGE, TopAbs_FACE, TopAbs_FORWARD, TopAbs_REVERSED,
    TopAbs_SHELL, TopAbs_SOLID, TopAbs_WIRE,
)
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopTools import TopTools_ListOfShape
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Face, TopoDS_Shell, topods
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt


def count_shapes(shape, kind):
    explorer = TopExp_Explorer(shape, kind)
    count = 0
    while explorer.More():
        count += 1
        explorer.Next()
    return count


def read_step(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError("OpenCascade could not read the faceted STEP")
    if not reader.TransferRoots():
        raise ValueError("The STEP contains no importable geometry")
    shape = reader.OneShape()
    if shape.IsNull() or not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("The input STEP is not a valid B-rep")
    return shape


def volume(shape):
    properties = GProp_GProps()
    brepgprop.VolumeProperties(shape, properties)
    return properties.Mass()


def merge_planes(shape, tolerance=0.01):
    if not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("Cannot merge an invalid B-rep")
    best = shape
    face_count = count_shapes(shape, TopAbs_FACE)
    solid_count = count_shapes(shape, TopAbs_SOLID)
    original_volume = volume(shape)
    properties = GProp_GProps()
    brepgprop.SurfaceProperties(shape, properties)
    linear = min(1e-3, tolerance / 10)
    for angular in (1e-10, 1e-6, 1e-5, 1e-4, 1e-3):
        merger = ShapeUpgrade_UnifySameDomain(shape, True, True, False)
        merger.SetLinearTolerance(linear)
        merger.SetAngularTolerance(angular)
        merger.Build()
        merged = merger.Shape()
        if not BRepCheck_Analyzer(merged).IsValid():
            continue
        if count_shapes(merged, TopAbs_SOLID) != solid_count:
            continue
        if not math.isclose(
            volume(merged), original_volume, rel_tol=1e-6,
            abs_tol=properties.Mass() * linear,
        ):
            continue
        merged_faces = count_shapes(merged, TopAbs_FACE)
        if merged_faces < face_count:
            best, face_count = merged, merged_faces
    return best


def fit_cylinder(mesh, face_indices, axis):
    axis = axis / np.linalg.norm(axis)
    if axis[np.argmax(np.abs(axis))] < 0:
        axis = -axis
    reference = np.eye(3)[np.argmin(np.abs(axis))]
    horizontal = np.cross(reference, axis)
    horizontal /= np.linalg.norm(horizontal)
    vertical = np.cross(axis, horizontal)
    basis = np.column_stack((horizontal, vertical))
    points = mesh.vertices[np.unique(mesh.faces[face_indices])]
    local = points @ basis
    offset = local.mean(axis=0)
    centered = local - offset
    design = np.column_stack((2 * centered, np.ones(len(centered))))
    solution, _, rank, _ = np.linalg.lstsq(
        design, np.square(centered).sum(axis=1), rcond=None,
    )
    radius_squared = solution[2] + np.square(solution[:2]).sum()
    if rank != 3 or radius_squared <= 0:
        return None
    center = (solution[:2] + offset) @ basis.T
    return axis, basis, center, math.sqrt(radius_squared)


def cylinder_bands(mesh, tolerance):
    normals = mesh.face_normals
    neighbors = [[] for _ in mesh.faces]
    for first, second in mesh.face_adjacency:
        neighbors[first].append(second)
        neighbors[second].append(first)
    consumed = set()
    examined = set()
    bands = []
    fit_tolerance = min(tolerance * 0.1, 1e-3)

    for (first, second), angle in zip(
        mesh.face_adjacency, mesh.face_adjacency_angles,
    ):
        if first in consumed or second in consumed:
            continue
        if not math.radians(0.5) < angle < math.radians(20):
            continue
        fit = fit_cylinder(mesh, [first, second], np.cross(
            normals[first], normals[second]))
        if fit is None:
            continue
        selected = [first, second]
        for _ in range(3):
            axis, basis, center, radius = fit
            relative = mesh.vertices - center
            distances = np.linalg.norm(relative @ basis, axis=1)
            eligible = (
                (np.abs(distances[mesh.faces] -
                 radius).max(axis=1) <= fit_tolerance)
                & (np.abs(normals @ axis) <= 1e-3)
            )
            selected_set = set()
            pending = [first]
            while pending:
                current = pending.pop()
                if (current in selected_set or not eligible[current]
                        or current in consumed):
                    continue
                selected_set.add(current)
                pending.extend(neighbors[current])
            selected = sorted(selected_set)
            if len(selected) < 12:
                break
            _, eigenvectors = np.linalg.eigh(
                normals[selected].T @ normals[selected])
            fit = fit_cylinder(mesh, selected, eigenvectors[:, 0])
            if fit is None:
                break
        if fit is None or len(selected) < 12:
            continue
        identity = tuple(selected)
        if identity in examined:
            continue
        examined.add(identity)
        axis, basis, center, radius = fit
        point_indices = np.unique(mesh.faces[selected])
        points = mesh.vertices[point_indices]
        local = (points - center) @ basis
        radii = np.linalg.norm(local, axis=1)
        residual = np.abs(radii - radius).max()
        if residual > fit_tolerance:
            continue
        heights = points @ axis
        bottom, top = heights.min(), heights.max()
        height = top - bottom
        if height <= tolerance:
            continue
        edges = np.sort(np.concatenate([
            mesh.faces[selected][:, [0, 1]],
            mesh.faces[selected][:, [1, 2]],
            mesh.faces[selected][:, [2, 0]],
        ]), axis=1)
        unique_edges, counts = np.unique(edges, axis=0, return_counts=True)
        boundary = unique_edges[counts == 1]
        if not len(boundary):
            continue
        edge_heights = (mesh.vertices @ axis)[boundary]
        on_cap = (
            (np.abs(edge_heights - bottom) <= fit_tolerance).all(axis=1)
            | (np.abs(edge_heights - top) <= fit_tolerance).all(axis=1)
        )
        full_circle = np.all(on_cap)
        angles = np.mod(np.arctan2(local[:, 1], local[:, 0]), 2 * math.pi)
        sorted_angles = np.sort(angles)
        gaps = np.diff(
            np.append(sorted_angles, sorted_angles[0] + 2 * math.pi))
        gap_index = int(np.argmax(gaps))
        start = float(sorted_angles[(gap_index + 1) % len(sorted_angles)])
        sweep = 2 * math.pi if full_circle else 2 * math.pi - gaps[gap_index]
        if sweep < math.radians(15):
            continue
        all_local = (mesh.vertices - center) @ basis
        all_angles = np.arctan2(all_local[:, 1], all_local[:, 0])
        triangle_angles = all_angles[mesh.faces[selected]]
        unwrapped = triangle_angles[:, :1] + (
            triangle_angles - triangle_angles[:, :1] + math.pi
        ) % (2 * math.pi) - math.pi
        triangle_heights = (mesh.vertices @ axis)[mesh.faces[selected]]
        first_delta = unwrapped[:, 1] - unwrapped[:, 0]
        second_delta = unwrapped[:, 2] - unwrapped[:, 0]
        parameter_area = np.abs(
            first_delta * (triangle_heights[:, 2] - triangle_heights[:, 0])
            - second_delta * (triangle_heights[:, 1] - triangle_heights[:, 0])
        ).sum() / 2
        if not math.isclose(parameter_area, sweep * height,
                            rel_tol=1e-4, abs_tol=1e-7):
            continue
        if not full_circle:
            boundary_angles = np.mod(all_angles[boundary] - start, 2 * math.pi)
            endpoint_distance = np.minimum(
                np.minimum(boundary_angles, 2 * math.pi - boundary_angles),
                np.abs(boundary_angles - sweep),
            ) * radius
            on_side = (endpoint_distance <= fit_tolerance).all(axis=1)
            if not np.all(on_cap | on_side):
                continue
        maximum_angle = np.ptp(unwrapped, axis=1).max()
        deviation = float(residual + radius *
                          (1 - math.cos(maximum_angle / 2)))
        if deviation + 2 * fit_tolerance > tolerance:
            continue
        radial_normals = (
            (mesh.triangles_center[selected] - center) @ basis) @ basis.T
        orientation = np.einsum("ij,ij->i", normals[selected], radial_normals)
        if not (np.all(orientation > 0) or np.all(orientation < 0)):
            continue
        bands.append({
            "faces": selected, "axis": axis, "basis": basis,
            "center": center, "radius": float(radius),
            "bottom": float(bottom), "height": float(height),
            "start": start, "sweep": float(sweep),
            "outward": bool(orientation.mean() > 0),
            "deviation": deviation, "residual": float(residual),
            "angles": angles,
        })
        consumed.update(selected)
    return bands


def cylinder_face_count(shape):
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    count = 0
    while explorer.More():
        surface = BRepAdaptor_Surface(topods.Face(explorer.Current()))
        count += surface.GetType() == GeomAbs_Cylinder
        explorer.Next()
    return count


def boolean_shape(operation_type, original, tool, tolerance):
    arguments = TopTools_ListOfShape()
    arguments.Append(original)
    tools = TopTools_ListOfShape()
    tools.Append(tool)
    operation = operation_type()
    operation.SetArguments(arguments)
    operation.SetTools(tools)
    operation.SetFuzzyValue(tolerance)
    operation.Build()
    if not operation.IsDone():
        raise ValueError("Cylinder Boolean operation failed")
    return operation.Shape()


def cylinder_skin(band):
    axis = band["axis"]
    basis = band["basis"]
    start = band["start"]
    direction = basis[:, 0] * math.cos(start) + basis[:, 1] * math.sin(start)
    origin = band["center"] + axis * band["bottom"]
    placement = gp_Ax2(gp_Pnt(*origin), gp_Dir(*axis), gp_Dir(*direction))
    radius = band["radius"] + band["residual"] + 1e-6
    thickness = max(2 * band["deviation"], 1e-4)
    if radius <= thickness:
        raise ValueError("Cylinder is too small for reliable reconstruction")
    sweep = band["sweep"]
    dimensions = [band["height"]]
    if sweep < 2 * math.pi - 1e-8:
        dimensions.append(sweep)
    outer = BRepPrimAPI_MakeCylinder(placement, radius, *dimensions).Shape()
    inner = BRepPrimAPI_MakeCylinder(
        placement, radius - thickness, *dimensions).Shape()
    return boolean_shape(BRepAlgoAPI_Cut, outer, inner, 1e-7)


def reconstruct_cylinders(shape, mesh, tolerance):
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError(
            "Fit tolerance must be a positive finite distance in millimetres")
    bands = cylinder_bands(mesh, tolerance)
    recovered = []
    skipped = []
    for band in bands:
        try:
            tool = cylinder_skin(band)
            operation = (BRepAlgoAPI_Fuse if band["outward"]
                         else BRepAlgoAPI_Cut)
            candidate = boolean_shape(
                operation, shape, tool, min(1e-5, tolerance * 0.001))
            candidate = merge_planes(candidate, tolerance)
            if count_shapes(candidate, TopAbs_FACE) >= count_shapes(
                    shape, TopAbs_FACE):
                raise ValueError(
                    "Cylinder reconstruction did not reduce face count")
            if count_shapes(candidate, TopAbs_SOLID) != count_shapes(
                    shape, TopAbs_SOLID):
                raise ValueError(
                    "Cylinder reconstruction changed the number of bodies")
            change = volume(candidate) - volume(shape)
            maximum_change = (
                band["radius"] * band["sweep"] * band["height"] * tolerance
            )
            expected_sign = 1 if band["outward"] else -1
            if (abs(change) > maximum_change
                    or change * expected_sign < -1e-6):
                raise ValueError(
                    "Cylinder reconstruction exceeded its volume-change bound")
            if cylinder_face_count(candidate) <= cylinder_face_count(shape):
                raise ValueError(
                    "Cylinder reconstruction did not create an analytic face")
            shape = candidate
            recovered.append({
                "radius_mm": band["radius"],
                "height_mm": band["height"],
                "sweep_degrees": math.degrees(band["sweep"]),
                "type": "outer" if band["outward"] else "hole",
                "replaced_triangles": len(band["faces"]),
                "estimated_deviation_mm": band["deviation"],
            })
        except (ValueError, RuntimeError) as error:
            skipped.append(str(error))
    return shape, recovered, skipped


def subshapes(shape, kind):
    result = []
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        result.append(explorer.Current())
        explorer.Next()
    return result


def curved_face_groups(faces):
    normals, owners = {}, {}
    for index, face in enumerate(faces):
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() != GeomAbs_Plane:
            continue
        normal = np.array(surface.Plane().Axis().Direction().Coord())
        if face.Orientation() == TopAbs_REVERSED:
            normal = -normal
        normals[index] = normal
        for edge in subshapes(face, TopAbs_EDGE):
            owners.setdefault(edge.Oriented(TopAbs_FORWARD), []).append(index)
    neighbors = {index: [] for index in normals}
    for adjacent in owners.values():
        if len(adjacent) != 2:
            continue
        first, second = adjacent
        if normals[first] @ normals[second] > math.cos(math.radians(8)):
            neighbors[first].append(second)
            neighbors[second].append(first)
    visited, groups = set(), []
    for seed in normals:
        pending, selected = [seed], []
        while pending:
            current = pending.pop()
            if current in visited:
                continue
            if normals[current] @ normals[seed] < 0.65:
                continue
            visited.add(current)
            selected.append(current)
            pending.extend(neighbors[current])
        if len(selected) >= 6:
            groups.append(selected)
    return groups


def patch_mesh(faces, tolerance):
    vertices, triangles = [], []
    for face in faces:
        BRepMesh_IncrementalMesh(face, tolerance / 10, False, 0.1, False)
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation(face, location)
        if triangulation is None:
            raise ValueError("Could not sample a curved patch")
        offset = len(vertices)
        transform = location.Transformation()
        for index in range(1, triangulation.NbNodes() + 1):
            vertices.append(
                triangulation.Node(index).Transformed(transform).Coord())
        for index in range(1, triangulation.NbTriangles() + 1):
            triangle = [offset + vertex - 1 for vertex in
                        triangulation.Triangle(index).Get()]
            if face.Orientation() == TopAbs_REVERSED:
                triangle.reverse()
            triangles.append(triangle)
    return trimesh.Trimesh(vertices=vertices, faces=triangles, process=False)


def bernstein_weights(parameters, degree_u, degree_v):
    weights = []
    for axis, degree in enumerate((degree_u, degree_v)):
        values = parameters[:, axis]
        weights.append(np.column_stack([
            math.comb(degree, index) * values ** index
            * (1 - values) ** (degree - index)
            for index in range(degree + 1)
        ]))
    return (weights[0][:, :, None] * weights[1][:, None, :]).reshape(
        len(parameters), -1,
    )


def fit_patch_surface(mesh, tolerance):
    points = np.unique(mesh.vertices, axis=0)
    origin = points.mean(axis=0)
    normal = (mesh.face_normals * mesh.area_faces[:, None]).sum(axis=0)
    normal /= np.linalg.norm(normal)
    if np.min(mesh.face_normals @ normal) < 0.5:
        raise ValueError("Curved patch folds over its fitting plane")
    reference = np.eye(3)[np.argmin(np.abs(normal))]
    horizontal = np.cross(reference, normal)
    horizontal /= np.linalg.norm(horizontal)
    basis = np.column_stack((horizontal, np.cross(normal, horizontal), normal))
    local = (points - origin) @ basis
    plane_deviation = float(np.max(np.abs(local[:, 2])))
    if plane_deviation <= min(1e-4, tolerance / 10):
        return Geom_Plane(gp_Pnt(*origin), gp_Dir(*normal)), plane_deviation
    span = np.ptp(local[:, :2], axis=0)
    if np.min(span) <= tolerance:
        raise ValueError("Curved patch is too narrow for spline fitting")
    lower = local[:, :2].min(axis=0) - span * 0.01
    span *= 1.02
    parameters = (local[:, :2] - lower) / span
    samples = np.concatenate([
        mesh.triangles[:, 0] * (first / 8)
        + mesh.triangles[:, 1] * (second / 8)
        + mesh.triangles[:, 2] * (1 - (first + second) / 8)
        for first in range(9) for second in range(9 - first)
    ])
    sample_local = (samples - origin) @ basis
    sample_parameters = (sample_local[:, :2] - lower) / span
    degrees = sorted(
        [(degree_u, degree_v) for degree_u in range(1, 6)
         for degree_v in range(1, 6) if degree_u + degree_v > 2],
        key=lambda pair: ((pair[0] + 1) * (pair[1] + 1), sum(pair)),
    )
    for degree_u, degree_v in degrees:
        weights = bernstein_weights(parameters, degree_u, degree_v)
        heights, _, rank, singular = np.linalg.lstsq(
            weights, local[:, 2], rcond=None,
        )
        if rank != weights.shape[1] or singular[-1] < singular[0] * 1e-8:
            continue
        predicted = bernstein_weights(
            sample_parameters, degree_u, degree_v,
        ) @ heights
        deviation = float(np.max(np.abs(predicted - sample_local[:, 2])))
        if deviation > tolerance / 2:
            continue
        poles = TColgp_Array2OfPnt(1, degree_u + 1, 1, degree_v + 1)
        heights = heights.reshape(degree_u + 1, degree_v + 1)
        for row in range(degree_u + 1):
            for column in range(degree_v + 1):
                position = origin + basis @ np.array([
                    lower[0] + span[0] * row / degree_u,
                    lower[1] + span[1] * column / degree_v,
                    heights[row, column],
                ])
                poles.SetValue(row + 1, column + 1, gp_Pnt(*position))
        knots = TColStd_Array1OfReal(1, 2)
        knots.SetValue(1, 0.0)
        knots.SetValue(2, 1.0)
        multiplicities = []
        for degree in (degree_u, degree_v):
            values = TColStd_Array1OfInteger(1, 2)
            values.Init(degree + 1)
            multiplicities.append(values)
        surface = Geom_BSplineSurface(
            poles, knots, knots, *multiplicities, degree_u, degree_v,
        )
        return surface, deviation
    raise ValueError("Curved patch exceeds the spline fit tolerance")


def trimmed_spline_face(faces, surface, tolerance):
    builder = BRep_Builder()
    shell = TopoDS_Shell()
    builder.MakeShell(shell)
    for face in faces:
        builder.Add(shell, face)
    copied = BRepBuilderAPI_Copy(shell).Shape()
    bounds = ShapeAnalysis_FreeBounds(copied)
    wires = subshapes(bounds.GetClosedWires(), TopAbs_WIRE)
    if not wires or count_shapes(bounds.GetOpenWires(), TopAbs_WIRE):
        raise ValueError("Curved patch does not have closed trimming loops")
    face = TopoDS_Face()
    builder.MakeFace(face, surface, tolerance / 2)
    for wire in wires:
        builder.Add(face, wire)
    fixer = ShapeFix_Face(face)
    fixer.SetPrecision(tolerance / 2)
    fixer.SetMaxTolerance(tolerance / 2)
    fixer.Perform()
    result = fixer.Face()
    if not BRepCheck_Analyzer(result).IsValid():
        raise ValueError("Spline trimming did not produce a valid face")
    return result


def sew_solid(faces, tolerance):
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for face in faces:
        builder.Add(compound, face)
    copied = BRepBuilderAPI_Copy(compound).Shape()
    sewing = BRepBuilderAPI_Sewing(tolerance / 2)
    for face in subshapes(copied, TopAbs_FACE):
        sewing.Add(face)
    sewing.Perform()
    shells = subshapes(sewing.SewedShape(), TopAbs_SHELL)
    if not shells or sewing.NbFreeEdges():
        raise ValueError("Spline replacement left an open shell")
    solid = BRepBuilderAPI_MakeSolid()
    for shell in shells:
        solid.Add(topods.Shell(shell))
    fixer = ShapeFix_Solid(solid.Solid())
    fixer.SetPrecision(tolerance / 2)
    fixer.SetMaxTolerance(tolerance / 2)
    fixer.Perform()
    result = fixer.Solid()
    if not BRepCheck_Analyzer(result).IsValid():
        raise ValueError("Spline replacement did not form a valid solid")
    return result


def reconstruct_splines(shape, tolerance):
    faces = [topods.Face(face) for face in subshapes(shape, TopAbs_FACE)]
    remaining = dict(enumerate(faces))
    recovered, skipped = [], []
    original_volume = volume(shape)
    properties = GProp_GProps()
    brepgprop.SurfaceProperties(shape, properties)
    bounds = Bnd_Box()
    brepbndlib.AddOptimal(shape, bounds, False, False)
    original_bounds = np.array(bounds.Get())
    for selected in curved_face_groups(faces):
        try:
            patch = [faces[index] for index in selected]
            surface, deviation = fit_patch_surface(
                patch_mesh(patch, tolerance), tolerance,
            )
            replacement = trimmed_spline_face(patch, surface, tolerance)
            retained = {index: face for index, face in remaining.items()
                        if index not in selected}
            retained[selected[0]] = replacement
            candidate = sew_solid(list(retained.values()), tolerance)
            if count_shapes(candidate, TopAbs_SOLID) != 1:
                raise ValueError("Spline replacement changed body count")
            if count_shapes(candidate, TopAbs_SHELL) != count_shapes(
                    shape, TopAbs_SHELL):
                raise ValueError("Spline replacement changed shell count")
            if count_shapes(candidate, TopAbs_FACE) >= count_shapes(
                    shape, TopAbs_FACE):
                raise ValueError("Spline replacement did not reduce faces")
            if abs(volume(candidate) - original_volume) > (
                    properties.Mass() * tolerance / 2):
                raise ValueError("Spline replacement changed too much volume")
            candidate_bounds = Bnd_Box()
            brepbndlib.AddOptimal(candidate, candidate_bounds, False, False)
            if np.max(np.abs(np.array(candidate_bounds.Get())
                             - original_bounds)) > tolerance:
                raise ValueError("Spline replacement changed body dimensions")
            shape, remaining = candidate, retained
            recovered.append({
                "type": "plane" if isinstance(surface, Geom_Plane)
                else "bspline",
                "replaced_faces": len(selected),
                "sampled_fit_deviation_mm": deviation,
            })
        except (ValueError, RuntimeError) as error:
            skipped.append(str(error))
    return shape, recovered, skipped


def reconstruct_components(shape, mesh, tolerance):
    parts = mesh.split(only_watertight=False)
    replacements = BRepTools_ReShape()
    recovered, skipped = [], []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        solid = topods.Solid(explorer.Current())
        bounds = Bnd_Box()
        brepbndlib.AddOptimal(solid, bounds, False, False)
        matches = [
            part for part in parts
            if part.is_watertight and np.allclose(
                part.bounds.flatten(), bounds.Get(), atol=1e-5, rtol=0,
            )
        ]
        if len(matches) == 1:
            part = matches[0]
            part.fix_normals(multibody=False)
            result, bands, failures = reconstruct_cylinders(
                solid, part, tolerance,
            )
            result, patches, patch_failures = reconstruct_splines(
                result, tolerance,
            )
            result_solids = TopExp_Explorer(result, TopAbs_SOLID)
            replacements.Replace(solid, result_solids.Current())
            recovered.extend(bands)
            recovered.extend(patches)
            skipped.extend(failures)
            skipped.extend(patch_failures)
        else:
            skipped.append("Kept a body whose source component was ambiguous")
        explorer.Next()
    result = replacements.Apply(shape)
    if not BRepCheck_Analyzer(result).IsValid():
        raise ValueError("Reconstructed components have invalid topology")
    if count_shapes(result, TopAbs_SOLID) != count_shapes(shape, TopAbs_SOLID):
        raise ValueError("Reconstruction changed the number of components")
    return result, recovered, skipped


def write_step(shape, output):
    output = Path(output)
    with tempfile.NamedTemporaryFile(
        dir=output.parent, prefix=".scancraft-", suffix=".step", delete=False,
    ) as temporary:
        temporary_path = Path(temporary.name)
    try:
        Interface_Static.SetCVal("write.step.schema", "AP214IS")
        writer = STEPControl_Writer()
        if writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone:
            raise ValueError("OpenCascade could not transfer the smooth B-rep")
        if writer.Write(str(temporary_path)) != IFSelect_RetDone:
            raise ValueError("OpenCascade could not write the smooth STEP")
        reimported = read_step(temporary_path)
        if count_shapes(reimported, TopAbs_SOLID) != count_shapes(
                shape, TopAbs_SOLID):
            raise ValueError(
                "STEP round-trip changed the number of solid bodies")
        os.replace(temporary_path, output)
    finally:
        temporary_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(
        description="Recover planes, cylinders and spline patches with OCC",
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stl", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.01)
    args = parser.parse_args()
    if not math.isfinite(args.tolerance) or args.tolerance <= 0:
        parser.error("--tolerance must be positive and finite")
    original = read_step(args.input)
    merged = merge_planes(original, args.tolerance)
    recovered, skipped = [], []
    if args.stl:
        mesh = trimesh.load_mesh(str(args.stl))
        merged, recovered, skipped = reconstruct_components(
            merged, mesh, args.tolerance,
        )
    write_step(merged, args.output)
    surface_types = [
        BRepAdaptor_Surface(topods.Face(face)).GetType()
        for face in subshapes(merged, TopAbs_FACE)
    ]
    print(json.dumps({
        "faces_before": count_shapes(original, TopAbs_FACE),
        "faces_after": count_shapes(merged, TopAbs_FACE),
        "solids": count_shapes(merged, TopAbs_SOLID),
        "cylindrical_faces": cylinder_face_count(merged),
        "bspline_faces": surface_types.count(GeomAbs_BSplineSurface),
        "planar_faces": surface_types.count(GeomAbs_Plane),
        "fit_tolerance_mm": args.tolerance,
        "reconstructed_bands": recovered,
        "skipped_bands": skipped,
        "volume_mm3": volume(merged),
        "valid_brep": True,
        "file": str(args.output),
    }, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Smoothing failed: {error}", file=sys.stderr)
        sys.exit(1)
