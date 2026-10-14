import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import trimesh
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GeomAbs import (
    GeomAbs_BSplineSurface, GeomAbs_Cylinder, GeomAbs_Plane,
)
from OCC.Core.GProp import GProp_GProps
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_WIRE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import topods


def count_shapes(shape, kind):
    explorer = TopExp_Explorer(shape, kind)
    count = 0
    while explorer.More():
        count += 1
        explorer.Next()
    return count


def tessellate_shape(shape, deflection):
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.1, True)
    points, triangles = [], []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation(face, location)
        if triangulation is None:
            raise ValueError("A STEP face could not be tessellated")
        offset = len(points)
        transform = location.Transformation()
        for index in range(1, triangulation.NbNodes() + 1):
            points.append(
                triangulation.Node(index).Transformed(transform).Coord()
            )
        for index in range(1, triangulation.NbTriangles() + 1):
            triangles.append([
                offset + vertex - 1
                for vertex in triangulation.Triangle(index).Get()
            ])
        explorer.Next()
    return trimesh.Trimesh(vertices=points, faces=triangles, process=False)


def sampled_distance(source, target):
    samples = np.unique(np.vstack((
        source.vertices, source.triangles_center,
    )), axis=0)
    maximum = 0.0
    for start in range(0, len(samples), 32):
        _, distances, _ = trimesh.proximity.closest_point_naive(
            target, samples[start:start + 32],
        )
        maximum = max(maximum, float(distances.max()))
    return maximum


def validate_step(source, output, smooth_tolerance=None):
    mesh = trimesh.load_mesh(str(source))
    reader = STEPControl_Reader()
    if reader.ReadFile(str(output)) != IFSelect_RetDone:
        raise ValueError("OpenCascade could not read the STEP file")
    roots = reader.TransferRoots()
    shape = reader.OneShape()
    if not roots or shape is None or shape.IsNull():
        raise ValueError("The STEP file contains no importable geometry")
    if not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("OpenCascade reports invalid B-rep topology")

    faces = count_shapes(shape, TopAbs_FACE)
    solids = count_shapes(shape, TopAbs_SOLID)
    if smooth_tolerance is None and faces != len(mesh.faces):
        raise ValueError(f"Expected {len(mesh.faces)} faces, imported {faces}")
    if smooth_tolerance is not None and faces > len(mesh.faces):
        raise ValueError("Smoothing increased the total face count")

    bounds = Bnd_Box()
    brepbndlib.AddOptimal(shape, bounds, False, False)
    actual_bounds = bounds.Get()
    for actual, expected in zip(actual_bounds, mesh.bounds.flatten()):
        if not math.isclose(
            actual, expected, abs_tol=smooth_tolerance or 1e-5, rel_tol=0,
        ):
            raise ValueError("STEP dimensions do not match the source STL")

    properties = GProp_GProps()
    brepgprop.VolumeProperties(shape, properties)
    volume = properties.Mass()
    components = mesh.split(only_watertight=False)
    expected_solids = sum(part.is_watertight for part in components)
    if solids != expected_solids:
        raise ValueError(f"Expected {expected_solids} solids, got {solids}")
    if mesh.is_watertight:
        expected_volume = sum(abs(part.volume) for part in components)
        if not math.isclose(
            volume, expected_volume, rel_tol=1e-7,
            abs_tol=mesh.area * (smooth_tolerance or 0),
        ):
            raise ValueError("STEP volume does not match the source STL")

    cylinder_faces = 0
    spline_faces = 0
    planar_faces = 0
    spline_faces_with_holes = 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
        surface = BRepAdaptor_Surface(face)
        cylinder_faces += surface.GetType() == GeomAbs_Cylinder
        spline_faces += surface.GetType() == GeomAbs_BSplineSurface
        planar_faces += surface.GetType() == GeomAbs_Plane
        if surface.GetType() == GeomAbs_BSplineSurface:
            spline_faces_with_holes += count_shapes(face, TopAbs_WIRE) > 1
        explorer.Next()
    deviation = None
    if smooth_tolerance is not None:
        tessellation = tessellate_shape(shape, smooth_tolerance / 10)
        deviation = max(
            sampled_distance(mesh, tessellation),
            sampled_distance(tessellation, mesh),
        )
        if deviation > smooth_tolerance:
            raise ValueError(
                f"Sampled surface deviation {deviation:.6g} mm exceeds "
                f"{smooth_tolerance:.6g} mm"
            )

    return {
        "file": str(output),
        "valid_brep": True,
        "transferred_roots": roots,
        "solids": solids,
        "faces": faces,
        "cylindrical_faces": cylinder_faces,
        "bspline_faces": spline_faces,
        "planar_faces": planar_faces,
        "bspline_faces_with_holes": spline_faces_with_holes,
        "sampled_surface_deviation_mm": deviation,
        "volume_mm3": volume if mesh.is_watertight else None,
        "bounds_mm": actual_bounds,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Convert binary STL with ScanCraft and validate using OCC"
    )
    parser.add_argument("stl", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--release", action="store_true")
    parser.add_argument("--smooth", action="store_true")
    parser.add_argument("--fit-tolerance", type=float, default=0.01)
    args = parser.parse_args()
    if not math.isfinite(args.fit_tolerance) or args.fit_tolerance <= 0:
        parser.error("--fit-tolerance must be positive and finite")
    source = args.stl.resolve()
    output = (
        args.output.resolve() if args.output
        else source.with_name(source.stem + "_fixed.step")
    )
    if not args.check_only:
        command = ["cargo", "run", "--quiet", "-p", "scancraft-kernel"]
        if args.release:
            command.append("--release")
        command.extend([
            "--", "mesh-to-nurbs", str(source), "--output", str(output)
        ])
        if args.smooth:
            command.extend([
                "--smooth", "--fit-tolerance", str(args.fit_tolerance),
                "--python", sys.executable,
            ])
        result = subprocess.run(
            command, cwd=Path(__file__).resolve().parent,
            capture_output=True, text=True,
        )
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        print(result.stdout.strip())
    print(json.dumps(validate_step(
        source, output, args.fit_tolerance if args.smooth else None,
    ), indent=2))


class SmoothStepTests(unittest.TestCase):
    @staticmethod
    def curved_panel():
        mesh = trimesh.creation.box(extents=[10, 8, 4])
        for _ in range(3):
            mesh = mesh.subdivide()
        top = mesh.vertices[:, 2] > 1.99
        points = mesh.vertices[top]
        mesh.vertices[top, 2] += (
            0.008 * points[:, 0] ** 2 + 0.004 * points[:, 1] ** 2
        )
        return mesh

    def convert(self, mesh, tolerance=0.01):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        source = Path(temporary.name) / "part.stl"
        output = Path(temporary.name) / "part.step"
        mesh.export(str(source))
        result = subprocess.run([
            "cargo", "run", "--quiet", "-p", "scancraft-kernel", "--",
            "mesh-to-nurbs", str(source), "--output", str(output),
            "--smooth", "--fit-tolerance", str(tolerance),
            "--python", sys.executable,
        ], cwd=Path(__file__).resolve().parent, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr[-4000:])
        return validate_step(source, output, tolerance)

    def test_tube_has_two_cylinders_and_two_caps(self):
        mesh = trimesh.creation.annulus(
            r_min=2, r_max=3, height=8, sections=64,
        )
        report = self.convert(mesh)
        self.assertEqual(report["solids"], 1)
        self.assertEqual(report["cylindrical_faces"], 2)
        self.assertEqual(report["faces"], 4)

    def test_tilted_reversed_cylinder_is_preserved(self):
        mesh = trimesh.creation.cylinder(radius=3, height=8, sections=64)
        mesh.apply_transform(trimesh.transformations.rotation_matrix(
            0.63, [1, 2, 3],
        ))
        mesh.faces = mesh.faces[:, ::-1]
        report = self.convert(mesh)
        self.assertEqual(report["solids"], 1)
        self.assertEqual(report["cylindrical_faces"], 1)

    def test_rounded_planar_triangles_are_merged(self):
        mesh = trimesh.creation.box(extents=[10, 8, 4]).subdivide()
        center = np.linalg.norm(mesh.vertices - [0, 0, 2], axis=1).argmin()
        mesh.vertices[center, 2] += 0.0002
        mesh.apply_transform(trimesh.transformations.rotation_matrix(
            0.63, [1, 2, 3],
        ))
        report = self.convert(mesh)
        self.assertEqual(report["solids"], 1)
        self.assertEqual(report["faces"], 6)
        self.assertEqual(report["planar_faces"], 6)

    def test_curved_panel_becomes_one_spline_face(self):
        report = self.convert(self.curved_panel())
        self.assertEqual(report["solids"], 1)
        self.assertEqual(report["bspline_faces"], 1)
        self.assertEqual(report["faces"], 6)

    def test_strict_tolerance_keeps_curved_panel_facets(self):
        report = self.convert(self.curved_panel(), 0.0001)
        self.assertEqual(report["bspline_faces"], 0)
        self.assertGreater(report["faces"], 6)

    def test_curved_panel_preserves_its_hole(self):
        mesh = trimesh.creation.annulus(
            r_min=2, r_max=3, height=8, sections=64,
        )
        top = mesh.vertices[:, 2] > 3.99
        points = mesh.vertices[top]
        mesh.vertices[top, 2] += (
            0.008 * points[:, 0] ** 2 + 0.004 * points[:, 1] ** 2
        )
        report = self.convert(mesh)
        self.assertEqual(report["solids"], 1)
        self.assertEqual(report["bspline_faces_with_holes"], 1)

    def test_strict_tolerance_keeps_coarse_cylinder_facets(self):
        mesh = trimesh.creation.cylinder(radius=3, height=8, sections=16)
        report = self.convert(mesh, 0.0001)
        self.assertEqual(report["cylindrical_faces"], 0)
        self.assertEqual(report["faces"], 18)

    def test_disconnected_bodies_remain_separate(self):
        cylinder = trimesh.creation.cylinder(radius=3, height=8, sections=64)
        box = trimesh.creation.box()
        box.apply_translation([10, 0, 0])
        report = self.convert(trimesh.util.concatenate([cylinder, box]))
        self.assertEqual(report["solids"], 2)
        self.assertEqual(report["cylindrical_faces"], 1)
        self.assertEqual(report["faces"], 9)

    def test_missing_python_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "part.stl"
            output = Path(temporary) / "existing.step"
            trimesh.creation.box().export(str(source))
            output.write_text("existing output", encoding="ascii")
            result = subprocess.run([
                "cargo", "run", "--quiet", "-p", "scancraft-kernel", "--",
                "mesh-to-nurbs", str(source), "--output", str(output),
                "--smooth", "--python", str(Path(temporary) / "missing.exe"),
            ], cwd=Path(__file__).resolve().parent,
                capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_text(), "existing output")

    def test_open_cylinder_remains_a_surface_body(self):
        cylinder = trimesh.creation.cylinder(radius=3, height=8, sections=64)
        side_faces = np.flatnonzero(np.abs(cylinder.face_normals[:, 2]) < 0.5)
        surface = cylinder.submesh([side_faces], append=True)
        report = self.convert(surface)
        self.assertEqual(report["solids"], 0)
        self.assertEqual(report["cylindrical_faces"], 0)
        self.assertEqual(report["faces"], 64)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Validation failed: {error}", file=sys.stderr)
        sys.exit(1)
