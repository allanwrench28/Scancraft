import io
import json
import os
from pathlib import Path
import struct
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from scancraft_gui import (
    Application,
    conversion_command,
    conversion_error,
    create_server,
    export_options,
    inspect_stl,
)


def tetrahedron_stl(duplicate=False):
    points = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
    faces = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]
    if duplicate:
        faces.append(faces[0])
    return (
        bytes(80)
        + struct.pack("<I", len(faces))
        + b"".join(
            struct.pack(
                "<12fH",
                0,
                0,
                0,
                *(
                    coordinate
                    for index in face
                    for coordinate in points[index]
                ),
                0,
            )
            for face in faces
        )
    )


class ExportSettingsTests(unittest.TestCase):
    def test_native_bridge_exposes_only_save_and_reveal(self):
        from scancraft_gui import DesktopAPI

        bridge = DesktopAPI(None)
        self.assertEqual(
            sorted(name for name in dir(bridge) if not name.startswith("_")),
            ["reveal_export", "save_export"],
        )

    def test_exact_export_keeps_paths_with_spaces_as_single_arguments(self):
        command = conversion_command(
            "converter.exe",
            "Just hand.stl",
            "Just hand.step",
        )
        self.assertEqual(
            command,
            [
                "converter.exe",
                "mesh-to-nurbs",
                "Just hand.stl",
                "--output",
                "Just hand.step",
            ],
        )

    def test_smoothing_passes_the_selected_tolerance_and_python(self):
        command = conversion_command(
            "converter.exe",
            "part.stl",
            "part.step",
            True,
            0.01,
            "CAD runtime/python.exe",
        )
        self.assertEqual(
            command[-5:],
            [
                "--smooth",
                "--fit-tolerance",
                "0.01",
                "--python",
                "CAD runtime/python.exe",
            ],
        )

    def test_export_options_validate_settings(self):
        self.assertEqual(
            export_options({"name": "My part", "smooth": True}),
            (True, 0.01, "My part.step"),
        )
        for payload in [
            {"tolerance": 0},
            {"tolerance": "nan"},
            {"tolerance": "oops"},
            {"name": "../part.step"},
            {"name": "C:\\part.step"},
            {"smooth": "false"},
            {"name": "CON.step"},
            [],
        ]:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                export_options(payload)

    def test_invalid_topology_has_an_actionable_error(self):
        message = conversion_error(
            'Error: "Mesh has non-manifold edges or inconsistent triangle'
            ' winding"',
        )
        self.assertIn("needs repair", message)
        self.assertIn("No STEP was exported", message)


class ImportTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "nt", "Requires Windows mesh repair")
    def test_import_repairs_invalid_mesh_before_enabling_export(self):
        application = Application(probe=False)
        self.addCleanup(application.close)
        content = tetrahedron_stl(duplicate=True)
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "Needs repair.stl"
            source.write_bytes(content)
            with source.open("rb") as stream:
                mesh = application.import_mesh(
                    stream, len(content), source.name,
                )
            self.assertTrue(mesh["exportable"], mesh.get("repair"))
            self.assertEqual(mesh["topology"], "closed")
            self.assertEqual(mesh["repair"]["status"], "repaired")
            repaired = application.directory / f"{mesh['id']}.stl"
            self.assertEqual(inspect_stl(repaired)["topology"], "closed")
            self.assertEqual(source.read_bytes(), content)
            job = application.start_export({
                "mesh_id": mesh["id"], "name": "repaired.step",
                "smooth": False,
            })
            application.threads[-1].join(timeout=30)
            exported = application.jobs[job["id"]]
            self.assertEqual(exported.status, "complete", exported.error)
            from test_gen import validate_step

            report = validate_step(repaired, exported.output)
            self.assertEqual(report["solids"], 1)
            self.assertTrue(report["valid_brep"])

    @unittest.skipUnless(os.name == "nt", "Requires Windows mesh repair")
    def test_open_mesh_is_repaired_on_import(self):
        application = Application(probe=False)
        self.addCleanup(application.close)
        original = tetrahedron_stl()
        content = original[:80] + struct.pack("<I", 3) + original[84:-50]
        mesh = application.import_mesh(
            io.BytesIO(content), len(content), "Open tetrahedron.stl",
        )
        self.assertEqual(mesh["topology"], "closed", mesh.get("repair"))
        self.assertTrue(mesh["exportable"])
        self.assertEqual(mesh["repair"]["status"], "repaired")

    def test_valid_mesh_is_not_sent_for_repair(self):
        application = Application(converter="missing-converter", probe=False)
        self.addCleanup(application.close)
        content = tetrahedron_stl()
        mesh = application.import_mesh(
            io.BytesIO(content), len(content), "Valid.stl",
        )
        self.assertNotIn("repair", mesh)
        self.assertEqual(
            (application.directory / f"{mesh['id']}.stl").read_bytes(),
            content,
        )

    def test_failed_repair_preserves_mesh_and_cleans_candidate(self):
        application = Application(converter="missing-converter", probe=False)
        self.addCleanup(application.close)
        content = tetrahedron_stl(duplicate=True)
        mesh = application.import_mesh(
            io.BytesIO(content), len(content), "Needs repair.stl",
        )
        self.assertFalse(mesh["exportable"])
        self.assertEqual(mesh["repair"]["status"], "failed")
        self.assertEqual(
            (application.directory / f"{mesh['id']}.stl").read_bytes(),
            content,
        )
        self.assertEqual(len(list(application.directory.glob("*.stl"))), 1)

    @unittest.skipUnless(os.name == "nt", "Requires Windows mesh repair")
    def test_native_repair_rejects_empty_result_without_losing_input(self):
        application = Application(probe=False)
        self.addCleanup(application.close)
        content = bytes(80) + struct.pack("<I", 1) + bytes(50)
        mesh = application.import_mesh(
            io.BytesIO(content), len(content), "Collapsed.stl",
        )
        self.assertFalse(mesh["exportable"])
        self.assertEqual(mesh["repair"]["status"], "failed")
        self.assertEqual(
            (application.directory / f"{mesh['id']}.stl").read_bytes(),
            content,
        )

    def test_closed_mesh_and_bad_topology_are_distinguished(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "part.stl"
            path.write_bytes(tetrahedron_stl())
            details = inspect_stl(path)
            self.assertEqual(details["topology"], "closed")
            self.assertEqual(details["dimensions"], [1, 1, 1])
            self.assertEqual(details["triangles"], 4)
            path.write_bytes(tetrahedron_stl(duplicate=True))
            details = inspect_stl(path)
            self.assertEqual(details["topology"], "invalid")
            self.assertFalse(details["exportable"])

    def test_failed_import_keeps_the_previous_mesh(self):
        application = Application(probe=False)
        self.addCleanup(application.close)
        content = tetrahedron_stl()
        mesh = application.import_mesh(
            io.BytesIO(content), len(content), "My part.stl"
        )
        with self.assertRaises(ValueError):
            application.import_mesh(io.BytesIO(bytes(100)), 100, "broken.stl")
        self.assertEqual(application.mesh, mesh)
        self.assertEqual(len(list(application.directory.glob("*.stl"))), 1)


class LocalServiceTests(unittest.TestCase):
    def setUp(self):
        self.application = Application(probe=False)
        self.server = create_server(self.application)
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.application.close()

    def test_browser_upload_requires_session_token(self):
        content = tetrahedron_stl()
        request = Request(
            self.base + "/api/import?name=part.stl", data=content
        )
        with self.assertRaises(HTTPError) as caught:
            urlopen(request)
        self.assertEqual(caught.exception.code, 403)
        request.add_header("X-ScanCraft-Token", self.application.token)
        with urlopen(request) as response:
            result = json.load(response)
        self.assertEqual(result["triangles"], 4)
        self.assertEqual(result["topology"], "closed")

    def test_foreign_origin_is_rejected_even_with_token(self):
        request = Request(
            self.base + "/api/export",
            data=b"{}",
            headers={
                "Origin": "https://untrusted.example",
                "X-ScanCraft-Token": self.application.token,
            },
        )
        with self.assertRaises(HTTPError) as caught:
            urlopen(request)
        self.assertEqual(caught.exception.code, 403)

    def test_incomplete_export_has_no_download(self):
        with self.assertRaises(HTTPError) as caught:
            urlopen(self.base + "/api/jobs/not-a-job/download")
        self.assertEqual(caught.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
