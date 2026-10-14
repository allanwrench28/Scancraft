import argparse
from collections import deque
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import secrets
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import parse_qs, quote, urlsplit
import uuid
import webbrowser

ROOT = Path(__file__).resolve().parent
ASSETS = Path(getattr(sys, "_MEIPASS", ROOT))
MAX_UPLOAD = 100 * 1024 * 1024
CREATE_FLAGS = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0


def export_options(payload):
    if not isinstance(payload, dict):
        raise ValueError("Invalid export settings")
    smooth = payload.get("smooth", False)
    if not isinstance(smooth, bool):
        raise ValueError("Choose an export mode")
    try:
        tolerance = float(payload.get("tolerance", 0.01))
    except (TypeError, ValueError) as error:
        raise ValueError(
            "Tolerance must be a positive number in mm"
        ) from error
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("Tolerance must be a positive number in mm")
    name = str(payload.get("name", "model.step")).strip()
    if not name or any(character in name for character in '/\\<>:"|?*\x00'):
        raise ValueError(
            "Use a filename without folders or special characters"
        )
    if any(ord(character) < 32 for character in name):
        raise ValueError("Invalid output filename")
    if Path(name).suffix.lower() not in (".step", ".stp"):
        name += ".step"
    reserved = {"CON", "PRN", "AUX", "NUL"}
    reserved.update(
        f"{prefix}{number}"
        for prefix in ("COM", "LPT")
        for number in range(1, 10)
    )
    if len(name) > 160 or name.split(".")[0].upper() in reserved:
        raise ValueError("Choose a shorter, valid Windows filename")
    return smooth, tolerance, name


def conversion_command(
    converter, source, output, smooth=False, tolerance=0.01, python=None
):
    command = [
        str(converter),
        "mesh-to-nurbs",
        str(source),
        "--output",
        str(output),
    ]
    if smooth:
        command.extend(["--smooth", "--fit-tolerance", str(tolerance)])
        if python:
            command.extend(["--python", str(python)])
    return command


def conversion_error(log):
    lowered = log.lower()
    if "non-manifold" in lowered or "inconsistent triangle winding" in lowered:
        return (
            "This mesh needs repair: some edges or triangle directions are"
            " invalid. No STEP was exported."
        )
    if "degenerate" in lowered:
        return (
            "This mesh contains collapsed triangles. Repair the STL before"
            " exporting."
        )
    if "non-finite" in lowered:
        return "This STL contains invalid coordinates. No STEP was exported."
    if "no module named" in lowered or "cannot start" in lowered:
        return (
            "The CAD Python runtime is unavailable. Exact export is still"
            " available."
        )
    if "permission denied" in lowered or "access is denied" in lowered:
        return (
            "ScanCraft could not write the output. Check the folder"
            " permissions."
        )
    return (
        "Conversion failed. No STEP was exported. Open the activity log for"
        " details."
    )


def inspect_stl(path):
    size = path.stat().st_size
    with path.open("rb") as stream:
        header = stream.read(84)
        if len(header) != 84:
            raise ValueError("This is not a complete binary STL file")
        triangle_count = struct.unpack_from("<I", header, 80)[0]
        if not triangle_count or 84 + triangle_count * 50 != size:
            raise ValueError(
                "This file is not a valid binary STL. Use binary STL format."
            )
        if size > MAX_UPLOAD:
            raise ValueError("This STL exceeds the 100 MB import limit")
        vertices, edges = {}, {}
        lower, upper = [math.inf] * 3, [-math.inf] * 3
        degenerate = 0
        while block := stream.read(50 * 4096):
            for values in struct.iter_unpack("<12fH", block):
                points = [values[start:start + 3] for start in (3, 6, 9)]
                if not all(
                    math.isfinite(value) for point in points for value in point
                ):
                    raise ValueError("This STL contains invalid coordinates")
                indices = []
                for point in points:
                    indices.append(vertices.setdefault(point, len(vertices)))
                    for axis, value in enumerate(point):
                        lower[axis] = min(lower[axis], value)
                        upper[axis] = max(upper[axis], value)
                first = [
                    points[1][axis] - points[0][axis] for axis in range(3)
                ]
                second = [
                    points[2][axis] - points[0][axis] for axis in range(3)
                ]
                cross = [
                    first[1] * second[2] - first[2] * second[1],
                    first[2] * second[0] - first[0] * second[2],
                    first[0] * second[1] - first[1] * second[0],
                ]
                degenerate += sum(value * value for value in cross) <= 1e-24
                for corner in range(3):
                    start, end = indices[corner], indices[(corner + 1) % 3]
                    key = (min(start, end), max(start, end))
                    count, balance = edges.get(key, (0, 0))
                    edges[key] = count + 1, balance + (
                        1 if start < end else -1
                    )
    boundary = sum(count == 1 for count, _ in edges.values())
    non_manifold = sum(count > 2 for count, _ in edges.values())
    winding = sum(
        count == 2 and balance != 0 for count, balance in edges.values()
    )
    valid = not (degenerate or non_manifold or winding)
    return {
        "triangles": triangle_count,
        "vertices": len(vertices),
        "bytes": size,
        "bounds": [lower, upper],
        "dimensions": [end - start for start, end in zip(lower, upper)],
        "exportable": valid,
        "topology": (
            "closed"
            if valid and not boundary
            else "open" if valid else "invalid"
        ),
        "boundary_edges": boundary,
        "non_manifold_edges": non_manifold,
        "winding_errors": winding,
        "degenerate_faces": degenerate,
    }


def find_converter():
    filename = (
        "scancraft-kernel.exe" if os.name == "nt" else "scancraft-kernel"
    )
    for candidate in (
        ASSETS / filename,
        ROOT / "target" / "release" / filename,
    ):
        if candidate.is_file():
            return candidate
    return ROOT / "target" / "release" / filename


def find_python():
    if getattr(sys, "frozen", False):
        return Path(sys.executable).with_name("scancraft-cad.exe")
    configured = os.environ.get("SCANCRAFT_PYTHON")
    if configured:
        return Path(configured)
    candidate = (
        Path.home()
        / "miniconda3"
        / ("python.exe" if os.name == "nt" else "bin/python")
    )
    return candidate if candidate.is_file() else Path(sys.executable)


def runtime_available(python):
    arguments = (
        [str(python), "--check-runtime"]
        if getattr(sys, "frozen", False)
        else [
            str(python),
            "-c",
            "import OCC.Core.BRep; import numpy; import trimesh",
        ]
    )
    try:
        return (
            subprocess.run(
                arguments,
                capture_output=True,
                timeout=30,
                creationflags=CREATE_FLAGS,
            ).returncode
            == 0
        )
    except (OSError, subprocess.TimeoutExpired):
        return False


def repair_imported_mesh(source, converter, details):
    if details["topology"] == "closed":
        return details
    repair = {"status": "failed", "engine": "Windows Printing3D"}
    if os.name != "nt":
        repair["message"] = "Automatic native repair requires Windows."
        return {**details, "repair": repair}
    candidate = source.with_name(source.stem + "-repair.stl")
    try:
        result = subprocess.run(
            [str(converter), "repair-mesh", str(source),
             "--output", str(candidate)],
            capture_output=True, text=True, encoding="utf-8",
            errors="replace", timeout=180, creationflags=CREATE_FLAGS,
        )
        repair["log"] = (result.stdout + result.stderr)[-6000:]
        if result.returncode:
            raise ValueError("Windows could not repair this mesh.")
        repaired = inspect_stl(candidate)
        if repaired["topology"] != "closed":
            raise ValueError(
                "Windows repair did not produce a closed, valid mesh."
            )
        os.replace(candidate, source)
        repair.update({
            "status": "repaired", "before_triangles": details["triangles"],
            "after_triangles": repaired["triangles"],
        })
        return {**repaired, "repair": repair}
    except subprocess.TimeoutExpired:
        repair["message"] = "Windows repair exceeded the three-minute limit."
    except (OSError, ValueError) as error:
        repair["message"] = str(error)
    finally:
        candidate.unlink(missing_ok=True)
    return {**details, "repair": repair}


@dataclass
class ExportJob:
    identifier: str
    name: str
    output: Path
    smooth: bool
    status: str = "queued"
    stage: str = "Preparing export"
    started: float = field(default_factory=time.time)
    finished: float | None = None
    error: str | None = None
    report: dict = field(default_factory=dict)
    log: deque = field(default_factory=lambda: deque(maxlen=300))
    cancelled: bool = False
    process: subprocess.Popen | None = None

    def snapshot(self):
        return {
            "id": self.identifier,
            "name": self.name,
            "smooth": self.smooth,
            "status": self.status,
            "stage": self.stage,
            "error": self.error,
            "elapsed": round((self.finished or time.time()) - self.started, 1),
            "report": self.report,
            "log": list(self.log),
            "bytes": (
                self.output.stat().st_size if self.status == "complete" else 0
            ),
        }


class Application:
    def __init__(self, converter=None, python=None, probe=True):
        self.converter = Path(converter) if converter else find_converter()
        self.python = Path(python) if python else find_python()
        self.smoothing_available = (
            runtime_available(self.python) if probe else False
        )
        self.temporary = tempfile.TemporaryDirectory(prefix="scancraft-gui-")
        self.directory = Path(self.temporary.name)
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.mesh = None
        self.jobs = {}
        self.threads = []
        self.saved = {}

    def busy(self):
        return any(
            job.status in ("queued", "running", "cancelling")
            for job in self.jobs.values()
        )

    def configuration(self):
        with self.lock:
            return {
                "token": self.token,
                "version": "0.2.1",
                "mesh": self.mesh,
                "converter_ready": self.converter.is_file(),
                "smoothing_available": self.smoothing_available,
                "max_upload_bytes": MAX_UPLOAD,
                "busy": self.busy(),
                "job": (
                    next(reversed(self.jobs.values())).snapshot()
                    if self.jobs
                    else None
                ),
            }

    def import_mesh(self, stream, length, name):
        if Path(name).suffix.lower() != ".stl":
            raise ValueError("Choose an STL file")
        if length <= 84 or length > MAX_UPLOAD:
            raise ValueError("Choose a non-empty STL smaller than 100 MB")
        with self.lock:
            if self.busy():
                raise ValueError(
                    "Wait for the export or cancel it before importing another"
                    " mesh"
                )
            identifier = uuid.uuid4().hex
            destination = self.directory / f"{identifier}.stl"
            try:
                with destination.open("wb") as output:
                    remaining = length
                    while remaining:
                        block = stream.read(min(1024 * 1024, remaining))
                        if not block:
                            raise ValueError(
                                "The import was interrupted. Please try again."
                            )
                        output.write(block)
                        remaining -= len(block)
                details = inspect_stl(destination)
                details = repair_imported_mesh(
                    destination, self.converter, details,
                )
            except Exception:
                destination.unlink(missing_ok=True)
                raise
            if self.mesh:
                (self.directory / f"{self.mesh['id']}.stl").unlink(
                    missing_ok=True
                )
            self.mesh = {
                "id": identifier,
                "name": Path(name.replace("\\", "/")).name,
                **details,
            }
            return self.mesh

    def start_export(self, payload):
        smooth, tolerance, name = export_options(payload)
        with self.lock:
            if self.busy():
                raise ValueError("An export is already running")
            if not self.converter.is_file():
                raise ValueError(
                    "The converter is missing. Reinstall ScanCraft or build"
                    " the release converter."
                )
            if not self.mesh or payload.get("mesh_id") != self.mesh["id"]:
                raise ValueError("Import an STL before exporting")
            if not self.mesh["exportable"]:
                raise ValueError(
                    "This mesh needs repair before export. Check the mesh"
                    " status."
                )
            if smooth and not self.smoothing_available:
                raise ValueError(
                    "The smoothing runtime is missing. Select exact export or"
                    " reinstall ScanCraft."
                )
            identifier = uuid.uuid4().hex
            folder = self.directory / identifier
            folder.mkdir()
            job = ExportJob(identifier, name, folder / name, smooth)
            self.jobs[identifier] = job
            command = conversion_command(
                self.converter,
                self.directory / f"{self.mesh['id']}.stl",
                job.output,
                smooth,
                tolerance,
                self.python,
            )
            thread = threading.Thread(
                target=self._convert, args=(job, command), daemon=True
            )
            self.threads.append(thread)
            thread.start()
            return job.snapshot()

    def _convert(self, job, command):
        captured = deque(maxlen=4000)
        try:
            with self.lock:
                if job.cancelled:
                    job.status = "cancelled"
                    return
                job.status, job.stage = "running", "Building STEP geometry"
                job.process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    creationflags=CREATE_FLAGS,
                )
            for line in job.process.stdout:
                line = line.rstrip()
                captured.append(line)
                with self.lock:
                    job.log.append(line[:2000])
                    if "Open CASCADE" in line or "Transfer Mode" in line:
                        job.stage = (
                            "Fitting and validating surfaces"
                            if job.smooth
                            else "Writing STEP"
                        )
            exit_code = job.process.wait()
            with self.lock:
                if job.cancelled:
                    job.status, job.stage = "cancelled", "Export cancelled"
                elif (
                    exit_code
                    or not job.output.is_file()
                    or not job.output.stat().st_size
                ):
                    job.status, job.stage = "failed", "Export failed"
                    job.error = conversion_error("\n".join(captured))
                else:
                    text = "\n".join(captured)
                    for position, character in enumerate(text):
                        if character != "{":
                            continue
                        try:
                            report, _ = json.JSONDecoder().raw_decode(
                                text[position:]
                            )
                            if (
                                isinstance(report, dict)
                                and "faces_after" in report
                            ):
                                job.report = {
                                    key: report.get(key)
                                    for key in (
                                        "faces_before",
                                        "faces_after",
                                        "solids",
                                        "cylindrical_faces",
                                        "bspline_faces",
                                        "planar_faces",
                                        "valid_brep",
                                        "fit_tolerance_mm",
                                    )
                                }
                                break
                        except json.JSONDecodeError:
                            continue
                    job.status, job.stage = "complete", "STEP ready"
        except (OSError, ValueError) as error:
            with self.lock:
                job.status, job.stage = "failed", "Export failed"
                job.log.append(str(error))
                job.error = conversion_error(str(error))
        finally:
            with self.lock:
                job.finished = time.time()
                job.process = None
                if job.status != "complete":
                    job.output.unlink(missing_ok=True)

    def cancel(self, identifier):
        with self.lock:
            job = self.jobs.get(identifier)
            if not job:
                raise ValueError("Export not found")
            if job.status not in ("queued", "running", "cancelling"):
                return job.snapshot()
            job.cancelled, job.status, job.stage = (
                True,
                "cancelling",
                "Cancelling export",
            )
            process = job.process
        if process and process.poll() is None:
            if os.name == "nt":
                subprocess.run(
                    ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                    capture_output=True,
                    creationflags=CREATE_FLAGS,
                )
            else:
                process.terminate()
        return job.snapshot()

    def close(self):
        for identifier in list(self.jobs):
            self.cancel(identifier)
        for thread in self.threads:
            thread.join(timeout=10)
        self.temporary.cleanup()


class RequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format_string, *arguments):
        pass

    def _trusted(self, mutate=False):
        origin = f"http://127.0.0.1:{self.server.server_port}"
        if self.headers.get("Host") != origin.removeprefix("http://"):
            return False
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            return False
        if self.headers.get("Origin") not in (None, origin):
            return False
        return not mutate or secrets.compare_digest(
            self.headers.get("X-ScanCraft-Token", ""),
            self.server.application.token,
        )

    def _headers(self, status, content_type, length):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'"
            " 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'"
            " blob:; worker-src 'self' blob:; frame-ancestors 'none'",
        )

    def _json(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self._headers(status, "application/json; charset=utf-8", len(body))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not self._trusted():
            self._json(
                {
                    "error": (
                        "Only the local ScanCraft window can access this"
                        " service"
                    )
                },
                403,
            )
            return
        path = urlsplit(self.path).path
        application = self.server.application
        if path == "/api/session":
            self._json(application.configuration())
            return
        if path.startswith("/api/mesh/"):
            with application.lock:
                if (
                    application.mesh
                    and path == f"/api/mesh/{application.mesh['id']}"
                ):
                    source = (
                        application.directory / f"{application.mesh['id']}.stl"
                    )
                    self._headers(
                        200, "application/octet-stream", source.stat().st_size
                    )
                    self.end_headers()
                    with source.open("rb") as stream:
                        shutil.copyfileobj(stream, self.wfile)
                    return
            self._json({"error": "Mesh not found"}, 404)
            return
        if path.startswith("/api/jobs/"):
            pieces = path.strip("/").split("/")
            with application.lock:
                job = (
                    application.jobs.get(pieces[2])
                    if len(pieces) >= 3
                    else None
                )
                if not job:
                    self._json({"error": "Export not found"}, 404)
                    return
                if len(pieces) == 3:
                    self._json(job.snapshot())
                    return
                if pieces[3:] == ["download"] and job.status == "complete":
                    self._headers(
                        200, "application/step", job.output.stat().st_size
                    )
                    self.send_header(
                        "Content-Disposition",
                        f"attachment; filename*=UTF-8''{quote(job.name)}",
                    )
                    self.end_headers()
                    with job.output.open("rb") as stream:
                        shutil.copyfileobj(stream, self.wfile)
                    return
            self._json({"error": "The export is not ready"}, 409)
            return
        root = self.server.assets.resolve()
        relative = "index.html" if path == "/" else path.lstrip("/")
        resource = (root / relative).resolve()
        if not resource.is_relative_to(root) or not resource.is_file():
            self._json({"error": "Not found"}, 404)
            return
        content_type = (
            mimetypes.guess_type(resource.name)[0]
            or "application/octet-stream"
        )
        self._headers(200, content_type, resource.stat().st_size)
        self.end_headers()
        with resource.open("rb") as stream:
            shutil.copyfileobj(stream, self.wfile)

    def do_POST(self):
        if not self._trusted(mutate=True):
            self.close_connection = True
            self._json(
                {"error": "Request rejected. Reopen the ScanCraft window."},
                403,
            )
            return
        parsed = urlsplit(self.path)
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if parsed.path == "/api/import":
                name = parse_qs(parsed.query).get("name", [""])[0]
                self._json(
                    self.server.application.import_mesh(
                        self.rfile, length, name
                    )
                )
                return
            if length < 0 or length > 16384:
                raise ValueError("Invalid request size")
            payload = json.loads(self.rfile.read(length) or b"{}")
            if parsed.path == "/api/export":
                self._json(self.server.application.start_export(payload), 202)
                return
            pieces = parsed.path.strip("/").split("/")
            if (
                len(pieces) == 4
                and pieces[:2] == ["api", "jobs"]
                and pieces[3] == "cancel"
            ):
                self._json(self.server.application.cancel(pieces[2]))
                return
            self._json({"error": "Not found"}, 404)
        except (ValueError, OSError) as error:
            self.close_connection = True
            self._json({"error": str(error)}, 400)


def create_server(application, port=0, assets=None):
    server = ThreadingHTTPServer(("127.0.0.1", port), RequestHandler)
    server.application = application
    server.assets = Path(assets) if assets else ASSETS / "gui" / "dist"
    server.daemon_threads = True
    return server


class DesktopAPI:
    def __init__(self, application):
        self._application = application
        self._window = None

    def save_export(self, identifier):
        import webview

        with self._application.lock:
            job = self._application.jobs.get(identifier)
            if not job or job.status != "complete":
                return {"error": "The export is not ready"}
        selection = self._window.create_file_dialog(
            webview.FileDialog.SAVE,
            save_filename=job.name,
            file_types=("STEP files (*.step;*.stp)",),
        )
        if not selection:
            return {"cancelled": True}
        destination = Path(
            selection if isinstance(selection, str) else selection[0]
        )
        try:
            shutil.copyfile(job.output, destination)
            self._application.saved[identifier] = destination
            return {
                "saved": True,
                "path": str(destination),
                "name": destination.name,
            }
        except OSError as error:
            return {"error": str(error)}

    def reveal_export(self, identifier):
        destination = self._application.saved.get(identifier)
        if destination and destination.is_file() and os.name == "nt":
            subprocess.Popen(["explorer.exe", "/select,", str(destination)])
            return True
        return False


def main():
    if sys.argv[1:3] == ["-u", "-c"]:
        from smooth_step import main as smooth_main

        sys.argv = [sys.argv[0], *sys.argv[4:]]
        smooth_main()
        return
    if sys.argv[1:] == ["--check-runtime"]:
        from smooth_step import read_step

        return read_step is not None
    parser = argparse.ArgumentParser(description="ScanCraft desktop interface")
    parser.add_argument("--port", type=int, default=4317)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--browser", action="store_true")
    parser.add_argument("--converter", type=Path)
    parser.add_argument("--python", type=Path)
    args = parser.parse_args()
    application = Application(args.converter, args.python)
    try:
        try:
            server = create_server(application, args.port)
        except OSError:
            server = create_server(application)
        url = f"http://127.0.0.1:{server.server_port}"
        if sys.stdout:
            print(f"ScanCraft: {url}", flush=True)
        if args.no_browser or args.browser:
            if args.browser:
                webbrowser.open(url)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
        else:
            import webview

            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            api = DesktopAPI(application)
            api._window = webview.create_window(
                "ScanCraft",
                url,
                js_api=api,
                width=1360,
                height=900,
                min_size=(760, 600),
                background_color="#f3f5f4",
            )
            webview.start(gui="edgechromium" if os.name == "nt" else None)
            server.shutdown()
        server.server_close()
    finally:
        application.close()


if __name__ == "__main__":
    main()
