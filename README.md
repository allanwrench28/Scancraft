# ScanCraft

Convert binary STL meshes to millimetre-unit STEP (AP214) B-reps using the
Monstertruck CAD kernel and STEP serializer.
<img width="1917" height="1015" alt="Screenshot 2026-10-14 001918" src="https://github.com/user-attachments/assets/96f29f81-0288-4bbe-9e9e-a9ce5bc1e552" />
<img width="953" height="676" alt="Screenshot 2026-10-14 002021" src="https://github.com/user-attachments/assets/94e2291f-9b54-4690-9d2c-c1b835b0a781" />
<img width="810" height="552" alt="Screenshot 2026-10-14 002010" src="https://github.com/user-attachments/assets/321d92f1-0691-4e46-b7de-46fceda8ff2a" />



## Windows App

Run [ScanCraft Setup](dist/installer/ScanCraft-Setup-0.2.1.exe) to install for
your Windows account. The installer creates a Start menu entry and offers a
desktop shortcut; administrator privileges are not required. Windows 10/11 x64
and the Microsoft Edge WebView2 runtime are required. Current Windows systems
usually already have WebView2. This development build is not code-signed, so
Windows may display an unknown-publisher warning.

The portable alternative is [ScanCraft.exe](dist/ScanCraft/ScanCraft.exe).
Keep the whole `dist/ScanCraft` folder together, including `_internal` and
`scancraft-cad.exe`. The package bundles the converter, Python, OpenCascade,
NumPy, trimesh, fonts, and preview assets. Rust, Node, Miniconda, and internet
access are not needed to convert files in the installed app.

1. Choose **Import STL** or drop one binary STL into the window.
2. Inspect the model with rotate, zoom, view buttons, and triangle edges.
3. Enable **Smooth faces** for reconstructed surfaces, or disable it for an
	exact faceted B-rep. The default fit tolerance is 0.01 mm.
4. Choose **Export STEP**, then **Save STEP** to select a destination.

Imports are copied into a temporary local workspace; source files are not
modified. On Windows, importing a mesh with non-manifold edges, winding errors,
collapsed triangles, or open boundaries automatically runs the native Windows
Printing3D repair service. No separate repair button or external slicer is
required. Already-valid closed meshes are left unchanged.

The app rechecks the repaired STL before using it, refreshes the preview and
counts, shows **Repaired by Windows**, and enables export. Both exact and smooth
exports use that repaired working copy. Windows repair can fill holes and
retessellate geometry; it is not restricted by the smoothing fit tolerance.
The repair log is available under **Activity**.

Repair has a three-minute limit. If Windows repair is unavailable, fails, or
returns unusable geometry, the original import is retained with a warning;
invalid meshes remain blocked, while valid open meshes can still export as
surface bodies. Malformed and non-finite STL data is rejected. The preview
shows the working STL (repaired when applicable), not the smooth STEP result.
One export runs at a time and can be cancelled. Files larger than 100 MB and
ASCII STLs are not supported by this version of the interface.

For GUI development in this checkout:

```powershell
npm ci
npm run build
& "$env:USERPROFILE\miniconda3\python.exe" -m pip install -r requirements-desktop.txt
& "$env:USERPROFILE\miniconda3\python.exe" .\scancraft_gui.py
```

Use `--browser` for the local browser version or `--no-browser --port 4317`
for automated tests. The service listens only on loopback, rejects foreign
origins, and requires a per-session token for mutations.

```powershell
& "$env:USERPROFILE\miniconda3\python.exe" -m unittest test_gui -v
npm run test:ui -- --project=browser
.\build_desktop.ps1
npm run test:ui -- --project=packaged
```

Building the installer requires the CAD Python environment described below,
PyInstaller, Node/npm, Rust, and Inno Setup 6. The build script checks the
packaged CAD runtime before creating the installer. The packaged smoke test
repairs the local hand STL when available, exports the mount, and launches the
executable without Python environment or Conda paths. Native repair is compiled
into the Rust converter and uses the Windows service, not Python packages.

For explicit repair from the command line on Windows:

```powershell
cargo run --release -p scancraft-kernel -- repair-mesh "C:\path\part.stl" --output "C:\path\part_repaired.stl"
```

## Convert

Run from the workspace root in PowerShell:

```powershell
cargo run --release -p scancraft-kernel -- mesh-to-nurbs "C:\path\part.stl" --output "C:\path\part.step"
```

Closed, consistently wound mesh components become solid bodies. Open components
become surface bodies. Vertices and edges are shared between adjacent faces, and
face geometry is derived from the vertices rather than the STL normal field.
Disconnected components are exported separately; nested shells are not combined
into cavities. Empty, degenerate, non-finite, non-manifold, or inconsistently
wound meshes produce errors instead of invalid solids. Conversion errors return
a nonzero exit code; input validation occurs before the output file is written.

The command name `mesh-to-nurbs` is retained for compatibility. Default export
preserves the STL triangles as planar CAD faces. Without `--output`, the command
runs experimental NURBS fit diagnostics. Those fits are not used for STEP export.

## Smooth Faces

Add `--smooth` to merge planar regions and chamfers, reconstruct cylindrical
walls as analytic CAD faces, and fit curved panels as trimmed B-spline faces.
Exact faceted export remains the default.

```powershell
cargo run --release -p scancraft-kernel -- mesh-to-nurbs "C:\path\part.stl" --output "C:\path\part_smooth.step" --smooth --fit-tolerance 0.01
```

`--fit-tolerance` is in millimetres (default `0.01`). Curved reconstruction changes
the original faceted geometry slightly; regions that cannot fit within this
tolerance stay faceted. Solid components are processed independently. Each
accepted replacement must reduce face count, preserve body count, pass B-rep
validation, and stay within a volume-change bound. The final STEP is reimported
and validated before replacing the destination file.

Full or partial cylinders with constant axial height are supported, including
cylindrical holes and tilted cylinders. Gently curved panels are fitted to
low-degree B-spline surfaces with their original outlines and holes as trimming
loops. Near-planar facets affected by STL rounding can become single plane faces,
including chamfers. Facet groups stop at creases of eight degrees or more.

Panel fits are sampled across the original triangles using half the tolerance
budget, then checked for closed trimming loops, valid sewn solids, preserved
shell count, dimensions, and bounded volume change. Unsupported, folded, or
poorly fitting regions retain their facets. Open meshes receive planar cleanup
only. This is bounded patch reconstruction, not unrestricted reverse engineering:
tightly curved regions may need multiple patches, and a full cylinder still has
a normal periodic seam. Caps and neighboring features remain separate faces.

Smooth mode uses Python with `pythonocc-core`, `numpy`, and `trimesh`. The helper
is embedded in the Rust executable; Python and these packages must be installed.
The converter uses `--python`, then `SCANCRAFT_PYTHON`, then a detected Miniconda
installation, or finally Python on PATH. To select this machine's working CAD
environment explicitly:

```powershell
cargo run --release -p scancraft-kernel -- mesh-to-nurbs "C:\path\part.stl" --output "C:\path\part_smooth.step" --smooth --python "$env:USERPROFILE\miniconda3\python.exe"
```

## Verify

```powershell
cargo test --workspace
```

The independent end-to-end validator runs the actual converter, imports its
output with OpenCascade, and checks B-rep validity, face and solid counts,
dimensions, and (for closed meshes) volume against the STL.

It requires Python with `pythonocc-core` and `trimesh`. This machine already has
both in its Miniconda environment:

```powershell
& "$env:USERPROFILE\miniconda3\python.exe" .\test_gen.py "C:\path\part.stl" --output "C:\path\part.step" --release
```

Use `--check-only` to validate an existing STEP without rerunning the converter.
The validator exits nonzero if import or geometry validation fails. Open the
freshly generated file in Fusion; an older STEP is not updated by rebuilding.

For smooth exports, add `--smooth` and the same `--fit-tolerance` to the validator.
It checks reduced face count, plane/cylinder/B-spline counts, dimensions, volume, and
sampled bidirectional surface deviation against the STL. The distance check uses
an OCC tessellation at one tenth of the fit tolerance; it is a sampled check, not
a formal maximum-distance proof.

```powershell
& "$env:USERPROFILE\miniconda3\python.exe" .\test_gen.py "C:\path\part.stl" --output "C:\path\part_smooth.step" --smooth --release
& "$env:USERPROFILE\miniconda3\python.exe" -m unittest test_gen.SmoothStepTests -v
```
