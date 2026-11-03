# Adaptive Mesh Helper

Optional PC-side helper for adaptive mesh viewing. Uses packaged MIT-licensed
meshoptimizer 1.3 sources pinned at
`9e1f07b159d3cb777f1c67ed31fc11fd117986f4`; see `vendor/LICENSE.md` and
`vendor/PROVENANCE.md`. The existing source-vendor provenance also describes the
SDK native core; this Python helper does not require that core or its CMake build.

`ensure_native()` performs an offline preflight build with the existing C++17
compiler. It stores a source/platform-fingerprinted shared library under
`${XDG_CACHE_HOME:-~/.cache}/horus/adaptive_mesh/`, outside installed Python package
directories. No system library, service or compiler is installed by this helper.
The build precedes live mapping, never runs inside a stream callback, and fails
explicitly if a compatible compiler/runtime is unavailable.

The native simplifier changes only derived context meshes. Original fine page
positions, colours, triangle indices and source files remain unchanged. Context
targets approximately one ninth of the source indices with a 1.5 cm simplification
error limit, colour-aware scoring and boundary safeguards; geometry constraints
can retain more. These are display approximation settings, not a physical sensor
accuracy guarantee or a claim that every source achieves the target count.

After stopping adaptive processes, deleting the adaptive helper cache removes
its compiled artifacts; preflight recreates the required version. To remove the
feature entirely, remove its Python/native source, registration integration and
package-data entries together. The pre-existing renderer/native core and normal
`add_live_mesh` path are separate and need not be uninstalled.
