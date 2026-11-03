# Native dependency sources

MCAP C++ 2.1.3, MIT license, is copied from
https://github.com/foxglove/mcap/tree/1420296ffcfdcde4b6894c0c1aba0ad083f93dde/cpp/mcap.
Only the public headers and license are included. Archive SHA-256:
`f72059d1e926563b04500e51d3f89b4f590c7d52eff3205b191705ff4ec62050`.
The local empty-write fix is documented in `mcap/HORUS_PATCHES.md`.

The recording implementation uses MCAP's indexed, CRC-protected writer/reader
with Zstandard compression. This is a project-only source dependency, not a
system installation. Remove `vendor/mcap` only together with the native Teach
recorder and its build dependency. No configure-time download is required.
