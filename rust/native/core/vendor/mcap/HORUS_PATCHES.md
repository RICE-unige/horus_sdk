# Local MCAP changes

Base: MCAP C++ 2.1.3, commit `1420296ffcfdcde4b6894c0c1aba0ad083f93dde`.

- `IWritable::write` returns immediately for zero-byte writes. Empty schema data
  can expose a null pointer, which must not be passed to `fwrite` even when its
  size is zero. This preserves the file format, byte count and CRC and is covered
  by the native Teach recording tests under UndefinedBehaviorSanitizer.
