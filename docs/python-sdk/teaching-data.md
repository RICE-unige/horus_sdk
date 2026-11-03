# Teach Recordings

HORUS Teach captures episodes and reports whether they were saved and validated.

Optional [task management and custom actions](./teaching-actions-tasks.md) add a
host-confirmed task prompt, counts across sessions, and a callback after durable
save. Callback failures are reported independently of the recording result.
It does not provide a recorded-data viewer or choose an analysis tool for you.
Use the native reader or MCAP tooling with your preferred workflow. Rerun and
learning frameworks are not required for capture, saving or validation.

Use the [hardware-free humanoid demo](teaching-demo.md) to exercise the live
collection workflow. No physical robot profile, training export or viewer-specific
loader is qualified by this documentation.

## Files and Storage

Set `TeachConfig.recording_root` to an absolute directory on the **SDK recording
host**, not the headset. Each episode has a new UUID and is never overwritten.
While capturing, its directory ends in `.inprogress`. Successful finalization
flushes the data and publishes the episode directory without that suffix.

| File | Contents |
| --- | --- |
| `data.mcap` | Native ROS samples, receipt timing and Teach events. |
| `manifest.json` | Schema version, task/outcome, configuration/hash, stream counts, quality issues and artifact checksums. |
| `journal.json` | Initial recording configuration retained for interrupted-capture recovery. |
| `capture.lock` | Host-local recorder/recovery coordination; not a data stream. |

The manifest schema is `horus.teach.episode.v1`. Keep the entire episode directory
together. `capture`, `quality` and `outcome` describe different things: a saved
episode can contain missing samples, and a failed task can still have a valid
recording. Integrity validation alone does not establish training suitability.

## Streams and Time

- ROS channels use configured `stream_id` names. Their `source_topic` metadata
  records the original topic. Payloads retain ROS 2 CDR serialization and embedded
  `ros2msg` definitions. Standard `.msg` interfaces are supported; IDL-only
  interfaces currently fail explicitly rather than recording an empty schema.
- `__events` contains JSON events such as episode lifecycle, operator intent,
  anchoring, hold requests and adapter command results. A published command is
  not evidence of controller acceptance or physical execution. Consult the
  recorded adapter contract and independently recorded feedback.
- `__timing` pairs samples with recorder-side monotonic receipt times, original
  source timestamps and source sequence numbers where known. `receipt_index`
  identifies the sample within its channel.
- MCAP `log_time` is recorder Unix receipt time. `publish_time` uses the source
  stamp when available, otherwise zero; use the nullable `source_ns` in the
  timing record to distinguish unknown timestamps. A header stamp's semantics
  come from the configured stream, not an assumption about exposure time.
- Clock offset/uncertainty and upstream drop counts remain unknown unless
  explicitly measured. Never subtract unrelated clocks or treat recorder
  receipt indices as the publisher's frame counters.

This mixed ROS/JSON archive is not a rosbag2 directory and is not automatically
a LeRobot, RLDS or other ready-to-train dataset. Tool-specific import/conversion
may be required; HORUS does not silently resample, invent action labels or
install a viewer.

### Calibration Snapshots

The runtime retains the latest CameraInfo and transient-local TF calibration
received before recording. TF entries are keyed by child frame, so separate
publishers/messages are not collapsed to only the last message. Each new episode
includes the retained original CDR messages and their original source and receipt
times, plus `calibration_snapshot` events identifying those samples. These are
prior calibration observations, not fresh acquisitions at recording start.
Messages containing multiple children may overlap; process snapshots in original
receipt order so a newer child transform supersedes its older value.

The cache is bounded to 2048 keys and 16 MiB of accounted payload/schema storage.
Overflow reports unacceptable quality and requests hold rather than silently
discarding calibration. A declared required calibration stream must have been
received; retained calibration does not expire merely because it publishes once.
Calibration accuracy, clock synchronization and whether a mount is measured or
nominal remain source responsibilities. The runtime does not invent missing TF.

Current-session alignment is also recorded when capture starts after Align.
Accepted operator intent contains calibrated poses and the original tracking-frame
identity; alignment events retain the transform needed to interpret them. This
does not turn command targets into measured executed robot actions.

Opt-in [Teach input v2](teaching-input.md) retains head/controller poses, sticks,
buttons and their validity flags in accepted `operator_intent` events. The
manifest's resolved configuration records the input version and required
features. Version 1 recordings and readers keep their existing schema. These
events describe accepted operator intent, not a continuous unengaged XR stream
or proof of a robot executing the requested action.

## Validate or Recover

In an environment with the SDK's `teaching` extra installed:

```bash
python3 -m horus.teaching validate /absolute/dataset/EPISODE_UUID
python3 -m horus.teaching recover /absolute/dataset/EPISODE_UUID.inprogress
```

Validation prints machine-readable integrity and quality results. Its exit code
is nonzero for an integrity failure; inspect `quality` and `quality_issues`
separately. Recovery refuses an active recorder, preserves the original and
writes a **new** interrupted episode containing recoverable complete records.
It never labels recovered data as complete or training-ready.

## Read in Your Own Tool

```python
from horus.teaching.dataset import Episode

episode = Episode("/absolute/dataset/EPISODE_UUID")
issues = episode.validate()
if issues:
    raise ValueError(issues)

print(episode.manifest["task"], episode.manifest["quality"])
for schema, channel, sample in episode.samples(stream_ids=["front", "joints"]):
    # Forward sample.data to your chosen decoder or analysis pipeline.
    print(channel.topic, channel.message_encoding, sample.log_time)

for event in episode.events():
    print(event["kind"], event["data"])
```

The reader streams samples instead of loading an episode into memory. For ROS
messages with the corresponding local interface installed, use
`rclpy.serialization.deserialize_message` and
`rosidl_runtime_py.utilities.get_message(schema.name)` to decode CDR payloads.
Your analysis should retain source timing, quality flags and provenance.

## Offline Recorder Check

With ROS 2 sourced and the SDK available:

```bash
python3 -m horus.teaching fixture /absolute/test-data
```

This creates a short, explicitly synthetic `fixture` episode and prints its
path. It publishes no robot commands and opens no viewer. It checks recording
and reading only, not hardware control, sensor synchronization or policy quality.
