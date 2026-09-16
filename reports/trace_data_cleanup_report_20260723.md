> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# Trace Data Cleanup Report (2026-07-23)

Date: 2026-07-23

Status: applied successfully. The cleanup removed 145 exact paths from 22
trace families. A post-cleanup scan found only the five intended trace
families listed as preserved in this report.

## Applied policy

- For June traces, preserve every trace containing real camera payload files,
  regardless of its success/failure label.
- Treat MP4, NPY, PNG and JPEG files as camera payload. A `camera/` directory
  containing only logs, timestamps or summaries is not camera data.
- Delete a June camera trace only when its RGB duration is shorter than 30
  seconds.
- For July traces, preserve only the trace explicitly labeled `success`.
- Delete the complete trace family together: raw CSV, diagnostics, metadata,
  label, camera directory, cleaned output, and LeRobot v2.1/v3.0 derivatives.
- Apply the same exact deletion to the legacy directory and to files already
  migrated into the normalized project copy.

## Preserved camera traces

| Trace suffix | Camera payload files | Approximate RGB duration | Label handling |
| --- | ---: | ---: | --- |
| `20260616_164117` | 32,492 | 180.5 s | Preserved even though labeled failure |
| `20260616_170217` | 20,869 | 115.9 s | Preserved even though labeled failure |
| `20260616_173955` | 108,856 | about 605 s from timestamps | Preserved |
| `20260616_174453` | 52,322 | 290.7 s | Preserved even though labeled failure |
| `20260715_133454` | 0 | No camera payload | Preserved because labeled success |

## Deleted candidates

Short camera traces:

- `20260616_162816`: about 12.0 seconds
- `20260616_163231`: about 12.2 seconds

No camera payload:

- `20260715_091945`
- `20260715_092011`
- `20260715_092127`
- `20260715_092849`
- `20260715_093004`
- `20260715_094337`
- `20260715_094459`
- `20260715_095245`
- `20260715_095552`
- `20260715_104616`
- `20260715_113739`
- `20260715_114123`
- `20260715_115852`
- `20260715_120908`
- `20260715_123523`
- `20260715_125345`
- `20260715_125810`
- `20260715_131110`
- `20260715_133013`
- `20260715_133243`

The 22 candidate families occupied approximately 6.80 GB across the split
legacy/project locations before deletion. The `latest` aliases pointing to the
preserved `20260715_133454` success trace are also preserved.

Deletion is destructive and is not recoverable from the normalized project
unless an independent external backup exists.
