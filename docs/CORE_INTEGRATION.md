# FINN Guidance → FINN Core Integration

Reviewed: 19 July 2026

How the tractor guidance app integrates into `finn-core`. Guidance integrates
by contracts, not by merging: Core is the system of record for field runs,
telemetry uploads, and analysis tasks, but nothing in the tractor depends on
Core being online. Steering never waits for the network.

The canonical Core contract is `../../finn-core/docs/CORE_CONTRACTS.md`. Core
API routes are root-level (no `/api` prefix); `/api/...` aliases exist only for
stale clients.

## Node Identity

Per the Core contract, guidance registers as:

- `node_type`: `guidance`
- `tier`: 1
- capabilities: `guidance.telemetry`, `guidance.steering`
- example `node_id`: `tractor-guidance-01`
- `system_info`: machine and hardware detail
  (e.g. `LC29H BA direct + motor ESP32`)
- `metadata.project`: `finn-guidance`

Non-worker nodes appear on the Core dashboard without becoming LLM task
capacity.

TBD — confirm with Tom: live registration/heartbeat from the cab app itself is
defined by the contract and covered by Core-side mock tests, but whether the
field app currently registers as a live node (vs. upload-only integration
below) needs verifying against the running code before this doc claims it.

## Implemented Today: Offline-First Field-Run Upload

After a run, the field laptop registers the run with Core:

```bash
python scripts/upload_field_run.py --core-url http://<core-pc>:8000 --field-name "North paddock"
```

By default the command:

- creates a guidance field run via `POST /field-runs`;
- summarises the latest `logs/steer_*.jsonl` telemetry file if present and
  uploads via `POST /field-runs/{id}/telemetry`;
- summarises `data/coverage.db` and uploads via
  `POST /field-runs/{id}/coverage` with `coverage_source: guidance_manual`;
- sets `create_analysis_task: true`, so Core creates worker analysis tasks
  linked back to the upload records.

Full files stay on the laptop; Core stores local file URIs plus summaries.
Results appear on the Core field-run dashboard
(`http://<core-pc>:8000/dashboard/field-runs`), and an architect session can be
opened with the run as context via
`POST /field-runs/{id}/architect-session`.

## Coverage Source Naming

Coverage uploads must declare their source explicitly so guidance and pilot
never silently both claim primary coverage:

- `guidance_manual` — today's default.
- `guidance_backup` — once pilot owns primary coverage.
- `pilot_coverage_assist` / future `pilot_primary` — pilot-side sources.

## Rules

- Guidance keeps steering locally without Core; uploads happen after the fact.
- Core analyses and recommends; it never commands steering. Any tuning
  recommendation is applied by the operator, on the tractor, deliberately.
- Telemetry files carry a schema version; required run metadata (machine,
  implement width, AB line, config snapshot, firmware version, GPS mode, RTK
  status, coverage source, run start/end) follows the unification roadmap
  contract.

## Future Work

- Wire the RTK/correction status into uploads once the NTRIP rover client
  exists, so Core can correlate fix quality with steering performance.
- GUI button: "Send latest telemetry to FINN Core" (today it is the script).
- Confirm/implement live node registration + heartbeat from the cab app.
