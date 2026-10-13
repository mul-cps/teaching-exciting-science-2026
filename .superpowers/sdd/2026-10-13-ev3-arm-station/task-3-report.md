# Task 3 report

Implemented `PybricksEV3(config, transport=None)` with connect/preflight/home/read_color/position/feed/stop/wait_stopped/close, plus the standalone packaged `resources/ev3_agent.py`. This is code and deterministic offline evidence; no SSH connection, hardware command or network modification was performed.

## Runtime prerequisites

Provision the EV3 MicroPython v2 microSD image and boot it on the EV3 before using this backend, following the [official EV3 v2 quick start](https://docs.pybricks.com/en/v2.0/start_ev3.html) and [Pybricks installation instructions](https://pybricks.com/install/mindstorms-ev3/installation/). EV3 uses Pybricks 2, whereas newer hubs use newer major versions. Existing USB Ethernet or WiFi routing, verified SSH known_hosts entry and working key-based SSH authentication are prerequisites. This implementation supplies no default password, scans no network and changes no network configuration. The existing source analysis in `pybricks-research.md` supports `brickrun -r -- pybricks-micropython`, ujson/uselect/stdin/stdout, StopWatch and finite asynchronous BRAKE run_time commands.

Configure a lowercase 32-hex `expected_machine_id`, the exact existing host/user/port, optional identity_file and an absolute safe remote_dir. Original EV3 USB serial is mandatory only for original firmware physical admission; arm USB serial remains mandatory. No original USB VID/PID is used to authenticate Pybricks.

## Interfaces and safety

Host verifies pinned SSH host key, independent `/etc/machine-id`, EV3 ARM platform evidence and actual sysfs port driver inventory before uploading or launching the agent. Packaged upload is automatic over the same SSH destination. The session nonce/protocol/version/runtime/port ready handshake cross-checks the discovered port inventory against this independent identity. Runtime version is the live MicroPython sys.version string; no physical image/runtime was verified here.

Transport seam: connect(deadline, nonce) returns independently discovered identity; send(request, deadline); receive(deadline); close(). Responses use integer monotonic request ids, matching id and Boolean ok. No timed-out request is retried. Requests and motor admission use finite locks and checks after ownership acquisition. Low-level host requests are capped at 150 ms; agent motor slices are capped at 100 ms and end with BRAKE, never HOLD. Host continuously drains protocol/diagnostic pipes during requests with bounded buffers, without a background unbounded queue. Idle sessions expire after 500 ms on the brick and must reconnect; they do not resume a previous feed.

Agent `feed` is the idempotent finite feeder-slice operation; public PybricksEV3.feed performs the full commissioned 180-degree outbound then return-to-actual-start sequence under one deadline. Agent holds only the latest reply and a monotonic high-water mark, so duplicates cannot repeat movement and older requests remain rejected after cache replacement. Changed duplicate ids fault. Partial/oversized/malformed framing, EOF, lease expiry and errors attempt to brake both motors even if one stop fails. Motor firmware duration bounds remain effective if protocol output blocks.

Factored existing admission/homing/encoder tracking/color stability/feed/cleanup logic into backend-neutral `ev3_motion.py`; OriginalEV3 still exposes its existing API and EV3Error/read_stable_color imports. Only hardware-specific value/zero/slice/stop/busy hooks remain in backends. No USB algorithms were duplicated. Existing USB tests pass unchanged. Station is new/unindexed: traced exact source callers via AST extraction and `rg` over workshop/tests before moving symbols.

## Verification

- `python -m pytest -q`: **208 passed**, 5.07 s (final full-suite run; see command output).
- `python -m py_compile workshop/station/ev3_motion.py workshop/station/ev3_usb.py workshop/station/ev3_pybricks.py workshop/station/resources/ev3_agent.py`: passed.
- `git diff --check`: passed.
- New tests cover handshake/remote identity mismatch, sysfs/platform mismatch, deployment failure, pinned SSH/safe paths, missing/malformed/wrong acknowledgements, EOF/partial/oversized framing, duplicate feed and changed/old ids, finite slice constraints, both-stop attempts, busy motor rejection, idle/partial lease expiry, shared feed deadlines/start angle, cancellation and stop generation, switch release/stuck homing, stall/backoff and homing timeout.

Not verified: actual EV3 MicroPython boot image, remote SSH route/host key, brickrun redirect behavior on a physical brick, measured motor/switch/color/encoder behavior or physical commissioning. The 150 ms acknowledgement budget must be qualified on the actual link; timeout faults and finite brick-side movement remain conservative when it is exceeded.
