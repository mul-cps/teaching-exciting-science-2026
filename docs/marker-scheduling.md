# Latest synchronized marker cohorts

The combined marker pipeline keeps at most one processing RGB/depth/CameraInfo
cohort and one pending cohort. Newer synchronized exposures replace the pending
cohort without copying image buffers. Both RGB and depth timestamps must advance;
duplicate/reordered exposures cannot count as new observations.

The three message-filter subscriptions share a separate mutually exclusive
callback group. Their callback only offers message references to the bounded
scheduler. A 10 ms ROS timer in the default callback group drains the newest
pending cohort. The board detector and its stale-status timer therefore remain
serialized. The existing TF publication callback group is unchanged. The
executor retains two threads so input subscriptions can progress during native
OpenCV computation. There is no new worker thread or camera/device I/O.

Malformed/future, mismatched-frame and unpaired input metadata is rejected before
exposure high-water marks advance, so a bad future stamp cannot suppress later
current frames.

The configured rate remains capped at 5 Hz. Starts are separated by at least its
period using monotonic time; missed deadlines do not trigger catch-up bursts.
Original exposure timestamps are retained. RGB, depth and CameraInfo freshness
(including future rejection), frames and pair deltas are checked at drain time;
stale queued observations are discarded. No observation or classifier admission
threshold has changed. The shared grayscale path continues to bypass the child
nodes' standalone rate gates.

When timing publication is enabled, the existing timing topic adds `queue_wait_ms`
and cumulative `scheduler` counters: offered, overwritten, duplicate/reordered,
started and stale-dropped. These counters describe scheduling, not detection
success. Shutdown closes intake and drops pending references, cancels the drain
timer, then stops executor submission without an unbounded thread join. Cleanup
joins the installed executor's worker snapshot using one shared 2 s deadline.
Nodes/context are destroyed only after all workers terminate. Timeout or missing
runtime introspection raises a clear fault while retaining those resources.
Python cannot terminate a wedged native callback: bounded cleanup does not imply
bounded process exit. ROS launch provides external process escalation; direct CLI
use requires an external caller/supervisor to enforce that policy. There is no
unbounded application queue or extra shutdown worker.

A latest-cohort scheduler cannot recover exposures that the camera producer never
published. The recorded aligned-CameraInfo trace is only an arrival proxy for an
offline scheduling replay; it does not establish that every RGB-D triple was
available or that physical object confirmation succeeds. Live commissioning must
check independent, fresh board/object observations after deployment.

The installed RoboStack `launch.actions.ExecuteLocal` API uses finite defaults:
5 s from SIGINT to SIGTERM and another 5 s to SIGKILL (`sigterm_timeout` and
`sigkill_timeout`). The existing marker launch inherits these defaults. They are
process-supervisor escalation, separate from the 2 s in-process cleanup budget.

The direct marker entrypoint defaults to ROS domain 76 and localhost-only before
ROS initialization. Explicit domain 0 and invalid domains outside 1..232 fail
before the node starts; an explicit valid isolated test domain is preserved.
