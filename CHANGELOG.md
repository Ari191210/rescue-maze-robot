# Changelog

No git history exists for this project — it was developed as a series of numbered `.ino` files. This changelog is reconstructed from the version history documented inline in the firmware's own header comments, plus file timestamps. Entries before v13 are summarized from those comments only; intermediate files (v10–v12) exist on disk but their individual diffs weren't separately documented, so they're grouped here rather than itemized.

## v15 — 2026-07-07 (current build, `firmware/maze_v15_pure_hardcode.ino`)
"100% pure hardcode" — removed wall-following and PID correction entirely. Drive is now pure encoder-count dead reckoning against a hardcoded `PATH[]` string. Everything depends on `COUNTS_PER_CELL`, `STOP_MARGIN_TURN`, and `PATH[]` being calibrated correctly, since nothing corrects drift anymore.

## v14 — 2026-07-06/07
Added synchronized braking (both wheels brake together, fixing a 45° swing at each stop) and speed tapering (crawl the last `SLOW_ZONE` encoder counts before stopping, instead of slamming to a stop).

## v13 — 2026-07-07
Added `TEST_MODE=8`, a hardcoded-path mode where the route is typed as a string and run in order, while still using the wall-following PID drive underneath to self-correct against walls. Also: hardened encoder reads (reject 0-sentinel and glitch jumps), brownout detector at boot, corrected I2C sensor address→side mapping, and motion timeouts so a bad sensor read can't run the robot forever.

## v10–v12 — 2026-07-06
Diagnostic and stabilization pass on top of the wall-following PID build: brownout detector added, `TEST_MODE=6` combined motor/encoder diagnostic added, hardened encoder reads against sentinel/glitch values that were causing circling and bad cell-distance stops.
