# Changelog

## tune9 — beta 1 (2026-11-13)

On top of ESP-Miner v2.15.3 (12 patches, in order):

1. Require 3 s of high chip readings before overheat safe mode (one bad read no longer trips it).
2. Heat throttle: step 25 MHz below the setting while the auto fan is maxed and the chip is over target.
3. Auto-tuner: find the lowest clean voltage per frequency and keep the best point.
4. Drop late results from the previous block's jobs.
5. Auto fan also watches the regulator: fan floor from 0 % at VR 75 °C to 100 % at 95 °C.
6. Fan control: retry failed writes; fault only after 5 s, and clear it when the fan responds again.
7. Heat throttle steps back up after mostly-cool minutes; a blip no longer restarts the clock.
8. Auto-tuner watches every hash domain: a stopped domain ends the trial, sets a voltage floor and re-inits the chip.
9. Auto-tuner check: skip points the voltage margin would push over the power cap; on a power or heat fail,
   fall back to the next best point; say why a check failed.
10. Stalled-domain watchdog: when no tune runs and a hash domain sits under 25 % for 2 min, re-init the chip;
    restart only if that fails and uptime > 30 min; at most 3 re-inits per 6 h.
11. Keep the last tune result and the watchdog's lifetime counts in flash (own NVS namespace, written only
    when a tune ends or the watchdog acts), so they survive a restart.
12. Do those flash reads/writes in a small internal-RAM task (flash operations from a PSRAM stack crash the miner).

Web page solo45-ui-4: Overview, Chips & domains, Auto-tune (with the domain watchdog card), Pool,
Miner setup and Logs tabs.

Tested: 10 Bitaxes (Gamma 601/602 and a Hex) for 3+ days, 0 crashes from the firmware.
Next (not in this release): tune10 — the power-stage temperature must also stay high for 3 s before
safe mode, and safe mode comes back at the same voltage instead of 100 mV lower.
