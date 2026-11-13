# Solo45 ESP-Miner — tune9 (beta 1)

Custom firmware for the **Bitaxe Gamma** (BM1370, boards 601 and 602), built on the official
[ESP-Miner v2.15.3](https://github.com/bitaxeorg/ESP-Miner/releases/tag/v2.15.3) by the Bitaxe team.
It adds a heat throttle, a per-chip auto-tuner, a stalled-domain watchdog and a new web page,
and keeps everything else from stock AxeOS (pools, Wi-Fi, settings, API).

> **Beta — use at your own risk.** It has run for days on the author's own Bitaxes without a crash,
> but only on Gamma 601/602 boards. Other boards (Supra, Ultra, Hex, GT, …) are **not tested**: please wait.
> There is no automatic rollback: if a flash fails, you need a USB re-flash (see INSTALL.md).

> ⚠️ **Install with the Bitaxe's own Settings → Update page only.** These are update files, not full flash
> images: don't use the web flasher, esptool or any USB flasher with them.

## What's new compared with stock AxeOS 2.15.3

**Heat**
- **Heat throttle** — when the auto fan is already at full speed and the chip is still over its target
  (or the power stage runs hot, VR ≥ 95 °C), the clock steps down 25 MHz at a time (at most 200 MHz,
  never below 400 MHz) instead of letting the miner overheat. It steps back up once it has been cool for a while.
  Your saved frequency is never changed.
- **No false overheat trips from a single bad sensor read** — the chip has to read over 75 °C for 3 seconds
  before safe mode starts.
- **The auto fan also watches the power stage** — a fan floor that rises from 0 % at VR 75 °C to 100 % at 95 °C.
- **Fan control is more robust** — failed fan writes are retried; a fan fault is only declared after 5 s and
  clears when the fan answers again.

**Auto-tuner** (Auto-tune tab, you start it)
- Tries frequencies one by one and finds the **lowest clean voltage** for each, keeps the best point
  (hashrate per watt, within your power cap) and saves it.
- Watches **every hash domain** of the chip: a domain that stops ends the trial and marks a voltage floor.
- Skips points that would go over the power cap, explains why a check failed, and falls back to the next best point.
- The last result is kept across restarts.

**Stalled-domain watchdog** (always on, when no tune runs)
- If one of the chip's 4 hash domains sits under 25 % for 2 minutes, it re-initialises the chip
  (pause/resume); only if that fails and the miner has been up over 30 minutes does it restart the miner.
  At most 3 re-inits per 6 hours, so it can't loop.

**Mining**
- Drops late results from the previous block's jobs instead of sending stale shares.

**New web page (solo45-ui-4)** — a single page with Overview, Chips & domains, Auto-tune, Pool,
Miner setup and Logs tabs: live hashrate/power/temps, per-domain health, the watchdog's counts,
tuner controls, and a settings backup button. The stock page stays built in (see INSTALL.md to switch back).

## Files

| File | What |
|---|---|
| `esp-miner-solo45-tune9.bin` | the firmware (upload as **Firmware**) |
| `www-solo45-ui-4.bin` | the web page (upload as **Website**, after the firmware) |
| `SHA256SUMS.txt` | checksums — check them before flashing |
| `INSTALL.md` | install, check, and how to go back to stock |
| `CHANGELOG.md` | every change, one line each |
| `source/patches/` | the source changes as 12 git patches on top of ESP-Miner v2.15.3 |
| `source/webui/index.html` | the web page's source |
| `LICENSE` | GNU GPL v3 (same as ESP-Miner) |

## Build it yourself

```
git clone https://github.com/bitaxeorg/ESP-Miner.git && cd ESP-Miner
git checkout v2.15.3
git am /path/to/source/patches/*.patch
# ESP-IDF v6.0.2, target esp32s3
idf.py set-target esp32s3 build      # -> build/esp-miner.bin
```
The web page is `source/webui/index.html`, gzipped into a 3 MB SPIFFS image for the `www` partition.

## License and credit

GPL-3.0, like ESP-Miner. All credit for the base firmware and the open hardware goes to the
[Bitaxe team](https://github.com/bitaxeorg). Not affiliated with or endorsed by them.
Made for solo miners by [@race45race](https://x.com/race45race) — [Solo45](https://github.com/race45race/solo45).
Feedback and bug reports welcome.
