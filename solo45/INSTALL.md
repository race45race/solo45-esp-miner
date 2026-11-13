# Install, check, and go back

**For Bitaxe Gamma 601 / 602 only.** Takes about 3 minutes. Your Wi-Fi, pools and settings are kept.

## Before you start
1. Check the checksums (Windows PowerShell: `Get-FileHash esp-miner-solo45-tune9.bin`; Mac/Linux: `shasum -a 256 *.bin`)
   against `SHA256SUMS.txt`. Don't flash if they differ.
2. Note your current settings (frequency, core voltage, pools), or take screenshots.
3. Download the **official** files too, so you can go back any time:
   `esp-miner.bin` and `www.bin` from the [ESP-Miner v2.15.3 release](https://github.com/bitaxeorg/ESP-Miner/releases/tag/v2.15.3).

## Install
1. Open your Bitaxe's page in a browser (its IP address) → **Settings → Update** (or **System → Update**).
2. **Firmware:** upload `esp-miner-solo45-tune9.bin`. Wait for the restart (~30 s).
3. **Website:** upload `www-solo45-ui-4.bin`. Wait for the restart (~20 s).
   Always do the website **after** the firmware: a firmware update switches the custom page off,
   and uploading the website switches it back on.
4. Reload the page: you should see the **Solo45** page, and the firmware version
   **v2.15.3-solo45-tune9** in the header.

## Check it's working (after ~5 minutes)
- Hashrate near your normal, **4/4 domains hashing**, shares accepted, no rejects piling up.
- Temperatures and fan where you'd expect.

## Try the auto-tuner (optional)
Auto-tune tab → set your power cap → **Start**. It takes a while (it tests each point); leave the
miner alone until it says done. You can stop it any time; your old settings come back.

## Go back to stock
- **Just the page:** upload the official `www.bin` as **Website**. (Or from a terminal:
  `curl -X PATCH -H 'Content-Type: application/json' -d '{"useCustomWWW":0}' http://<bitaxe-ip>/api/system`
  then `curl -X POST http://<bitaxe-ip>/api/system/restart` — the stock page is built into the firmware.)
- **Everything:** upload the official `esp-miner.bin` as **Firmware**, then the official `www.bin` as **Website**.

## If it won't boot
There is no automatic rollback. If the miner doesn't come back after a flash, re-flash it over USB with
the official factory image using the [Bitaxe web flasher](https://bitaxeorg.github.io/bitaxe-web-flasher/)
(you'll need to set Wi-Fi and pools again). This has not happened in testing, but be ready for it.

## Reporting problems
Tell us your board (601/602), the firmware version, what you saw, and the miner's **Logs** tab if you can.
Never post your Wi-Fi password or a full flash dump — they contain your Wi-Fi details.
