# Zenoh Plugin (native)

A first-class OpenPLC Runtime v4 **native plugin** that bridges Zenoh
pub/sub to PLC I/O, driven by the editor-generated `conf/zenoh.json` — the
same shape as the `ethercat/` and `s7comm/` plugins under
`core/src/drivers/plugins/native/`.

## Layout

```
zenoh-plugin/
├── CMakeLists.txt          # build config (mirrors ethercat's)
├── zenoh_plugin.c          # plugin: init / start_loop / cycle_end / stop_loop / cleanup
├── zenoh_config.json       # default (empty) config, for reference
├── docs/                   # JSON schema + usage notes
└── libs/zenoh-pico/        # vendored zenoh-pico 1.9.0 source (static)
```

## How it integrates

1. `install.sh` runs `build_native_plugins()`, which scans
   `core/src/drivers/plugins/native/` for any directory with a
   `CMakeLists.txt` and builds it into `build/plugins/libzenoh_plugin.so`
   — exactly like ethercat and s7comm.
2. Register in `plugins.conf`:
   ```
   zenoh,./build/plugins/libzenoh_plugin.so,1,1,./build/plugins/zenoh.json
   ```
   The runtime matches uploaded config files to plugins **by name**, so an
   editor program that includes `conf/zenoh.json` is auto-copied to the
   plugin dir and handed to the plugin via
   `plugin_specific_config_file_path`. No webserver changes needed.
3. `start_loop()` loads the config (picking up re-uploaded configs on every
   program start), opens the zenoh session, and declares one publisher per
   `publish` topic (published each scan in `cycle_end`) and one subscriber
   per `subscribe` topic (writes into the PLC image under the image lock).

## Config schema

See the editor's `frontend/utils/zenoh/generate-zenoh-config.ts`
(`ZENOH_CONFIG_FORMAT_VERSION = 1`):

```json
{
  "version": 1,
  "enabled": true,
  "mode": "client",
  "connect": "tcp/localhost:7447",
  "listen": "",
  "topics": [
    { "topic": "openplc/plc1/output/0", "direction": "publish", "variable": "%QX0.0", "data_type": "BOOL" },
    { "topic": "openplc/plc1/cmd/0", "direction": "subscribe", "variable": "%IX0.0", "data_type": "BOOL" }
  ]
}
```

IEC locations supported: `%IX/%QX/%MX` (bit), `%IB/%QB` (byte),
`%IW/%QW/%MW` (word), `%ID/%QD/%MD` (dword), `%IL/%QL/%ML` (lword).
