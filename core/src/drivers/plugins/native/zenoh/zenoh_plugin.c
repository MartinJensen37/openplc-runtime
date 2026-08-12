/*
 * zenoh_plugin.c — OpenPLC Runtime v4 native plugin bridging Zenoh
 * pub/sub to PLC I/O, driven by the editor-generated conf/zenoh.json.
 *
 * The editor stores Zenoh connectors in the project and, at compile time,
 * writes conf/zenoh.json into the upload bundle. The runtime extracts it to
 * core/generated/conf/zenoh.json and, because this plugin is registered in
 * plugins.conf under the name "zenoh", update_plugins_from_config_dir copies
 * it to ./plugins/zenoh.json and hands the path to the plugin through
 * plugin_runtime_args_t.plugin_specific_config_file_path.
 *
 * Config schema (see the editor's frontend/utils/zenoh/generate-zenoh-config.ts):
 *
 *   {
 *     "version": 1,
 *     "enabled": true,
 *     "mode": "client",
 *     "connect": "tcp/localhost:7447",
 *     "listen": "",
 *     "topics": [
 *       { "topic": "openplc/plc1/output/0", "direction": "publish",
 *         "variable": "%QX0.0", "data_type": "BOOL" },
 *       { "topic": "openplc/plc1/cmd/0", "direction": "subscribe",
 *         "variable": "%IX0.0", "data_type": "BOOL" }
 *     ]
 *   }
 *
 * "variable" is an IEC location: %IX/%QX/%MX (bit), %IB/%QB (byte),
 * %IW/%QW/%MW (word), %ID/%QD/%MD (dword), %IL/%QL/%ML (lword).
 *
 * When no config file exists / is unparseable the plugin falls back to the
 * legacy env-var behaviour (ZENOH_MODE / ZENOH_CONNECT / ZENOH_LISTEN plus
 * the two default topics) so the pre-editor workflow keeps working. A config
 * that exists but carries an unsupported "version" is refused loudly (like
 * the OPC-UA plugin's format_version gate) rather than half-applied.
 *
 * Config is loaded in start_loop(), not init(), so a re-uploaded
 * conf/zenoh.json is picked up on the next program start — same lifecycle
 * as the s7comm / opcua / modbus plugins. Logging goes through the runtime's
 * central plugin_logger so messages appear in the editor's log viewer.
 *
 * Targets zenoh-pico 1.x owned/loaned API (examples/unix/c11/z_pub.c,
 * z_sub.c). Re-verify against those if you pin a different tag.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "plugin_logger.h"
#include "plugin_types.h"
#include <zenoh-pico.h>

#define ZENOH_MAX_TOPICS 64
#define ZENOH_FALLBACK_CONFIG "core/generated/conf/zenoh.json"

/* Mirror of ZENOH_CONFIG_FORMAT_VERSION in the editor's
 * frontend/utils/zenoh/generate-zenoh-config.ts: a config written by an
 * older/newer editor is refused loudly instead of half-applied (same
 * contract gate the OPC-UA plugin applies to opcua.json). */
#define ZENOH_CONFIG_MIN_FORMAT_VERSION 1

/* plugin_runtime_args_t comes from the runtime's core/src/drivers/
 * plugin_types.h (canonical struct — mutex_take/mutex_give were replaced
 * by image_lock()/image_unlock()). init() copies it by value because the
 * runtime frees the args pointer right after init() returns. */
static plugin_logger_t       g_logger;
static plugin_runtime_args_t g_args;
static z_owned_session_t     g_session;
static int g_active = 0;
static int g_configured_enabled = 1;

/* ------------------------------------------------------------------ */
/* Parsed configuration                                                */
/* ------------------------------------------------------------------ */

typedef enum { Z_AREA_I, Z_AREA_Q, Z_AREA_M } zenoh_area_t;
typedef enum { Z_SIZE_X, Z_SIZE_B, Z_SIZE_W, Z_SIZE_D, Z_SIZE_L } zenoh_size_t;

typedef struct {
    zenoh_area_t area;
    zenoh_size_t size;
    int byte_idx; /* buffer index */
    int bit_idx;  /* only for X (bit) size */
} zenoh_iec_addr_t;

typedef struct {
    char topic[256];
    int direction;   /* 0 = publish (PLC -> bus), 1 = subscribe (bus -> PLC) */
    char variable[64];
    char data_type[16];
    zenoh_iec_addr_t addr; /* parsed at init */
    int addr_valid;
    /* Declared endpoint handle for this topic (avoids index juggling
     * between the topics array and parallel handle arrays). */
    z_owned_publisher_t  pub;
    z_owned_subscriber_t sub;
    int pub_valid;
    int sub_valid;
} zenoh_topic_t;

typedef struct {
    int enabled;
    char mode[16];
    char connect[256];
    char listen[256];
    int num_topics;
    zenoh_topic_t topics[ZENOH_MAX_TOPICS];
} zenoh_config_t;

static zenoh_config_t g_cfg;

/* ------------------------------------------------------------------ */
/* IEC location parsing                                               */
/* ------------------------------------------------------------------ */

/* Parse "%QX0.0", "%IW3", "%MD42", ... Returns 0 on success. */
static int zenoh_parse_iec_addr(const char *s, zenoh_iec_addr_t *out) {
    if (!s || !out || s[0] != '%' || s[1] == '\0' || s[2] == '\0') return -1;
    const char area_c = s[1];
    const char size_c = s[2];
    const char *num = s + 3;
    char *end = NULL;
    long idx = strtol(num, &end, 10);
    if (end == num || idx < 0 || idx > 1023) return -1;

    int bit = 0;
    if (size_c == 'X') {
        if (*end != '.') return -1;
        char *bend = NULL;
        bit = (int)strtol(end + 1, &bend, 10);
        if (bend == end + 1 || bit < 0 || bit > 7) return -1;
    } else if (*end != '\0') {
        return -1;
    }

    switch (area_c) {
        case 'I': out->area = Z_AREA_I; break;
        case 'Q': out->area = Z_AREA_Q; break;
        case 'M': out->area = Z_AREA_M; break;
        default: return -1;
    }
    switch (size_c) {
        case 'X': out->size = Z_SIZE_X; break;
        case 'B': out->size = Z_SIZE_B; break;
        case 'W': out->size = Z_SIZE_W; break;
        case 'D': out->size = Z_SIZE_D; break;
        case 'L': out->size = Z_SIZE_L; break;
        default: return -1;
    }
    out->byte_idx = (int)idx;
    out->bit_idx = bit;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Buffer read / write under the image lock                           */
/* ------------------------------------------------------------------ */

static int zenoh_read_var(const zenoh_iec_addr_t *a, char *out, size_t outsz) {
    if (!a || !out) return -1;
    if (g_args.image_lock) g_args.image_lock();
    int rc = -1;
    switch (a->area) {
    case Z_AREA_I:
        switch (a->size) {
        case Z_SIZE_X:
            if (g_args.bool_input[a->byte_idx] && g_args.bool_input[a->byte_idx][a->bit_idx]) {
                snprintf(out, outsz, "%d", *g_args.bool_input[a->byte_idx][a->bit_idx] ? 1 : 0);
                rc = 0;
            }
            break;
        case Z_SIZE_B:
            if (g_args.byte_input && g_args.byte_input[a->byte_idx]) {
                snprintf(out, outsz, "%u", (unsigned)*g_args.byte_input[a->byte_idx]);
                rc = 0;
            }
            break;
        case Z_SIZE_W:
            if (g_args.int_input && g_args.int_input[a->byte_idx]) {
                snprintf(out, outsz, "%u", (unsigned)*g_args.int_input[a->byte_idx]);
                rc = 0;
            }
            break;
        case Z_SIZE_D:
            if (g_args.dint_input && g_args.dint_input[a->byte_idx]) {
                snprintf(out, outsz, "%lu", (unsigned long)*g_args.dint_input[a->byte_idx]);
                rc = 0;
            }
            break;
        case Z_SIZE_L:
            if (g_args.lint_input && g_args.lint_input[a->byte_idx]) {
                snprintf(out, outsz, "%llu", (unsigned long long)*g_args.lint_input[a->byte_idx]);
                rc = 0;
            }
            break;
        }
        break;
    case Z_AREA_Q:
        switch (a->size) {
        case Z_SIZE_X:
            if (g_args.bool_output[a->byte_idx] && g_args.bool_output[a->byte_idx][a->bit_idx]) {
                snprintf(out, outsz, "%d", *g_args.bool_output[a->byte_idx][a->bit_idx] ? 1 : 0);
                rc = 0;
            }
            break;
        case Z_SIZE_B:
            if (g_args.byte_output && g_args.byte_output[a->byte_idx]) {
                snprintf(out, outsz, "%u", (unsigned)*g_args.byte_output[a->byte_idx]);
                rc = 0;
            }
            break;
        case Z_SIZE_W:
            if (g_args.int_output && g_args.int_output[a->byte_idx]) {
                snprintf(out, outsz, "%u", (unsigned)*g_args.int_output[a->byte_idx]);
                rc = 0;
            }
            break;
        case Z_SIZE_D:
            if (g_args.dint_output && g_args.dint_output[a->byte_idx]) {
                snprintf(out, outsz, "%lu", (unsigned long)*g_args.dint_output[a->byte_idx]);
                rc = 0;
            }
            break;
        case Z_SIZE_L:
            if (g_args.lint_output && g_args.lint_output[a->byte_idx]) {
                snprintf(out, outsz, "%llu", (unsigned long long)*g_args.lint_output[a->byte_idx]);
                rc = 0;
            }
            break;
        }
        break;
    case Z_AREA_M:
        switch (a->size) {
        case Z_SIZE_X:
            if (g_args.bool_memory[a->byte_idx] && g_args.bool_memory[a->byte_idx][a->bit_idx]) {
                snprintf(out, outsz, "%d", *g_args.bool_memory[a->byte_idx][a->bit_idx] ? 1 : 0);
                rc = 0;
            }
            break;
        case Z_SIZE_W:
            if (g_args.int_memory && g_args.int_memory[a->byte_idx]) {
                snprintf(out, outsz, "%u", (unsigned)*g_args.int_memory[a->byte_idx]);
                rc = 0;
            }
            break;
        case Z_SIZE_D:
            if (g_args.dint_memory && g_args.dint_memory[a->byte_idx]) {
                snprintf(out, outsz, "%lu", (unsigned long)*g_args.dint_memory[a->byte_idx]);
                rc = 0;
            }
            break;
        case Z_SIZE_L:
            if (g_args.lint_memory && g_args.lint_memory[a->byte_idx]) {
                snprintf(out, outsz, "%llu", (unsigned long long)*g_args.lint_memory[a->byte_idx]);
                rc = 0;
            }
            break;
        default:
            break;
        }
        break;
    }
    if (g_args.image_unlock) g_args.image_unlock();
    return rc;
}

static int zenoh_write_var(const zenoh_iec_addr_t *a, const char *val) {
    if (!a || !val) return -1;
    if (g_args.image_lock) g_args.image_lock();
    int rc = -1;
    switch (a->area) {
    case Z_AREA_I:
        switch (a->size) {
        case Z_SIZE_X:
            if (g_args.bool_input[a->byte_idx] && g_args.bool_input[a->byte_idx][a->bit_idx]) {
                *g_args.bool_input[a->byte_idx][a->bit_idx] = (val[0] == '1') ? 1 : 0;
                rc = 0;
            }
            break;
        case Z_SIZE_B:
            if (g_args.byte_input && g_args.byte_input[a->byte_idx]) {
                *g_args.byte_input[a->byte_idx] = (IEC_BYTE)strtoul(val, NULL, 10);
                rc = 0;
            }
            break;
        case Z_SIZE_W:
            if (g_args.int_input && g_args.int_input[a->byte_idx]) {
                *g_args.int_input[a->byte_idx] = (IEC_UINT)strtoul(val, NULL, 10);
                rc = 0;
            }
            break;
        case Z_SIZE_D:
            if (g_args.dint_input && g_args.dint_input[a->byte_idx]) {
                *g_args.dint_input[a->byte_idx] = (IEC_UDINT)strtoul(val, NULL, 10);
                rc = 0;
            }
            break;
        case Z_SIZE_L:
            if (g_args.lint_input && g_args.lint_input[a->byte_idx]) {
                *g_args.lint_input[a->byte_idx] = (IEC_ULINT)strtoull(val, NULL, 10);
                rc = 0;
            }
            break;
        }
        break;
    case Z_AREA_Q:
        switch (a->size) {
        case Z_SIZE_X:
            if (g_args.bool_output[a->byte_idx] && g_args.bool_output[a->byte_idx][a->bit_idx]) {
                *g_args.bool_output[a->byte_idx][a->bit_idx] = (val[0] == '1') ? 1 : 0;
                rc = 0;
            }
            break;
        case Z_SIZE_B:
            if (g_args.byte_output && g_args.byte_output[a->byte_idx]) {
                *g_args.byte_output[a->byte_idx] = (IEC_BYTE)strtoul(val, NULL, 10);
                rc = 0;
            }
            break;
        case Z_SIZE_W:
            if (g_args.int_output && g_args.int_output[a->byte_idx]) {
                *g_args.int_output[a->byte_idx] = (IEC_UINT)strtoul(val, NULL, 10);
                rc = 0;
            }
            break;
        case Z_SIZE_D:
            if (g_args.dint_output && g_args.dint_output[a->byte_idx]) {
                *g_args.dint_output[a->byte_idx] = (IEC_UDINT)strtoul(val, NULL, 10);
                rc = 0;
            }
            break;
        case Z_SIZE_L:
            if (g_args.lint_output && g_args.lint_output[a->byte_idx]) {
                *g_args.lint_output[a->byte_idx] = (IEC_ULINT)strtoull(val, NULL, 10);
                rc = 0;
            }
            break;
        }
        break;
    case Z_AREA_M:
        switch (a->size) {
        case Z_SIZE_X:
            if (g_args.bool_memory[a->byte_idx] && g_args.bool_memory[a->byte_idx][a->bit_idx]) {
                *g_args.bool_memory[a->byte_idx][a->bit_idx] = (val[0] == '1') ? 1 : 0;
                rc = 0;
            }
            break;
        case Z_SIZE_W:
            if (g_args.int_memory && g_args.int_memory[a->byte_idx]) {
                *g_args.int_memory[a->byte_idx] = (IEC_UINT)strtoul(val, NULL, 10);
                rc = 0;
            }
            break;
        case Z_SIZE_D:
            if (g_args.dint_memory && g_args.dint_memory[a->byte_idx]) {
                *g_args.dint_memory[a->byte_idx] = (IEC_UDINT)strtoul(val, NULL, 10);
                rc = 0;
            }
            break;
        case Z_SIZE_L:
            if (g_args.lint_memory && g_args.lint_memory[a->byte_idx]) {
                *g_args.lint_memory[a->byte_idx] = (IEC_ULINT)strtoull(val, NULL, 10);
                rc = 0;
            }
            break;
        default:
            break;
        }
        break;
    }
    if (g_args.image_unlock) g_args.image_unlock();
    return rc;
}

/* ------------------------------------------------------------------ */
/* Config file loading (editor-generated JSON via cJSON)              */
/* ------------------------------------------------------------------ */

/* Read an entire file into a heap buffer. Returns NULL on failure. */
static char *zenoh_read_file(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long len = ftell(f);
    if (len <= 0 || len > 1024 * 1024) {
        fclose(f);
        return NULL;
    }
    rewind(f);
    char *buf = (char *)malloc((size_t)len + 1);
    if (!buf) {
        fclose(f);
        return NULL;
    }
    size_t got = fread(buf, 1, (size_t)len, f);
    fclose(f);
    buf[got] = '\0';
    return buf;
}

/* Defaults matching the legacy hard-coded behaviour. */
static void zenoh_config_set_defaults(zenoh_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = 1;
    snprintf(cfg->mode, sizeof(cfg->mode), "%s", "client");
    const char *connect = getenv("ZENOH_CONNECT");
    const char *listen = getenv("ZENOH_LISTEN");
    if (connect && connect[0]) snprintf(cfg->connect, sizeof(cfg->connect), "%s", connect);
    if (listen && listen[0]) snprintf(cfg->listen, sizeof(cfg->listen), "%s", listen);
    const char *mode = getenv("ZENOH_MODE");
    if (mode && mode[0]) snprintf(cfg->mode, sizeof(cfg->mode), "%s", mode);
}

static int zenoh_config_add_topic(zenoh_config_t *cfg, const char *topic, const char *direction,
                                  const char *variable, const char *data_type) {
    if (!topic || !direction || !variable || cfg->num_topics >= ZENOH_MAX_TOPICS) return -1;
    zenoh_topic_t *t = &cfg->topics[cfg->num_topics];
    memset(t, 0, sizeof(*t));
    snprintf(t->topic, sizeof(t->topic), "%s", topic);
    snprintf(t->variable, sizeof(t->variable), "%s", variable);
    if (data_type) snprintf(t->data_type, sizeof(t->data_type), "%s", data_type);
    if (strcmp(direction, "subscribe") == 0)
        t->direction = 1;
    else
        t->direction = 0;
    t->addr_valid = (zenoh_parse_iec_addr(t->variable, &t->addr) == 0);
    if (!t->addr_valid)
        plugin_logger_warn(&g_logger, "invalid IEC location '%s' for topic '%s', skipping",
                           t->variable, t->topic);
    cfg->num_topics++;
    return 0;
}

/* Load the editor-generated config. Returns 0 when a config was applied
 * (even if partially), -1 when nothing usable was found. */
static int zenoh_load_config(const char *path, zenoh_config_t *cfg) {
    char *buf = zenoh_read_file(path);
    if (!buf) return -1;

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        plugin_logger_error(&g_logger, "failed to parse config %s", path);
        return -1;
    }

    /* Contract gate (mirror of the editor's ZENOH_CONFIG_FORMAT_VERSION):
     * refuse configs from an incompatible editor rather than half-applying
     * them. -2 signals "config present but unsupported" so the caller does
     * NOT fall back to env defaults (the config exists; it is just stale). */
    const cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "version");
    int fmt_ver = cJSON_IsNumber(version) ? version->valueint : 0;
    if (fmt_ver < ZENOH_CONFIG_MIN_FORMAT_VERSION) {
        plugin_logger_error(&g_logger,
                            "unsupported zenoh.json format_version %d (runtime requires >= %d); "
                            "bridge stays down",
                            fmt_ver, ZENOH_CONFIG_MIN_FORMAT_VERSION);
        cJSON_Delete(root);
        return -2;
    }

    zenoh_config_set_defaults(cfg);

    const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(root, "enabled");
    if (cJSON_IsBool(enabled)) cfg->enabled = enabled->valueint != 0;

    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(root, "mode");
    if (cJSON_IsString(mode) && mode->valuestring && mode->valuestring[0])
        snprintf(cfg->mode, sizeof(cfg->mode), "%s", mode->valuestring);

    const cJSON *connect = cJSON_GetObjectItemCaseSensitive(root, "connect");
    if (cJSON_IsString(connect) && connect->valuestring)
        snprintf(cfg->connect, sizeof(cfg->connect), "%s", connect->valuestring);

    const cJSON *listen = cJSON_GetObjectItemCaseSensitive(root, "listen");
    if (cJSON_IsString(listen) && listen->valuestring)
        snprintf(cfg->listen, sizeof(cfg->listen), "%s", listen->valuestring);

    const cJSON *topics = cJSON_GetObjectItemCaseSensitive(root, "topics");
    if (cJSON_IsArray(topics)) {
        const cJSON *item = NULL;
        cJSON_ArrayForEach(item, topics) {
            const cJSON *topic = cJSON_GetObjectItemCaseSensitive(item, "topic");
            const cJSON *direction = cJSON_GetObjectItemCaseSensitive(item, "direction");
            const cJSON *variable = cJSON_GetObjectItemCaseSensitive(item, "variable");
            const cJSON *data_type = cJSON_GetObjectItemCaseSensitive(item, "data_type");
            if (cJSON_IsString(topic) && cJSON_IsString(direction) && cJSON_IsString(variable)) {
                zenoh_config_add_topic(cfg, topic->valuestring, direction->valuestring,
                                       variable->valuestring,
                                       cJSON_IsString(data_type) ? data_type->valuestring : NULL);
            }
        }
    }

    cJSON_Delete(root);
    plugin_logger_info(&g_logger, "loaded config %s (version=%d, enabled=%d, mode=%s, "
                                  "connect=%s, topics=%d)",
                       path, fmt_ver, cfg->enabled, cfg->mode, cfg->connect, cfg->num_topics);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Subscriber callback: bus -> PLC                                    */
/* ------------------------------------------------------------------ */

static void on_zenoh_msg(z_loaned_sample_t *sample, void *ctx) {
    zenoh_topic_t *t = (zenoh_topic_t *)ctx;
    if (!t || !t->addr_valid) return;

    z_owned_string_t s;
    z_bytes_to_string(z_sample_payload(sample), &s);
    const char *val = z_string_data(z_loan(s));
    size_t vlen = val ? z_string_len(z_loan(s)) : 0;

    /* Copy into a bounded, NUL-terminated buffer so we never read past the
     * payload (and the log stays readable). */
    char vbuf[64];
    if (!val || vlen == 0) {
        snprintf(vbuf, sizeof(vbuf), "0");
    } else {
        size_t n = vlen < sizeof(vbuf) - 1 ? vlen : sizeof(vbuf) - 1;
        memcpy(vbuf, val, n);
        vbuf[n] = '\0';
    }

    zenoh_write_var(&t->addr, vbuf);
    plugin_logger_info(&g_logger, "'%s' <- '%s' (wrote %s to %s)", t->topic, vbuf, vbuf,
                       t->variable);

    z_drop(z_move(s));
}

/* ------------------------------------------------------------------ */
/* Plugin lifecycle                                                    */
/* ------------------------------------------------------------------ */

/* The runtime gates cycle_start/cycle_end on plugin->running, which it
 * only sets when start_loop() returns 0, and it calls stop_loop()/start_loop()
 * around program start/stop. So the real work happens in start_loop() —
 * matching s7comm/opcua/modbus: it (re)loads the config (picking up a
 * freshly re-uploaded conf/zenoh.json), opens the session and declares the
 * topics. init() only stores the runtime args and sets up logging. */

static void zenoh_undeclare_all(void) {
    for (int i = 0; i < g_cfg.num_topics; i++) {
        zenoh_topic_t *t = &g_cfg.topics[i];
        if (t->pub_valid) {
            z_undeclare_publisher(z_move(t->pub));
            t->pub_valid = 0;
        }
        if (t->sub_valid) {
            z_undeclare_subscriber(z_move(t->sub));
            t->sub_valid = 0;
        }
    }
}

int init(void *args) {
    if (!args) {
        plugin_logger_init(&g_logger, "ZENOH", NULL);
        plugin_logger_error(&g_logger, "init args is NULL");
        return -1;
    }
    /* The runtime frees args right after init() returns — copy it now. */
    memcpy(&g_args, args, sizeof(plugin_runtime_args_t));

    /* Route log messages through the runtime's central logging so they
     * appear in the editor's log viewer (falls back to stdout). */
    plugin_logger_init(&g_logger, "ZENOH", args);
    plugin_logger_info(&g_logger, "Zenoh bridge loaded (config read at start_loop)");
    return 0;
}

int start_loop(void) {
    if (g_active) return 0;

    /* Prefer the config path the runtime hands us (plugins.conf entry,
     * updated on every upload); fall back to the raw uploaded location. */
    const char *config_path = g_args.plugin_specific_config_file_path;
    if (!config_path || config_path[0] == '\0') config_path = ZENOH_FALLBACK_CONFIG;

    int rc = zenoh_load_config(config_path, &g_cfg);
    if (rc == -2) {
        /* Config exists but was written by an incompatible editor. Refuse
         * loudly instead of bridging the wrong data (same stance as the
         * OPC-UA plugin's format_version gate). */
        g_configured_enabled = 0;
        return 0;
    }
    if (rc != 0) {
        /* No editor config (first boot / no connector) — legacy env mode. */
        zenoh_config_set_defaults(&g_cfg);
        zenoh_config_add_topic(&g_cfg, "openplc/plc1/output/0", "publish", "%QX0.0", "BOOL");
        zenoh_config_add_topic(&g_cfg, "openplc/plc1/cmd/0", "subscribe", "%IX0.0", "BOOL");
        plugin_logger_warn(&g_logger, "no config file, using env defaults (mode=%s, connect=%s)",
                           g_cfg.mode, g_cfg.connect);
    }

    if (!g_cfg.enabled) {
        g_configured_enabled = 0;
        plugin_logger_info(&g_logger, "disabled by config (%s) — bridging off", config_path);
        return 0;
    }
    g_configured_enabled = 1;

    z_owned_config_t zconfig;
    z_config_default(&zconfig);
    zp_config_insert(z_loan_mut(zconfig), Z_CONFIG_MODE_KEY, g_cfg.mode);
    if (g_cfg.connect[0] != '\0')
        zp_config_insert(z_loan_mut(zconfig), Z_CONFIG_CONNECT_KEY, g_cfg.connect);
    if (g_cfg.listen[0] != '\0')
        zp_config_insert(z_loan_mut(zconfig), Z_CONFIG_LISTEN_KEY, g_cfg.listen);

    if (z_open(&g_session, z_move(zconfig), NULL) < 0) {
        plugin_logger_error(&g_logger, "failed to open session (connect=%s)", g_cfg.connect);
        return -1;
    }

    /* z_open() auto-starts the read/lease tasks. Do NOT call
     * zp_start_read_task()/zp_start_lease_task() — in zenoh-pico 1.9.0
     * the session registers at the router but no data ever flows. */

    for (int i = 0; i < g_cfg.num_topics; i++) {
        zenoh_topic_t *t = &g_cfg.topics[i];
        if (!t->addr_valid) continue;

        z_view_keyexpr_t ke;
        z_view_keyexpr_from_str(&ke, t->topic);

        if (t->direction == 0) {
            if (z_declare_publisher(z_loan(g_session), &t->pub, z_loan(ke), NULL) < 0) {
                plugin_logger_error(&g_logger, "failed to declare publisher for '%s'", t->topic);
                continue;
            }
            t->pub_valid = 1;
            plugin_logger_info(&g_logger, "publishing %s -> '%s'", t->variable, t->topic);
        } else {
            z_owned_closure_sample_t callback;
            z_closure(&callback, on_zenoh_msg, NULL, (void *)t);
            if (z_declare_subscriber(z_loan(g_session), &t->sub, z_loan(ke), z_move(callback), NULL) < 0) {
                plugin_logger_error(&g_logger, "failed to declare subscriber for '%s'", t->topic);
                continue;
            }
            t->sub_valid = 1;
            plugin_logger_info(&g_logger, "subscribing '%s' -> %s", t->topic, t->variable);
        }
    }

    g_active = 1;
    plugin_logger_info(&g_logger, "bridge started (mode=%s, connect=%s, topics=%d)",
                       g_cfg.mode, g_cfg.connect, g_cfg.num_topics);
    return 0;
}

/* End of scan: publish every configured output. The runtime holds the
 * image lock across cycle hooks, so the (recursive) lock in
 * zenoh_read_var is a re-entrant no-op. Keep fast — scan critical path. */
void cycle_end(void) {
    if (!g_active || !g_configured_enabled) return;
    for (int i = 0; i < g_cfg.num_topics; i++) {
        zenoh_topic_t *t = &g_cfg.topics[i];
        if (t->direction != 0 || !t->addr_valid || !t->pub_valid) continue;
        char v[32];
        if (zenoh_read_var(&t->addr, v, sizeof(v)) != 0) continue;
        z_owned_bytes_t payload;
        /* z_bytes_copy_from_str, not _buf (which produced garbage payloads). */
        z_bytes_copy_from_str(&payload, v);
        z_publisher_put(z_loan(t->pub), z_move(payload), NULL);
    }
}

void stop_loop(void) {
    if (!g_active) return;
    g_active = 0;
    zenoh_undeclare_all();
    /* z_close() stops the auto-started read/lease tasks, so a subsequent
     * start_loop() re-reads config and opens a fresh session. */
    z_close(z_loan_mut(g_session), NULL);
}

void cleanup(void) {
    if (g_active) stop_loop();
}
