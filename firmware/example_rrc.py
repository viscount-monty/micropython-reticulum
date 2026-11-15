"""
µReticulum Example: RRC Relay Chat Command Bot
==============================================
Joins an RRC (Reticulum Relay Chat) hub (`rrcd`), joins `commands`, and
emulates a command bot that responds to commands providing information about:
  - the ESP32-S3 MCU (`!esp32`)
  - the SX1262 LoRa radio (`!sx1262`)
  - the micropython-reticulum (`urns`) instance (`!rns`)
  - GPIO pin control and reading (`!gpio get`, `!gpio set`, `!gpio list`, `!led`)

RRC specification: https://rrc.kc1awv.net/
Reference hub (`rrcd`): https://github.com/kc1awv/rrcd

Note on command prefix:
  `rrcd` intercepts any message starting with `/` as a hub slash command and
  never forwards unrecognized `/` commands to room members. This bot uses `!`
  (`BOT_PREFIX = "!"`) so bot commands never conflict with RRC `/` commands.

Running on Seeed XIAO ESP32-S3 with `mpremote`:
  # Run directly from host without copying to flash:
  mpremote run firmware/example_rrc.py

  # Or copy to device flash and run via REPL / exec:
  mpremote cp firmware/example_rrc.py :
  mpremote exec "import example_rrc"

Bot commands in `commands`:
  !help                                      - List bot commands
  !esp32   (or !esp32s3, !mcu, !sys)         - ESP32-S3 MCU & memory status
  !sx1262  (or !radio, !lora)                - SX1262 LoRa radio configuration & stats
  !rns     (or !reticulum, !urns, !net)      - micropython-reticulum & RRC status
  !gpio get <pin> [in|pu|pd]                 - Read digital state of GPIO <pin> (e.g. 4 or d3)
  !gpio set <pin> <0|1|on|off|high|low|toggle> - Set GPIO <pin> output state
  !gpio list                                 - Show configured & reserved GPIOs
  !led [on|off|toggle|status]                - Control or read built-in LED (GPIO21)
"""

from config import WIFI_SSID, WIFI_PASS, NODE_NAME, DEBUG, CONFIG

import gc
import os
import sys
import time

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

from machine import Pin

gc.collect()

_BOOT_TIME = time.time()

# RRC timestamps are Unix-epoch milliseconds. Bare-metal MicroPython ports
# use a 2000-01-01 epoch for time.time()/gmtime(), so shift when needed.
try:
    UNIX_EPOCH_OFFSET = 946684800 if time.gmtime(0)[0] == 2000 else 0
except Exception:
    UNIX_EPOCH_OFFSET = 0


def _unix_ms():
    return int((time.time() + UNIX_EPOCH_OFFSET) * 1000)


# ---- RRC settings ----
HUB_DEST_HASH = "a8b7055fc51c6d9e317ecd839b969981"
ROOM_NAME = "commands"
RRC_NICK = NODE_NAME or "esp32s3-bot"
CLIENT_NAME = "micropython-rrc-bot"
CLIENT_VERSION = "0.2"
RECONNECT_DELAY = 10

# Bot command prefix. Must NOT be "/" because rrcd intercepts "/" for hub
# operator/slash commands (/help, /rooms, /join, /part, /who, /topic, etc.).
BOT_PREFIX = "!"
if BOT_PREFIX.startswith("/"):
    raise ValueError("BOT_PREFIX must not start with '/' (reserved by RRC hub)")

# Handshake & half-duplex LoRa pacing parameters (seconds)
IDENTIFY_DELAY = 1.5
HELLO_DELAY = 1.5
JOIN_DELAY = 2.0
HANDSHAKE_RETRY_MIN = 12.0
MAX_HANDSHAKE_RETRIES = 5
TX_PACE_DELAY = 0.8
RX_SETTLE_DELAY = 0.5

# Built-in LED. XIAO ESP32-S3 uses GPIO21 and is active-low.
LED_PIN = 21
LED_ACTIVE_LOW = True

# ESP32-S3 valid GPIO range and system-reserved pins:
#   19, 20: USB D-/D+ (native USB Serial/JTAG used by mpremote!)
#   26..37: Internal SPI Flash / Octal PSRAM
# Active SX1262 SPI/control pins from LoRaInterface are added dynamically.
GPIO_MIN_PIN = 0
GPIO_MAX_PIN = 48
SYSTEM_RESERVED_PINS = set([19, 20] + list(range(26, 38)))
DEFAULT_RADIO_PINS = set([7, 8, 9, 39, 40, 41, 42])

# Seeed XIAO ESP32-S3 header silkscreen pin aliases -> ESP32-S3 GPIO numbers
XIAO_PIN_ALIASES = {
    "d0": 1, "a0": 1,
    "d1": 2, "a1": 2,
    "d2": 3, "a2": 3,
    "d3": 4, "a3": 4,
    "d4": 5, "a4": 5, "sda": 5,
    "d5": 6, "a5": 6, "scl": 6,
    "d6": 43, "tx": 43,
    "d7": 44, "rx": 44,
    "d8": 7, "sck": 7,
    "d9": 8, "miso": 8,
    "d10": 9, "mosi": 9,
    "led": 21,
}

XIAO_PIN_LABELS = {
    1: "D0/A0",
    2: "D1/A1",
    3: "D2/A2",
    4: "D3/A3",
    5: "D4/SDA",
    6: "D5/SCL",
    7: "D8/SCK",
    8: "D9/MISO",
    9: "D10/MOSI",
    21: "LED",
    43: "D6/TX",
    44: "D7/RX",
}


def _parse_pin(token):
    s = str(token).strip().lower()
    if s in XIAO_PIN_ALIASES:
        return XIAO_PIN_ALIASES[s]
    if s.startswith("gpio"):
        s = s[4:]
    return int(s)


def _pin_label(pin_no):
    lbl = XIAO_PIN_LABELS.get(pin_no)
    return ("GPIO%d(%s)" % (pin_no, lbl)) if lbl else ("GPIO%d" % pin_no)

# ---- RRC protocol constants (https://rrc.kc1awv.net/) ----
RRC_VERSION = 1

# Envelope keys (1-RRC)
K_V = 0
K_T = 1
K_ID = 2
K_TS = 3
K_SRC = 4
K_ROOM = 5
K_BODY = 6
K_NICK = 7
K_DST = 8

# Message types (1-RRC)
T_HELLO = 1
T_WELCOME = 2
T_JOIN = 10
T_JOINED = 11
T_PART = 12
T_PARTED = 13
T_MSG = 20
T_NOTICE = 21
T_ACTION = 22
T_PING = 30
T_PONG = 31
T_ERROR = 40
T_RESOURCE_ENVELOPE = 50

# HELLO body keys (3-RRC)
B_HELLO_NAME = 0
B_HELLO_VER = 1
B_HELLO_CAPS = 2

# WELCOME body keys (3-RRC)
B_WELCOME_HUB = 0
B_WELCOME_VER = 1
B_WELCOME_CAPS = 2
B_WELCOME_LIMITS = 3

# Capability keys (3-RRC)
CAP_RESOURCE_ENVELOPE = 0
CAP_ACTION = 1
CAP_DIRECT_NOTICE = 2

# Limit keys in B_WELCOME_LIMITS (3-RRC / rrcd)
B_LIMIT_MAX_NICK_BYTES = 0
B_LIMIT_MAX_ROOM_NAME_BYTES = 1
B_LIMIT_MAX_MSG_BODY_BYTES = 2
B_LIMIT_MAX_ROOMS_PER_SESSION = 3
B_LIMIT_RATE_LIMIT_MSGS_PER_MINUTE = 4

DEFAULT_LIMITS = {
    B_LIMIT_MAX_NICK_BYTES: 32,
    B_LIMIT_MAX_ROOM_NAME_BYTES: 64,
    B_LIMIT_MAX_MSG_BODY_BYTES: 350,
    B_LIMIT_MAX_ROOMS_PER_SESSION: 32,
    B_LIMIT_RATE_LIMIT_MSGS_PER_MINUTE: 240,
}

# RESOURCE_ENVELOPE body keys (2-RRC)
B_RES_ID = 0
B_RES_KIND = 1
B_RES_SIZE = 2
B_RES_SHA256 = 3
B_RES_ENCODING = 4

LIMIT_NAMES = {
    B_LIMIT_MAX_NICK_BYTES: "max_nick_bytes",
    B_LIMIT_MAX_ROOM_NAME_BYTES: "max_room_name_bytes",
    B_LIMIT_MAX_MSG_BODY_BYTES: "max_msg_body_bytes",
    B_LIMIT_MAX_ROOMS_PER_SESSION: "max_rooms_per_session",
    B_LIMIT_RATE_LIMIT_MSGS_PER_MINUTE: "rate_limit_msgs_per_minute",
}

CAP_NAMES = {
    CAP_RESOURCE_ENVELOPE: "resource_envelope",
    CAP_ACTION: "action",
    CAP_DIRECT_NOTICE: "direct_notice",
}

# ---- LED & GPIO state ----
try:
    led = Pin(LED_PIN, Pin.OUT)
except Exception:
    led = None

_led_state = False


def led_set(on):
    global _led_state
    _led_state = bool(on)
    if led is not None:
        led.value((0 if _led_state else 1) if LED_ACTIVE_LOW else (1 if _led_state else 0))


def led_get():
    if led is not None:
        raw = led.value()
        return (raw == 0) if LED_ACTIVE_LOW else (raw != 0)
    return _led_state


led_set(False)


# ---- Minimal CBOR (RRC uses CBOR maps per RFC 8949, not msgpack) ----


def _cbor_head(major, value):
    if value < 24:
        return bytes([(major << 5) | value])
    if value < 256:
        return bytes([(major << 5) | 24, value])
    if value < 65536:
        return bytes([(major << 5) | 25]) + value.to_bytes(2, "big")
    if value < 4294967296:
        return bytes([(major << 5) | 26]) + value.to_bytes(4, "big")
    return bytes([(major << 5) | 27]) + value.to_bytes(8, "big")


def cbor_encode(obj):
    if obj is None:
        return b"\xf6"
    if obj is True:
        return b"\xf5"
    if obj is False:
        return b"\xf4"
    if isinstance(obj, int):
        return _cbor_head(0, obj) if obj >= 0 else _cbor_head(1, -1 - obj)
    if isinstance(obj, (bytes, bytearray)):
        return _cbor_head(2, len(obj)) + bytes(obj)
    if isinstance(obj, str):
        raw = obj.encode("utf-8")
        return _cbor_head(3, len(raw)) + raw
    if isinstance(obj, (list, tuple)):
        out = _cbor_head(4, len(obj))
        for item in obj:
            out += cbor_encode(item)
        return out
    if isinstance(obj, dict):
        out = _cbor_head(5, len(obj))
        for key in obj:
            out += cbor_encode(key) + cbor_encode(obj[key])
        return out
    raise TypeError("cannot CBOR encode " + str(type(obj)))


def _cbor_read(data, pos):
    if pos >= len(data):
        raise ValueError("unexpected end of CBOR data")
    initial = data[pos]
    pos += 1
    major = initial >> 5
    info = initial & 31

    if info == 31:
        if major in (2, 3):
            chunks = b""
            text = ""
            while data[pos] != 0xFF:
                value, pos = _cbor_read(data, pos)
                if major == 3:
                    text += value
                else:
                    chunks += value
            pos += 1
            return (text if major == 3 else chunks), pos
        if major == 4:
            items = []
            while data[pos] != 0xFF:
                value, pos = _cbor_read(data, pos)
                items.append(value)
            return items, pos + 1
        if major == 5:
            items = {}
            while data[pos] != 0xFF:
                key, pos = _cbor_read(data, pos)
                value, pos = _cbor_read(data, pos)
                items[key] = value
            return items, pos + 1
        raise ValueError("unsupported indefinite CBOR value")

    if info < 24:
        value = info
    elif info == 24:
        value = data[pos]
        pos += 1
    elif info == 25:
        value = int.from_bytes(data[pos : pos + 2], "big")
        pos += 2
    elif info == 26:
        value = int.from_bytes(data[pos : pos + 4], "big")
        pos += 4
    elif info == 27:
        value = int.from_bytes(data[pos : pos + 8], "big")
        pos += 8
    else:
        raise ValueError("unsupported CBOR additional info")

    if major == 0:
        return value, pos
    if major == 1:
        return -1 - value, pos
    if major == 2:
        end = pos + value
        if end > len(data):
            raise ValueError("truncated CBOR bytes")
        return bytes(data[pos:end]), end
    if major == 3:
        end = pos + value
        if end > len(data):
            raise ValueError("truncated CBOR text")
        return bytes(data[pos:end]).decode("utf-8"), end
    if major == 4:
        items = []
        for _ in range(value):
            item, pos = _cbor_read(data, pos)
            items.append(item)
        return items, pos
    if major == 5:
        items = {}
        for _ in range(value):
            key, pos = _cbor_read(data, pos)
            item, pos = _cbor_read(data, pos)
            items[key] = item
        return items, pos
    if major == 7:
        if info == 20:
            return False, pos
        if info == 21:
            return True, pos
        if info in (22, 23):
            return None, pos
    raise ValueError("unsupported CBOR major type " + str(major))


def cbor_decode(data):
    value, pos = _cbor_read(bytes(data), 0)
    if pos != len(data):
        raise ValueError("trailing CBOR bytes")
    return value


# ---- Formatting & validation helpers ----


def _is_int(val):
    return isinstance(val, int) and not isinstance(val, bool)


def _hex(data):
    if isinstance(data, (bytes, bytearray)):
        return bytes(data).hex()
    return "?" if data is None else str(data)


def _short(text, limit=160):
    text = str(text)
    return text if len(text) <= limit else text[:limit] + "...(" + str(len(text)) + " chars)"


def _fmt_ms(ms):
    if not _is_int(ms):
        return str(ms)
    try:
        sec = ms // 1000 - UNIX_EPOCH_OFFSET
        tm = time.gmtime(sec)
        return "%04d-%02d-%02d %02d:%02d:%02d.%03d UTC" % (
            tm[0], tm[1], tm[2], tm[3], tm[4], tm[5], ms % 1000,
        )
    except Exception:
        return str(ms) + " ms"


def _fmt_uptime(seconds):
    seconds = max(0, int(seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    if h > 0:
        return "%dh%02dm%02ds" % (h, m, s)
    return ("%dm%02ds" % (m, s)) if m > 0 else ("%ds" % s)


def _describe(value, depth=0):
    if isinstance(value, dict):
        if depth >= 2:
            return "{%d entries}" % len(value)
        return "{" + ", ".join("%s=%s" % (k, _describe(value[k], depth + 1)) for k in value) + "}"
    if isinstance(value, (list, tuple)):
        if depth >= 2:
            return "[%d items]" % len(value)
        return "[" + ", ".join(_describe(x, depth + 1) for x in value) + "]"
    if isinstance(value, (bytes, bytearray)):
        raw = bytes(value)
        return ("0x" + raw.hex()) if len(raw) <= 8 else ("0x%s...(%dB)" % (raw[:8].hex(), len(raw)))
    if isinstance(value, str):
        return _short(value, 120)
    return str(value)


def _describe_named(value, names, depth=0):
    if isinstance(value, dict):
        return "{" + ", ".join("%s=%s" % (names.get(k, str(k)), _describe(value[k], depth + 1)) for k in value) + "}"
    if isinstance(value, (list, tuple)):
        return _describe(value, depth)
    return str(value)


def _body_text(body):
    if isinstance(body, str):
        return body
    if isinstance(body, (bytes, bytearray)):
        try:
            return bytes(body).decode("utf-8")
        except Exception:
            return "0x" + bytes(body).hex()
    return _describe(body)


def _identity_list(body):
    if not isinstance(body, (list, tuple)):
        return []
    return [bytes(x) for x in body if isinstance(x, (bytes, bytearray))]


def _truncate_utf8(text, max_bytes):
    if not isinstance(text, str):
        text = str(text)
    raw = text.encode("utf-8")
    if len(raw) <= max_bytes:
        return text
    cut = raw[:max_bytes]
    while cut:
        try:
            return cut.decode("utf-8")
        except Exception:
            cut = cut[:-1]
    return ""


def _split_utf8(text, max_bytes):
    """Split text into chunks whose UTF-8 byte lengths are each <= max_bytes."""
    if not isinstance(text, str):
        text = str(text)
    max_bytes = max(16, max_bytes)
    if len(text.encode("utf-8")) <= max_bytes:
        return [text]

    chunks = []
    remaining = text
    while remaining:
        if len(remaining.encode("utf-8")) <= max_bytes:
            chunks.append(remaining)
            break
        piece = _truncate_utf8(remaining, max_bytes)
        if not piece:
            break
        split_at = -1
        for sep in ("\n", " | ", " "):
            idx = piece.rfind(sep)
            if idx >= len(piece) // 2:
                split_at = idx
                break
        if split_at > 0:
            head = remaining[:split_at].rstrip()
            remaining = remaining[split_at:].lstrip()
        else:
            head = piece
            remaining = remaining[len(piece):]
        if head:
            chunks.append(head)
    return chunks or [""]


def validate_envelope(env):
    """Validate an incoming RRC envelope per 1-RRC / rrcd envelope.py."""
    if not isinstance(env, dict):
        raise ValueError("envelope must be a CBOR map")
    for req in (K_V, K_T, K_ID, K_TS, K_SRC):
        if req not in env:
            raise ValueError("missing required key " + str(req))
    v = env[K_V]
    if not _is_int(v) or v != RRC_VERSION:
        raise ValueError("unsupported protocol version " + str(v))
    if not _is_int(env[K_T]):
        raise ValueError("invalid t type")
    mid = env[K_ID]
    if not isinstance(mid, (bytes, bytearray)) or len(mid) != 8:
        raise ValueError("id must be 8 bytes")
    ts = env[K_TS]
    if not _is_int(ts) or ts < 0:
        raise ValueError("invalid ts")
    src = env[K_SRC]
    if not isinstance(src, (bytes, bytearray)) or len(src) != 16:
        raise ValueError("src must be 16-byte identity hash")
    if K_ROOM in env and env[K_ROOM] is not None:
        if not isinstance(env[K_ROOM], str) or not env[K_ROOM].strip():
            raise ValueError("room must be non-empty string")
    if K_NICK in env and env[K_NICK] is not None and not isinstance(env[K_NICK], str):
        raise ValueError("nick must be string")
    if K_DST in env and env[K_DST] is not None and not isinstance(env[K_DST], (bytes, bytearray)):
        raise ValueError("dst must be bytes")
    return True


def _hub_hash_bytes():
    value = HUB_DEST_HASH.strip().lower()
    if value.startswith("0x"):
        value = value[2:]
    if len(value) != 32:
        raise ValueError("HUB_DEST_HASH must be exactly 32 hex characters")
    return bytes.fromhex(value)


# ---- RRC Command Bot & Client ----


class RRCClient:
    def __init__(self, rns, hub_hash, room, nick):
        self.rns = rns
        self.identity = rns.identity
        self.hub_hash = hub_hash
        self.room = room.strip().lower()
        self.nick = nick
        self.link = None
        self.connected = False
        self.identified = False
        self.welcomed = False
        self.joined = False
        self.awaiting_joined = False
        self.closed = True
        self.members = {}
        self.sent_ids = []
        self.pending_resource = None
        self.connected_at = 0
        self.welcomed_at = 0
        self.joined_at = 0
        self.hello_sent_at = 0
        self.hello_attempts = 0
        self.join_sent_at = 0
        self.join_attempts = 0
        self.last_rx_at = 0
        self.last_tx_at = 0
        self.tx_queue = []
        self.hub_caps = {}
        self.limits = dict(DEFAULT_LIMITS)
        self.session_no = 0
        self.msgs_in = 0
        self.msgs_out = 0
        self.bytes_in = 0
        self.bytes_out = 0
        self.cmds_handled = 0
        self.gpio_pins = {}
        self.gpio_modes = {}

    def _find_lora_iface(self):
        for iface in getattr(self.rns, "interfaces", ()):
            if hasattr(iface, "_freq_khz") or iface.__class__.__name__ == "LoRaInterface":
                return iface
        return None

    def _reserved_gpio_pins(self):
        reserved = set(SYSTEM_RESERVED_PINS)
        iface = self._find_lora_iface()
        if iface is not None:
            for attr in ("_sck_pin", "_mosi_pin", "_miso_pin", "_cs_pin", "_busy_pin", "_dio1_pin", "_reset_pin"):
                val = getattr(iface, attr, None)
                if isinstance(val, int):
                    reserved.add(val)
        else:
            reserved.update(DEFAULT_RADIO_PINS)
        return reserved

    def _cmd_help(self):
        p = BOT_PREFIX
        return (
            "Bot commands (prefix '%s', '/' is reserved for RRC hub): "
            "%shelp | %sesp32 | %ssx1262 | %srns | "
            "%sgpio get <pin> [in|pu|pd] | %sgpio set <pin> <0|1|on|off|toggle> | "
            "%sgpio list | %sled [on|off|toggle|status]"
            % (p, p, p, p, p, p, p, p, p)
        )

    def _cmd_esp32(self):
        gc.collect()
        free_b = gc.mem_free() if hasattr(gc, "mem_free") else 0
        alloc_b = gc.mem_alloc() if hasattr(gc, "mem_alloc") else 0
        total_b = free_b + alloc_b
        used_pct = (alloc_b * 100 // total_b) if total_b > 0 else 0

        freq_mhz, rst_cause = "?", "?"
        try:
            import machine
            if hasattr(machine, "freq"):
                freq_mhz = "%dMHz" % (machine.freq() // 1000000)
            if hasattr(machine, "reset_cause"):
                rst_cause = str(machine.reset_cause())
        except Exception:
            pass

        chip, fw_ver = sys.platform, "?"
        try:
            u = os.uname()
            chip = getattr(u, "machine", u[4])
            fw_ver = getattr(u, "release", u[2])
        except Exception:
            pass

        fs_info = "?"
        try:
            st = os.statvfs("/")
            bsize = st[0]
            fs_info = "%dKB/%dKB free" % ((st[3] * bsize) // 1024, (st[2] * bsize) // 1024)
        except Exception:
            pass

        temp_str = "n/a"
        try:
            import esp32
            if hasattr(esp32, "mcu_temperature"):
                temp_str = "%.1fC" % esp32.mcu_temperature()
            elif hasattr(esp32, "raw_temperature"):
                temp_str = "%.1fC" % ((esp32.raw_temperature() - 32.0) * 5.0 / 9.0)
        except Exception:
            pass

        up_str = _fmt_uptime(time.time() - _BOOT_TIME)
        return (
            "[ESP32-S3] %s (uPy %s) cpu=%s up=%s | mem: free=%dB alloc=%dB (%d%%) | fs=%s | temp=%s rst=%s"
            % (chip, fw_ver, freq_mhz, up_str, free_b, alloc_b, used_pct, fs_info, temp_str, rst_cause)
        )

    def _cmd_sx1262(self):
        iface = self._find_lora_iface()
        if iface is None:
            return "[SX1262] No LoRaInterface registered on this Reticulum instance"

        freq_khz = getattr(iface, "_freq_khz", 0)
        freq_mhz = ("%.3fMHz" % (freq_khz / 1000.0)) if freq_khz else "?"
        sf = getattr(iface, "_sf", "?")
        bw = getattr(iface, "_bw", "?")
        cr = getattr(iface, "_coding_rate", "?")
        tx_pwr = getattr(iface, "_tx_power", "?")
        pre = getattr(iface, "_preamble_len", "?")
        crc = getattr(iface, "_crc_en", "?")
        sw = getattr(iface, "_syncword", 0)
        sw_str = ("0x%04x" % sw) if isinstance(sw, int) else str(sw)
        br = int(getattr(iface, "bitrate", 0) or 0)
        online = getattr(iface, "online", False)

        rssi = getattr(iface, "rssi", None)
        snr = getattr(iface, "snr", None)
        rssi_str = ("%sdBm" % rssi) if rssi is not None else "n/a"
        snr_str = ("%.1fdB" % snr) if isinstance(snr, (int, float)) else (str(snr) if snr is not None else "n/a")

        lbt_thr = getattr(iface, "_lbt_rssi", None)
        lbt_str = ("%.1fdBm" % lbt_thr) if isinstance(lbt_thr, (int, float)) else str(lbt_thr)
        lbt_w = getattr(iface, "_lbt_waits", 0)
        lbt_f = getattr(iface, "_lbt_forced", 0)

        tx_pkts = getattr(iface, "tx", 0)
        rx_pkts = getattr(iface, "rx", 0)
        tx_bytes = getattr(iface, "txb", 0)
        rx_bytes = getattr(iface, "rxb", 0)
        modem = getattr(iface, "_modem", None)
        crc_errs = getattr(modem, "crc_errors", 0) if modem is not None else 0

        pins = "sck=%s,mosi=%s,miso=%s,cs=%s,busy=%s,dio1=%s,rst=%s" % (
            getattr(iface, "_sck_pin", "?"),
            getattr(iface, "_mosi_pin", "?"),
            getattr(iface, "_miso_pin", "?"),
            getattr(iface, "_cs_pin", "?"),
            getattr(iface, "_busy_pin", "?"),
            getattr(iface, "_dio1_pin", "?"),
            getattr(iface, "_reset_pin", "?"),
        )
        return (
            "[SX1262] %s online=%s | %s SF%s BW%s CR4/%s pwr=%sdBm pre=%s sw=%s crc=%s br=%dbps"
            " | sig: rssi=%s snr=%s lbt=%s(w=%d,f=%d) | tx=%d(%dB) rx=%d(%dB) crc_err=%d | %s"
            % (
                getattr(iface, "name", "LoRa"), online, freq_mhz, sf, bw, cr,
                tx_pwr, pre, sw_str, crc, br, rssi_str, snr_str, lbt_str,
                lbt_w, lbt_f, tx_pkts, tx_bytes, rx_pkts, rx_bytes, crc_errs, pins,
            )
        )

    def _cmd_rns(self):
        try:
            from urns.transport import Transport
        except Exception:
            Transport = None

        id_hex = getattr(self.identity, "hexhash", "?")
        t_enabled = getattr(Transport, "transport_enabled", False) if Transport else False

        ifaces = getattr(self.rns, "interfaces", ())
        if_parts = [
            "%s(%s,tx=%d,rx=%d)" % (
                getattr(i, "name", "?"),
                "up" if getattr(i, "online", False) else "down",
                getattr(i, "tx", 0),
                getattr(i, "rx", 0),
            )
            for i in ifaces
        ]
        if_summary = ", ".join(if_parts) if if_parts else "none"

        paths_n = len(getattr(Transport, "path_table", {})) if Transport else 0
        dests_n = len(getattr(Transport, "destination_table", {})) if Transport else 0
        reach_n = len(getattr(Transport, "reachable_destinations", {})) if Transport else 0
        act_links = len(getattr(Transport, "active_links", ())) if Transport else 0
        pend_links = len(getattr(Transport, "pending_links", ())) if Transport else 0

        rel_ann = getattr(Transport, "relayed_announces", 0) if Transport else 0
        rel_dat = getattr(Transport, "relayed_data", 0) if Transport else 0
        rel_lnk = getattr(Transport, "relayed_links", 0) if Transport else 0
        rel_prf = getattr(Transport, "relayed_proofs", 0) if Transport else 0

        link_info = "none"
        if self.link is not None:
            rtt_ms = int((getattr(self.link, "rtt", 0) or 0) * 1000)
            link_info = "id=%s rtt=%dms mdu=%sB hops=%s" % (
                self.link.link_id.hex()[:8],
                rtt_ms,
                getattr(self.link, "mdu", "?"),
                getattr(self.link, "expected_hops", "?"),
            )

        return (
            "[uRNS] id=%s transport=%s ifaces=%s | tables: paths=%d dests=%d reach=%d"
            " | links: act=%d pend=%d (%s) | relay: ann=%d data=%d lnk=%d prf=%d"
            " | rrc: sess=%d room=%s peers=%d in=%d(%dB) out=%d(%dB)"
            % (
                id_hex, t_enabled, if_summary, paths_n, dests_n, reach_n,
                act_links, pend_links, link_info, rel_ann, rel_dat, rel_lnk, rel_prf,
                self.session_no, self.room, len(self.members),
                self.msgs_in, self.bytes_in, self.msgs_out, self.bytes_out,
            )
        )

    def _cmd_led(self, args):
        if not args or args[0] in ("status", "get", "read", "?"):
            on = led_get()
            raw = led.value() if led is not None else (0 if on else 1)
            return "[LED] GPIO%d=%s (raw=%d, active_low=%s)" % (
                LED_PIN, "ON" if on else "OFF", raw, LED_ACTIVE_LOW,
            )
        act = args[0]
        if act in ("on", "1", "true", "yes"):
            led_set(True)
        elif act in ("off", "0", "false", "no"):
            led_set(False)
        elif act == "toggle":
            led_set(not led_get())
        else:
            return "[LED] Usage: %sled [on|off|toggle|status]" % BOT_PREFIX
        on = led_get()
        raw = led.value() if led is not None else (0 if on else 1)
        return "[LED] GPIO%d set to %s (raw=%d)" % (LED_PIN, "ON" if on else "OFF", raw)

    def _cmd_gpio(self, args):
        reserved = self._reserved_gpio_pins()

        if not args or args[0] in ("list", "status", "info"):
            led_on = led_get()
            led_raw = led.value() if led is not None else (0 if led_on else 1)
            cfg_items = []
            for p in sorted(self.gpio_pins.keys()):
                try:
                    val = self.gpio_pins[p].value()
                except Exception:
                    val = "?"
                cfg_items.append("GPIO%d=%s(%s)" % (p, val, self.gpio_modes.get(p, "?")))
            cfg_str = ", ".join(cfg_items) if cfg_items else "none"
            return "[GPIO] LED=GPIO%d(%s,raw=%d) | configured: %s | reserved(radio/sys): %s" % (
                LED_PIN, "ON" if led_on else "OFF", led_raw, cfg_str,
                ",".join(str(x) for x in sorted(reserved)),
            )

        sub = args[0]
        if sub.isdigit() or sub in XIAO_PIN_ALIASES or (sub.startswith("gpio") and sub[4:].isdigit()):
            sub = "get"
            args = ["get"] + args

        if sub in ("get", "read"):
            if len(args) < 2:
                return "[GPIO] Usage: %sgpio get <pin> [in|pu|pd]" % BOT_PREFIX
            try:
                pin_no = _parse_pin(args[1])
            except Exception:
                return "[GPIO] Invalid pin: " + str(args[1])
            if pin_no < GPIO_MIN_PIN or pin_no > GPIO_MAX_PIN:
                return "[GPIO] Pin out of range (%d..%d): %d" % (GPIO_MIN_PIN, GPIO_MAX_PIN, pin_no)
            if pin_no in reserved:
                return "[GPIO] Refusing to touch reserved radio/system %s" % _pin_label(pin_no)

            if pin_no == LED_PIN and led is not None and len(args) == 2:
                raw = led.value()
                return "[GPIO] %s: value=%d (%s, LED=%s, mode=OUT)" % (
                    _pin_label(pin_no), raw, "HIGH" if raw else "LOW", "ON" if led_get() else "OFF",
                )

            pull_arg = args[2].lower() if len(args) >= 3 else None
            try:
                if pull_arg is not None or pin_no not in self.gpio_pins:
                    if pull_arg in ("pu", "pullup", "pull_up", "up"):
                        p, mode_name = Pin(pin_no, Pin.IN, Pin.PULL_UP), "IN_PU"
                    elif pull_arg in ("pd", "pulldown", "pull_down", "down"):
                        p, mode_name = Pin(pin_no, Pin.IN, Pin.PULL_DOWN), "IN_PD"
                    elif pull_arg in (None, "in", "input"):
                        p, mode_name = Pin(pin_no, Pin.IN), "IN"
                    else:
                        return "[GPIO] Unknown pull/mode '%s' (use in, pu, or pd)" % pull_arg
                    self.gpio_pins[pin_no] = p
                    self.gpio_modes[pin_no] = mode_name
                else:
                    p = self.gpio_pins[pin_no]
                    mode_name = self.gpio_modes.get(pin_no, "IN")
                val = int(p.value())
                return "[GPIO] %s: value=%d (%s) mode=%s" % (
                    _pin_label(pin_no), val, "HIGH" if val else "LOW", mode_name,
                )
            except Exception as e:
                return "[GPIO] Error reading %s: %s" % (_pin_label(pin_no), e)

        if sub in ("set", "write"):
            if len(args) < 3:
                return "[GPIO] Usage: %sgpio set <pin> <0|1|on|off|high|low|toggle>" % BOT_PREFIX
            try:
                pin_no = _parse_pin(args[1])
            except Exception:
                return "[GPIO] Invalid pin: " + str(args[1])
            if pin_no < GPIO_MIN_PIN or pin_no > GPIO_MAX_PIN:
                return "[GPIO] Pin out of range (%d..%d): %d" % (GPIO_MIN_PIN, GPIO_MAX_PIN, pin_no)
            if pin_no in reserved:
                return "[GPIO] Refusing to modify reserved radio/system %s" % _pin_label(pin_no)

            val_arg = args[2].lower()
            if val_arg in ("1", "on", "high", "true", "hi"):
                target = 1
            elif val_arg in ("0", "off", "low", "false", "lo"):
                target = 0
            elif val_arg == "toggle":
                target = None
            else:
                return "[GPIO] Invalid state '%s' (use 0, 1, on, off, high, low, or toggle)" % args[2]

            try:
                if pin_no == LED_PIN and led is not None:
                    cur = int(led.value())
                    new_val = (0 if cur else 1) if target is None else target
                    led.value(new_val)
                    global _led_state
                    _led_state = (new_val == 0) if LED_ACTIVE_LOW else (new_val != 0)
                    return "[GPIO] %s set to %d (%s, LED=%s) mode=OUT" % (
                        _pin_label(pin_no), new_val, "HIGH" if new_val else "LOW", "ON" if _led_state else "OFF",
                    )

                p = self.gpio_pins.get(pin_no)
                if p is None or self.gpio_modes.get(pin_no) != "OUT":
                    cur = int(p.value()) if p is not None else 0
                    new_val = (0 if cur else 1) if target is None else target
                    p = Pin(pin_no, Pin.OUT, value=new_val)
                    self.gpio_pins[pin_no] = p
                    self.gpio_modes[pin_no] = "OUT"
                else:
                    cur = int(p.value())
                    new_val = (0 if cur else 1) if target is None else target
                    p.value(new_val)

                actual = int(p.value())
                return "[GPIO] %s set to %d (%s) mode=OUT" % (
                    _pin_label(pin_no), actual, "HIGH" if actual else "LOW",
                )
            except Exception as e:
                return "[GPIO] Error setting %s: %s" % (_pin_label(pin_no), e)

        return "[GPIO] Usage: %sgpio get <pin> [in|pu|pd] | %sgpio set <pin> <0|1|on|off|toggle> | %sgpio list" % (
            BOT_PREFIX, BOT_PREFIX, BOT_PREFIX,
        )

    def handle_bot_command(self, text):
        """Parse and execute a bot command starting with BOT_PREFIX.

        Returns a response string if `text` is a bot command, or None if not.
        Messages starting with '/' are RRC hub commands and are never matched.
        """
        if not isinstance(text, str):
            return None
        stripped = text.strip()
        if not stripped or stripped.startswith("/") or not stripped.startswith(BOT_PREFIX):
            return None
        cmdline = stripped[len(BOT_PREFIX):].strip()
        if not cmdline:
            return None

        parts = cmdline.split()
        cmd = parts[0].lower()
        args = [p.lower() for p in parts[1:]]

        if cmd in ("help", "commands", "?", "h"):
            return self._cmd_help()
        if cmd in ("esp32", "esp32s3", "mcu", "sys", "system", "status"):
            return self._cmd_esp32()
        if cmd in ("sx1262", "radio", "lora", "rf"):
            return self._cmd_sx1262()
        if cmd in ("rns", "reticulum", "urns", "net", "network"):
            return self._cmd_rns()
        if cmd in ("gpio", "pin", "pins"):
            return self._cmd_gpio(args)
        if cmd == "led":
            return self._cmd_led(args)
        return "Unknown command '%s%s'. Type %shelp for available commands." % (
            BOT_PREFIX, cmd, BOT_PREFIX,
        )

    # ---- RRC session & link management ----

    def close(self):
        """Cleanly part the room and close the active RRC link (e.g. on Ctrl+C in mpremote)."""
        self.closed = True
        if self.link is not None and getattr(self.link, "status", 0) == 1:
            if self.joined:
                try:
                    self._send_env(T_PART, room=self.room)
                except Exception:
                    pass
            try:
                self.link.teardown()
            except Exception:
                pass

    async def run(self):
        while True:
            self.session_no += 1
            print("\n[RRC] Session %d starting" % self.session_no)
            try:
                await self._session()
            except Exception as e:
                print("[RRC] Session error: " + str(e))
            self.link = None
            self.connected = False
            self.identified = False
            self.welcomed = False
            self.joined = False
            self.awaiting_joined = False
            self.members = {}
            self.tx_queue = []
            self.pending_resource = None
            gc.collect()
            print("[RRC] Reconnecting in %ds" % RECONNECT_DELAY)
            await asyncio.sleep(RECONNECT_DELAY)

    def _handshake_retry_interval(self):
        rtt = (getattr(self.link, "rtt", 0) or 0) if self.link else 0
        return max(HANDSHAKE_RETRY_MIN, rtt * 3.0 + 4.0)

    def _resource_in_flight(self):
        if self.link is not None:
            inc = getattr(self.link, "incoming_resources", None)
            if inc and len(inc) > 0:
                return True
        if self.pending_resource is not None:
            if time.time() - self.pending_resource.get("at", 0) < 30:
                return True
            self.pending_resource = None
        return False

    async def _session(self):
        hub_identity = await self._wait_for_hub()

        from urns import Destination
        from urns.link import OutgoingLink

        destination = Destination(
            hub_identity, Destination.OUT, Destination.SINGLE, "rrc", "hub",
        )
        if destination.hash != self.hub_hash:
            raise ValueError(
                "computed hub destination hash %s does not match HUB_DEST_HASH %s"
                % (destination.hexhash, self.hub_hash.hex())
            )

        print("[RRC] Hub identity: " + hub_identity.hexhash)
        print("[RRC] Hub destination: " + destination.hexhash)

        self.closed = False
        self.connected = False
        self.identified = False
        self.welcomed = False
        self.joined = False
        self.awaiting_joined = False
        self.hello_sent_at = 0
        self.hello_attempts = 0
        self.join_sent_at = 0
        self.join_attempts = 0
        self.tx_queue = []
        self.limits = dict(DEFAULT_LIMITS)
        self.hub_caps = {}

        def on_established(link):
            now = time.time()
            self.link = link
            self.connected = True
            self.connected_at = now
            self.last_rx_at = now
            self.last_tx_at = now
            self.members = {self.identity.hash: {"nick": self.nick, "seen": now}}
            print("\n[RRC] LINK ESTABLISHED")
            print("      link_id=" + link.link_id.hex())
            print("      hub=" + self.hub_hash.hex())
            print(
                "      mtu=%s mdu=%s rtt_ms=%d hops=%s"
                % (
                    link.mtu, link.mdu, int((link.rtt or 0) * 1000),
                    getattr(link, "expected_hops", "?"),
                )
            )
            if hasattr(link, "set_packet_callback"):
                link.set_packet_callback(self._on_packet)
            else:
                link.packet_callback = self._on_packet
            if hasattr(link, "set_resource_concluded_callback"):
                link.set_resource_concluded_callback(self._on_resource)
            else:
                link.resource_concluded_callback = self._on_resource

        def on_closed(link):
            self.closed = True
            self.connected = False
            self.identified = False
            self.welcomed = False
            self.joined = False
            uptime = int(time.time() - self.connected_at) if self.connected_at else 0
            print("\n[RRC] LINK CLOSED")
            print("      link_id=" + link.link_id.hex())
            print("      status=" + str(getattr(link, "status", "?")))
            print("      uptime=%ds" % uptime)
            print(
                "      msgs_in=%d msgs_out=%d bytes_in=%d bytes_out=%d"
                % (self.msgs_in, self.msgs_out, self.bytes_in, self.bytes_out)
            )
            print("      known_members=%d" % len(self.members))

        OutgoingLink(
            destination,
            established_callback=on_established,
            closed_callback=on_closed,
        )
        print("[RRC] Link request sent")

        while not self.closed:
            await asyncio.sleep(0.25)
            if not self.connected or self.link is None or self.link.status != 1:
                continue

            now = time.time()

            # Step 1: Paced IDENTIFY -> HELLO handshake (with retry if WELCOME not received)
            if not self.welcomed:
                if now - self.connected_at < IDENTIFY_DELAY:
                    continue
                retry_iv = self._handshake_retry_interval()
                if self.hello_attempts == 0 or (now - self.hello_sent_at >= retry_iv):
                    if self.hello_attempts >= MAX_HANDSHAKE_RETRIES:
                        print("[RRC] Handshake timed out waiting for WELCOME, closing link")
                        try:
                            self.link.teardown()
                        except Exception:
                            pass
                        self.closed = True
                        break

                    try:
                        self.link.identify(self.identity)
                        self.identified = True
                        self.last_tx_at = time.time()
                        print("[RRC] -> IDENTIFY %s (attempt %d)" % (self.identity.hexhash, self.hello_attempts + 1))
                    except Exception as e:
                        print("[RRC] Identify failed: " + str(e))

                    await asyncio.sleep(HELLO_DELAY)
                    if self.closed or self.welcomed:
                        continue

                    self.hello_attempts += 1
                    self.hello_sent_at = time.time()
                    self._send_hello()
                continue

            # Step 2: Paced JOIN after WELCOME (waiting for any MOTD/resource to settle)
            if self.welcomed and not self.joined:
                if (
                    now - self.welcomed_at < JOIN_DELAY
                    or self._resource_in_flight()
                    or now - self.last_rx_at < RX_SETTLE_DELAY
                ):
                    continue

                retry_iv = self._handshake_retry_interval()
                if self.join_attempts == 0 or (now - self.join_sent_at >= retry_iv):
                    if self.join_attempts >= MAX_HANDSHAKE_RETRIES:
                        print("[RRC] Timed out waiting for JOINED on %s, closing link" % self.room)
                        try:
                            self.link.teardown()
                        except Exception:
                            pass
                        self.closed = True
                        break

                    self.join_attempts += 1
                    self.join_sent_at = time.time()
                    self._send_join()

            # Step 3: Drain queued outbound messages (PONGs, bot command replies)
            if (
                self.tx_queue
                and not self._resource_in_flight()
                and (now - self.last_tx_at >= TX_PACE_DELAY)
                and (now - self.last_rx_at >= RX_SETTLE_DELAY)
            ):
                msg_type, room, body, dst = self.tx_queue.pop(0)
                self._send_env(msg_type, room=room, body=body, dst=dst)

    async def _wait_for_hub(self):
        from urns.identity import Identity
        from urns.transport import Transport

        last_request = 0
        while True:
            if Transport.has_path(self.hub_hash):
                identity = Identity.recall(self.hub_hash)
                if identity is not None:
                    print("[RRC] Path to hub: hops=" + str(Transport.hops_to(self.hub_hash)))
                    return identity
                if time.time() - last_request >= 10:
                    print("[RRC] Path known, waiting for hub identity/announce")
            if time.time() - last_request >= 15:
                print("[RRC] Requesting path to hub " + self.hub_hash.hex())
                try:
                    Transport.request_path(self.hub_hash)
                except Exception as e:
                    print("[RRC] Path request failed: " + str(e))
                last_request = time.time()
            await asyncio.sleep(1)

    def _send_hello(self):
        body = {
            B_HELLO_NAME: CLIENT_NAME,
            B_HELLO_VER: CLIENT_VERSION,
            B_HELLO_CAPS: {
                CAP_RESOURCE_ENVELOPE: True,
                CAP_ACTION: True,
                CAP_DIRECT_NOTICE: True,
            },
        }
        if self._send_env(T_HELLO, body=body):
            print("[RRC] -> HELLO name=%s version=%s nick=%s" % (CLIENT_NAME, CLIENT_VERSION, self.nick))

    def _send_join(self):
        max_room_b = int(self.limits.get(B_LIMIT_MAX_ROOM_NAME_BYTES, DEFAULT_LIMITS[B_LIMIT_MAX_ROOM_NAME_BYTES]))
        room_norm = self.room.strip().lower()
        if not room_norm or len(room_norm.encode("utf-8")) > max_room_b:
            print("[RRC] Invalid or oversized room name: " + str(self.room))
            return
        self.room = room_norm
        self.awaiting_joined = True
        if self._send_env(T_JOIN, room=self.room):
            print("[RRC] -> JOIN %s (attempt %d)" % (self.room, self.join_attempts))
        else:
            self.awaiting_joined = False

    def _max_body_bytes(self):
        hub_max = int(self.limits.get(B_LIMIT_MAX_MSG_BODY_BYTES, DEFAULT_LIMITS[B_LIMIT_MAX_MSG_BODY_BYTES]))
        mdu = getattr(self.link, "mdu", 431) if self.link else 431
        return min(hub_max, max(64, mdu - 75))

    def queue_text(self, msg_type, room, text, dst=None):
        """Split text into RRC-compliant chunks (<= max_msg_body_bytes and <= link.mdu)
        and queue them for paced transmission."""
        for chunk in _split_utf8(text, self._max_body_bytes()):
            self.tx_queue.append((msg_type, room, chunk, dst))

    def _send_env(self, msg_type, room=None, body=None, dst=None):
        if self.link is None or self.link.status != 1:
            print("[RRC] Cannot send type %s: link not active" % msg_type)
            return False

        mid = os.urandom(8)
        env = {
            K_V: RRC_VERSION,
            K_T: msg_type,
            K_ID: mid,
            K_TS: _unix_ms(),
            K_SRC: self.identity.hash,
        }
        if room is not None:
            max_room_b = int(self.limits.get(B_LIMIT_MAX_ROOM_NAME_BYTES, DEFAULT_LIMITS[B_LIMIT_MAX_ROOM_NAME_BYTES]))
            env[K_ROOM] = _truncate_utf8(room.strip().lower(), max_room_b)
        if body is not None:
            if msg_type in (T_MSG, T_ACTION) and isinstance(body, str):
                max_body_b = int(self.limits.get(B_LIMIT_MAX_MSG_BODY_BYTES, DEFAULT_LIMITS[B_LIMIT_MAX_MSG_BODY_BYTES]))
                body = _truncate_utf8(body, max_body_b)
            env[K_BODY] = body
        if self.nick:
            max_nick_b = int(self.limits.get(B_LIMIT_MAX_NICK_BYTES, DEFAULT_LIMITS[B_LIMIT_MAX_NICK_BYTES]))
            nick_clean = _truncate_utf8(str(self.nick).strip(), max_nick_b)
            if nick_clean:
                env[K_NICK] = nick_clean
        if dst is not None:
            env[K_DST] = bytes(dst)

        payload = cbor_encode(env)
        tries = 0
        while len(payload) > self.link.mdu and isinstance(body, str) and tries < 4:
            over = len(payload) - self.link.mdu
            target_b = max(1, len(body.encode("utf-8")) - over - 4)
            body = _truncate_utf8(body, target_b)
            env[K_BODY] = body
            payload = cbor_encode(env)
            tries += 1

        if len(payload) > self.link.mdu:
            print("[RRC] Outgoing type %s too large (%dB > %dB mdu), dropped" % (msg_type, len(payload), self.link.mdu))
            return False

        try:
            self.link.send(payload)
            self.last_tx_at = time.time()
            self.msgs_out += 1
            self.bytes_out += len(payload)
            if msg_type in (T_MSG, T_ACTION, T_NOTICE):
                self.sent_ids.append(bytes(mid))
                if len(self.sent_ids) > 64:
                    self.sent_ids.pop(0)
            return True
        except Exception as e:
            print("[RRC] Send failed type %s: %s" % (msg_type, e))
            return False

    def _on_packet(self, plaintext, packet):
        self.last_rx_at = time.time()
        self.msgs_in += 1
        self.bytes_in += len(plaintext)
        try:
            env = cbor_decode(plaintext)
        except Exception as e:
            print("[RRC] RX decode error: " + str(e))
            print("      raw=" + _describe(plaintext))
            return
        try:
            validate_envelope(env)
        except Exception as e:
            print("[RRC] RX invalid envelope (%s): %s" % (e, _describe(env)))
            return
        try:
            self._handle_env(env)
        except Exception as e:
            print("[RRC] Handler error: " + str(e))
            print("      envelope=" + _describe(env))

    def _handle_env(self, env):
        msg_type = env.get(K_T)
        room = env.get(K_ROOM)
        body = env.get(K_BODY)
        src = bytes(env.get(K_SRC))
        nick = env.get(K_NICK)
        mid = bytes(env.get(K_ID))
        ts = env.get(K_TS)
        dst = env.get(K_DST)

        if isinstance(room, str):
            room = room.strip().lower()

        now = time.time()
        if src in self.members:
            self.members[src]["seen"] = now
            if nick:
                self.members[src]["nick"] = nick

        meta = " msg_id=%s ts=%s src=%s" % (_hex(mid), _fmt_ms(ts), _hex(src))

        if msg_type == T_WELCOME:
            self.welcomed = True
            self.welcomed_at = now
            print("[RRC] WELCOME" + meta)
            if isinstance(body, dict):
                print("      hub_name=" + str(body.get(B_WELCOME_HUB, "?")))
                print("      hub_version=" + str(body.get(B_WELCOME_VER, "?")))
                if B_WELCOME_CAPS in body and isinstance(body[B_WELCOME_CAPS], dict):
                    self.hub_caps = dict(body[B_WELCOME_CAPS])
                    print("      capabilities=" + _describe_named(self.hub_caps, CAP_NAMES))
                if B_WELCOME_LIMITS in body and isinstance(body[B_WELCOME_LIMITS], dict):
                    for k, v in body[B_WELCOME_LIMITS].items():
                        if _is_int(k) and _is_int(v) and v > 0:
                            self.limits[k] = v
                    print("      limits=" + _describe_named(self.limits, LIMIT_NAMES))
            else:
                print("      body=" + _describe(body))

        elif msg_type == T_JOINED:
            hashes = _identity_list(body)
            if self.awaiting_joined or (
                not self.joined and room == self.room and self.identity.hash in hashes
            ):
                self.awaiting_joined = False
                self.joined = True
                self.joined_at = now
                print("[RRC] ROOM JOINED room=%s%s" % (room, meta))
                if hashes:
                    print("      members=%d" % len(hashes))
                    for item in hashes:
                        if item not in self.members:
                            self.members[item] = {"nick": None, "seen": now}
                        self.members[item]["seen"] = now
                        known = self.members[item].get("nick") or "?"
                        print("        %s nick=%s" % (_hex(item), known))
                else:
                    print("      member list: (hub did not include identities)")
                print("      known_members=%d" % len(self.members))
            else:
                print("[RRC] MEMBER JOINED room=%s nick=%s%s" % (room, nick or "?", meta))
                if hashes:
                    for item in hashes:
                        self.members[item] = {"nick": nick, "seen": now}
                    print("      member_id=" + ", ".join(_hex(x) for x in hashes))
                else:
                    print("      member_id=(not provided by hub)")
                print("      known_members=%d" % len(self.members))

        elif msg_type == T_PARTED:
            hashes = _identity_list(body)
            if self.identity.hash in hashes and (room is None or room == self.room):
                self.joined = False
                self.awaiting_joined = False
                self.join_attempts = 0
                print("[RRC] ROOM LEFT (self) room=%s%s" % (room, meta))
                return
            print("[RRC] MEMBER LEFT room=%s nick=%s%s" % (room, nick or "?", meta))
            if hashes:
                for item in hashes:
                    info = self.members.pop(item, None)
                    known = (info.get("nick") or "?") if info is not None else "?"
                    print("      member_id=%s known_nick=%s" % (_hex(item), known))
            else:
                print("      member_id=(not provided by hub)")
            print("      known_members=%d" % len(self.members))

        elif msg_type == T_MSG:
            content = _body_text(body)
            sender = nick or _hex(src)[:8]
            if src not in self.members:
                self.members[src] = {"nick": nick, "seen": now}
            elif nick:
                self.members[src]["nick"] = nick

            print("[RRC] MSG room=%s <%s> %s%s" % (room, sender, _short(content, 300), meta))

            if mid in self.sent_ids:
                self.sent_ids.remove(mid)
                if DEBUG >= 2:
                    print("[RRC] Ignoring own reflected message (by id)")
                return
            if src == self.identity.hash:
                if DEBUG >= 2:
                    print("[RRC] Ignoring own reflected message (by src)")
                return

            reply = self.handle_bot_command(content)
            if reply is not None:
                self.cmds_handled += 1
                target_room = room or self.room
                print("[BOT] Command from %s in %s: %s" % (sender, target_room, _short(content, 80)))
                self.queue_text(T_MSG, target_room, reply)

        elif msg_type == T_NOTICE:
            content = _body_text(body)
            scope = ("room=" + str(room)) if room else (("dst=" + _hex(dst)) if dst else "hub")
            print("[RRC] NOTICE %s %s%s" % (scope, _short(content, 400), meta))

        elif msg_type == T_ACTION:
            if src not in self.members:
                self.members[src] = {"nick": nick, "seen": now}
            print("[RRC] ACTION room=%s %s%s" % (room, _short(_body_text(body), 400), meta))

        elif msg_type == T_PING:
            if DEBUG >= 2:
                print("[RRC] PING" + meta)
            self.tx_queue.insert(0, (T_PONG, None, body if K_BODY in env else None, None))

        elif msg_type == T_PONG:
            if DEBUG >= 2:
                print("[RRC] PONG" + meta)

        elif msg_type == T_ERROR:
            err_text = _body_text(body)
            print("[RRC] ERROR%s: %s" % (meta, err_text))
            lower_err = err_text.lower()
            if "must send hello" in lower_err:
                self.welcomed = False
                self.joined = False
                self.awaiting_joined = False
            elif "not in room" in lower_err and (room is None or room == self.room):
                self.joined = False
                self.awaiting_joined = False
                self.join_attempts = 0

        elif msg_type == T_RESOURCE_ENVELOPE:
            self._handle_resource_envelope(room, body, meta)

        else:
            if DEBUG >= 1:
                print("[RRC] Unhandled type=%s room=%s%s body=%s" % (msg_type, room, meta, _describe(body)))

    def _handle_resource_envelope(self, room, body, meta):
        if not isinstance(body, dict):
            print("[RRC] RESOURCE ENVELOPE malformed room=%s%s body=%s" % (room, meta, _describe(body)))
            return
        res_id = body.get(B_RES_ID)
        res_kind = body.get(B_RES_KIND)
        res_size = body.get(B_RES_SIZE)
        res_sha = body.get(B_RES_SHA256)
        res_enc = body.get(B_RES_ENCODING)

        if not (isinstance(res_id, (bytes, bytearray)) and isinstance(res_kind, str) and _is_int(res_size) and res_size >= 0):
            print("[RRC] RESOURCE ENVELOPE invalid fields: " + _describe(body))
            return

        info = {
            "id": bytes(res_id),
            "kind": res_kind,
            "size": res_size,
            "sha256": bytes(res_sha) if isinstance(res_sha, (bytes, bytearray)) else None,
            "encoding": res_enc if isinstance(res_enc, str) else "utf-8",
            "room": room,
            "at": time.time(),
        }
        self.pending_resource = info
        print("[RRC] RESOURCE EXPECTED room=%s%s" % (room, meta))
        print("      id=%s kind=%s size=%s encoding=%s" % (_hex(info["id"]), info["kind"], info["size"], info["encoding"]))
        print("      sha256=" + _hex(info["sha256"]))

    def _on_resource(self, resource):
        self.last_rx_at = time.time()
        status = getattr(resource, "status", None)
        data = getattr(resource, "data", b"") or b""
        print("[RRC] RESOURCE CONCLUDED status=%s size=%d hash=%s" % (status, len(data), _hex(getattr(resource, "hash", b""))))

        info = self.pending_resource
        self.pending_resource = None

        try:
            from urns.resource import COMPLETE
            complete = status == COMPLETE
        except Exception:
            complete = status == 5

        if not complete:
            print("      resource transfer did not complete")
            return

        if info is not None:
            kind = info.get("kind")
            encoding = info.get("encoding") or "utf-8"
            exp_size = info.get("size")
            exp_sha = info.get("sha256")
            print("      kind=%s expected_size=%s room=%s sha256=%s" % (kind, exp_size, info.get("room"), _hex(exp_sha)))
            if _is_int(exp_size) and len(data) != exp_size:
                print("      warning: resource size mismatch (%d != %d)" % (len(data), exp_size))
                return
            if exp_sha is not None:
                try:
                    from urns.crypto.hashes import sha256
                    actual_sha = sha256(data)
                    if actual_sha != exp_sha:
                        print("      warning: resource sha256 mismatch (%s != %s)" % (_hex(actual_sha), _hex(exp_sha)))
                        return
                except Exception:
                    pass
            if kind in ("notice", "motd", "blob"):
                try:
                    print("[RRC] RESOURCE TEXT room=%s: %s" % (info.get("room"), _short(data.decode(encoding), 800)))
                    return
                except Exception as e:
                    print("      text decode failed: " + str(e))

        try:
            print("[RRC] RESOURCE TEXT: " + _short(data.decode("utf-8"), 800))
        except Exception:
            print("      preview=" + _describe(data))


# ---- WiFi / startup ----


def connect_wifi(ssid, password, timeout=15):
    import network

    platform = sys.platform
    if platform == "esp32":
        ap = network.WLAN(network.AP_IF)
        if ap.active():
            ap.active(False)

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        if DEBUG >= 1:
            print("Connecting to WiFi:", ssid)
        wlan.connect(ssid, password)
        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout:
                raise RuntimeError("WiFi connection timed out")
            time.sleep(0.5)

    if platform == "esp32":
        wlan.config(pm=0)
    elif platform == "rp2":
        wlan.config(pm=0xA11140)

    ip = wlan.ifconfig()[0]
    if DEBUG >= 1:
        print("Connected! IP:", ip, "(" + platform + ")")

    try:
        import ntptime
        ntptime.settime()
        if DEBUG >= 1:
            print("NTP synced")
    except Exception as e:
        if DEBUG >= 1:
            print("NTP sync failed:", e)

    return ip


def needs_wifi(config):
    for iface in config.get("interfaces", []):
        if iface.get("enabled", False) and iface.get("type", "") in (
            "UDPInterface",
            "TCPClientInterface",
        ):
            return True
    return False


def main():
    try:
        hub_hash = _hub_hash_bytes()
    except Exception as e:
        print("[RRC] Invalid HUB_DEST_HASH: " + str(e))
        print("[RRC] Edit example_rrc.py and set HUB_DEST_HASH")
        return

    if hub_hash == b"\x00" * 16:
        print("[RRC] HUB_DEST_HASH is not configured")
        print("[RRC] Edit example_rrc.py and set the hub's 32-char hex hash")
        return

    if needs_wifi(CONFIG):
        connect_wifi(WIFI_SSID, WIFI_PASS)
    gc.collect()

    from urns import Reticulum
    from urns.log import LOG_NONE, LOG_NOTICE, LOG_DEBUG

    log_map = {0: LOG_NONE, 1: LOG_NONE, 2: LOG_DEBUG}
    rns = Reticulum(loglevel=log_map.get(DEBUG, LOG_NOTICE))
    rns.config = CONFIG
    gc.collect()

    rns.setup_interfaces()
    gc.collect()

    client = RRCClient(rns, hub_hash, ROOM_NAME.strip().lower(), RRC_NICK)

    print("[RRC] Local identity: " + rns.identity.hexhash)
    print("[RRC] Hub: " + hub_hash.hex())
    print("[RRC] Room: " + client.room)
    print("[RRC] Nick: " + str(RRC_NICK))
    print("[RRC] Bot prefix: " + BOT_PREFIX)
    if hasattr(gc, "mem_free"):
        print("[RRC] Free memory:", gc.mem_free(), "bytes")

    async def runner():
        asyncio.create_task(client.run())
        await rns.run()

    try:
        asyncio.run(runner())
    except KeyboardInterrupt:
        client.close()
        rns.shutdown()
        print("Shutdown complete")


main()
