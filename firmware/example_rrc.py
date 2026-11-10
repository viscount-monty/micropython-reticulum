"""
µReticulum Example: RRC Relay Chat Echo Node
=============================================
Joins an RRC (Reticulum Relay Chat) hub, creates or joins a room, echoes
messages sent to that room, reports room state changes, and lets members
control the built-in LED with a chat command.

RRC wire format: https://rrc.kc1awv.net/

Edit the settings below, then run with: import example_rrc

Commands understood in the room:
  on | off | !led on | !led off | /led on | /led off | led on | led off
"""

from config import WIFI_SSID, WIFI_PASS, NODE_NAME, DEBUG, CONFIG

import gc
import os
import time

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

from machine import Pin

gc.collect()

# RRC timestamps are Unix-epoch milliseconds. Bare-metal MicroPython ports
# use a 2000-01-01 epoch for time.time()/gmtime(), so shift when needed.
try:
    UNIX_EPOCH_OFFSET = 946684800 if time.gmtime(0)[0] == 2000 else 0
except Exception:
    UNIX_EPOCH_OFFSET = 0


def _unix_ms():
    return int((time.time() + UNIX_EPOCH_OFFSET) * 1000)

# ---- RRC settings (edit these) ----
# 32 hex characters: the hub's rrc.hub destination hash, shown by the hub
# at startup and carried in its announce.
HUB_DEST_HASH = "a8b7055fc51c6d9e317ecd839b969981"
ROOM_NAME = "#esp32s3"
RRC_NICK = NODE_NAME or "mcu-echo"
CLIENT_NAME = "micropython-rrc"
CLIENT_VERSION = "0.1"
RECONNECT_DELAY = 10
ECHO_PREFIX = "Echo: "

# Built-in LED. XIAO ESP32-S3 uses GPIO21 and is active-low.
LED_PIN = 21
LED_ACTIVE_LOW = True

# ---- RRC constants ----
RRC_VERSION = 1

K_V = 0
K_T = 1
K_ID = 2
K_TS = 3
K_SRC = 4
K_ROOM = 5
K_BODY = 6
K_NICK = 7

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

B_HELLO_NAME = 0
B_HELLO_VER = 1
B_HELLO_CAPS = 2

B_WELCOME_HUB = 0
B_WELCOME_VER = 1
B_WELCOME_CAPS = 2
B_WELCOME_LIMITS = 3

B_RES_ID = 0
B_RES_KIND = 1
B_RES_SIZE = 2
B_RES_SHA256 = 3
B_RES_ENCODING = 4

LIMIT_NAMES = {
    0: "max_nick_bytes",
    1: "max_room_name_bytes",
    2: "max_msg_body_bytes",
    3: "max_rooms_per_session",
    4: "rate_limit_msgs_per_minute",
}

CAP_NAMES = {
    0: "resource_envelope",
    1: "action",
    2: "direct_notice",
}

led = Pin(LED_PIN, Pin.OUT)


def led_set(on):
    if LED_ACTIVE_LOW:
        led.value(0 if on else 1)
    else:
        led.value(1 if on else 0)


led_set(False)


# ---- Minimal CBOR (RRC uses CBOR maps, not msgpack) ----


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
        if obj >= 0:
            return _cbor_head(0, obj)
        return _cbor_head(1, -1 - obj)
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
    initial = data[pos]
    pos += 1
    major = initial >> 5
    info = initial & 31

    if info == 31:
        if major == 2 or major == 3:
            chunks = b""
            text = ""
            while data[pos] != 0xFF:
                value, pos = _cbor_read(data, pos)
                if major == 3:
                    text += value
                else:
                    chunks += value
            pos += 1
            if major == 3:
                return text, pos
            return chunks, pos
        if major == 4:
            items = []
            while data[pos] != 0xFF:
                value, pos = _cbor_read(data, pos)
                items.append(value)
            pos += 1
            return items, pos
        if major == 5:
            items = {}
            while data[pos] != 0xFF:
                key, pos = _cbor_read(data, pos)
                value, pos = _cbor_read(data, pos)
                items[key] = value
            pos += 1
            return items, pos
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
        return bytes(data[pos : pos + value]), pos + value
    if major == 3:
        return bytes(data[pos : pos + value]).decode("utf-8"), pos + value
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


# ---- Formatting helpers ----


def _hex(data):
    if isinstance(data, (bytes, bytearray)):
        return bytes(data).hex()
    if data is None:
        return "?"
    return str(data)


def _short(text, limit=160):
    text = str(text)
    if len(text) <= limit:
        return text
    return text[:limit] + "...(" + str(len(text)) + " chars)"


def _fmt_ms(ms):
    if not isinstance(ms, int):
        return str(ms)
    try:
        sec = ms // 1000 - UNIX_EPOCH_OFFSET
        tm = time.gmtime(sec)
        return "%04d-%02d-%02d %02d:%02d:%02d.%03d UTC" % (
            tm[0],
            tm[1],
            tm[2],
            tm[3],
            tm[4],
            tm[5],
            ms % 1000,
        )
    except Exception:
        return str(ms) + " ms"


def _describe(value, depth=0):
    if isinstance(value, dict):
        if depth >= 2:
            return "{" + str(len(value)) + " entries}"
        parts = []
        for key in value:
            parts.append(str(key) + "=" + _describe(value[key], depth + 1))
        return "{" + ", ".join(parts) + "}"
    if isinstance(value, (list, tuple)):
        if depth >= 2:
            return "[" + str(len(value)) + " items]"
        parts = []
        for item in value:
            parts.append(_describe(item, depth + 1))
        return "[" + ", ".join(parts) + "]"
    if isinstance(value, (bytes, bytearray)):
        raw = bytes(value)
        if len(raw) <= 8:
            return "0x" + raw.hex()
        return "0x" + raw[:8].hex() + "...(" + str(len(raw)) + "B)"
    if isinstance(value, str):
        return _short(value, 120)
    return str(value)


def _describe_named(value, names, depth=0):
    if isinstance(value, dict):
        parts = []
        for key in value:
            name = names.get(key, str(key))
            parts.append(name + "=" + _describe(value[key], depth + 1))
        return "{" + ", ".join(parts) + "}"
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
    result = []
    if isinstance(body, (list, tuple)):
        for item in body:
            if isinstance(item, (bytes, bytearray)):
                result.append(bytes(item))
    return result


def _parse_led_command(text):
    value = text.strip().lower()
    if value == "on":
        return True
    if value == "off":
        return False
    for prefix in ("!led ", "/led ", "led "):
        if value.startswith(prefix):
            arg = value[len(prefix) :].strip()
            if arg in ("on", "1", "true", "yes"):
                return True
            if arg in ("off", "0", "false", "no"):
                return False
    return None


def _hub_hash_bytes():
    value = HUB_DEST_HASH.strip().lower()
    if value.startswith("0x"):
        value = value[2:]
    if len(value) != 32:
        raise ValueError("HUB_DEST_HASH must be exactly 32 hex characters")
    return bytes.fromhex(value)


# ---- RRC client ----


class RRCClient:
    def __init__(self, rns, hub_hash, room, nick):
        self.rns = rns
        self.identity = rns.identity
        self.hub_hash = hub_hash
        self.room = room
        self.nick = nick
        self.link = None
        self.connected = False
        self.joined = False
        self.awaiting_joined = False
        self.closed = True
        self.members = {}
        self.sent_ids = []
        self.pending_resource = None
        self.connected_at = 0
        self.joined_at = 0
        self.session_no = 0
        self.msgs_in = 0
        self.msgs_out = 0
        self.bytes_in = 0
        self.bytes_out = 0

    async def run(self):
        while True:
            self.session_no += 1
            print()
            print("[RRC] Session " + str(self.session_no) + " starting")
            try:
                await self._session()
            except Exception as e:
                print("[RRC] Session error: " + str(e))
            self.link = None
            self.connected = False
            self.joined = False
            self.awaiting_joined = False
            self.members = {}
            self.pending_resource = None
            gc.collect()
            print("[RRC] Reconnecting in " + str(RECONNECT_DELAY) + "s")
            await asyncio.sleep(RECONNECT_DELAY)

    async def _session(self):
        hub_identity = await self._wait_for_hub()

        from urns import Destination
        from urns.link import OutgoingLink

        destination = Destination(
            hub_identity,
            Destination.OUT,
            Destination.SINGLE,
            "rrc",
            "hub",
        )

        if destination.hash != self.hub_hash:
            raise ValueError(
                "computed hub destination hash "
                + destination.hexhash
                + " does not match HUB_DEST_HASH "
                + self.hub_hash.hex()
            )

        print("[RRC] Hub identity: " + hub_identity.hexhash)
        print("[RRC] Hub destination: " + destination.hexhash)

        self.closed = False
        self.awaiting_joined = False

        def on_established(link):
            self.link = link
            self.connected = True
            self.connected_at = time.time()
            self.members = {
                self.identity.hash: {"nick": self.nick, "seen": self.connected_at},
            }
            print()
            print("[RRC] LINK ESTABLISHED")
            print("      link_id=" + link.link_id.hex())
            print("      hub=" + self.hub_hash.hex())
            print(
                "      mtu="
                + str(link.mtu)
                + " mdu="
                + str(link.mdu)
                + " rtt_ms="
                + str(int((link.rtt or 0) * 1000))
                + " hops="
                + str(getattr(link, "expected_hops", "?"))
            )
            link.set_packet_callback(self._on_packet)
            link.set_resource_concluded_callback(self._on_resource)
            try:
                link.identify(self.identity)
                print("[RRC] -> IDENTIFY " + self.identity.hexhash)
            except Exception as e:
                print("[RRC] Identify failed: " + str(e))
            self._send_hello()

        def on_closed(link):
            self.closed = True
            self.connected = False
            self.joined = False
            uptime = 0
            if self.connected_at:
                uptime = int(time.time() - self.connected_at)
            print()
            print("[RRC] LINK CLOSED")
            print("      link_id=" + link.link_id.hex())
            print("      status=" + str(getattr(link, "status", "?")))
            print("      uptime=" + str(uptime) + "s")
            print(
                "      msgs_in="
                + str(self.msgs_in)
                + " msgs_out="
                + str(self.msgs_out)
                + " bytes_in="
                + str(self.bytes_in)
                + " bytes_out="
                + str(self.bytes_out)
            )
            print("      known_members=" + str(len(self.members)))

        OutgoingLink(
            destination,
            established_callback=on_established,
            closed_callback=on_closed,
        )
        print("[RRC] Link request sent")

        while not self.closed:
            await asyncio.sleep(1)

    async def _wait_for_hub(self):
        from urns.identity import Identity
        from urns.transport import Transport

        last_request = 0
        while True:
            if Transport.has_path(self.hub_hash):
                identity = Identity.recall(self.hub_hash)
                if identity is not None:
                    print(
                        "[RRC] Path to hub: hops="
                        + str(Transport.hops_to(self.hub_hash))
                    )
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
            B_HELLO_CAPS: {1: True},
        }
        if self._send_env(T_HELLO, body=body):
            print(
                "[RRC] -> HELLO name="
                + CLIENT_NAME
                + " version="
                + CLIENT_VERSION
                + " nick="
                + str(self.nick)
            )

    def _send_join(self):
        self.awaiting_joined = True
        if self._send_env(T_JOIN, room=self.room):
            print("[RRC] -> JOIN " + self.room)
        else:
            self.awaiting_joined = False

    def _send_env(self, msg_type, room=None, body=None):
        if self.link is None or self.link.status != 1:
            print("[RRC] Cannot send type " + str(msg_type) + ": link not active")
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
            env[K_ROOM] = room
        if body is not None:
            env[K_BODY] = body
        if self.nick:
            env[K_NICK] = self.nick

        payload = cbor_encode(env)
        tries = 0
        while len(payload) > self.link.mdu and isinstance(body, str) and tries < 4:
            over = len(payload) - self.link.mdu
            cut = len(body) - over - 4
            if cut < 1:
                cut = 1
            body = body[:cut] + "..."
            env[K_BODY] = body
            payload = cbor_encode(env)
            tries += 1

        if len(payload) > self.link.mdu:
            print(
                "[RRC] Outgoing type "
                + str(msg_type)
                + " too large ("
                + str(len(payload))
                + "B > "
                + str(self.link.mdu)
                + "B mdu), dropped"
            )
            return False

        try:
            self.link.send(payload)
            self.msgs_out += 1
            self.bytes_out += len(payload)
            if msg_type in (T_MSG, T_ACTION):
                self.sent_ids.append(bytes(mid))
                if len(self.sent_ids) > 64:
                    self.sent_ids.pop(0)
            return True
        except Exception as e:
            print("[RRC] Send failed type " + str(msg_type) + ": " + str(e))
            return False

    def _on_packet(self, plaintext, packet):
        self.msgs_in += 1
        self.bytes_in += len(plaintext)
        try:
            env = cbor_decode(plaintext)
        except Exception as e:
            print("[RRC] RX decode error: " + str(e))
            print("      raw=" + _describe(plaintext))
            return
        if not isinstance(env, dict):
            print("[RRC] RX non-map envelope: " + _describe(env))
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
        src = env.get(K_SRC)
        nick = env.get(K_NICK)
        mid = env.get(K_ID)
        ts = env.get(K_TS)

        version = env.get(K_V)
        if version != RRC_VERSION:
            print(
                "[RRC] Warning: protocol version "
                + str(version)
                + " (expected "
                + str(RRC_VERSION)
                + ")"
            )

        meta = " msg_id=" + _hex(mid) + " ts=" + _fmt_ms(ts) + " src=" + _hex(src)

        if msg_type == T_WELCOME:
            print("[RRC] WELCOME" + meta)
            if isinstance(body, dict):
                print("      hub_name=" + str(body.get(B_WELCOME_HUB, "?")))
                print("      hub_version=" + str(body.get(B_WELCOME_VER, "?")))
                if B_WELCOME_CAPS in body:
                    print(
                        "      capabilities="
                        + _describe_named(body[B_WELCOME_CAPS], CAP_NAMES)
                    )
                if B_WELCOME_LIMITS in body:
                    print(
                        "      limits="
                        + _describe_named(body[B_WELCOME_LIMITS], LIMIT_NAMES)
                    )
            else:
                print("      body=" + _describe(body))
            self._send_join()

        elif msg_type == T_JOINED:
            hashes = _identity_list(body)
            now = time.time()

            if self.awaiting_joined:
                self.awaiting_joined = False
                self.joined = True
                self.joined_at = now
                print("[RRC] ROOM JOINED room=" + str(room) + meta)
                if hashes:
                    print("      members=" + str(len(hashes)))
                    for item in hashes:
                        if item not in self.members:
                            self.members[item] = {"nick": None, "seen": now}
                        self.members[item]["seen"] = now
                        known = self.members[item].get("nick") or "?"
                        print("        " + _hex(item) + " nick=" + str(known))
                    unique = len(set(hashes))
                    if unique != len(hashes):
                        print(
                            "      note: "
                            + str(len(hashes) - unique)
                            + " duplicate identity link(s); devices sharing "
                            "one Reticulum identity cannot be distinguished "
                            "by hash"
                        )
                else:
                    print("      member list: (hub did not include identities)")
                print("      known_members=" + str(len(self.members)))
            else:
                print(
                    "[RRC] MEMBER JOINED room="
                    + str(room)
                    + " nick="
                    + str(nick or "?")
                    + meta
                )
                if hashes:
                    for item in hashes:
                        self.members[item] = {"nick": nick, "seen": now}
                    print("      member_id=" + ", ".join(_hex(x) for x in hashes))
                else:
                    print("      member_id=(not provided by hub)")
                print("      known_members=" + str(len(self.members)))

        elif msg_type == T_PARTED:
            hashes = _identity_list(body)
            if self.identity.hash in hashes:
                self.joined = False
                print("[RRC] ROOM LEFT (self) room=" + str(room) + meta)
                return
            print(
                "[RRC] MEMBER LEFT room="
                + str(room)
                + " nick="
                + str(nick or "?")
                + meta
            )
            if hashes:
                for item in hashes:
                    info = self.members.pop(item, None)
                    known = "?"
                    if info is not None:
                        known = info.get("nick") or "?"
                    print("      member_id=" + _hex(item) + " known_nick=" + str(known))
            else:
                print("      member_id=(not provided by hub)")
            print("      known_members=" + str(len(self.members)))

        elif msg_type == T_MSG:
            content = _body_text(body)
            sender = nick or _hex(src)[:8]
            print(
                "[RRC] MSG room="
                + str(room)
                + " <"
                + str(sender)
                + "> "
                + _short(content, 300)
                + meta
            )

            own_mid = bytes(mid) if isinstance(mid, (bytes, bytearray)) else None
            if own_mid is not None and own_mid in self.sent_ids:
                self.sent_ids.remove(own_mid)
                if DEBUG >= 2:
                    print("[RRC] Ignoring own reflected message")
                return

            command = _parse_led_command(content)
            if command is not None:
                self._handle_led_command(command, room, nick, src, mid)
                return

            if content.startswith(ECHO_PREFIX):
                if DEBUG >= 2:
                    print("[RRC] Not echoing an echo")
                return

            self._send_env(T_MSG, room=room or self.room, body=ECHO_PREFIX + content)

        elif msg_type == T_NOTICE:
            print(
                "[RRC] NOTICE room="
                + str(room)
                + " "
                + _short(_body_text(body), 400)
                + meta
            )

        elif msg_type == T_ACTION:
            print(
                "[RRC] ACTION room="
                + str(room)
                + " "
                + _short(_body_text(body), 400)
                + meta
            )

        elif msg_type == T_PING:
            if DEBUG >= 2:
                print("[RRC] PING" + meta)
            if K_BODY in env:
                self._send_env(T_PONG, body=body)
            else:
                self._send_env(T_PONG)

        elif msg_type == T_PONG:
            if DEBUG >= 2:
                print("[RRC] PONG" + meta)

        elif msg_type == T_ERROR:
            print("[RRC] ERROR" + meta + ": " + _body_text(body))

        elif msg_type == T_RESOURCE_ENVELOPE:
            self._handle_resource_envelope(room, body, meta)

        else:
            if DEBUG >= 1:
                print(
                    "[RRC] Unhandled type="
                    + str(msg_type)
                    + " room="
                    + str(room)
                    + meta
                    + " body="
                    + _describe(body)
                )

    def _handle_led_command(self, on, room, nick, src, mid):
        state = "ON" if on else "OFF"
        led_set(on)
        who = nick or _hex(src)[:8]
        print(
            "[LED] "
            + str(who)
            + " ("
            + _hex(src)
            + ")"
            + " turned the LED "
            + state
            + " msg_id="
            + _hex(mid)
        )
        self._send_env(
            T_ACTION,
            room=room or self.room,
            body="* " + str(who) + " turned the LED " + state,
        )

    def _handle_resource_envelope(self, room, body, meta):
        if not isinstance(body, dict):
            print(
                "[RRC] RESOURCE ENVELOPE malformed room="
                + str(room)
                + meta
                + " body="
                + _describe(body)
            )
            return
        info = {
            "id": body.get(B_RES_ID),
            "kind": body.get(B_RES_KIND),
            "size": body.get(B_RES_SIZE),
            "sha256": body.get(B_RES_SHA256),
            "encoding": body.get(B_RES_ENCODING),
            "room": room,
            "at": time.time(),
        }
        self.pending_resource = info
        print("[RRC] RESOURCE EXPECTED room=" + str(room) + meta)
        print(
            "      id="
            + _hex(info["id"])
            + " kind="
            + str(info["kind"])
            + " size="
            + str(info["size"])
            + " encoding="
            + str(info["encoding"])
        )
        print("      sha256=" + _hex(info["sha256"]))

    def _on_resource(self, resource):
        status = getattr(resource, "status", None)
        data = getattr(resource, "data", b"") or b""
        print(
            "[RRC] RESOURCE CONCLUDED status="
            + str(status)
            + " size="
            + str(len(data))
            + " hash="
            + _hex(getattr(resource, "hash", b""))
        )

        info = self.pending_resource
        self.pending_resource = None

        complete = False
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
            print(
                "      kind="
                + str(kind)
                + " expected_size="
                + str(info.get("size"))
                + " room="
                + str(info.get("room"))
                + " sha256="
                + _hex(info.get("sha256"))
            )
            if kind in ("notice", "motd"):
                try:
                    print(
                        "[RRC] RESOURCE TEXT room="
                        + str(info.get("room"))
                        + ": "
                        + _short(data.decode(encoding), 800)
                    )
                    return
                except Exception as e:
                    print("      text decode failed: " + str(e))

        try:
            print("[RRC] RESOURCE TEXT: " + _short(data.decode("utf-8"), 800))
        except Exception:
            print("      preview=" + _describe(data))


# ---- WiFi / startup ----


def connect_wifi(ssid, password, timeout=15):
    import sys
    import network
    import time

    platform = sys.platform

    if platform == "esp32":
        ap = network.WLAN(network.AP_IF)
        if ap.active():
            ap.active(False)
            if DEBUG >= 2:
                print("AP_IF deactivated")

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
    print("[RRC] Room: " + ROOM_NAME.strip().lower())
    print("[RRC] Nick: " + str(RRC_NICK))
    print("[RRC] Free memory:", gc.mem_free(), "bytes")

    async def runner():
        asyncio.create_task(client.run())
        await rns.run()

    try:
        asyncio.run(runner())
    except KeyboardInterrupt:
        rns.shutdown()
        print("Shutdown complete")


main()
