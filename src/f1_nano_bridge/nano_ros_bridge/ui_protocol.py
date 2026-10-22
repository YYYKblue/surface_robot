"""ROS-independent f1-ui-v2 gateway. All calls run on one executor thread."""
from dataclasses import dataclass, field
import json
import math
import secrets
import time

from .session import Command

PROTOCOL = "f1-ui-v2"
COMMAND_FIELDS = {"token", "ticket", "sequence", "direction", "speed_percent", "mode",
                  "brush_enabled", "brush_percent", "thrust_enabled", "thrust_percent"}


@dataclass
class Client:
    last_seen: float
    last_input: float = 0.
    token: str = ""
    pending: str = ""
    sequence: int = -1
    seen_active: bool = False
    tickets: dict = field(default_factory=dict)


class GatewayCore:
    TICKET_TTL_NS = 100_000_000
    INPUT_TIMEOUT = .250
    STATE_TIMEOUT = .300
    MAX_CLIENTS = 32

    def __init__(self, backend, send, *, allow_fifth_thruster=False,
                 clock=time.monotonic, unix_ns=time.time_ns):
        self.backend, self.send = backend, send
        self.allow_fifth = allow_fifth_thruster
        self.clock, self.unix_ns = clock, unix_ns
        self.gateway_id = secrets.token_hex(16)
        self.clients = {}
        self.state = {}
        self.state_at = None
        self.bridge_key = None
        self.closed = False
        self.clock_fault = False
        self.clock_offset = None
        self._check_clock()

    def _check_clock(self):
        before = self.clock()
        unix = self.unix_ns()
        after = self.clock()
        # A scheduling pause between reads is not a system-clock adjustment.
        if after - before > .005:
            return
        offset = unix - round((before + after) * .5 * 1e9)
        if self.clock_offset is None:
            self.clock_offset = offset
        elif abs(offset - self.clock_offset) > 50_000_000:
            self.clock_fault = True

    def fifth_capability(self):
        if not self.allow_fifth:
            return False, "网关 allow_fifth_thruster=false"
        if self.state.get("allow_fifth_thruster") is not True:
            return False, "bridge 未启用第五路或尚未报告能力"
        return True, "网关和 bridge 已启用"

    def _reply(self, ident, packet, success, message="", **fields):
        self.send(ident, dict(protocol=PROTOCOL, type="reply", op=packet["op"],
                              request_id=packet["request_id"], success=success,
                              message=message, **fields))

    def _revoke(self, client):
        token = client.token
        client.token = client.pending = ""
        client.sequence, client.seen_active = -1, False
        client.tickets.clear()
        if token:
            self.backend.release(token, lambda result: None)

    def _fresh(self):
        if self.clock_fault or self.state_at is None:
            return False
        ages = (self.clock() - self.state_at, self.state.get("ctrl_age_sec", -1),
                self.state.get("ack_age_sec", -1))
        return (self.state.get("connected") is True and self.state.get("source") == "NANO"
                and self.state.get("armed") is True and self.state.get("result") == "OK"
                and all(type(age) in (int, float) and math.isfinite(age)
                        and 0 <= age < self.STATE_TIMEOUT for age in ages))

    def update_state(self, state):
        key = (state.get("connection_id"), state.get("epoch"), state.get("source"))
        if self.bridge_key is not None and key != self.bridge_key:
            for client in self.clients.values():
                self._revoke(client)
        self.bridge_key = key
        self.state, self.state_at = dict(state), self.clock()
        self._expire()

    def _expire(self):
        self._check_clock()
        now = self.clock()
        for ident, client in list(self.clients.items()):
            for ticket, (created, _) in list(client.tickets.items()):
                if (now - created) * 1e9 >= self.TICKET_TTL_NS:
                    del client.tickets[ticket]
            if now - client.last_seen >= self.INPUT_TIMEOUT:
                self._revoke(client)
                del self.clients[ident]
                continue
            if client.token or client.pending:
                invalid = not self._fresh() or self.state.get("state") not in ("READY", "ACTIVE")
                if client.token:
                    if self.state.get("motion_authorized") is True:
                        client.seen_active = True
                    invalid |= client.seen_active and not self.state.get("motion_authorized")
                if invalid or now - client.last_input >= self.INPUT_TIMEOUT:
                    self._revoke(client)

    @staticmethod
    def _decode(payload):
        if not isinstance(payload, bytes) or len(payload) > 32768:
            raise ValueError("invalid payload size")

        def pairs(items):
            result = {}
            for key, value in items:
                if key in result:
                    raise ValueError("duplicate JSON key")
                result[key] = value
            return result

        packet = json.loads(payload, object_pairs_hook=pairs)
        if type(packet) is not dict or packet.get("protocol") != PROTOCOL:
            raise ValueError("invalid protocol")
        if packet.get("op") not in ("hello", "acquire", "release", "stop", "command"):
            raise ValueError("invalid operation")
        request = packet.get("request_id")
        if type(request) is not str or not 1 <= len(request) <= 128:
            raise ValueError("invalid request_id")
        return packet

    def handle(self, ident, payload):
        if self.closed:
            return
        self._expire()  # A queued packet must not revive a timed-out lease.
        try:
            packet = self._decode(payload)
        except (ValueError, TypeError, UnicodeError, RecursionError):
            return
        client = self.clients.get(ident)
        if client is None:
            # Commands alone cannot create a session or recover a lost lease.
            if packet["op"] != "hello" or len(self.clients) >= self.MAX_CLIENTS:
                self._reply(ident, packet, False, "send hello before requesting control")
                return
            client = self.clients[ident] = Client(self.clock())
        client.last_seen = self.clock()
        op = packet["op"]
        fields = set(packet) - {"protocol", "op", "request_id"}
        expected = {"hello": set(), "acquire": {"owner"}, "stop": {"token"},
                    "release": {"token"}, "command": COMMAND_FIELDS}[op]
        if fields != expected:
            if client.token and packet.get("token") == client.token:
                self._revoke(client)
            self._reply(ident, packet, False, "unexpected or missing fields")
            return
        if op == "hello":
            return  # tick sends a fresh state/ticket, without a reply backlog.
        if op == "acquire":
            self._acquire(ident, client, packet)
        elif op in ("stop", "release"):
            self._release(ident, client, packet)
        else:
            self._command(ident, client, packet)

    def _acquire(self, ident, client, packet):
        owner = packet["owner"]
        if type(owner) is not str or not 1 <= len(owner) <= 128:
            self._reply(ident, packet, False, "invalid owner")
            return
        if client.token or client.pending or not self._fresh() or self.state.get("state") != "READY":
            self._reply(ident, packet, False, "bridge not ready or request already active")
            return
        pending = secrets.token_hex(16)
        client.pending, client.last_input = pending, self.clock()

        def complete(result):
            self._expire()
            token = result.get("token", "")
            if (self.closed or self.clients.get(ident) is not client or client.pending != pending
                    or not self._fresh()):
                if token:
                    self.backend.release(token, lambda response: None)
                return
            client.pending = ""
            if result.get("success") is True and type(token) is str and token:
                if result.get("max_command_ttl_ms", 0) < self.TICKET_TTL_NS // 1_000_000:
                    self.backend.release(token, lambda response: None)
                    self._reply(ident, packet, False, "bridge TTL is too short")
                    return
                client.token, client.last_input = token, self.clock()
                client.sequence = -1
                self._reply(ident, packet, True, result.get("message", "control acquired"),
                            token=token, epoch=result.get("epoch", 0))
            else:
                self._reply(ident, packet, False, result.get("message", "acquire failed"))

        self.backend.acquire(owner, complete)

    def _release(self, ident, client, packet):
        token = packet["token"]
        if type(token) is not str or (client.token and token != client.token):
            self._reply(ident, packet, False, "token does not own this connection")
            return
        # An empty token cancels a pending acquire. Its late response is released.
        if client.pending and not token:
            self._revoke(client)
            self._reply(ident, packet, True, "pending acquire cancelled")
            return
        if not client.token or token != client.token:
            self._reply(ident, packet, False, "no owned lease")
            return
        client.token = ""
        self._revoke(client)
        # A UI stop uses token-scoped ReleaseControl, never the global /f1/stop.
        self.backend.release(token, lambda result: self._reply(
            ident, packet, result.get("success") is True,
            result.get("message", "release requested; physical stop is not confirmed")))

    def _command(self, ident, client, packet):
        if not client.token or packet["token"] != client.token:
            self._reply(ident, packet, False, "token does not own this connection")
            return
        try:
            sequence = packet["sequence"]
            if type(sequence) is not int or not client.sequence < sequence < 2**64:
                raise ValueError("sequence must increase within uint64")
            ticket = packet["ticket"]
            if type(ticket) is not str or ticket not in client.tickets:
                raise ValueError("unknown or consumed ticket")
            created, computed_ns = client.tickets.pop(ticket)
            if ((self.clock() - created) * 1e9 >= self.TICKET_TTL_NS
                    or self.unix_ns() >= computed_ns + self.TICKET_TTL_NS or not self._fresh()):
                raise ValueError("expired ticket or bridge state")
            command = Command(client.token, sequence, computed_ns, computed_ns + self.TICKET_TTL_NS,
                              **{key: packet[key] for key in COMMAND_FIELDS
                                 - {"token", "ticket", "sequence"}})
            command.channels()  # Reuse strict direction/gear/boolean/effort validation.
            allow_fifth, reason = self.fifth_capability()
            if command.thrust_enabled and not allow_fifth:
                raise ValueError("fifth thruster is disabled: " + reason)
        except (ValueError, TypeError, OverflowError) as error:
            self._revoke(client)
            self._reply(ident, packet, False, str(error))
            return
        client.sequence, client.last_input = sequence, self.clock()
        self.backend.publish(command)
        self._reply(ident, packet, True, "forwarded to bridge; execution is not confirmed")

    def tick(self):
        if self.closed:
            return
        self._expire()
        now = self.clock()
        for ident, client in self.clients.items():
            ticket = secrets.token_hex(16)
            if self.clock_fault:
                ticket = ""
            else:
                client.tickets[ticket] = (now, self.unix_ns())
                while len(client.tickets) > 8:
                    del client.tickets[next(iter(client.tickets))]
            allow_fifth, fifth_reason = self.fifth_capability()
            self.send(ident, dict(protocol=PROTOCOL, type="state", gateway_id=self.gateway_id,
                                  state=self.state,
                                  bridge_age_sec=-1. if self.state_at is None else now - self.state_at,
                                  gateway_fault="system clock changed; restart gateway required"
                                  if self.clock_fault else "",
                                  ticket=ticket,
                                  capabilities={"allow_fifth_thruster": allow_fifth,
                                                "fifth_thruster_reason": fifth_reason}))

    def close(self):
        self.closed = True
        for client in self.clients.values():
            self._revoke(client)
        self.clients.clear()
