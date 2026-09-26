"""Feed valve control.

Opening the feed valve is gated: the seal has to be up, the compressor state
has to be durable in the record stream, and an operator confirmation has to be
redeemed.  Everything else in this module is bookkeeping around that gate.
"""

from __future__ import annotations

from typing import Any

from line_control.feed.arbiter import GOVERNOR, PROTECTION, Demand, arbitrate
from line_control.interlock.board import InterlockBoard
from line_control.registry.confirmations import ConfirmationBoard
from line_control.registry.parameters import Bounds, ParameterRegistry, ParameterSpec
from line_control.runtime.errors import (
    GateBlockedError,
    LimitViolationError,
    UnknownReferenceError,
    ValidationError,
)
from line_control.runtime.keys import scope_key
from line_control.store.stream import RecordStream

MAX_LIMIT = 100
OPEN_SUBJECT = "feed.open"


class FeedController:
    """Valve position, setpoint and the per unit feed latch."""

    def __init__(
        self,
        stream: RecordStream,
        registry: ParameterRegistry,
        board: InterlockBoard,
        confirmations: ConfirmationBoard,
    ) -> None:
        self._stream = stream
        self._registry = registry
        self._board = board
        self._confirmations = confirmations

    def scope(self, unit: str) -> str:
        """Return the parameter scope this module uses for a unit."""
        return scope_key("feed", unit)

    def declare_unit(self, unit: str) -> None:
        """Declare the tunable limits of one unit."""
        scope = self.scope(unit)
        self._registry.declare(
            ParameterSpec(scope, "setpoint", "int", 0, Bounds(0, MAX_LIMIT), "percent")
        )
        self._registry.declare(
            ParameterSpec(scope, "governor_demand", "int", MAX_LIMIT, Bounds(0, MAX_LIMIT), "percent")
        )
        self._registry.declare(
            ParameterSpec(
                scope, "protection_demand", "int", MAX_LIMIT, Bounds(0, MAX_LIMIT), "percent"
            )
        )
        self._registry.declare(
            ParameterSpec(scope, "low_limit", "int", 0, Bounds(0, MAX_LIMIT), "percent")
        )
        self._registry.declare(
            ParameterSpec(scope, "high_limit", "int", MAX_LIMIT, Bounds(0, MAX_LIMIT), "percent")
        )
        self._registry.declare(
            ParameterSpec(scope, "valve_position", "int", 0, Bounds(0, MAX_LIMIT), "percent")
        )

    # ------------------------------------------------------------ read paths
    def _valve_key(self, unit: str) -> str:
        return scope_key("feed", unit, "valve")

    def _limit(self, unit: str, name: str, fallback: int) -> int:
        try:
            return int(self._registry.value(self.scope(unit), name))
        except UnknownReferenceError:
            return fallback

    def valve_open(self, unit: str) -> bool:
        """Report whether the feed valve is open."""
        record = self._stream.visible_view().current(self._valve_key(unit))
        return bool(record and record.payload.get("state") == "open")

    def position(self, unit: str) -> int:
        """Return the commanded valve position."""
        return self._limit(unit, "valve_position", 0)

    def setpoint(self, unit: str) -> int:
        """Return the effective feed setpoint."""
        return self._limit(unit, "setpoint", 0)

    def low_limit(self, unit: str) -> int:
        """Return the lower clamp applied to the setpoint."""
        return self._limit(unit, "low_limit", 0)

    def high_limit(self, unit: str) -> int:
        """Return the upper clamp applied to the setpoint."""
        return self._limit(unit, "high_limit", MAX_LIMIT)

    def flow(self, unit: str) -> int:
        """Return the delivered feed as setpoint times position."""
        if not self.valve_open(unit):
            return 0
        return self.setpoint(unit) * self.position(unit) // 100

    def latched(self, unit: str) -> bool:
        """Report whether the feed latch is engaged."""
        return self._board.latches.is_set(unit, "feed")

    def latch_reason(self, unit: str) -> str:
        """Return why the feed latch was set."""
        return self._board.latches.reason(unit, "feed")

    def status(self, unit: str) -> dict[str, Any]:
        """Return a snapshot of the feed module for one unit."""
        return {
            "unit": unit,
            "valve": "open" if self.valve_open(unit) else "closed",
            "position": self.position(unit),
            "setpoint": self.setpoint(unit),
            "lowLimit": self.low_limit(unit),
            "highLimit": self.high_limit(unit),
            "flow": self.flow(unit),
            "latched": self.latched(unit),
            "latchReason": self.latch_reason(unit),
        }

    # ----------------------------------------------------------- write paths
    def open(self, unit: str, ticket: str) -> dict[str, Any]:
        """Open the feed valve once the pre-gate and the ticket both agree."""
        self._board.require_latch_clear(unit, "feed")
        self._board.require_gate("feed_open", unit)
        confirmation = self._confirmations.consume(ticket, subject=OPEN_SUBJECT)
        record = self._stream.append(
            "feed.open",
            self._valve_key(unit),
            {
                "unit": unit,
                "state": "open",
                "ticket": confirmation.ticket,
                "watermark": self._stream.watermark,
            },
        )
        self._stream.commit_upto(record.seq)
        return self.status(unit)

    def close(self, unit: str) -> dict[str, Any]:
        """Close the feed valve."""
        record = self._stream.append(
            "feed.close",
            self._valve_key(unit),
            {"unit": unit, "state": "closed"},
        )
        self._stream.commit_upto(record.seq)
        return self.status(unit)

    def set_position(self, unit: str, position: int) -> dict[str, Any]:
        """Command a valve position, refusing one outside the travel range."""
        requested = int(position)
        if not 0 <= requested <= MAX_LIMIT:
            raise LimitViolationError(
                f"feed valve position {position} is outside 0..{MAX_LIMIT}",
                unit=unit,
                value=requested,
                low=0,
                high=MAX_LIMIT,
            )
        self._registry.set(self.scope(unit), "valve_position", requested)
        return self.status(unit)

    def set_setpoint(self, unit: str, value: int, source: str = GOVERNOR) -> Demand:
        """Apply a setpoint demand from one of the two channels."""
        if source not in {GOVERNOR, PROTECTION}:
            raise ValidationError("unknown setpoint source", source=source)
        requested = int(value)
        if requested < 0:
            raise LimitViolationError(
                "a feed setpoint cannot be negative",
                unit=unit,
                value=requested,
                low=0,
                high=MAX_LIMIT,
            )
        register = "governor_demand" if source == GOVERNOR else "protection_demand"
        self._registry.set(self.scope(unit), register, min(requested, MAX_LIMIT))
        return self._apply_demand(unit)

    def arbitrate(self, unit: str, governor: int, protection: int) -> Demand:
        """Return the arbitrated demand without applying it."""
        return arbitrate(unit, governor, protection)

    def lower_limit(self, unit: str, value: int) -> int:
        """Pin the lower clamp, never letting it rise above the upper clamp."""
        pinned = min(int(value), self.high_limit(unit))
        self._registry.set(self.scope(unit), "low_limit", pinned)
        return pinned

    def raise_limit(self, unit: str, value: int) -> int:
        """Lift the lower clamp toward a ceiling, never above the upper clamp."""
        pinned = min(max(self.low_limit(unit), int(value)), self.high_limit(unit))
        self._registry.set(self.scope(unit), "low_limit", pinned)
        return pinned

    def set_high_limit(self, unit: str, value: int) -> int:
        """Pin the upper clamp, never letting it fall below the lower clamp."""
        requested = int(value)
        if not 0 <= requested <= MAX_LIMIT:
            raise LimitViolationError(
                f"feed high limit {value} is outside 0..{MAX_LIMIT}",
                unit=unit,
                value=requested,
                low=0,
                high=MAX_LIMIT,
            )
        pinned = max(requested, self.low_limit(unit))
        self._registry.set(self.scope(unit), "high_limit", pinned)
        if self.setpoint(unit) > pinned:
            self._registry.set(self.scope(unit), "setpoint", pinned)
        return pinned

    def latch(self, unit: str, reason: str) -> dict[str, Any]:
        """Engage the feed latch."""
        self._board.set_latch(unit, "feed", reason)
        return self.status(unit)

    def unlatch(self, unit: str, released: bool = True) -> dict[str, Any]:
        """Release the feed latch once its cause is gone."""
        self._board.clear_latch(unit, "feed", released, note="feed latch released")
        return self.status(unit)

    def retry_open(self, unit: str, ticket: str, seal_ok: bool, ignition_latch: str) -> dict[str, Any]:
        """Retry a failed start without pretending the earlier failure never happened."""
        if not seal_ok:
            raise GateBlockedError(
                "a feed retry needs an established seal",
                gate="feed_open",
                unit=unit,
                blocked_by=["seal_established"],
            )
        if ignition_latch:
            raise ValidationError(
                "the ignition latch has to be cleared before a retry",
                unit=unit,
                reason=ignition_latch,
            )
        before = self.retry_count(unit)
        self.unlatch(unit)
        result = self.open(unit, ticket)
        result["retry"] = before + 1
        return result

    def retry_count(self, unit: str) -> int:
        """Return how many feed retries were recorded for a unit."""
        record = self._stream.visible_view().current(scope_key("feed", unit, "retry"))
        if record is None:
            return 0
        return int(record.payload.get("count", 0))

    def note_retry(self, unit: str) -> int:
        """Record one more retry attempt."""
        previous = self.retry_count(unit)
        record = self._stream.append(
            "feed.retry",
            scope_key("feed", unit, "retry"),
            {"unit": unit, "count": previous + 1},
        )
        self._stream.commit_upto(record.seq)
        return previous + 1

    def _apply_demand(self, unit: str) -> Demand:
        """Resolve the two channel registers into the applied setpoint."""
        governor = self._limit(unit, "governor_demand", MAX_LIMIT)
        protection = self._limit(unit, "protection_demand", MAX_LIMIT)
        demand = arbitrate(unit, governor, protection)
        applied = self._clamp(unit, demand.value)
        self._registry.set(self.scope(unit), "setpoint", applied)
        return Demand(unit, applied, demand.source, governor, protection)

    def _clamp(self, unit: str, value: int) -> int:
        """Keep a setpoint inside the clamps the unit currently carries."""
        return min(max(value, self.low_limit(unit)), self.high_limit(unit))
