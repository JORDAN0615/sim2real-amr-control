"""Pure mission/runner helpers shared by ROS 2 nodes and tests."""

from dataclasses import dataclass
import math


DEFAULT_PRIORITY_TARGETS = [
    "person",
    "traffic cone",
    "grey barrel",
    "blue barrel",
]


@dataclass(frozen=True)
class TargetSnapshot:
    """Minimal target data needed by scheduler selection logic."""

    class_name: str
    received_at_sec: float


def clamp(value, lower, upper):
    """Limit a numeric command to a safe min/max range."""
    return max(lower, min(value, upper))


def parse_targets(raw_targets):
    """Parse comma-separated priority targets, preserving configured order."""
    if isinstance(raw_targets, list):
        return raw_targets
    targets = [target.strip() for target in raw_targets.split(",") if target.strip()]
    return targets or DEFAULT_PRIORITY_TARGETS


def planar_distance(position):
    """Return planar base_link distance for a message-like position object."""
    return math.hypot(float(position.x), float(position.y))


def is_distance_visible(distance, min_distance, max_distance):
    """Return whether a target distance is inside runner/mission visibility gates."""
    return min_distance <= distance <= max_distance


def prune_stale_targets(visible_targets, now_sec, timeout_sec):
    """Return a copy without targets older than timeout_sec."""
    return {
        class_name: target
        for class_name, target in visible_targets.items()
        if now_sec - target.received_at_sec <= timeout_sec
    }


def select_priority_target(priority_targets, visible_targets, cooldown_until, now_sec):
    """Select the first visible target by priority that is not cooling down."""
    cooling = []
    for class_name in priority_targets:
        if class_name not in visible_targets:
            continue
        cooldown_expiry = cooldown_until.get(class_name, 0.0)
        if cooldown_expiry > now_sec:
            cooling.append((class_name, cooldown_expiry))
            continue
        return visible_targets[class_name], cooling
    return None, cooling


def select_acquisition_candidate(candidates, preferred_order, tie_epsilon_m):
    """Pick nearest non-front candidate with deterministic tie preference."""
    valid_candidates = [candidate for candidate in candidates if candidate is not None]
    if not valid_candidates:
        return None

    nearest_distance = min(candidate.distance for candidate in valid_candidates)
    near_tie = [
        candidate
        for candidate in valid_candidates
        if candidate.distance <= nearest_distance + tie_epsilon_m
    ]
    for preferred_camera in preferred_order:
        for candidate in near_tie:
            if candidate.camera == preferred_camera:
                return candidate
    return min(valid_candidates, key=lambda candidate: candidate.distance)
