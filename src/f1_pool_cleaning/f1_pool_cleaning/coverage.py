"""Directed cleaning intervals, independent of where a task starts."""
from dataclasses import dataclass


@dataclass(frozen=True)
class CleaningLeg:
    edge_id: str
    start: float
    end: float


def merge_intervals(intervals):
    result = []
    for low, high in sorted(intervals):
        if high <= low:
            continue
        if result and low <= result[-1][1]:
            result[-1] = (result[-1][0], max(result[-1][1], high))
        else:
            result.append((low, high))
    return result


def missing_intervals(required, covered, tolerance):
    low, high = required
    cursor, gaps = low, []
    for a, b in merge_intervals(covered):
        a, b = max(low, a), min(high, b)
        if b <= low or a >= high:
            continue
        if a-cursor > tolerance:
            gaps.append((cursor, a))
        cursor = max(cursor, b)
    if high-cursor > tolerance:
        gaps.append((cursor, high))
    return gaps


def initial_legs(edges, required, first_id, contact_s, overlap):
    """Suffix, all other edges, then the first edge's uncleaned prefix.

    For an open route the last-to-first transition is navigation, never a
    fictitious closing wall. Edge identities and directions are preserved.
    """
    ids = [edge.id for edge in edges]
    index = ids.index(first_id)
    low, high = required[first_id]
    contact_s = min(high, max(low, contact_s))
    legs = [CleaningLeg(first_id, contact_s, high)]
    for edge_id in ids[index+1:]+ids[:index]:
        legs.append(CleaningLeg(edge_id, *required[edge_id]))
    if contact_s > low:
        legs.append(CleaningLeg(first_id, low, min(high, contact_s+overlap)))
    return [leg for leg in legs if leg.end-leg.start > 1e-6]
