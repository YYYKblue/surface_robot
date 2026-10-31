"""Observed cleaning interval union and gaps; no repair-route generation."""


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
