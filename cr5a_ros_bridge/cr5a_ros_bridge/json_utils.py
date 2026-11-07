"""Strict JSON and finite-number helpers shared by the CR5A bridge."""

import math
import numbers
from collections.abc import Mapping
from decimal import Decimal
from enum import Enum


def json_safe(value, invalid_paths=None, path="$"):
    """Return JSON-compatible containers and replace non-finite numbers with None.

    When supplied, invalid_paths is populated with paths of NaN/Infinity
    values. This keeps telemetry serialization non-fatal while preserving a
    diagnostic trail for callers that need to reject unsafe data.
    """
    if value is None or isinstance(value, (str, bool, int)):
        return value

    if isinstance(value, (float, Decimal, numbers.Real)):
        number = float(value)
        if not math.isfinite(number):
            if invalid_paths is not None:
                invalid_paths.append(path)
            return None
        return number

    if isinstance(value, Enum):
        return json_safe(value.value, invalid_paths, path)

    if isinstance(value, Mapping):
        result = {}
        for key, item in value.items():
            safe_key = key if isinstance(key, str) else str(key)
            result[safe_key] = json_safe(
                item, invalid_paths, f"{path}.{safe_key}"
            )
        return result

    if isinstance(value, (list, tuple)):
        return [
            json_safe(item, invalid_paths, f"{path}[{index}]")
            for index, item in enumerate(value)
        ]

    if isinstance(value, (bytes, bytearray, memoryview)):
        return list(value)

    # Support NumPy-like scalars/arrays without making NumPy a bridge dependency.
    item_method = getattr(value, "item", None)
    if callable(item_method):
        try:
            item = item_method()
        except (TypeError, ValueError):
            item = value
        if item is not value:
            return json_safe(item, invalid_paths, path)

    list_method = getattr(value, "tolist", None)
    if callable(list_method):
        try:
            items = list_method()
        except (TypeError, ValueError):
            items = value
        if items is not value:
            return json_safe(items, invalid_paths, path)

    return value


def finite_float(value, field_name):
    """Convert a numeric input to float, rejecting booleans and non-finite values."""
    if isinstance(value, bool):
        raise ValueError(f"{field_name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field_name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be a finite number")
    return number


def finite_vector(values, length, field_name):
    """Validate and convert a fixed-length numeric vector."""
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f"{field_name} must contain exactly {length} numbers")
    return [
        finite_float(value, f"{field_name}[{index}]")
        for index, value in enumerate(values)
    ]
