import json
import math
import unittest
from decimal import Decimal

from cr5a_ros_bridge.json_utils import (
    finite_float, finite_quaternion, finite_vector, json_safe,
)


class JSONSafeTests(unittest.TestCase):
    def test_non_finite_numbers_are_null_and_reported(self):
        overflow_number = json.loads("1e999")
        self.assertTrue(math.isinf(overflow_number))
        self.assertIsNone(json_safe(overflow_number))

        invalid_paths = []
        value = {
            "nan": float("nan"),
            "positive_infinity": float("inf"),
            "negative_infinity": float("-inf"),
            "nested": [1.5, {"bad": float("nan")}],
        }

        safe = json_safe(value, invalid_paths)

        self.assertEqual(safe, {
            "nan": None,
            "positive_infinity": None,
            "negative_infinity": None,
            "nested": [1.5, {"bad": None}],
        })
        self.assertEqual(
            invalid_paths,
            ["$.nan", "$.positive_infinity", "$.negative_infinity", "$.nested[1].bad"],
        )
        json.dumps(safe, allow_nan=False)

    def test_finite_values_and_containers_are_preserved(self):
        safe = json_safe({"integer": 4, "float": 1.25, "tuple": (True, "ok")})
        self.assertEqual(safe, {"integer": 4, "float": 1.25, "tuple": [True, "ok"]})
        json.dumps(safe, allow_nan=False)

    def test_decimal_non_finite_values_are_null(self):
        self.assertIsNone(json_safe(Decimal("NaN")))
        self.assertIsNone(json_safe(Decimal("Infinity")))
        self.assertEqual(json_safe(Decimal("2.5")), 2.5)

    def test_scalar_and_array_adapters_are_sanitized(self):
        class Scalar:
            def item(self):
                return float("inf")

        class Array:
            def tolist(self):
                return [1.0, float("nan")]

        self.assertIsNone(json_safe(Scalar()))
        self.assertEqual(json_safe(Array()), [1.0, None])

    def test_finite_float_rejects_invalid_values(self):
        for value in (float("nan"), float("inf"), float("-inf"), True, None, "not-a-number"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    finite_float(value, "value")

        self.assertEqual(finite_float(2, "value"), 2.0)
        self.assertEqual(finite_float("0.25", "value"), 0.25)

    def test_finite_vector_checks_length_and_each_component(self):
        self.assertEqual(finite_vector([1, 2.5, "3"], 3, "vector"), [1.0, 2.5, 3.0])
        for values in ([1, 2], [1, float("nan"), 3], [1, True, 3], None):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    finite_vector(values, 3, "vector")

    def test_finite_quaternion_is_normalized_and_rejects_invalid_values(self):
        self.assertEqual(finite_quaternion([0, 0, 0, 2], "quaternion"), [0.0, 0.0, 0.0, 1.0])
        for values in (
            [0, 0, 0, 0],
            [0, 0, 0, float("inf")],
            [1e308, 1e308, 1e308, 1e308],
        ):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    finite_quaternion(values, "quaternion")


if __name__ == "__main__":
    unittest.main()
