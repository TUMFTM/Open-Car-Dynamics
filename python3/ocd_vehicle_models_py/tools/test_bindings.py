#!/usr/bin/env python3
"""Test script that instantiates every vehicle model and calls all
`get_` and `construct_` methods to ensure Python bindings are correct.

Run from repository root with:

    python3 python3/ocd_vehicle_models_py/tools/test_bindings.py

"""
import sys
import traceback
import itertools
from typing import List

# Import package components
from ocd_vehicle_models_py import (
    VehicleFactory,
    DrivetrainType,
    SteeringActuatorType,
    VehicleDynamicsType,
    TireType,
    AerodynamicsType,
)

ALL_DRIVETRAINS = list(DrivetrainType)
ALL_STEERINGS = list(SteeringActuatorType)
ALL_VDMS = list(VehicleDynamicsType)
ALL_TIRES = list(TireType)
ALL_AEROS = list(AerodynamicsType)

failures = []

def call_noarg(method):
    try:
        result = method()
        return True, result
    except TypeError as e:
        # Method expects args - treat as failure for this test
        return False, f"TypeError: {e}"
    except Exception as e:
        return False, f"Exception: {e}\n{traceback.format_exc()}"


def main():
    total = 0
    success = 0

    combos = list(itertools.product(ALL_DRIVETRAINS, ALL_STEERINGS, ALL_VDMS, ALL_TIRES, ALL_AEROS))
    print(f"Testing {len(combos)} vehicle combinations")

    for dt, sa, vdm, tire, aero in combos:
        total += 1
        name = f"OCD_Vehicle_{dt.name}__{sa.name}__{vdm.name}__{tire.name}__{aero.name}"
        print(f"\n[{total}/{len(combos)}] Creating {name}")
        try:
            vehicle = VehicleFactory.create_vehicle(dt, sa, vdm, tire, aero)
        except Exception as e:
            failures.append((name, "constructor", f"Exception: {e}\n{traceback.format_exc()}"))
            print(f"  ERROR creating vehicle: {e}")
            continue

        # Find get_ and construct_ methods
        candidates: List[str] = [
            n for n in dir(vehicle)
            if (n.startswith("get_") or n.startswith("construct_")) and not n.startswith("__")
        ]
        print(f"  Found methods: {candidates}")

        for m in candidates:
            total += 1
            attr = getattr(vehicle, m)
            if not callable(attr):
                # skip non-callables
                continue
            ok, info = call_noarg(attr)
            if ok:
                success += 1
                print(f"    OK {m} -> {type(info)}")
            else:
                failures.append((name, m, info))
                print(f"    FAIL {m} -> {info}")

    print("\nSummary:")
    print(f"  Total calls attempted: {total}")
    print(f"  Successful calls: {success}")
    print(f"  Failures: {len(failures)}")

    if failures:
        print("\nFailures detail:")
        for name, method, err in failures:
            print(f"- {name} :: {method} -> {err}")
        sys.exit(2)
    else:
        print("All binding checks passed")


if __name__ == '__main__':
    main()
