#!/usr/bin/env python3
"""Dump parameters for every vehicle combination to ../config/<component_name>.json

Run from repository root with:

    python3 python3/ocd_vehicle_models_py/tools/dump_all_vehicle_params.py

Files are written to the `../config` folder relative to this script.
"""
from pathlib import Path
import json
import itertools
import traceback

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


def safe_name(dt, sa, vdm, tire, aero):
    return f"OCD_Vehicle_{dt.name}__{sa.name}__{vdm.name}__{tire.name}__{aero.name}"


def main():
    script_dir = Path(__file__).parent
    out_dir = (script_dir / '..' / 'config').resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    combos = list(itertools.product(ALL_DRIVETRAINS, ALL_STEERINGS, ALL_VDMS, ALL_TIRES, ALL_AEROS))
    print(f"Found {len(combos)} vehicle combinations. Writing to {out_dir}")

    written = 0
    failures = []

    for dt, sa, vdm, tire, aero in combos:
        name = safe_name(dt, sa, vdm, tire, aero)
        print(f"Creating {name}")
        try:
            veh = VehicleFactory.create_vehicle(dt, sa, vdm, tire, aero)
        except Exception as e:
            failures.append((name, 'constructor', f"Exception: {e}\n{traceback.format_exc()}"))
            print(f"  ERROR creating vehicle: {e}")
            continue

        try:
            params_dict = veh.get_param_manager().get_parameters_as_dict()
            # Sort keys for stable output
            json_obj = dict(sorted(params_dict.items()))
            out_path = out_dir / f"{name}.json"
            with out_path.open('w', encoding='utf-8') as fh:
                json.dump(json_obj, fh, indent=4)
            written += 1
            print(f"  Wrote {out_path}")
        except Exception as e:
            failures.append((name, 'dump', f"Exception: {e}\n{traceback.format_exc()}"))
            print(f"  ERROR dumping params: {e}")

    print('\nSummary:')
    print(f"  Written: {written}")
    print(f"  Failures: {len(failures)}")
    if failures:
        print('\nFailures detail:')
        for name, method, err in failures:
            print(f"- {name} :: {method} -> {err}")


if __name__ == '__main__':
    main()
