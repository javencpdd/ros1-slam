"""多场景回归：同一套检查器跑「平层」与「三级楼梯」，只依赖设计层与契约层。"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'planner'))
sys.path.insert(0, os.path.join(HERE, 'contracts'))
import support_planner  # noqa: E402
import validate  # noqa: E402

MAX_STEP = 0.2


def flat_map():
    return {"schema_version": 1, "map_id": "flat_two", "map_version": 1, "frame": "map", "kind": "support_map",
            "resolution": 0.1, "origin": [-2.0, -2.0, 0.0], "extent": {"width": 80, "height": 40},
            "cell_states": ["free", "occupied", "unknown"], "unknown_policy": "unknown_not_traversable",
            "multiple_surfaces_per_xy": True, "source": {"uri": "flat.world", "sha256": "0" * 64},
            "surfaces": [
                {"surface_id": "floor", "z_reference": 0.0, "thickness_m": 0.1, "representation": "mesh",
                 "data_uri": "world://floor", "traversability": "supported", "clearance_min_m": 0.4,
                 "footprint": {"min_x": -2.0, "min_y": -2.0, "max_x": 2.0, "max_y": 2.0},
                 "edges": [{"to": "step1", "kind": "step", "max_step_height_m": MAX_STEP, "bidirectional": True}]},
                {"surface_id": "step1", "z_reference": 0.18, "thickness_m": 0.1, "representation": "mesh",
                 "data_uri": "world://step1", "traversability": "supported", "clearance_min_m": 0.4,
                 "footprint": {"min_x": 1.5, "min_y": -0.6, "max_x": 3.0, "max_y": 0.6},
                 "edges": [{"to": "floor", "kind": "step", "max_step_height_m": MAX_STEP, "bidirectional": True},
                           {"to": "landing", "kind": "step", "max_step_height_m": MAX_STEP, "bidirectional": True}]},
                {"surface_id": "landing", "z_reference": 0.36, "thickness_m": 0.1, "representation": "mesh",
                 "data_uri": "world://landing", "traversability": "supported", "clearance_min_m": 0.4,
                 "footprint": {"min_x": 2.5, "min_y": -0.6, "max_x": 4.0, "max_y": 0.6},
                 "edges": [{"to": "step1", "kind": "step", "max_step_height_m": MAX_STEP, "bidirectional": True}]}]}


def single_scene():
    value = flat_map()
    value["map_id"] = "flat_single"
    value["surfaces"] = value["surfaces"][:1]
    value["surfaces"][0]["edges"] = []
    value["multiple_surfaces_per_xy"] = False
    return value


def check_route(value, route):
    ids = {s["surface_id"]: s for s in value["surfaces"]}
    for point in route["path"]:
        x, y, z, sid = point
        box = ids[sid]["footprint"]
        assert box["min_x"] <= x <= box["max_x"] and box["min_y"] <= y <= box["max_y"], ("off_surface", point)
        assert abs(z - ids[sid]["z_reference"]) < 1e-6, ("wrong_z", point)
    for move in route["transitions"]:
        src, dst = ids[move["from"]], ids[move["to"]]
        edge = [e for e in src["edges"] if e["to"] == move["to"]]
        assert edge, ("no_edge", move)
        drop = abs(dst["z_reference"] - src["z_reference"])
        assert drop <= edge[0].get("max_step_height_m", 0) + 1e-9, ("over_step", move, drop)


def run():
    report = {}
    flat = single_scene()
    validate.validate("map", flat)
    route = support_planner.plan(flat, (0, 0, 0), (1.5, 0, 0))
    check_route(flat, route)
    report["flat"] = {"points": len(route["path"]), "segments": len(route["segments"]),
                      "transitions": len(route["transitions"])}

    stairs = flat_map()
    validate.validate("map", stairs)
    route2 = support_planner.plan(stairs, (0, 0, 0), (3.5, 0, 0.36))
    check_route(stairs, route2)
    report["stairs"] = {"points": len(route2["path"]), "segments": len(route2["segments"]),
                        "transitions": len(route2["transitions"])}

    again = support_planner.plan(stairs, (0, 0, 0), (3.5, 0, 0.36))
    assert json.dumps(again, sort_keys=True) == json.dumps(route2, sort_keys=True), "non-deterministic"
    report["deterministic"] = True

    bad = json.loads(json.dumps(stairs))
    bad["surfaces"][0]["edges"][0]["max_step_height_m"] = 0.05
    try:
        validate.validate("map", bad)
    except validate.ContractError as error:
        report["over_step_rejected"] = error.code
    else:
        raise AssertionError("over-step map accepted")
    print(json.dumps(report, ensure_ascii=False))
    return report


if __name__ == '__main__':
    run()
