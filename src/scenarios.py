from __future__ import annotations

from typing import Dict, List, Tuple

from src.models import CarPose, Cone


_SCENARIOS: Dict[str, Tuple[List[Cone], CarPose]] = {
    "1": (
        [],
        CarPose(x=0.0, y=0.0, yaw=0.0),
    ),

    "2": (
        [Cone(x=1.0, y=3.0, color=1), Cone(x=1.0, y=1.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.4),
    ),

    "3": (
        [
            Cone(x=1.0, y=3.0, color=1),
            Cone(x=3.0, y=3.0, color=1),
            Cone(x=1.0, y=1.0, color=0),
            Cone(x=3.0, y=1.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.8),
    ),

    "4": (
        [Cone(x=4.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.2),
    ),

    "5": (
        [Cone(x=4.0, y=2.0, color=0), Cone(x=3.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.6),
    ),

    "6": (
        [
            Cone(x=4.0, y=4.0, color=1),
            Cone(x=4.0, y=2.0, color=0),
            Cone(x=3.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=2.0),
    ),

    "7": (
        [
            Cone(x=2.0, y=3.0, color=1),
            Cone(x=4.0, y=3.0, color=1),
            Cone(x=2.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "8": (
        [Cone(x=3.0, y=3.0, color=1), Cone(x=5.0, y=3.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=1.8),
    ),

    "9": (
        [Cone(x=3.0, y=3.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.2),
    ),

    "10": (
        [Cone(x=5.0, y=3.0, color=1), Cone(x=5.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.6),
    ),

    "11": (
        [
            Cone(x=1.0, y=3.0, color=1),
            Cone(x=4.0, y=5.0, color=1),
            Cone(x=1.0, y=2.0, color=0),
            Cone(x=4.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.0),
    ),

    "12": (
        [Cone(x=5.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "13": (
        [Cone(x=5.0, y=3.0, color=0), Cone(x=5.0, y=1.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.1),
    ),

    "14": (
        [
            Cone(x=3.0, y=5.0, color=1),
            Cone(x=3.0, y=2.0, color=0),
            Cone(x=5.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=5.2),
    ),

    "15": (
        [
            Cone(x=2.0, y=5.0, color=1),
            Cone(x=3.0, y=4.0, color=1),
            Cone(x=3.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.6),
    ),

    "16": (
        [Cone(x=0.0, y=3.0, color=1), Cone(x=2.0, y=5.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.2),
    ),

    "17": (
        [Cone(x=3.0, y=5.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.6),
    ),

    "18": (
        [Cone(x=2.0, y=4.0, color=1), Cone(x=4.0, y=3.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.0),
    ),

    "19": (
        [
            Cone(x=2.0, y=3.0, color=1),
            Cone(x=5.0, y=3.0, color=1),
            Cone(x=2.0, y=0.0, color=0),
            Cone(x=5.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "20": (
        [Cone(x=0.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.8),
    ),

    "21": (
        [
            Cone(x=2.0, y=3.0, color=1),
            Cone(x=3.0, y=3.0, color=1),
            Cone(x=4.0, y=3.0, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),
    
    "22": (
        # Three yellow (right) cones in a straight line
        [
            Cone(x=2.0, y=1.0, color=0),
            Cone(x=3.5, y=1.0, color=0),
            Cone(x=5.0, y=1.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.3),
    ),

    "23": (
        # Three blue (left) cones in a straight line
        [
            Cone(x=2.0, y=3.0, color=1),
            Cone(x=3.5, y=3.5, color=1),
            Cone(x=5.0, y=4.0, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.5),
    ),

    "24": (
        # Three yellow (right) cones forming a curve
        [
            Cone(x=2.0, y=1.5, color=0),
            Cone(x=3.5, y=1.0, color=0),
            Cone(x=5.0, y=1.5, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.2),
    ),

    "25": (
        # Three blue (left) cones with one yellow (right) cone
        [
            Cone(x=2.0, y=4.0, color=1),
            Cone(x=3.0, y=4.5, color=1),
            Cone(x=4.0, y=5.0, color=1),
            Cone(x=2.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.8),
    ),

    "26": (
        # Three yellow (right) cones with two blue (left) cones
        [
            Cone(x=2.0, y=3.5, color=1),
            Cone(x=4.0, y=4.0, color=1),
            Cone(x=2.0, y=1.5, color=0),
            Cone(x=3.5, y=1.0, color=0),
            Cone(x=5.0, y=1.5, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.4),
    ),

    "27": (
        # Three blue (left) cones forming a tight curve
        [
            Cone(x=1.5, y=3.0, color=1),
            Cone(x=2.5, y=4.0, color=1),
            Cone(x=4.0, y=4.5, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.0),
    ),

    "28": (
        # Three yellow (right) cones with sharp angle change
        [
            Cone(x=3.0, y=2.0, color=0),
            Cone(x=4.0, y=1.5, color=0),
            Cone(x=5.5, y=2.5, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.3),
    ),

    "29": (
        # Three cones on each side (full track)
        [
            Cone(x=1.5, y=4.0, color=1),
            Cone(x=3.0, y=4.5, color=1),
            Cone(x=4.5, y=5.0, color=1),
            Cone(x=1.5, y=1.5, color=0),
            Cone(x=3.0, y=1.0, color=0),
            Cone(x=4.5, y=1.5, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.5),
    ),

    "30": (
        # Three blue (left) cones at different angles
        [
            Cone(x=2.0, y=2.5, color=1),
            Cone(x=3.0, y=3.5, color=1),
            Cone(x=5.0, y=4.0, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.7),
    ),

    "31": (
        # Challenge: Three yellow cones with wide spacing
        [
            Cone(x=2.0, y=1.0, color=0),
            Cone(x=4.0, y=0.5, color=0),
            Cone(x=6.0, y=1.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.1),
    ),
}


def get_scenario_names() -> List[str]:
    return sorted(_SCENARIOS.keys(), key=lambda s: int(s))


def make_scenario(name: str) -> Tuple[List[Cone], CarPose]:
    if name not in _SCENARIOS:
        valid = ", ".join(get_scenario_names())
        raise ValueError(f"Unknown scenario '{name}'. Valid options: {valid}")
    cones, car = _SCENARIOS[name]
    return list(cones), CarPose(x=car.x, y=car.y, yaw=car.yaw)