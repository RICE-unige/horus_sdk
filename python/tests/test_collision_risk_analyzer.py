"""Unit tests for collision risk analyzer math helpers."""

from horus.dataviz.collision_risk_analyzer import (
    analyze_laser_scan,
    analyze_point_cloud,
    risk_from_distance,
)


def test_risk_from_distance_bounds():
    assert risk_from_distance(None, 1.2) == 0.0
    assert risk_from_distance(5.0, 1.2) == 0.0
    assert risk_from_distance(0.0, 1.2) == 1.0
    assert risk_from_distance(0.6, 1.2) == 0.5


def test_analyze_laser_scan_finds_nearest_direction():
    ranges = [2.0, 1.5, 0.5, 1.1]
    min_distance, direction = analyze_laser_scan(
        ranges=ranges,
        angle_min=-1.0,
        angle_increment=0.5,
        min_valid_range=0.1,
        max_valid_range=4.0,
    )
    assert min_distance == 0.5
    assert direction[0] > 0.0
    assert direction[2] == 0.0


def test_analyze_point_cloud_filters_z_window_and_returns_closest():
    points = [
        (1.0, 0.0, 0.0),
        (0.5, 0.1, 1.0),  # filtered out by z-window
        (0.3, 0.4, 0.1),  # closest accepted
    ]
    min_distance, direction = analyze_point_cloud(points, z_window_m=0.35)
    assert round(min_distance, 4) == 0.5
    assert direction[0] > 0.0
    assert direction[1] > 0.0
    assert direction[2] == 0.0
