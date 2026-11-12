import copy
import pytest
from dock_planner import Pose2D
from dock_planner.task3_mission_policy import (
    KeepOutCircle,
    Task3NavigationConstraint,
    plan_keep_out_detour,
    plan_keep_out_exit,
    resume_phase,
)


def snapshot(**extra):
    return dict(run_id=1, revision=1, active=True,
                frame_id='rtk_ned', center_ned={'north': 0, 'east': 0},
                radius_m=2, clearance_m=1, **extra)


def test_segment_not_just_endpoint_and_terminal_has_no_bypass():
    policy = Task3NavigationConstraint()
    policy.apply(snapshot())
    assert policy.segment_block_reason((-10, 0), (10, 0), 'rtk_ned') == 'KEEP_OUT_REPLAN_REQUIRED'
    assert policy.segment_block_reason((-10, 3), (10, 3), 'rtk_ned') == 'KEEP_OUT_REPLAN_REQUIRED'
    assert policy.segment_block_reason((-10, 4), (10, 4), 'rtk_ned') is None
    assert policy.segment_block_reason((1, 0), (10, 0), 'rtk_ned') == 'KEEP_OUT_REPLAN_REQUIRED'


def test_all_clear_revision_and_run_are_guarded():
    policy = Task3NavigationConstraint()
    policy.apply(snapshot())
    clear = dict(run_id=1, revision=2, active=False)
    assert policy.apply(clear)
    assert not policy.apply(snapshot())  # late zone cannot resurrect it
    assert not policy.apply(clear)  # replay is idempotent
    assert policy.zone is None
    with pytest.raises(ValueError):
        policy.apply(dict(clear, revision=3, run_id=2))
    with pytest.raises(ValueError):
        policy.apply(dict(clear, active=True))  # same version, conflicting contents


@pytest.mark.parametrize('changes', [
    {'radius_m': -1}, {'radius_m': float('nan')},
    {'clearance_m': -1}, {'frame_id': ''},
    {'center_ned': {'latitude': 31, 'longitude': 121}}, {'revision': True},
])
def test_invalid_update_preserves_existing_zone(changes):
    policy = Task3NavigationConstraint()
    policy.apply(snapshot())
    old = copy.deepcopy(policy.to_dict())
    with pytest.raises(ValueError):
        policy.apply(dict(snapshot(), revision=2, **changes) if 'revision' not in changes else dict(snapshot(), **changes))
    assert policy.to_dict() == old


def test_frame_mismatch_blocks():
    policy = Task3NavigationConstraint()
    policy.apply(snapshot())
    assert policy.segment_block_reason((20, 20), (30, 30), 'odom') == 'KEEP_OUT_FRAME_MISMATCH'


def test_four_resume_phases_keep_success_facts():
    assert resume_phase('SEARCH_DOCK', {}).value == 'EXPLORING'
    assert resume_phase('SEARCH_DOCK', {'dock_id': 'a'}).value == 'DOCK_KNOWN'
    assert resume_phase('SPRAYING', {}).value == 'CORRECT_BAY_KNOWN'
    assert resume_phase('IDLE', {'watering_completed': True}).value == 'RETURNING'
    assert resume_phase('EXIT_RETURN', {}).value == 'RETURNING'


def test_detour_uses_expanded_circle_and_stays_in_geofence():
    zone = KeepOutCircle(10, 5, 2, 1, 'rtk_ned')
    route = plan_keep_out_detour(
        Pose2D(1, 5, 0), Pose2D(19, 5, 0), zone,
        boundary=((0, 0), (20, 0), (20, 10), (0, 10)),
        detour_margin_m=.5, arc_spacing_m=.5,
    )
    assert route.side in {'CLOCKWISE', 'COUNTERCLOCKWISE'}
    assert route.clearance_radius_m == 3.5
    assert route.waypoints[-1] == Pose2D(19, 5, 0)
    for waypoint in route.waypoints[:-1]:
        assert 0 <= waypoint.x <= 20 and 0 <= waypoint.y <= 10
        assert ((waypoint.x - 10) ** 2 + (waypoint.y - 5) ** 2) ** .5 >= 3.5 - 1e-8


def test_detour_rejects_start_or_target_inside_and_boundary_with_no_route():
    zone = KeepOutCircle(10, 5, 2, 1, 'rtk_ned')
    with pytest.raises(ValueError, match='START_INSIDE'):
        plan_keep_out_detour(Pose2D(10, 5, 0), Pose2D(19, 5, 0), zone,
                             boundary=((0, 0), (20, 0), (20, 10), (0, 10)))
    with pytest.raises(ValueError, match='TARGET_INSIDE'):
        plan_keep_out_detour(Pose2D(1, 5, 0), Pose2D(10, 5, 0), zone,
                             boundary=((0, 0), (20, 0), (20, 10), (0, 10)))
    edge_zone = KeepOutCircle(10, 5, 4.9, 0, 'rtk_ned')
    with pytest.raises(ValueError, match='NO_DETOUR'):
        plan_keep_out_detour(Pose2D(1, 5, 0), Pose2D(19, 5, 0), edge_zone,
                             boundary=((0, 0), (20, 0), (20, 10), (0, 10)),
                             detour_margin_m=.5)


def test_exit_uses_nearest_safe_radial_point_inside_geofence():
    zone = KeepOutCircle(10, 5, 2, 1, 'rtk_ned')
    target = plan_keep_out_exit(
        Pose2D(11, 5, 0), zone,
        boundary=((0, 0), (20, 0), (20, 10), (0, 10)),
        exit_margin_m=.5,
    )
    assert target.x == pytest.approx(13.5)
    assert target.y == pytest.approx(5.0)
