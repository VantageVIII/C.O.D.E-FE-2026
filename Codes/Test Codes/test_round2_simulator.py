import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Main Codes"))

from round2_simulator import Round2Simulator


def test_start_and_orientation_use_button_and_down_camera():
    robot = Round2Simulator()
    robot.frame(0, down_color="blue")
    assert robot.state == "WAITING"

    robot.press_start(1)
    robot.frame(2, down_color="blue")
    assert robot.state == "RUNNING"
    assert robot.orientation == "blue"


def test_forward_pillar_runs_settle_then_pulse_and_wraps_lap():
    robot = Round2Simulator()
    robot.press_start(0)
    robot.frame(1, down_color="orange", forward_color="green")
    assert robot.state == "TURN"
    assert robot.turn_phase == "SETTLE"

    for index in range(3):
        robot.frame(2 + index * 2)
        robot.frame(3 + index * 2)
        robot.frame(4 + index * 2)
    assert robot.lap_count == 0

    robot.frame(8, forward_color="red")
    robot.frame(9)
    robot.frame(10)
    assert robot.lap_count == 1
    assert robot.rotation_index == 0


def test_three_laps_enter_parking_and_purple_finishes():
    robot = Round2Simulator()
    robot.press_start(0)
    for turn in range(12):
        robot.frame(turn * 3 + 1, forward_color="green")
        robot.frame(turn * 3 + 2)
        robot.frame(turn * 3 + 3)
    assert robot.lap_count == 3
    assert robot.state == "PARKING"

    robot.frame(40, forward_color="purple")
    assert robot.state == "PARKED"
    assert robot.parking_complete


def test_round_timer_stops_the_run():
    robot = Round2Simulator()
    robot.press_start(0)
    robot.frame(180)
    assert robot.state == "TIMEOUT"
