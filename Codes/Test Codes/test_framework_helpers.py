import ast
from pathlib import Path


def load_helpers():
    framework_path = Path(__file__).resolve().parents[1] / "Main Codes" / "Round2 _Framework.py"
    source = framework_path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(framework_path))

    target_names = {"is_blue_line", "is_orange_line", "normalize_angle_error", "detect_orientation"}
    selected_nodes = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in target_names:
            selected_nodes.append(node)

    module = ast.Module(body=selected_nodes, type_ignores=[])
    namespace = {"__name__": "test_helpers"}
    exec(compile(module, str(framework_path), "exec"), namespace)
    return namespace


def test_is_blue_line_matches_expected_color_values():
    helpers = load_helpers()
    assert helpers["is_blue_line"](100, 90, 140) is True
    assert helpers["is_blue_line"](80, 70, 90) is False


def test_is_orange_line_matches_expected_color_values():
    helpers = load_helpers()
    assert helpers["is_orange_line"](220, 140, 100) is True
    assert helpers["is_orange_line"](100, 90, 130) is False


def test_normalize_angle_error_wraps_across_180_degrees():
    helpers = load_helpers()
    assert helpers["normalize_angle_error"](350, 10) == -20
    assert helpers["normalize_angle_error"](10, 350) == 20


def test_detect_orientation_returns_expected_camera_assignment():
    helpers = load_helpers()
    assert helpers["detect_orientation"](220, 140, 100) == ("orange", "RightCam")
    assert helpers["detect_orientation"](100, 90, 140) == ("blue", "LeftCam")
