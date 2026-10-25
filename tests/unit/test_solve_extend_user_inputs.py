from osdagbridge_core.core.bridge_types.plate_girder.defaults import solve_extend_basic_input_dict
from osdagbridge_core.core.utils.common import (
    KEY_SPAN,
    KEY_CARRIAGEWAY_WIDTH,
    KEY_FOOTPATH,
    KEY_DESIGN_MODE,
    KEY_TS_NO_OF_GIRDERS,
    KEY_TS_GIRDER_SPACING,
    KEY_TS_DECK_OVERHANG,
    KEY_TS_DECK_THICKNESS,
    KEY_MP_GIRDER_DEPTH,
    KEY_MP_GIRDER_TOP_FLANGE_WIDTH,
    KEY_MP_CB_NO_OF_CROSS_BRACINGS,
    KEY_MP_CB_SPACING,
    KEY_PL_SELF_WEIGHT_FACTOR,
    KEY_DO_GAMMA_C_BASIC,
)


def _base_input_dict():
    return {
        KEY_SPAN: 30.0,
        KEY_CARRIAGEWAY_WIDTH: 7.5,
        KEY_FOOTPATH: "None",
        KEY_DESIGN_MODE: "Custom",
    }


def test_solve_extend_preserves_user_girder_count():
    d = _base_input_dict()
    # 7.5m carriageway + 2*0.45m barriers = 8.4m overall.
    # Seed gives 5 girders (8.4/2 = 4.2 -> 5).
    # User selects 6 girders (spacing = 8.4/6 = 1.4m >= 1.0m).
    d[KEY_TS_NO_OF_GIRDERS] = 6
    solve_extend_basic_input_dict(d)

    assert d[KEY_TS_NO_OF_GIRDERS] == 6
    # Girders up to G6 should be present
    assert f"{KEY_MP_GIRDER_DEPTH}.G6.M1" in d


def test_solve_extend_preserves_ten_girders_on_wide_bridge():
    d = _base_input_dict()
    d[KEY_CARRIAGEWAY_WIDTH] = 14.1
    # User selects 10 girders
    d[KEY_TS_NO_OF_GIRDERS] = 10
    solve_extend_basic_input_dict(d)

    assert d[KEY_TS_NO_OF_GIRDERS] == 10
    assert f"{KEY_MP_GIRDER_DEPTH}.G10.M1" in d


def test_solve_extend_preserves_user_deck_thickness():
    d = _base_input_dict()
    d[KEY_TS_DECK_THICKNESS] = 320.0
    solve_extend_basic_input_dict(d)

    assert d[KEY_TS_DECK_THICKNESS] == 320.0


def test_solve_extend_preserves_user_girder_member_properties():
    d = _base_input_dict()
    d[KEY_TS_NO_OF_GIRDERS] = 6
    d[f"{KEY_MP_GIRDER_DEPTH}.G1.M1"] = 1850
    d[f"{KEY_MP_GIRDER_TOP_FLANGE_WIDTH}.G1.M1"] = 550
    solve_extend_basic_input_dict(d)

    assert d[f"{KEY_MP_GIRDER_DEPTH}.G1.M1"] == 1850
    assert d[f"{KEY_MP_GIRDER_TOP_FLANGE_WIDTH}.G1.M1"] == 550


def test_solve_extend_preserves_user_cross_bracing_and_loading():
    d = _base_input_dict()
    d[KEY_MP_CB_NO_OF_CROSS_BRACINGS] = 7
    d[KEY_MP_CB_SPACING] = 3.75
    d[KEY_PL_SELF_WEIGHT_FACTOR] = "1.15"
    d[KEY_DO_GAMMA_C_BASIC] = "1.35"
    solve_extend_basic_input_dict(d)

    assert d[KEY_MP_CB_NO_OF_CROSS_BRACINGS] == 7
    assert d[KEY_MP_CB_SPACING] == 3.75
    assert d[KEY_PL_SELF_WEIGHT_FACTOR] == "1.15"
    assert d[KEY_DO_GAMMA_C_BASIC] == "1.35"
