from pathlib import Path


def test_filter_is_reapplied_after_can_start():
    source = (Path(__file__).resolve().parents[1] / "BSP" / "bsp_bxcan.c").read_text(
        encoding="utf-8"
    )
    body = source.split("void BSP_BXCAN_Init(void)", 1)[1].split("#else", 1)[0]

    start_pos = body.index("HAL_CAN_Start(&hcan1)")
    filter_positions = [
        pos for pos in range(len(body))
        if body.startswith("CAN1_Filter_Config();", pos)
    ]

    assert any(pos > start_pos for pos in filter_positions)
