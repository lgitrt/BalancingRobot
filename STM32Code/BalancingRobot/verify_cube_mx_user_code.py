# Author: Luca Obwegs
"""Verify CubeMX preserved the balancing application integration hooks."""

from __future__ import annotations

import re
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent


def user_section(source: str, section: str) -> str:
    begin = f"/* USER CODE BEGIN {section} */"
    end = f"/* USER CODE END {section} */"
    if source.count(begin) != 1 or source.count(end) != 1:
        raise ValueError(f"main.c must contain exactly one USER CODE {section} section")
    start = source.index(begin) + len(begin)
    finish = source.index(end, start)
    if finish < start:
        raise ValueError(f"USER CODE {section} section has reversed markers")
    return source[start:finish]


def require_once(pattern: str, source: str, description: str) -> None:
    if len(re.findall(pattern, source)) != 1:
        raise ValueError(f"expected exactly one {description} in its CubeMX user section")


def main() -> int:
    ioc = (PROJECT_DIR / "BalancingRobot.ioc").read_text(encoding="utf-8")
    main_source = (PROJECT_DIR / "Core" / "Src" / "main.c").read_text(
        encoding="utf-8"
    )

    if not re.search(r"(?m)^ProjectManager\.KeepUserCode=true$", ioc):
        raise ValueError("enable Keep User Code when re-generating code in CubeMX")

    include_section = user_section(main_source, "Includes")
    init_section = user_section(main_source, "2")
    run_section = user_section(main_source, "3")
    require_once(r'#include\s+"robot_app\.h"', include_section, "robot_app.h include")
    require_once(r"\bRobot_App_Init\s*\(", init_section, "Robot_App_Init call")
    require_once(r"\bRobot_App_Run\s*\(", run_section, "Robot_App_Run call")

    if main_source.count("Robot_App_Init(") != 1:
        raise ValueError("Robot_App_Init must only be called from USER CODE BEGIN 2")
    if main_source.count("Robot_App_Run(") != 1:
        raise ValueError("Robot_App_Run must only be called from USER CODE BEGIN 3")

    init_position = main_source.index("Robot_App_Init(")
    peripheral_initializers = (
        "MX_GPIO_Init",
        "MX_DMA_Init",
        "MX_LPUART1_UART_Init",
        "MX_I2C1_Init",
        "MX_TIM1_Init",
        "MX_TIM2_Init",
        "MX_TIM6_Init",
        "MX_IWDG_Init",
    )
    if any(
        main_source.index(f"{initializer}();") > init_position
        for initializer in peripheral_initializers
    ):
        raise ValueError("Robot_App_Init must run after all CubeMX peripherals initialize")
    if main_source.index("while (1)") > main_source.index("Robot_App_Run("):
        raise ValueError("Robot_App_Run must remain inside the main loop")

    print("CubeMX user-code preservation and robot application hooks are intact.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        raise SystemExit(f"CubeMX integration check failed: {error}") from error
