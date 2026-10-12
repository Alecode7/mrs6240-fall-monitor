from __future__ import annotations

from pathlib import Path

from matplotlib import font_manager, rcParams


CHINESE_FONT_CANDIDATES = (
    "Microsoft YaHei",
    "SimHei",
    "Microsoft JhengHei",
    "Noto Sans CJK SC",
    "Source Han Sans SC",
    "Arial Unicode MS",
    "DejaVu Sans",
)
CHINESE_FONT_FILES = (
    Path("C:/Windows/Fonts/msyh.ttc"),
    Path("C:/Windows/Fonts/simhei.ttf"),
    Path("C:/Windows/Fonts/msjh.ttc"),
)


def configure_chinese_matplotlib_font() -> None:
    for font_path in CHINESE_FONT_FILES:
        if font_path.exists():
            font_manager.fontManager.addfont(str(font_path))
    existing = [font for font in rcParams.get("font.sans-serif", []) if font not in CHINESE_FONT_CANDIDATES]
    rcParams["font.sans-serif"] = [*CHINESE_FONT_CANDIDATES, *existing]
    rcParams["axes.unicode_minus"] = False
