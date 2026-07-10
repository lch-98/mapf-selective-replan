# ─────────────────────────────────────────────────────────────────
# python/_pathsetup.py
#
# 컴파일된 mapf_py 확장 모듈(build/bindings/Release 또는 Debug 안에 있는
# .pyd)을 import 가능하게 sys.path에 등록한다. CMake가 매 빌드마다 이
# 파일을 건드리지 않도록, 여기서는 "리포 루트 기준 상대경로로 찾기"만
# 한다 — mapf_py.pyd를 python/gui/ 안으로 복사하는 설치 단계는 없다.
# ─────────────────────────────────────────────────────────────────
import sys
from pathlib import Path


def add_mapf_py_to_path() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    candidates = [
        repo_root / "build" / "bindings" / "Release",
        repo_root / "build" / "bindings" / "Debug",
    ]
    for candidate in candidates:
        if any(candidate.glob("mapf_py*.pyd")):
            sys.path.insert(0, str(candidate))
            return
    raise ImportError(
        "mapf_py.pyd를 찾을 수 없습니다. 먼저 빌드하세요:\n"
        "  cmake --build build --target mapf_py --config Release"
    )
