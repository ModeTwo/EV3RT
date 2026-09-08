"""Checks for the standard imports required by every feature module."""

import ast
from pathlib import Path
import unittest


REQUIRED_IMPORTS = {
    "Behaviour",
    "BottleColor",
    "Color",
    "Failure",
    "HeadingType",
    "Parallel",
    "ParallelPolicy",
    "Running",
    "Selector",
    "Sequence",
    "Status",
    "Success",
    "TargetInterested",
    "TraceSide",
    "runtime",
    "time",
}


class FeatureImportTest(unittest.TestCase):
    def test_all_feature_modules_have_standard_bt_imports(self) -> None:
        features_dir = Path(__file__).resolve().parents[1] / "features"
        excluded = {"__init__.py", "bt_imports.py"}

        for path in sorted(features_dir.glob("*.py")):
            if path.name in excluded:
                continue
            with self.subTest(feature=path.name):
                tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
                imported = set()
                for node in tree.body:
                    if (
                        isinstance(node, ast.ImportFrom)
                        and node.level == 1
                        and node.module == "bt_imports"
                    ):
                        imported.update(alias.name for alias in node.names)
                self.assertEqual(REQUIRED_IMPORTS, imported)


if __name__ == "__main__":
    unittest.main()
