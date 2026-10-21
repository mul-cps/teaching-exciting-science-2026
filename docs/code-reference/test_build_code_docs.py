"""Offline regression checks for the code-reference generator itself."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("build_code_docs", ROOT / "tools/build_code_docs.py")
docs = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(docs)


class CodeReferenceTests(unittest.TestCase):
    def test_static_visitor_never_imports_or_executes_source(self):
        code = '''"""Module documentation."""
import missing_robot_sdk
raise RuntimeError("must never execute")
class Motor:
    """Own a motor."""
    def move(self, amount: float, *, enabled=False) -> bool:
        """Request an offset."""
        return enabled
'''
        with patch("builtins.__import__", wraps=__import__) as imports:
            text, count, _ = docs.python_reference(Path("01_motor.launch.py"), code)
        self.assertEqual(count, 2)
        self.assertIn("def Motor.move(amount: float, *, enabled=False) -> bool", text)
        self.assertIn("Request an offset", text)
        self.assertNotIn("missing_robot_sdk", [call.args[0] for call in imports.call_args_list])

    def test_endpoint_expressions_are_preserved_without_evaluation(self):
        calls = docs.ros_calls('self.create_publisher(String, prefix + "/status", 10)\nself.declare_parameter("dry_run", True)')
        self.assertEqual([row["line"] for row in calls], [1, 2])
        self.assertIn("prefix + '/status'", calls[0]["expression"])
        self.assertEqual(calls[1]["kind"], "declare_parameter")

    def test_wrappers_configs_and_hidden_paths(self):
        self.assertEqual(docs.language(Path("scripts/operator_api"), "#!/usr/bin/env python3\n"), "python")
        self.assertEqual(docs.language(Path("CMakeLists.txt"), "project(example)"), "cmake")
        self.assertEqual(docs.language(Path("requirements-docs.txt"), "mkdocs==1.6.1"), "text")
        self.assertEqual(docs.language(Path("tutorial.ipynb"), "{}"), "notebook")
        self.assertEqual(docs.page_name(Path(".github/workflows/check.yml")), "source/_dot_github/workflows/check.yml.md")
        self.assertNotEqual(docs.page_name(Path("x/a.py")), docs.page_name(Path("x/a.sh")))
        self.assertIsNone(docs.language(Path("LICENSE"), "License"))

    def test_embedded_fences_are_escaped(self):
        self.assertTrue(docs.fenced("```python\ntext\n```", "text").startswith("````text"))

    def test_broken_local_anchor_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "site").mkdir()
            (output / "site/index.html").write_text('<a href="#absent">broken</a>')
            with patch.object(docs, "OUTPUT", output):
                with self.assertRaisesRegex(RuntimeError, "missing anchor"):
                    docs.check_site({"files": []})


if __name__ == "__main__":
    unittest.main()
