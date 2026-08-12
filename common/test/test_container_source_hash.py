#!/usr/bin/env python3

import os
import stat
import subprocess
import tempfile
import unittest


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")
)
PROJECT_ROOT = os.path.abspath(
    os.environ.get(
        "SIM2REAL_PROJECT_ROOT",
        os.path.join(PACKAGE_ROOT, ".."),
    )
)
HASH_HELPER = os.path.join(PROJECT_ROOT, "launch", "container_source_hash.sh")


def source_hash(root, *paths):
    completed = subprocess.run(
        [
            "bash",
            "-c",
            'source "$1"; shift; compute_container_source_hash "$@"',
            "bash",
            HASH_HELPER,
            root,
            *paths,
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return completed.stdout.strip()


class ContainerSourceHashTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not os.path.isfile(HASH_HELPER):
            raise unittest.SkipTest(
                "repository container hash helper is not mounted"
            )

    def test_content_path_and_executable_mode_change_digest(self):
        with tempfile.TemporaryDirectory() as root:
            os.makedirs(os.path.join(root, "source"))
            first = os.path.join(root, "source", "entry.py")
            with open(first, "w", encoding="utf-8") as stream:
                stream.write("print('one')\n")
            initial = source_hash(root, "source")

            with open(first, "w", encoding="utf-8") as stream:
                stream.write("print('two')\n")
            content_changed = source_hash(root, "source")
            self.assertNotEqual(initial, content_changed)

            os.chmod(first, os.stat(first).st_mode | stat.S_IXUSR)
            mode_changed = source_hash(root, "source")
            self.assertNotEqual(content_changed, mode_changed)

            second = os.path.join(root, "source", "renamed.py")
            os.rename(first, second)
            self.assertNotEqual(mode_changed, source_hash(root, "source"))

    def test_generated_caches_and_workspaces_do_not_change_digest(self):
        with tempfile.TemporaryDirectory() as root:
            os.makedirs(os.path.join(root, "source"))
            with open(
                os.path.join(root, "source", "entry.py"),
                "w",
                encoding="utf-8",
            ) as stream:
                stream.write("pass\n")
            initial = source_hash(root, "source")

            generated_files = (
                "source/__pycache__/entry.cpython-38.pyc",
                "source/build/object.o",
                "source/devel/setup.bash",
                "source/logs/build.log",
                "source/.pytest_cache/state",
            )
            for relative in generated_files:
                path = os.path.join(root, relative)
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, "wb") as stream:
                    stream.write(b"generated")
            self.assertEqual(initial, source_hash(root, "source"))

    def test_rejects_paths_outside_source_root(self):
        with tempfile.TemporaryDirectory() as root:
            completed = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1"; compute_container_source_hash "$2" ../outside',
                    "bash",
                    HASH_HELPER,
                    root,
                ],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.assertNotEqual(0, completed.returncode)
            self.assertIn("must stay below", completed.stderr)


if __name__ == "__main__":
    unittest.main()
