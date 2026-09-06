from pathlib import Path
import tempfile
import unittest

from doodle_bot.config import ExecutorConfig, PipelineConfig, StageConfig
from doodle_bot.contracts import Artifact, StageRequest
from doodle_bot.execution import ExecutorRouter, LocalExecutor
from doodle_bot.registry import StageRegistry
from doodle_bot.runner import PipelineRunner


class TestStage:
    type_name = "test.copy"

    def can_handle(self, artifact: Artifact) -> bool:
        return artifact.media_type == "image/png"

    def run(self, artifact: Artifact, request: StageRequest) -> Artifact:
        destination = request.run_dir / "result.png"
        destination.write_bytes(artifact.path.read_bytes())
        return Artifact(destination, "image/png", artifact.lineage + (request.stage_id,))


class RunnerTests(unittest.TestCase):
    def test_writes_artifact_and_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.png"
            source.write_bytes(b"png")
            registry = StageRegistry()
            registry.register(TestStage())
            config = PipelineConfig(
                "test", (StageConfig("copy", "test.copy", None, True, "local", {}),), {},
                {"local": ExecutorConfig("local", "local", None, None, 60)},
            )
            result = PipelineRunner(registry, ExecutorRouter({"local": LocalExecutor()})).run(config, source, root / "run")
            self.assertEqual(result.path.read_bytes(), b"png")
            self.assertTrue((root / "run" / "provenance.json").exists())


if __name__ == "__main__":
    unittest.main()
