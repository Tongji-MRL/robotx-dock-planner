import unittest
import xml.etree.ElementTree as ET

from examples.phase3_world import (
    EXIT_DELAY_SECONDS,
    RUNNING_STATE_SECONDS,
    build_phase3_world,
    source_world,
)


class Phase3WorldTests(unittest.TestCase):
    def test_only_lifecycle_override_is_present(self):
        root = build_phase3_world().getroot()
        world = root.find("./world")
        self.assertIsNotNone(world)
        self.assertEqual(world.get("name"), "scan_dock_deliver_task")
        plugin = world.find("./plugin[@name='vrx::ScanDockScoringPlugin']")
        self.assertIsNotNone(plugin)
        self.assertEqual(float(plugin.findtext("exit_delay")), EXIT_DELAY_SECONDS)
        self.assertEqual(plugin.findtext("initial_state_duration"), "10")
        self.assertEqual(plugin.findtext("ready_state_duration"), "10")
        self.assertEqual(
            float(plugin.findtext("running_state_duration")),
            RUNNING_STATE_SECONDS,
        )
        self.assertEqual(plugin.findtext("release_topic"), "/vrx/release")

        plugin.remove(plugin.find("exit_delay"))
        official = ET.parse(source_world()).getroot()
        official_plugin = official.find(
            "./world/plugin[@name='vrx::ScanDockScoringPlugin']"
        )
        plugin.find("running_state_duration").text = official_plugin.findtext(
            "running_state_duration"
        )

        def semantic_tree(element):
            return (
                element.tag,
                tuple(sorted(element.attrib.items())),
                (element.text or "").strip(),
                tuple(semantic_tree(child) for child in element),
            )

        self.assertEqual(semantic_tree(root), semantic_tree(official))


if __name__ == "__main__":
    unittest.main()
