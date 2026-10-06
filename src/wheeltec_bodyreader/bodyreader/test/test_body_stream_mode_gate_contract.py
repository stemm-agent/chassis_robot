"""Static contracts for the body stream mode gate.

These tests intentionally avoid the Astra SDK so they can run on a development
host while still protecting the control-flow and ROS QoS assumptions used by
the production launch file.
"""

from pathlib import Path
import re
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MAIN_SOURCE = (PACKAGE_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
FOLLOW_LAUNCH = (
    PACKAGE_ROOT / "launch" / "bodyfollow_nav2.launch.py"
).read_text(encoding="utf-8")


def compact(text: str) -> str:
    return re.sub(r"\s+", " ", text)


class BodyStreamModeGateContractTest(unittest.TestCase):
    def test_mode_subscription_matches_controller_latched_qos(self):
        subscription = re.search(
            r'create_subscription<std_msgs::msg::Int8>\(\s*"/mode",'
            r'\s*(rclcpp::QoS\(1\)[^,]+),',
            MAIN_SOURCE,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(subscription, "/mode subscription was not found")
        qos = compact(subscription.group(1))

        # bodyfollow_mode_controller publishes /mode at depth 1 with these
        # policies; both reliability and durability must match so a restarted
        # body_main receives the controller's retained mode immediately.
        self.assertEqual(
            qos,
            "rclcpp::QoS(1).reliable().transient_local()",
        )

    def test_follow_launch_enables_mode_gate_and_requires_mode_two(self):
        self.assertIn("{'mode_gated_body_stream': True}", FOLLOW_LAUNCH)
        self.assertNotIn("{'mode_gated_body_stream': False}", FOLLOW_LAUNCH)
        self.assertIn("{'mode_required': 2}", FOLLOW_LAUNCH)

    def test_follow_pipeline_has_bounded_active_rates(self):
        # The camera supplied roughly 10 useful frames/s during the captured
        # summon.  Do not enqueue camera/TensorRT work faster than that, and do
        # not double the production Nav2 control loop for the occasional
        # obstacle-takeover path.
        self.assertIn("{'max_processing_rate_hz': 10.0}", FOLLOW_LAUNCH)
        self.assertIn("'max_inference_fps': 10.0", FOLLOW_LAUNCH)
        self.assertIn("{'controller_frequency': 10.0}", FOLLOW_LAUNCH)
        self.assertNotIn("{'controller_frequency': 20.0}", FOLLOW_LAUNCH)

    def test_mode_two_starts_and_mode_one_stops_body_stream(self):
        source = compact(MAIN_SOURCE)
        self.assertIn(
            "const bool should_run = requested_mode == mode_required;",
            source,
        )
        self.assertRegex(
            source,
            re.compile(
                r"if \(should_run && !body_stream_running\).*?"
                r"astra_stream_start\(bodyStream\)",
            ),
        )
        self.assertRegex(
            source,
            re.compile(
                r"else if \(!should_run && body_stream_running\).*?"
                r"astra_stream_stop\(bodyStream\).*?"
                r"body_stream_running = false",
            ),
        )

    def test_rgb_stream_remains_active_when_body_stream_is_gated(self):
        source = compact(MAIN_SOURCE)

        # RGB starts independently of the body-stream gate.
        self.assertRegex(
            source,
            re.compile(
                r"if \(rgb_stream\).*?astra_stream_start\(colorStream\)"
            ),
        )

        # A stopped body stream only idles the loop when RGB is also disabled,
        # and RGB frame publication remains conditioned solely on rgb_stream.
        self.assertIn(
            "if (!body_stream_running && !rgb_stream)",
            source,
        )
        self.assertRegex(
            source,
            re.compile(
                r"if \(rgb_stream\).*?astra_frame_get_colorframe"
                r"\(frame, &colorFrame\).*?print_color\(colorFrame\)",
            ),
        )


if __name__ == "__main__":
    unittest.main()
