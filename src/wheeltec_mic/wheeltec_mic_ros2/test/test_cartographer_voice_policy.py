import unittest

from cartographer_voice_policy import (
    ENDING,
    IDLE,
    PAUSED,
    RUNNING,
    STARTING,
    apply_pause_latch,
    classify_external_state,
    command_action,
    command_in_cooldown,
    command_is_allowed,
    motion_is_blocked,
    service_discovery_decision,
    service_request_decision,
    should_preserve_confirmed_state,
    success_feedback_audio,
    COMPLETE,
    SEND,
    TIMEOUT,
    WAIT,
)


class CartographerVoicePolicyTest(unittest.TestCase):
    def test_external_state_recovery(self):
        self.assertEqual(
            IDLE,
            classify_external_state('inactive', False, None, False),
        )
        self.assertEqual(
            STARTING,
            classify_external_state('active', True, None, False),
        )
        self.assertEqual(
            RUNNING,
            classify_external_state(
                'active', True,
                {'mode': 'mapping', 'mapping_done': False}, True),
        )
        self.assertEqual(
            PAUSED,
            classify_external_state(
                'active', True,
                {'mode': 'stopped', 'mapping_done': False}, True),
        )
        self.assertEqual(
            ENDING,
            classify_external_state(
                'active', True,
                {'mode': 'returning_home', 'mapping_done': True}, True),
        )
        self.assertEqual(
            PAUSED,
            classify_external_state(
                'active', True,
                {'mode': 'stopped', 'mapping_done': True}, True),
        )

    def test_commands_are_state_scoped(self):
        self.assertEqual('start', command_action('开始建图'))
        self.assertEqual('pause', command_action('暂停建图'))
        self.assertEqual('end', command_action('结束建图'))
        self.assertEqual('save', command_action('保存地图'))
        self.assertEqual('continue', command_action('继续建图'))
        self.assertIsNone(command_action('开始画图'))
        self.assertTrue(command_is_allowed('start', IDLE))
        self.assertFalse(command_is_allowed('start', RUNNING))
        self.assertTrue(command_is_allowed('pause', RUNNING))
        self.assertFalse(command_is_allowed('pause', PAUSED))
        self.assertTrue(command_is_allowed('continue', PAUSED))
        self.assertTrue(command_is_allowed('save', RUNNING))
        self.assertTrue(command_is_allowed('save', PAUSED))
        self.assertTrue(command_is_allowed('end', RUNNING))
        self.assertTrue(command_is_allowed('end', PAUSED))
        for action in ('start', 'pause', 'continue', 'save', 'end'):
            self.assertFalse(command_is_allowed(action, STARTING))
            self.assertFalse(command_is_allowed(action, ENDING))

    def test_repeat_commands_are_deduplicated(self):
        history = {}
        self.assertFalse(command_in_cooldown(history, '保存地图', 10.0, 2.0))
        self.assertTrue(command_in_cooldown(history, '保存地图', 11.0, 2.0))
        self.assertFalse(command_in_cooldown(history, '保存地图', 12.1, 2.0))

    def test_service_waits_and_timeouts_are_bounded(self):
        self.assertEqual(WAIT, service_discovery_decision(False, 1.0, 2.0))
        self.assertEqual(SEND, service_discovery_decision(True, 1.0, 2.0))
        self.assertEqual(TIMEOUT, service_discovery_decision(False, 2.0, 2.0))
        self.assertEqual(WAIT, service_request_decision(False, 1.0, 2.0))
        self.assertEqual(COMPLETE, service_request_decision(True, 1.0, 2.0))
        self.assertEqual(TIMEOUT, service_request_decision(False, 2.0, 2.0))

    def test_motion_gate_follows_external_session(self):
        self.assertFalse(motion_is_blocked(IDLE, 'inactive', False))
        self.assertTrue(motion_is_blocked(STARTING, 'inactive', False))
        self.assertTrue(motion_is_blocked(IDLE, 'active', False))
        self.assertTrue(motion_is_blocked(IDLE, 'inactive', True))

    def test_confirmed_state_survives_nav_gap_while_lock_is_held(self):
        self.assertTrue(should_preserve_confirmed_state(
            RUNNING, 'inactive', True, True, False))
        self.assertTrue(should_preserve_confirmed_state(
            PAUSED, 'inactive', True, True, False))
        self.assertFalse(should_preserve_confirmed_state(
            STARTING, 'inactive', True, True, False))
        self.assertFalse(should_preserve_confirmed_state(
            RUNNING, 'inactive', False, True, False))
        self.assertFalse(should_preserve_confirmed_state(
            RUNNING, 'inactive', True, True, True))

    def test_pause_latch_survives_transient_navigation_states(self):
        self.assertEqual(PAUSED, apply_pause_latch(STARTING, True))
        self.assertEqual(PAUSED, apply_pause_latch(RUNNING, True))
        self.assertEqual(PAUSED, apply_pause_latch(PAUSED, True))
        self.assertEqual(ENDING, apply_pause_latch(ENDING, True))
        self.assertEqual(IDLE, apply_pause_latch(IDLE, True))
        self.assertEqual(RUNNING, apply_pause_latch(RUNNING, False))

    def test_success_feedback_audio(self):
        self.assertEqual('/OK.wav', success_feedback_audio('save'))
        self.assertEqual('/OK.wav', success_feedback_audio('pause'))
        self.assertEqual(
            '/mapping_return_home.wav',
            success_feedback_audio('end'),
        )
        self.assertEqual('/OK.wav', success_feedback_audio('continue'))


if __name__ == '__main__':
    unittest.main()
