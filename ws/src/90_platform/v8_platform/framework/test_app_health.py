"""Retain consuming component-health failures for application evidence."""
from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import app_runner


class AppHealthTests(unittest.TestCase):
    def test_first_dead_components_and_error_are_retained(self):
        assembly = mock.Mock()
        assembly.health.side_effect = [['carrier'], []]
        result = {'error': None, 'ok': False}
        self.assertTrue(app_runner.assembly_failed(assembly, result))
        assembly.health.assert_called_once_with()
        self.assertEqual(result['assembly_health'], ['carrier'])
        self.assertIn('carrier', result['error'])

    def test_healthy_poll_does_not_manufacture_a_component_failure(self):
        assembly = mock.Mock()
        assembly.health.return_value = []
        result = {'error': None}
        self.assertFalse(app_runner.assembly_failed(assembly, result))
        assembly.health.assert_called_once_with()
        self.assertEqual(result, {'error': None})

    def test_poll_exception_is_preserved_for_execute_error_handler(self):
        assembly = mock.Mock()
        assembly.health.side_effect = RuntimeError('health transport')
        with self.assertRaisesRegex(RuntimeError, 'health transport'):
            app_runner.assembly_failed(assembly, {'error': None})


if __name__ == '__main__':
    unittest.main()
