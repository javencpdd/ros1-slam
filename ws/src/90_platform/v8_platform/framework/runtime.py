"""Assembly plan -> existing ProcessRunner/Orchestrator (F3).

No ROS import: the default command builder emits rosrun/roslaunch argv; tests
inject a fake command factory so start/ready/stop/cleanup can be verified with
plain subprocesses.
"""
import json
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'orchestrator'))
import orchestrator as orch  # noqa: E402
from process_runner import ProcessRunner  # noqa: E402

import safety
from config import FrameworkError  # noqa: E402


def _resolve_launch(package, launch_file):
    """Return an absolute launch path when rospack can find it.

    roslaunch <pkg> <nested/file.launch> only resolves basenames; the
    generated skeleton uses $(find pkg)/<file>, so the real command
    must use the same absolute path rather than fail at launch time.
    """
    rospack = shutil.which('rospack')
    if not rospack:
        return None
    try:
        root = subprocess.check_output(
            [rospack, 'find', package], text=True,
            stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    candidate = Path(root) / launch_file
    return str(candidate) if candidate.is_file() else None


def default_command(instance, component):
    """Return (argv, ready_marker) for a plan instance."""
    spec = component['launch']
    namespace = instance.get('namespace')
    if spec['kind'] == 'include':
        if namespace:
            raise FrameworkError('E_RUNTIME_NAMESPACE',
                                 '%s: include cannot take a namespace' % instance['id'])
        args = dict(spec.get('args', {}))
        args.update(instance.get('params', {}))
        target = _resolve_launch(spec['package'], spec['launch_file'])
        command = (['roslaunch', target] if target else
                   ['roslaunch', spec['package'], spec['launch_file']])
        for key in sorted(args):
            value = args[key]
            if isinstance(value, bool):
                text = 'true' if value else 'false'
            elif isinstance(value, (dict, list)):
                text = json.dumps(value, separators=(',', ':'))
            else:
                text = str(value)
            command.append('%s:=%s' % (key, text))
        return command, spec.get('ready_marker')
    command = ['rosrun', spec['package'], spec['type']]
    if namespace:
        command.append('__ns:=' + namespace)
    for key in sorted(instance.get('params', {})):
        value = instance['params'][key]
        if isinstance(value, bool):
            text = 'true' if value else 'false'
        elif isinstance(value, (dict, list)):
            text = json.dumps(value, separators=(',', ':'))
        else:
            text = str(value)
        command.append('_%s:=%s' % (key, text))
    return command, spec.get('ready_marker')


def build_specs(plan, catalog):
    specs = []
    for instance in plan['instances']:
        component = catalog['components'][instance['component']]
        specs.append(orch.ComponentSpec(instance['id'], component['roles'],
                                        stop_method=component['stop'],
                                        resources=component.get('resources', [])))
    return specs


class Assembly:
    """Start an assembly plan through Orchestrator; stop/health are idempotent."""

    def __init__(self, plan, catalog, command_factory=None, budget=None,
                 ready_timeout=10.0, run_timeout=None):
        self.plan = plan
        self.catalog = catalog
        self.command_factory = command_factory or default_command
        self.budget = dict(budget or {})
        self.ready_timeout = ready_timeout
        self.run_timeout = run_timeout
        self.engine = None
        self.task = None
        self.runners = {}
        self.started = False

    def start(self):
        if self.started:
            raise FrameworkError('E_RUNTIME_STARTED', self.plan['name'])
        safety.check_plan(self.plan, self.catalog)
        specs = build_specs(self.plan, self.catalog)
        runners = {}
        for instance in self.plan['instances']:
            component = self.catalog['components'][instance['component']]
            command, marker = self.command_factory(instance, component)
            runners[instance['id']] = ProcessRunner(
                next(spec for spec in specs if spec.id == instance['id']),
                command, ready_marker=marker)
        engine = orch.Orchestrator(self.budget)
        task = engine.submit(self.plan['name'], specs)
        self.engine, self.task, self.runners = engine, task, runners
        try:
            engine.start(task, runners, ready_timeout=self.ready_timeout,
                         run_timeout=self.run_timeout)
        except Exception:
            self.started = False
            raise
        self.started = True
        return task

    def ready(self):
        return bool(self.runners) and all(r.ready() for r in self.runners.values())

    def health(self):
        if not self.started or self.task.state != orch.TaskState.RUNNING:
            return []
        dead = self.engine.health_check(self.task, self.runners)
        return dead

    def stop(self):
        if self.engine is not None and self.task is not None:
            if self.task.state == orch.TaskState.RUNNING:
                self.engine.cancel(self.task, self.runners)
            for runner in self.runners.values():
                try:
                    runner.reap()
                except orch.OrchestratorError:
                    pass
        self.started = False

    @property
    def stop_record(self):
        return list(self.task.stop_record) if self.task is not None else []
