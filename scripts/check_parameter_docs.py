#!/usr/bin/env python3
"""Check public parameter documentation without importing ROS or physics.

The Markdown meaning tables are maintained by humans. The default/source
appendices are generated from YAML, launch/CLI declarations and default data
classes. This is a documentation tool, never part of runtime configuration.
"""
import argparse
import ast
from collections import defaultdict
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

PACKAGES = ('dual_fr3_maniskill', 'dual_fr3_trunking_mtc', 'dual_fr3_cable_perception',
            'dual_fr3_moveit_config')
DATA_CLASSES = {
    'TrunkingDefaults': '', 'CameraSettings': 'camera_',
    'InsertionLimits': 'insertion.', 'GraspThresholds': 'grasp.',
    'GripperSafetyLimits': 'safety.',
}
DECLARATIONS = {'DeclareLaunchArgument', 'declare_argument', 'declare_parameter', 'add_argument'}


def normalize(name):
    name = name.split('ros__parameters.')[-1]
    name = re.sub(r'^profiles\.[^.]+\.', 'profiles.*.', name)
    name = re.sub(r'\[\d+\]', '[]', name)
    return name.lstrip('-').replace('-', '_')


def yaml_parameters(value, prefix=''):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from yaml_parameters(child, f'{prefix}.{key}' if prefix else str(key))
    elif isinstance(value, list) and value and isinstance(value[0], dict):
        for index, child in enumerate(value):
            yield from yaml_parameters(child, f'{prefix}[{index}]')
    else:
        yield prefix, repr(value)


def expression(node):
    if node is None:
        return '必填/由调用方提供'
    try:
        return repr(ast.literal_eval(node))
    except (ValueError, TypeError):
        return ast.unparse(node)


def yaml_locations(node, prefix=''):
    """Preserve source line and sequence index instead of flattening instances."""
    if isinstance(node, yaml.MappingNode):
        for key, child in node.value:
            yield from yaml_locations(child, f'{prefix}.{key.value}' if prefix else key.value)
    elif isinstance(node, yaml.SequenceNode) and node.value and isinstance(node.value[0], yaml.MappingNode):
        for index, child in enumerate(node.value):
            yield from yaml_locations(child, f'{prefix}[{index}]')
    else:
        yield prefix, node.start_mark.line + 1


def inventory(package):
    """Record every declaration, retaining intentional differences by source."""
    found = defaultdict(set)

    def add(name, path, value, line=None):
        if name and isinstance(name, str):
            found[normalize(name)].add((str(path.relative_to(package)), line or 1, value, name))

    for path in sorted((package / 'config').glob('*.yaml')):
        locations = dict(yaml_locations(yaml.compose(path.read_text())))
        for name, value in yaml_parameters(yaml.safe_load(path.read_text())):
            add(name, path, value, locations[name])
    for path in sorted((package / 'config').glob('*.xacro')):
        for argument in ET.parse(path).iter('{http://www.ros.org/wiki/xacro}arg'):
            add(argument.get('name'), path, argument.get('default', '必填'))
    sources = [p for p in package.rglob('*.py') if not any(
        part in ('test', 'scripts', '_compat', '__pycache__') for part in p.relative_to(package).parts)]
    for path in sorted(sources):
        tree = ast.parse(path.read_text())
        relative = str(path.relative_to(package))
        # Explicit section aliases used by the existing plain-dict loaders.
        # Restrict by file so observation/status dictionaries are never mistaken
        # for configuration. No runtime imports or evaluation of expressions.
        aliases = {'guide': 'guide', 'insertion': 'insertion', 'display': 'display'}
        if relative.endswith('cable/model.py'):
            aliases.update(c='cable', r='rope_actor', m='mpm')
        if relative.endswith('cable/rope_actor.py'):
            aliases['r'] = 'rope_actor'
        if relative.endswith('execution/gripper.py'):
            aliases.update(safety_raw='safety', values='profiles.*', epsilon='profiles.*.epsilon')
        if relative.endswith('stages/compiler.py'):
            aliases['settings'] = 'keypoints[].metadata.gripper'
        if relative.endswith('usb/scene.py') or '/insertion_task/' in relative:
            aliases.update(c='insertion', config='insertion', **{'self.config': 'insertion'})
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name in DATA_CLASSES:
                for field in node.body:
                    if isinstance(field, ast.AnnAssign) and isinstance(field.target, ast.Name):
                        add(DATA_CLASSES[node.name] + field.target.id, path, expression(field.value), field.lineno)
            if isinstance(node, ast.Call):
                function = getattr(node.func, 'id', getattr(node.func, 'attr', ''))
                if function in DECLARATIONS and node.args and isinstance(node.args[0], ast.Constant):
                    name = node.args[0].value
                    default = next((k.value for k in node.keywords if k.arg in ('default', 'default_value')), None)
                    if function in ('declare_parameter', 'declare_argument') and len(node.args) > 1:
                        default = node.args[1]
                    value = expression(default)
                    if function == 'add_argument' and default is None:
                        action = next((expression(k.value) for k in node.keywords if k.arg == 'action'), '')
                        value = 'False' if action == "'store_true'" else 'True' if action == "'store_false'" else 'None'
                    add(name, path, value, node.lineno)
                if function in ('get', 'setdefault') and len(node.args) > 1 and isinstance(node.args[0], ast.Constant):
                    receiver = ast.unparse(node.func.value)
                    section = aliases.get(receiver)
                    match = re.search(r"(?:config|cable_config)\[['\"](usb|cable|guide|mpm|rope_actor|insertion|display)['\"]\]$", receiver)
                    if match:
                        section = match.group(1)
                    if section and isinstance(node.args[0].value, str):
                        if not isinstance(node.args[1], ast.Dict):
                            add(section + '.' + node.args[0].value, path, expression(node.args[1]), node.lineno)
            # ROS bridge defaults and perception launch's dict comprehension.
            if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'defaults' for t in node.targets):
                if isinstance(node.value, ast.Dict) and path.name in ('bridge.py', 'node.py'):
                    for key, value in zip(node.value.keys, node.value.values):
                        if isinstance(key, ast.Constant):
                            add(key.value, path, expression(value), key.lineno)
                if isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) and node.value.func.id == 'dict':
                    if path.name == 'node.py' or path.parent.name == 'launch':
                        for key in node.value.keywords:
                            add(key.arg, path, expression(key.value), key.value.lineno)
                    if relative.endswith('cable/model.py'):
                        for key in node.value.keywords:
                            add('rope_actor.' + key.arg, path, expression(key.value), key.value.lineno)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'dict':
                if any(k.arg == 'perception_enabled' for k in node.keywords):
                    for key in node.keywords:
                        add(key.arg, path, expression(key.value), key.value.lineno)
    return found


def default_document(found):
    lines = ['# 参数默认值与声明位置（自动生成）', '',
             '由 `check_parameter_docs.py --write-defaults` 从当前源码生成。',
             '同名参数的不同入口逐行保留；表达式表示运行时求值，不代表唯一有效值。',
             '作用、单位、消费模块及验证见 [参数索引](parameters.md)。YAML 列出当前文件值，不继承其他预设。', '',
             '| 参数 | 源字段（区分实例） | 声明文件 | 默认值 / 表达式 |', '| --- | --- | --- | --- |']
    for name, entries in sorted(found.items()):
        for path, line, value, source_name in sorted(entries):
            value = value.replace('|', '\\|').replace('\n', ' ')
            lines.append(f'| `{name}` | `{source_name}` | [{path}:{line}](../{path}#L{line}) | `{value}` |')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--write-defaults', action='store_true')
    args = parser.parse_args()
    errors = []
    for name in PACKAGES:
        package = args.source_root / name
        found = inventory(package)
        document = package / 'docs/parameters.md'
        meanings = document.read_text() if document.exists() else ''
        # Only first-column keys count: mentioning a name in prose is insufficient.
        documented = set(re.findall(r'^\| `([^`]+)` \|', meanings, re.MULTILINE))
        missing = set(found) - documented
        if missing:
            errors.append(f'{name}: undocumented: ' + ', '.join(sorted(missing)))
        defaults = package / 'docs/parameter_defaults.md'
        expected = default_document(found)
        if args.write_defaults:
            defaults.parent.mkdir(exist_ok=True)
            defaults.write_text(expected)
        elif not defaults.exists() or defaults.read_text() != expected:
            errors.append(f'{name}: default/source appendix is stale; use --write-defaults')
        print(f'{name}: {len(found)} distinct parameters, {len(missing)} missing meanings')
    if errors:
        print('\n'.join(errors))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
