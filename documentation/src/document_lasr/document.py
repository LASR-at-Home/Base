import re
import os
from os import listdir
from os.path import join, exists
import xml.etree.ElementTree as ET
from collections import OrderedDict

def generate_readme(pkg_dir):
    # Prepare the data
    data = {
        'package_name': None,
        'description': 'No description provided.',
        'maintainers': [],
        'authors': [],
        'dependencies': []
    }

    # Load package.xml
    tree = ET.parse(join(pkg_dir, 'package.xml'))

    for child in tree.getroot():
        if child.tag == 'name':
            data['package_name'] = child.text
        elif child.tag == 'description':
            data['description'] = child.text
        elif child.tag == 'maintainer':
            data['maintainers'].append({
                'name': child.text,
                'email': child.attrib['email']
            })
        elif child.tag == 'author':
            data['authors'].append({
                'name': child.text,
                'email': child.attrib['email']
            })
        elif child.tag in ['buildtool_depend', 'depend', 'build_depend', 'exec_depend', 'test_depend', 'doc_depend']:
            data['dependencies'].append({
                'package': child.text,
                'type': child.tag
            })

    # Generate maintainers list
    MAINTAINERS = '''This package is maintained by:'''

    for maintainer in data['maintainers']:
        name = maintainer['name']
        email = maintainer['email']
        MAINTAINERS += f'\n- [{name}](mailto:{email})'

    data['maintainers'] = MAINTAINERS

    # Generate authors list
    AUTHORS = ''

    if len(data['authors']) > 0:
        AUTHORS = '''\nThe following people have contributed to this package:'''

        for author in data['authors']:
            name = author['name']
            email = author['email']
            AUTHORS += f'\n- [{name}](mailto:{email})'
        
        AUTHORS += '\n'

    data['authors'] = AUTHORS

    # Generate dependency list
    DEPENDENCIES = 'This package has no additional ROS dependencies.'

    if len(data['dependencies']) > 0:
        DEPENDENCIES = '''This package depends on the following ROS packages:'''

        for dependency in data['dependencies']:
            name = dependency['package']
            depend_type = dependency['type'].replace('depend', '').replace('_', '')

            if len(depend_type) > 0:
                depend_type = f' ({depend_type})'

            DEPENDENCIES += f'\n- {name}{depend_type}'

    # Try to detect required Python version if using catkin_virtualenv
    with open(join(pkg_dir, 'CMakeLists.txt')) as f:
        cmakelists = f.read()
        match = re.search(r'PYTHON_INTERPRETER ([\w\d\.]+)', cmakelists)
        if match is not None:
            python = match.group(1)
            if python.startswith('python'):
                version = python[6:]
                python = f'Python {version}'

            DEPENDENCIES += f'\n\nThis packages requires {python} to be present.'

    # Generate Python dependency list
    PATH = join(pkg_dir, 'requirements.txt')
    if exists(PATH):
        LOCKED_PATH = join(pkg_dir, 'requirements.in')
        LOCKED_DEPENDS = exists(LOCKED_PATH)

        python_requirements: list[str]
        all_requirements: list[str]
        if LOCKED_DEPENDS:
            with open(LOCKED_PATH, 'r') as f:
                python_requirements = f.read().strip().split('\n')
            
            with open(PATH, 'r') as f:
                all_requirements = f.read().strip().split('\n')
        else:
            with open(PATH, 'r') as f:
                python_requirements = f.read().strip().split('\n')
                all_requirements = python_dependencies
        
        def split_depend(input: str) -> (str, str):
            # get rid of any garbage
            input = input.strip().split(' ')[0]

            # skip comments and empty lines
            if input.startswith('#') or len(input) == 0:
                return None

            # determine package name and restriction
            for restriction in ['==', '>', '<']:
                if restriction in input:
                    name, ver = input.split(restriction)
                    return (name, f'{restriction}{ver}')

            # otherwise just return everything as the package name
            return (input, None)

        python_dependencies = [
            split_depend(depend) for depend in python_requirements if split_depend(depend) is not None
        ]

        all_python_dependencies = [
            split_depend(depend) for depend in all_requirements if split_depend(depend) is not None
        ]

        DEPENDENCIES += f'\n\nThis package has {len(all_python_dependencies)} Python dependencies:'

        for (name, restriction) in python_dependencies:
            DEPENDENCIES += f'\n- [{name}](https://pypi.org/project/{name}){restriction}'

        hidden_dependencies = [
            name for (name, _) in all_python_dependencies if not any(name == pkg for (pkg, _) in python_dependencies)
        ]

        if len(hidden_dependencies) > 0:
            DEPENDENCIES += f'\n- .. and {len(hidden_dependencies)} sub dependencies'

    data['dependencies'] = DEPENDENCIES

    # Load additional markdown files
    data['prerequisites'] = 'Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!'
    data['usage'] = 'Ask the package maintainer to write a `doc/USAGE.md` for their package!'
    data['example'] = 'Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!'
    data['technical'] = 'Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!'

    for key, file in [
        ('prerequisites', 'PREREQUISITES'),
        ('usage', 'USAGE'),
        ('example', 'EXAMPLE'),
        ('technical', 'TECHNICAL')
    ]:
        PATH = join(pkg_dir, f'doc/{file}.md')
        if exists(PATH):
            with open(PATH, 'r') as f:
                data[key] = f.read().strip()

    # Generate documentation for ROS-specific components
    data['launch_files'] = 'This package has no launch files.\n'
    data['messages'] = 'This package has no messages.\n'
    data['services'] = 'This package has no services.\n'
    data['actions'] = 'This package has no actions.\n'

    # Generate launch file table
    LAUNCH_FILES = join(pkg_dir, 'launch')
    if exists(LAUNCH_FILES):
        files = listdir(LAUNCH_FILES)

        data['launch_files'] = ''

        for file in files:
            if not file.endswith('.launch'):
                continue
                
            name = file[:-7]
            data['launch_files'] += f'#### `{name}`\n\n'

            # Load launch file XML
            tree = ET.parse(join(pkg_dir, 'launch', file))

            launch_data = {
                'description': 'No description provided.',
                'usage': [],
                'arguments': [],
            }

            for child in tree.getroot():
                if child.tag == 'description':
                    launch_data['description'] = child.text
                elif child.tag == 'usage':
                    launch_data['usage'].append({
                        'description': child.attrib['doc'],
                        'arguments': child.text
                    })
                elif child.tag == 'arg':
                    if 'value' in child.attrib:
                        continue
                    
                    launch_data['arguments'].append(child.attrib)

            # Generate list of example usages
            LAUNCH_USAGE = '\n\n```bash'
                
            for use in launch_data['usage']:
                pkg_name = data['package_name']
                description = use['description']
                arguments = use['arguments'] or ''
                LAUNCH_USAGE += f'\n# {description}\nroslaunch {pkg_name} {file} {arguments}\n'

            launch_data['usage'] = f'{LAUNCH_USAGE}```' if len(launch_data['usage']) > 0 else ''

            # Generate table of arguments
            table = '\n\n| Argument | Default | Description |\n|:-:|:-:|---|\n'
            for arg in launch_data['arguments']:
                argument = arg['name']
                default = arg['default'] if 'default' in arg else ''
                description = arg['doc'] if 'doc' in arg else ''
                table += f'| {argument} | {default} | {description} |\n'

            launch_data['arguments'] = table if len(launch_data['arguments']) > 0 else ''

            data['launch_files'] += '{description}{usage}{arguments}\n\n'.format(**launch_data)
    
    # Helper functions for ROS messages
    def parse_rosmsg_definition(input):
        '''
        Convert message definition into a dictionary
        '''

        COMMENT_BUFFER = ''
        MESSAGE = OrderedDict()

        for line in input.split('\n'):
            line = line.strip()
            # skip empty lines
            if len(line) == 0:
                continue
            # buffer comments
            elif line.startswith('#'):
                COMMENT_BUFFER += '\n' + line[1:].strip()
            # parse type / name
            else:
                # add comment to buffer if exists
                if '#' in line:
                    definition, comment = line.split('#', 1)
                    COMMENT_BUFFER += '\n' + comment
                else:
                    definition = line

                # destruct the type and name
                var_type, var_name = [seg for seg in definition.strip().split(' ') if len(seg) > 0]
                MESSAGE[var_name] = {
                    'type': var_type,
                    'comment': COMMENT_BUFFER.strip()
                }

                COMMENT_BUFFER = ''

        return MESSAGE

    def rosmsg_definition_to_table(fields):
        '''
        Convert a definition to a table
        '''

        table = '| Field | Type | Description |\n|:-:|:-:|---|\n'
        for name, definition in fields.items():
            var_type = definition['type']
            comment = '<br/>'.join(definition['comment'].split('\n'))
            table += f'| {name} | {var_type} | {comment} |\n'
        
        return table

    # Generate message tables
    MESSAGES = join(pkg_dir, 'msg')
    if exists(MESSAGES):
        messages = os.listdir(MESSAGES)

        data['messages'] = ''

        for message in messages:
            if not message.endswith('.msg'):
                continue
                
            name = message[:-4]
            data['messages'] += f'#### `{name}`\n\n'

            with open(join(pkg_dir, 'msg', message), 'r') as f:
                fields = parse_rosmsg_definition(f.read())
                data['messages'] += rosmsg_definition_to_table(fields)

            data['messages'] += '\n'

    # Generate service tables
    SERVICES = join(pkg_dir, 'srv')
    if exists(SERVICES):
        services = os.listdir(SERVICES)

        data['services'] = ''

        for service in services:
            if not service.endswith('.srv'):
                continue
                
            name = service[:-4]
            data['services'] += f'#### `{name}`\n\n'

            with open(join(pkg_dir, 'srv', service), 'r') as f:
                request, response = f.read().split('---')

                data['services'] += 'Request\n\n'
                request_fields = parse_rosmsg_definition(request)
                data['services'] += rosmsg_definition_to_table(request_fields)

                data['services'] += '\nResponse\n\n'
                response_fields = parse_rosmsg_definition(response)
                data['services'] += rosmsg_definition_to_table(response_fields)

            data['services'] += '\n'

    # Generate action tables
    ACTIONS = join(pkg_dir, 'action')
    if exists(ACTIONS):
        actions = os.listdir(ACTIONS)

        data['actions'] = ''

        for action in actions:
            if not action.endswith('.action'):
                continue
                
            name = action[:-7]
            data['actions'] += f'#### `{name}`\n\n'

            with open(join(pkg_dir, 'action', action), 'r') as f:
                goal, result, feedback = f.read().split('---')

                data['actions'] += 'Goal\n\n'
                goal_fields = parse_rosmsg_definition(goal)
                data['actions'] += rosmsg_definition_to_table(goal_fields)

                data['actions'] += '\nResult\n\n'
                result_fields = parse_rosmsg_definition(result)
                data['actions'] += rosmsg_definition_to_table(result_fields)

                data['actions'] += '\nFeedback\n\n'
                feedback_fields = parse_rosmsg_definition(feedback)
                data['actions'] += rosmsg_definition_to_table(feedback_fields)

            data['actions'] += '\n'

    # Generate the README
    README = '''# {package_name}

{description}

{maintainers}
{authors}
## Prerequisites

{dependencies}

{prerequisites}

## Usage

{usage}

## Example

{example}

## Technical Overview

{technical}

## ROS Definitions

### Launch Files

{launch_files}
### Messages

{messages}
### Services

{services}
### Actions

{actions}'''.format(**data)

    with open(join(pkg_dir, 'README.md'), 'w') as f:
        f.write(README)
