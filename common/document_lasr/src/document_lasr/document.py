import os
from os.path import join, abspath
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
        MAINTAINERS += f'\n- [{name}]({email})'

    data['maintainers'] = MAINTAINERS

    # Generate authors list
    AUTHORS = ''

    if len(data['authors']) > 0:
        AUTHORS = '''\nThe following people have contributed to this package:'''

        for author in data['authors']:
            name = author['name']
            email = author['email']
            AUTHORS += f'\n- [{name}]({email})'
        
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
        PATH = os.path.join(pkg_dir, f'doc/{file}.md')
        if os.path.exists(PATH):
            with open(PATH, 'r') as f:
                data[key] = f.read().strip()

    # Generate documentation for ROS-specific components
    data['messages'] = 'This package has no messages.\n'
    data['services'] = 'This package has no services.\n'
    data['actions'] = 'This package has no actions.\n'

    def parse_message_definition(input):
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
                var_type, var_name = definition.strip().split(' ')
                MESSAGE[var_name] = {
                    'type': var_type,
                    'comment': COMMENT_BUFFER.strip()
                }

                COMMENT_BUFFER = ''

        return MESSAGE

    def definition_to_table(fields):
        '''
        Convert a definition to a table
        '''

        table = '| Field | Type | Description |\n|:-:|:-:|---|\n'
        for name, definition in fields.items():
            var_type = definition['type']
            comment = '<br>'.join(definition['comment'].split('\n'))
            table += f'| {name} | {var_type} | {comment} |\n'
        
        return table

    # Generate message tables
    MESSAGES = os.path.join(pkg_dir, 'msg')
    if os.path.exists(MESSAGES):
        messages = os.listdir(MESSAGES)

        data['messages'] = ''

        for message in messages:
            if not message.endswith('.msg'):
                continue
                
            name = message[:-4]
            data['messages'] += f'#### `{name}`\n\n'

            with open(os.path.join(pkg_dir, 'msg', message), 'r') as f:
                fields = parse_message_definition(f.read())
                data['messages'] += definition_to_table(fields)

            data['messages'] += '\n'

    # Generate service tables
    SERVICES = os.path.join(pkg_dir, 'srv')
    if os.path.exists(SERVICES):
        services = os.listdir(SERVICES)

        data['services'] = ''

        for service in services:
            if not service.endswith('.srv'):
                continue
                
            name = service[:-4]
            data['services'] += f'#### `{name}`\n\n'

            with open(os.path.join(pkg_dir, 'srv', service), 'r') as f:
                request, response = f.read().split('---')

                data['services'] += 'Request\n\n'
                request_fields = parse_message_definition(request)
                data['services'] += definition_to_table(request_fields)

                data['services'] += '\nResponse\n\n'
                response_fields = parse_message_definition(response)
                data['services'] += definition_to_table(response_fields)

            data['services'] += '\n'

    # Generate action tables
    ACTIONS = os.path.join(pkg_dir, 'action')
    if os.path.exists(ACTIONS):
        actions = os.listdir(ACTIONS)

        data['actions'] = ''

        for action in actions:
            if not action.endswith('.action'):
                continue
                
            name = action[:-4]
            data['actions'] += f'#### `{name}`\n\n'

            with open(os.path.join(pkg_dir, 'action', action), 'r') as f:
                goal, result, feedback = f.read().split('---')

                data['actions'] += 'Goal\n\n'
                goal_fields = parse_message_definition(goal)
                data['actions'] += definition_to_table(request_fields)

                data['actions'] += '\nResult\n\n'
                result_fields = parse_message_definition(result)
                data['actions'] += definition_to_table(result_fields)

                data['actions'] += '\nFeedback\n\n'
                feedback_fields = parse_message_definition(result)
                data['actions'] += definition_to_table(feedback_fields)

            data['services'] += '\n'

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

### Messages

{messages}
### Services

{services}
### Actions

{actions}'''.format(**data)

    with open(os.path.join(pkg_dir, 'README.md'), 'w') as f:
        f.write(README)
