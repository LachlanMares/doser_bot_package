#!/usr/bin/env python3

from pathlib import Path


def parse_definitions_file(definitions_filepath: Path):
    def_dict = {}

    if definitions_filepath.is_file():
        definitions_file = open(definitions_filepath, "r")

        if definitions_file.mode == 'r':
            definitions = definitions_file.read().splitlines()

        definitions_file.close()
        dict_key = ''

        for line in [lines.split() for lines in definitions if len(lines.split()) > 0]:
            if line[0] == '//' and len(line) == 2:
                dict_key = line[1]
                def_dict[dict_key] = {}
            elif line[0] == '#define' and len(line) == 3:
                if "motor_" in dict_key:
                    motor_key = line[1].replace(f"{dict_key.upper()}_", '').lower()
                    if line[2].startswith('0x'):
                        def_dict[dict_key][motor_key] = int(line[2][2:], 16)
                    else:
                        def_dict[dict_key][motor_key] = int(line[2])

                elif "load_cell_" in dict_key:
                    motor_key = line[1].replace(f"{dict_key.upper()}_", '').lower()
                    if line[2].startswith('0x'):
                        def_dict[dict_key][motor_key] = int(line[2][2:], 16)
                    else:
                        def_dict[dict_key][motor_key] = int(line[2])

                elif dict_key == 'status_message_bits':
                    status_key = line[1].replace(f"STATUS_", '').replace(f"_BIT", '').lower()
                    if line[2].startswith('0x'):
                        def_dict[dict_key][status_key] = int(line[2][2:], 16)
                    else:
                        def_dict[dict_key][status_key] = int(line[2])

                else:
                    if line[2].startswith('0x'):
                        def_dict[dict_key][line[1]] = int(line[2][2:], 16)
                    else:
                        def_dict[dict_key][line[1]] = int(line[2])
    else:
        raise Exception("Filepath is not a file")

    return def_dict
