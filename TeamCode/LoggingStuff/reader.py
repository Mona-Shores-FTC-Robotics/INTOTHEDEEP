import enum
import struct
import typing as T
from dataclasses import dataclass, field
from collections import defaultdict
import csv
import sys
import argparse
import os

MessageSchema = T.Union['StructSchema', 'PrimitiveSchema']

@dataclass(frozen=True)
class StructSchema:
    fields: T.Dict[str, MessageSchema]

class PrimitiveSchema(enum.Enum):
    INT = 'int'
    LONG = 'long'
    DOUBLE = 'double'
    STRING = 'string'
    BOOLEAN = 'boolean'

@dataclass(frozen=True)
class EnumSchema:
    constants: T.List[str]

def read_file(f):
    def read(n):
        assert n > 0
        # assume this reads exactly n bytes or we reach EOF
        buf = f.read(n)
        if len(buf) == 0:
            raise EOFError
        if len(buf) < n:
            raise IOError('Short read')
        return buf

    def read_string():
        nbytes, = struct.unpack_from('!i', read(4))
        name, = struct.unpack_from(f'!{nbytes}s', read(nbytes))
        return name.decode('utf-8')

    def read_schema():
        schema_type, = struct.unpack_from('!i', read(4))
        # struct schema
        if schema_type == 0:
            nfields, = struct.unpack_from('!i', read(4))
            fields = {}
            for _ in range(nfields):
                name = read_string()
                fields[name] = read_schema()
            return StructSchema(fields)
        # primitive schema
        elif schema_type == 1:
            return PrimitiveSchema.INT
        elif schema_type == 2:
            return PrimitiveSchema.LONG
        elif schema_type == 3:
            return PrimitiveSchema.DOUBLE
        elif schema_type == 4:
            return PrimitiveSchema.STRING
        elif schema_type == 5:
            return PrimitiveSchema.BOOLEAN
        # enum schema
        elif schema_type == 6:
            nconstants, = struct.unpack_from('!i', read(4))
            constants = []
            for _ in range(nconstants):
                constants.append(read_string())
            return EnumSchema(constants)
        else:
            raise ValueError(f'Unknown schema type: {schema_type}')

    def read_msg(schema):
        if isinstance(schema, StructSchema):
            msg = {}
            for name, field_schema in schema.fields.items():
                msg[name] = read_msg(field_schema)
            return msg
        elif isinstance(schema, PrimitiveSchema):
            if schema == PrimitiveSchema.INT:
                return struct.unpack_from('!i', read(4))[0]
            elif schema == PrimitiveSchema.LONG:
                return struct.unpack_from('!q', read(8))[0]
            elif schema == PrimitiveSchema.DOUBLE:
                return struct.unpack_from('!d', read(8))[0]
            elif schema == PrimitiveSchema.STRING:
                return read_string()
            elif schema == PrimitiveSchema.BOOLEAN:
                return struct.unpack_from('!?', read(1))[0]
            else:
                raise ValueError(f'Unknown primitive schema: {schema}')
        elif isinstance(schema, EnumSchema):
            ordinal, = struct.unpack_from('!i', read(4))
            return schema.constants[ordinal]
        else:
            raise ValueError(f'Unknown schema: {schema}')

    magic, version = struct.unpack_from('!2sh', read(4))
    assert magic == b'RR'
    assert version == 0

    channels = []
    schemas = {}
    messages = defaultdict(list)

    while True:
        try:
            entry_type, = struct.unpack_from('!i', read(4))
            if entry_type == 0:
                # channel definition
                ch = read_string()
                schemas[ch] = read_schema()
                channels.append(ch)
            elif entry_type == 1:
                # message
                ch_index, = struct.unpack_from('!i', read(4))
                ch = channels[ch_index]
                messages[ch].append(read_msg(schemas[ch]))
            else:
                raise ValueError(f"Unknown entry type: {entry_type}")
        except EOFError:
            break

    return schemas, dict(messages)

def flatten_dict(d, parent_key='', sep='_'):
    """Flatten a nested dictionary, appending keys with a separator."""
    items = []
    for k, v in d.items():
        if k == 'timestamp':
            new_key = k
        else:
            new_key = f"{parent_key}{sep}{k}" if parent_key else k

        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)

def collect_csv_data(schemas, messages):
    all_fields = set()
    csv_data = []

    for ch, schema in schemas.items():
        if isinstance(schema, StructSchema) and 'timestamp' in schema.fields:
            for msg in messages[ch]:
                if 'timestamp' in msg:
                    flattened_msg = flatten_dict(msg, ch)
                    csv_data.append(flattened_msg)
                    all_fields.update(flattened_msg.keys())

    all_fields = sorted(all_fields, key=lambda x: (x != 'timestamp', x))
    csv_data.sort(key=lambda msg: msg.get('timestamp', 0))

    return all_fields, csv_data

def write_csv(output_path, fields, data):
    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fields)
        writer.writeheader()
        for row in data:
            writer.writerow(row)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filepath', help="Path to the log file")
    parser.add_argument('--csv', nargs='?', const=True, help="Output CSV file path (optional). If omitted, uses the input filename with .csv extension")
    args = parser.parse_args()

    with open(args.filepath, 'rb') as f:
        schemas, messages = read_file(f)

        if args.csv:
            output_csv = args.csv if isinstance(args.csv, str) else os.path.splitext(args.filepath)[0] + '.csv'
            fields, csv_data = collect_csv_data(schemas, messages)
            write_csv(output_csv, fields, csv_data)
        else:
            for ch, schema in schemas.items():
                print(f'Channel: {ch}  ({len(messages[ch])} messages)\n  {schema}')

