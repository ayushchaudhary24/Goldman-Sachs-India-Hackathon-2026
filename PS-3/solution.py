import sys
import json


PRIMITIVE_NAMES = {
    str: "string",
    int: "number",
    float: "number",
    bool: "boolean",
    type(None): "null",
}


class InterfaceNode:
    __slots__ = ("base_name", "name", "fields", "object_count")

    def __init__(self, base_name):
        self.base_name = base_name
        self.name = None
        self.fields = {}
        self.object_count = 0


class FieldInfo:
    __slots__ = (
        "present_count",
        "primitive_types",
        "object_node",
        "has_array",
        "array_primitive_types",
        "array_object_node",
    )

    def __init__(self):
        self.present_count = 0
        self.primitive_types = set()
        self.object_node = None
        self.has_array = False
        self.array_primitive_types = set()
        self.array_object_node = None


def interface_base_from_key(key):
    return key[0].upper() + key[1:]


def primitive_type(value):
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, (int, float)):
        return "number"
    if isinstance(value, str):
        return "string"
    return None


def get_field(node, key):
    field = node.fields.get(key)
    if field is None:
        field = FieldInfo()
        node.fields[key] = field
    return field


def merge_object_into_node(node, obj):
    node.object_count += 1

    for key, value in obj.items():
        field = get_field(node, key)
        field.present_count += 1
        merge_value_into_field(field, key, value)


def merge_value_into_field(field, key, value):
    if isinstance(value, dict):
        if field.object_node is None:
            field.object_node = InterfaceNode(interface_base_from_key(key))
        merge_object_into_node(field.object_node, value)
        return

    if isinstance(value, list):
        field.has_array = True
        for item in value:
            if isinstance(item, dict):
                if field.array_object_node is None:
                    field.array_object_node = InterfaceNode(interface_base_from_key(key))
                merge_object_into_node(field.array_object_node, item)
            else:
                item_type = primitive_type(item)
                if item_type is not None:
                    field.array_primitive_types.add(item_type)
        return

    value_type = primitive_type(value)
    if value_type is not None:
        field.primitive_types.add(value_type)


def assign_interface_names(root):
    root.name = root.base_name
    used_names = {root.name}
    interfaces = [root]

    def unique_name(base):
        if base not in used_names:
            used_names.add(base)
            return base

        suffix = 2
        while True:
            candidate = base + str(suffix)
            if candidate not in used_names:
                used_names.add(candidate)
                return candidate
            suffix += 1

    def visit(node):
        for key in sorted(node.fields):
            field = node.fields[key]
            children = []
            if field.object_node is not None:
                children.append(field.object_node)
            if field.array_object_node is not None:
                children.append(field.array_object_node)

            for child in children:
                child.name = unique_name(child.base_name)
                interfaces.append(child)
                visit(child)

    visit(root)
    return interfaces


def array_type_string(field):
    element_types = set(field.array_primitive_types)

    if field.array_object_node is not None:
        element_types.add(field.array_object_node.name)

    if not element_types:
        return "unknown[]"

    ordered = sorted(element_types)
    if len(ordered) == 1:
        return ordered[0] + "[]"

    return "(" + " | ".join(ordered) + ")[]"


def field_type_string(field):
    type_parts = set(field.primitive_types)

    if field.object_node is not None:
        type_parts.add(field.object_node.name)

    if field.has_array:
        type_parts.add(array_type_string(field))

    return " | ".join(sorted(type_parts))


def render_interface(node):
    if not node.fields:
        return f"export interface {node.name} {{}}"

    lines = [f"export interface {node.name} {{"]
    for key in sorted(node.fields):
        field = node.fields[key]
        optional = "?" if field.present_count < node.object_count else ""
        lines.append(f"  {key}{optional}: {field_type_string(field)};")
    lines.append("}")
    return "\n".join(lines)


def solve_case(root_name, json_text):
    data = json.loads(json_text)
    root = InterfaceNode(root_name)

    for obj in data:
        merge_object_into_node(root, obj)

    interfaces = assign_interface_names(root)
    interfaces.sort(key=lambda item: item.name)
    return "\n\n".join(render_interface(node) for node in interfaces)


def main():
    lines = sys.stdin.read().splitlines()
    if not lines:
        return

    test_count = int(lines[0])
    output_blocks = []

    index = 1
    for _ in range(test_count):
        root_name = lines[index]
        json_text = lines[index + 1]
        index += 2
        output_blocks.append(solve_case(root_name, json_text))

    sys.stdout.write("\n---\n".join(output_blocks) + "\n")


if __name__ == "__main__":
    main()
