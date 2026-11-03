"""Store ROS message definitions with raw CDR, including nested types."""

from functools import lru_cache
from pathlib import Path


@lru_cache(maxsize=256)
def message_schema(message_type: str) -> bytes:
    from rosidl_runtime_py.get_interfaces import get_interface_path
    from rosidl_adapter.parser import parse_message_string

    definitions = {}

    def visit(name):
        if name in definitions:
            return
        package, _, typename = name.split("/")
        path = Path(get_interface_path(name))
        if path.suffix != ".msg":
            raise ValueError(f"Teach recording requires installed .msg definitions for {name}")
        text = path.read_text()
        definitions[name] = text
        parsed = parse_message_string(package, typename, text)
        for item in parsed.fields:
            if item.type.pkg_name:
                visit(f"{item.type.pkg_name}/msg/{item.type.type}")

    visit(message_type)
    root = definitions.pop(message_type).rstrip() + "\n"
    for name, text in definitions.items():
        root += "\n" + "=" * 80 + "\nMSG: " + name + "\n" + text.rstrip() + "\n"
    return root.encode()
