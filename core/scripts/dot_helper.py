import os
import re
import sys

def filter_graph_lines(lines: list[str], module_name: str) -> list[str]:
    new_lines = []
    found_module = False
    for line in lines:
        if line.strip() == "digraph \"" + module_name + "\" {":
            assert not found_module
            found_module = True
        if found_module:
            new_lines.append(line)
        if line.strip() == "}":
            found_module = False
    return new_lines

def fix_param_labels(lines: list[str]) -> list[str]:
    param_label_pattern = re.compile(r'^(.*\|)(\w+)\\n\$paramod&#9586;(\w+)&#9586;(\w+)=s\d+\'([01]+)(\|.*)$')
    new_lines = []
    for line in lines:
        label_match = param_label_pattern.match(line)
        if label_match:
            prefix, module_name, module_type, module_param_name, module_param_value, suffix = label_match.groups()
            module_param_value = int(module_param_value, 2)
            module_type = f"{module_type}(.{module_param_name}({module_param_value}))"
            line = f"{prefix}{module_name}\\n{module_type}{suffix}"
        new_lines.append(line)
    return new_lines

def duplicate_signal(lines: list[str], target_signal: str) -> list[str]:
    node_line = None
    node_attrs = None
    edge_pattern = None
    node_pattern = re.compile(r'^\s*(\S*)\s*\[(.*label="%s".*)\];' % re.escape(target_signal))
    for i, line in enumerate(lines):
        m = node_pattern.match(line)
        if m:
            node_line = i
            node_attrs = m.group(2)
            edge_pattern = re.compile(r'^\s*%s:e\s*->\s*(.*);' % re.escape(m.group(1)))
            break

    if node_line is None:
        return lines

    new_lines = []
    clone_count = 0

    for line in lines:
        m = edge_pattern.match(line) if edge_pattern else None
        if not m:
            new_lines.append(line)
            continue

        if node_attrs:
            clone_count += 1
            clone_name = f"{target_signal}_clone_{clone_count}"
            new_line = f"{clone_name} -> {m.group(1)};"
            new_lines.append(new_line)

    clone_nodes_defs = []
    for i in range(1, clone_count + 1):
        name = f"{target_signal}_clone_{i}"
        clone_nodes_defs.append(f'{name} [{node_attrs}];')

    return new_lines[:node_line+1] + clone_nodes_defs + new_lines[node_line+1:]

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python dot_helper.py file.dot")
        sys.exit(-1)
    dot_path = sys.argv[1]

    with open(dot_path, 'r') as f:
        dot = f.read()
    lines = dot.splitlines()

    # Keep just the graph for the top-level module
    module_name = os.path.splitext(os.path.basename(dot_path))[0]
    lines = filter_graph_lines(lines, module_name)
    lines = fix_param_labels(lines)
    lines = duplicate_signal(lines, "clk")
    lines = duplicate_signal(lines, "reset_n")
    lines = duplicate_signal(lines, "enable_n")

    with open(dot_path, 'w') as f:
        f.write("\n".join(lines) + "\n")
