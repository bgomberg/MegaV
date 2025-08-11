import sys
import re

def main(lines: list[str], target_signal: str) -> list[str]:
    node_pattern = re.compile(r'^\s*(\S*)\s*\[(.*label="%s".*)\];' % re.escape(target_signal))

    node_line = None
    for i, line in enumerate(lines):
        if node_pattern.match(line):
            node_line = i
            break

    if node_line is None:
        return lines

    node_name = node_pattern.match(lines[node_line]).group(1)
    node_attrs = node_pattern.match(lines[node_line]).group(2)
    edge_pattern = re.compile(r'^\s*%s:e\s*->\s*(.*);' % re.escape(node_name))

    new_lines = []
    clone_count = 0

    for line in lines:
        m = edge_pattern.match(line)
        if not m:
            new_lines.append(line)
            continue

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

    lines = main(lines, "clk")
    lines = main(lines, "reset_n")

    with open(dot_path, 'w') as f:
        f.write("\n".join(lines) + "\n")
