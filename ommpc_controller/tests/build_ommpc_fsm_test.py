"""Build the isolated OMMPC state test with the existing catkin target's flags."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("package", type=Path)
    parser.add_argument("build", type=Path)
    parser.add_argument("test_source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    source = args.package / "src/ommpc_example.cpp"
    entries = json.loads((args.build / "compile_commands.json").read_text())
    entry = next(item for item in entries if Path(item["file"]) == source)
    flags = shlex.split(entry["command"])
    object_path = args.output.with_suffix(".o")
    flags[flags.index("-o")+1] = str(object_path)
    flags[flags.index(str(source))] = str(args.test_source)
    flags.insert(1, '-DOMMPC_FSM_SOURCE="%s"' % source)
    subprocess.run(flags, cwd=entry["directory"], check=True)
    link_file = args.build / "src/ommpc_controller/ommpc_controller/CMakeFiles/ommpc_controller_example_node.dir/link.txt"
    link = shlex.split(link_file.read_text())
    link[link.index("-o")+1] = str(args.output)
    object_index = next(i for i, flag in enumerate(link) if flag.endswith("/src/ommpc_example.cpp.o"))
    link[object_index] = str(object_path)
    subprocess.run(link, cwd=link_file.parent.parent.parent, check=True)


if __name__ == "__main__":
    main()
