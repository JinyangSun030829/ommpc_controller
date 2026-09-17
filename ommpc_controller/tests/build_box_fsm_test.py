"""Use each controller target's existing compiler/link flags for isolated tests."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("build",type=Path)
    parser.add_argument("source",type=Path)
    parser.add_argument("target")
    parser.add_argument("test",type=Path)
    parser.add_argument("output",type=Path)
    parser.add_argument("--tracking",action="store_true")
    args=parser.parse_args()
    entries=json.loads((args.build/"compile_commands.json").read_text())
    entry=next(item for item in entries if Path(item["file"])==args.source)
    flags=shlex.split(entry["command"])
    object_path=args.output.with_suffix(".o")
    flags[flags.index("-o")+1]=str(object_path)
    flags[flags.index(str(args.source))]=str(args.test)
    flags.insert(1,'-DFSM_BOX_SOURCE="%s"'%args.source)
    if args.tracking:
        flags.insert(1,"-DFSM_BOX_TRACKING")
    subprocess.run(flags,cwd=entry["directory"],check=True)
    link_file=args.build/("src/ommpc_controller/ommpc_controller/CMakeFiles/%s.dir/link.txt"%args.target)
    link=shlex.split(link_file.read_text())
    link[link.index("-o")+1]=str(args.output)
    object_index=next(i for i,flag in enumerate(link) if flag.endswith("/src/"+args.source.name+".o"))
    link[object_index]=str(object_path)
    subprocess.run(link,cwd=link_file.parent.parent.parent,check=True)


if __name__=="__main__":
    main()
