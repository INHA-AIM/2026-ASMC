#!/usr/bin/env python3
"""Build a review helper against the existing actual Main test library."""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import time
import xmlrpc.client


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    master=os.environ.get('ROS_MASTER_URI','')
    if master!='http://127.0.0.1:11314':
        raise ValueError('review helper requires its own localhost master 11314')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',11314))==0:
            raise ValueError('master already present; leave existing processes untouched')
    target=args.build/'Main/CMakeFiles/test_frame_boundary_ros.dir'
    flags={}
    for line in (target/'flags.make').read_text().splitlines():
        if ' = ' in line:
            key,value=line.split(' = ',1);flags[key]=shlex.split(value)
    source=Path(__file__).with_suffix('.cpp')
    obj=args.output/'helper.o';exe=args.output/'helper'
    compile_args=['/usr/bin/c++']+flags['CXX_FLAGS']+flags['CXX_DEFINES']+flags['CXX_INCLUDES']+['-c',str(source),'-o',str(obj)]
    link=shlex.split((target/'link.txt').read_text())
    original='CMakeFiles/test_frame_boundary_ros.dir/test/test_frame_boundary_ros.cpp.o'
    if link.count(original)!=1:raise ValueError('unexpected Main test link recipe')
    link[link.index(original)]=str(obj);link[link.index('-o')+1]=str(exe)
    (args.output/'build-commands.json').write_text(json.dumps([compile_args,link],indent=2)+'\n')
    with (args.output/'build.log').open('w') as log:
        subprocess.run(compile_args,cwd=args.build/'Main',stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run(link,cwd=args.build/'Main',stdout=log,stderr=subprocess.STDOUT,check=True)
    with (args.output/'roscore.log').open('w') as log:
        core=subprocess.Popen(['roscore','-p','11314'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            for _ in range(100):
                if core.poll() is not None:raise RuntimeError('own master exited')
                try:
                    if xmlrpc.client.ServerProxy(master).getPid('/review')[0]==1:break
                except (OSError,xmlrpc.client.Error):time.sleep(.1)
            else:raise RuntimeError('own master readiness timeout')
            for mode in ('legacy','common'):
                with (args.output/(mode+'.log')).open('w') as runlog:
                    subprocess.run([str(exe),mode,str(args.output/(mode+'.json'))],check=True,
                                   stdout=runlog,stderr=subprocess.STDOUT)
            a=json.loads((args.output/'legacy.json').read_text());b=json.loads((args.output/'common.json').read_text())
            assert a['input']==b['input'] and len(a['input'])==24
            assert not a['output'] and b['output']==b['input']
        finally:
            if core.poll() is None:
                os.killpg(core.pid,signal.SIGINT)
                try:core.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(core.pid,signal.SIGTERM);core.wait(timeout=5)


if __name__=='__main__':main()
