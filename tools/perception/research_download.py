#!/usr/bin/env python3
"""Download public research assets with process-local DNS fallback and SHA log.

Does not change host/container DNS settings or disable TLS verification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import socket

import requests


def enable_dns_fallback():
    original=socket.getaddrinfo
    cache={}
    def lookup(host,port,*args,**kwargs):
        if host in cache:
            return original(cache[host],port,*args,**kwargs)
        try:
            return original(host,port,*args,**kwargs)
        except socket.gaierror:
            response=requests.get('https://1.1.1.1/dns-query',params=dict(name=host,type='A'),
                                  headers={'Accept':'application/dns-json'},timeout=15)
            response.raise_for_status()
            answers=[a['data'] for a in response.json().get('Answer',[]) if a['type']==1]
            if not answers: raise
            cache[host]=answers[0]
            return original(answers[0],port,*args,**kwargs)
    socket.getaddrinfo=lookup


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('url');p.add_argument('output',type=Path)
    p.add_argument('--dns-fallback',action='store_true')
    a=p.parse_args()
    if a.dns_fallback: enable_dns_fallback()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    response=requests.get(a.url,stream=True,timeout=(30,90));response.raise_for_status()
    digest=hashlib.sha256();size=0
    with a.output.open('wb') as stream:
        for chunk in response.iter_content(1024*1024):
            stream.write(chunk);digest.update(chunk);size+=len(chunk)
    record=dict(url=a.url,bytes=size,sha256=digest.hexdigest())
    a.output.with_suffix(a.output.suffix+'.source.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record),flush=True)


if __name__=='__main__':main()
