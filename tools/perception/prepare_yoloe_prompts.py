#!/usr/bin/env python3
"""Prepare checkpoint-bound text embeddings offline, before measured inference."""
import argparse
import hashlib
import json
from pathlib import Path
import os
import numpy as np
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--encoder-directory',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.weights=args.weights.resolve()
    args.config=args.config.resolve()
    args.output=args.output.resolve()
    from ultralytics import YOLOE
    import torch
    torch.set_num_threads(2)
    names=yaml.safe_load(args.config.read_text())['allowed_names']
    os.environ['YOLO_AUTOINSTALL']='false'
    os.chdir(args.encoder_directory)
    model=YOLOE(str(args.weights))
    embeddings=model.get_text_pe(names).cpu().numpy()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    sha=hashlib.sha256(args.weights.read_bytes()).hexdigest()
    np.savez_compressed(args.output, names=np.array(names), embeddings=embeddings,
                        weights_sha256=np.array(sha))
    print(json.dumps(dict(names=len(names),shape=list(embeddings.shape),
                         weights_sha256=sha,embeddings_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest())))


if __name__=='__main__':main()
