#!/usr/bin/env python3
"""Validate local LeRobot parquet, metadata, RGB videos and metric depth sidecars."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
import pandas as pd

def validate(root):
    info=json.loads((root/'meta/info.json').read_text());fps=info['fps']
    data=pd.concat([pd.read_parquet(p) for p in sorted((root/'data').rglob('*.parquet'))],ignore_index=True)
    n=len(data);assert n==info['total_frames'] and n>0
    assert np.array_equal(data.frame_index,np.arange(n))
    assert np.allclose(data.timestamp,np.arange(n)/fps,atol=1e-5)
    for key,feat in info['features'].items():
        if feat['dtype']=='video':continue
        assert key in data,key
        a=np.stack(data[key]);assert np.isfinite(a).all(),key
        if len(feat['shape'])==1 and feat['shape'][0]>1:
            assert a.shape==(n,feat['shape'][0]),(key,a.shape)
            assert len(feat['names'])==feat['shape'][0],key
    videos={}
    for p in (root/'videos').rglob('*.mp4'):
        cap=cv2.VideoCapture(str(p));assert cap.isOpened(),p
        assert abs(cap.get(cv2.CAP_PROP_FPS)-fps)<.01,p
        count=0
        while True:
            ok,frame=cap.read()
            if not ok:break
            assert frame.shape==(480,640,3),frame.shape
            count+=1
        cap.release();assert count==n,(p,count,n);videos[str(p.relative_to(root))]=count
    assert len(videos)==3,videos
    if info['codebase_version']=='v3.0':
        episodes=pd.read_parquet(root/'meta/episodes/chunk-000/file-000.parquet')
        assert episodes.iloc[0]['dataset_to_index']==n
        for key,feat in info['features'].items():
            if feat['dtype']=='video':
                assert episodes.iloc[0][f'videos/{key}/from_timestamp']==0
                assert abs(episodes.iloc[0][f'videos/{key}/to_timestamp']-n/fps)<1e-6
    index=pd.read_parquet(root/'depth/index.parquet');depth={}
    for name,rows in index.groupby('camera_name'):
        for rel in rows.relative_path:
            p=root/rel;assert p.is_file(),p
        for rel in (rows.iloc[0].relative_path,rows.iloc[-1].relative_path):
            a=np.load(root/rel);assert a.shape==(480,640) and a.dtype==np.float32
        depth[name]=len(rows)
    assert len(depth)==3
    return {'version':info['codebase_version'],'frames':n,'fps':fps,'duration_s':n/fps,
            'rgb_videos':videos,'depth_sidecar_frames':depth,'passed':True,
            'scope':'Local schema/content/decode validation; official LeRobot training loader not executed.'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('roots',nargs='+',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    results=[validate(x) for x in a.roots]
    text=json.dumps(results,indent=2);print(text)
    if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text+'\n')
if __name__=='__main__':main()
