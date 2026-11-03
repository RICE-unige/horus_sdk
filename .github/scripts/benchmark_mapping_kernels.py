#!/usr/bin/env python3
"""CPU microbenchmark matching native inputs; not a ROS/headset capacity claim."""
import json
import time
import numpy as np
from horus.mapping.costmap import CostmapState, GridGeometry
from horus.mapping.mesh_protocol import BlockGeometry, encode_live_payload


def main():
    grid = CostmapState()
    grid.full(GridGeometry("map",1024,1024,0.05,(0,0,0,0,0,0,1)), np.zeros(1024*1024,dtype=np.int8),0)
    patch = (np.arange(128*128,dtype=np.uint32)%256).astype(np.uint8)
    geometry = BlockGeometry(tuple((i%256/16,i//256/16,1.0) for i in range(30000)),
                             ((255,128,0,255),)*30000,tuple(range(30000)))
    stamp = 0
    for round_number in range(6):
        start = time.perf_counter()
        for _ in range(1000):
            stamp += 1
            grid.patch("map",64,64,128,128,patch,stamp,raw=True)
        patch_us = (time.perf_counter()-start)*1e6/1000
        start = time.perf_counter()
        for _ in range(100):
            encoded = encode_live_payload(chunk_id=7,page_key=(0,0,0),epoch=1,revision=1,
                operation=0,geometry=geometry,max_payload_bytes=1048576)
        encode_us = (time.perf_counter()-start)*1e6/100
        if round_number:
            print(json.dumps(dict(language="python",round=round_number,patch_us=patch_us,
                encode_us=encode_us,wire_bytes=len(encoded),checksum=sum(encoded))))


if __name__ == "__main__":
    main()
