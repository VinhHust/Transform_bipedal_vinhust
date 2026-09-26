#!/usr/bin/env python3
# DO NHIP SERVER: hoi tung chan 300 lan, cach 50ms. Chi gui "feedback" - KHONG dong motor.
#   RTT           = laptop hoi -> Pi tra loi (mang + thread ZMQ tren Pi)
#   t_sample step = tem mau IMU nhay bao nhieu ms giua 2 lan hoi (= imu_loop tren Pi). Binh thuong ~20ms.
#   Timeout >3s thi in ra, tao lai socket va di tiep (REQ sau timeout phai tao lai).
#   python3 measure_server_timing.py
import zmq, time, numpy as np
ctx=zmq.Context()
def mk(h,p):
    s=ctx.socket(zmq.REQ); s.setsockopt(zmq.LINGER,0); s.setsockopt(zmq.RCVTIMEO,3000); s.connect(f"tcp://{h}:{p}"); return s
cfg={"RIGHT":("mobile1.local",5555),"LEFT":("mobile2.local",5556)}
S={n:mk(*c) for n,c in cfg.items()}
R={n:[] for n in S}; T={n:[] for n in S}; last={n:None for n in S}; TO={n:0 for n in S}
t0=time.monotonic()
for i in range(300):
    for n in list(S):
        s=S[n]; a=time.monotonic()
        try:
            s.send_json({"type":"feedback"}); r=s.recv_json()
        except zmq.Again:
            TO[n]+=1; print(f"  {n}: TIMEOUT >3s tai lan {i} (t={time.monotonic()-t0:.1f}s)"); s.close(); S[n]=mk(*cfg[n]); continue
        R[n].append((time.monotonic()-a)*1000)
        if last[n] is not None: T[n].append((r["t_sample"]-last[n])*1000)
        last[n]=r["t_sample"]
    time.sleep(0.05)
for n in S:
    r=np.array(R[n]); t=np.array(T[n])
    print(f"{n}: timeout={TO[n]} | RTT p50/p90/max={np.percentile(r,50):.1f}/{np.percentile(r,90):.1f}/{r.max():.1f}ms >100ms:{(r>100).sum()} | t_sample step p50/max={np.percentile(t,50):.0f}/{t.max():.0f}ms >=250:{(t>=250).sum()}/{len(t)}")
