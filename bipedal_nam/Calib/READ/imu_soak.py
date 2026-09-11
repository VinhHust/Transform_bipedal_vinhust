# SOAK / ANGLE-CHECK 1 IMU - chan doan mot IMU, DUNG TRUOC MADGWICK.
# Goc tinh truc tiep tu trong luc bang atan2 -> tach lo cam bien/calib khoi loi fusion.
# Vai tro, 7 tang xu ly, y nghia tung cot CSV: xem docs/analysis.md
#
#   python3 imu_soak.py --angles --dlpf low    # do sai so roll/pitch theo goc tu dat
#   python3 imu_soak.py --minutes 480 --full   # chay dai bat loi ngau nhien
#
# Ra logs/<mode>_<timestamp>/: all.csv events.csv summary.csv segments.csv report.txt

import argparse
import csv
import json
import math
import select
import sys
import time
from collections import deque, Counter
from datetime import datetime
from pathlib import Path

import numpy as np
import qwiic_icm20948 as Q

HERE = Path(__file__).parent
G = 9.80665
D2R = math.pi / 180.0

# ------------------------------------------------------------------ NGUONG
TH = {
    "norm": 0.40,      # |a| lech khoi 9.81 bao nhieu m/s^2 thi bao dong
    "jump": 1200,      # |delta raw accel| 1 mau (LSB)
    "gjump": 1500,     # |delta raw gyro| 1 mau (LSB)
    "gyro": 15.0,      # |omega| (deg/s) khi dang dung yen
    "angle": 3.0,      # nhay goc 1 mau (deg) - tinh tren goc CHUA loc
    "dt": 4.0,         # dt gap bao nhieu lan median
    "stale": 4,        # bao nhieu mau raw giong het nhau lien tiep
}
CTX = 40               # so mau ngu canh truoc/sau moi su kien
MAX_EVENTS = 400       # tran so su kien ghi ra dia
BAD_ANGLE = 2.0        # sai so goc (deg) coi la HONG trong che do --angles

DLPF = {
    "off": None,
    "low": (Q.acc_d23bw9_n34bw4, Q.gyr_d23bw9_n35bw9),      # ~24 Hz, hop 50 Hz
    "mid": (Q.acc_d111bw4_n136bw, Q.gyr_d119bw5_n154bw3),   # ~115 Hz
}

COLS = [
    "seg", "i", "t", "dt",
    # TANG 3 - raw
    "ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw", "tmp_raw",
    "magst1", "magst2",
    # TANG 4/5 - da calib, CHUA loc
    "ax", "ay", "az", "norm", "gx_dps", "gy_dps", "gz_dps",
    "gx_rad", "gy_rad", "gz_rad",
    # TANG 6 - sau low-pass EMA
    "fax", "fay", "faz", "fnorm", "fgx_rad", "fgy_rad", "fgz_rad",
    # TANG 7 - goc
    "roll", "pitch", "froll", "fpitch",
    "tempC", "exp_roll", "exp_pitch", "flags",
]
I_ROLL, I_PITCH = COLS.index("roll"), COLS.index("pitch")


def load_calib():
    """Nap calib. Uu tien calibfull.json (ban dung cho server).

    Tra ve them DLPF ghi trong meta luc calib. Banner doi chieu no voi DLPF
    dang chay, thay vi hardcode mot chuoi - hardcode thi doi cach calib la
    dong chu thich hoa sai lang le.
    """
    p = HERE.parent.parent / "src/leg_server/calibfull.json"
    if p.exists():
        c = json.load(open(p))
        return (np.array(c["accel"]["SM"]), np.array(c["accel"]["bias"]),
                np.array([c["gyro"]["gx_bias"], c["gyro"]["gy_bias"],
                          c["gyro"]["gz_bias"]]),
                c["gyro"]["gyro_sensitivity"], str(p),
                c.get("meta", {}).get("dlpf"))
    a = json.load(open(HERE / "vinh_accel_calib_ellipsoid.json"))
    g = json.load(open(HERE / "vinhgyrocalib.json"))
    return (np.array(a["accel"]["SM"]), np.array(a["accel"]["bias"]),
            np.array([g["gx_bias"], g["gy_bias"], g["gz_bias"]]),
            g["gyro_sensitivity"], "vinh_accel_calib_ellipsoid.json",
            a.get("meta", {}).get("dlpf"))


def tilt(a):
    """TANG 7 - goc nghieng TRUC TIEP tu trong luc. Khong gyro, khong Madgwick."""
    ax, ay, az = a
    return (math.degrees(math.atan2(ay, az)),
            math.degrees(math.atan2(-ax, math.hypot(ay, az))))


def near_multiple_256(d):
    """Delta co gan boi so cua 256 khong -> chu ky cua torn read."""
    if abs(d) < 128:
        return False
    r = abs(d) % 256
    return min(r, 256 - r) <= 24


class Pipeline:
    """7 tang xu ly, dung chung cho ca hai che do."""

    def __init__(self, SM, AB, GB, GS, a_acc, a_gyr, warm_start):
        self.SM, self.AB, self.GB, self.GS = SM, AB, GB, GS
        self.a_acc, self.a_gyr = a_acc, a_gyr
        self.fa = None if warm_start else np.zeros(3)
        self.fg = None if warm_start else np.zeros(3)
        self.prev = None
        self.stale_n = 0
        self.dts = deque(maxlen=400)
        self.ring = deque(maxlen=CTX)
        self.i = 0

    def step(self, araw, graw, tmp_raw, st1, st2, t, dt, seg="", exp=(None, None)):
        self.i += 1
        self.dts.append(dt)

        a = self.SM @ araw - self.AB                      # TANG 4
        norm = float(np.linalg.norm(a))
        gd = (graw - self.GB) / self.GS                   # TANG 5 (deg/s)
        gr = gd * D2R                                     # rad/s
        gmax = float(np.abs(gd).max())

        if self.fa is None:                               # TANG 6
            self.fa, self.fg = a.copy(), gr.copy()
        else:
            self.fa = self.a_acc * a + (1.0 - self.a_acc) * self.fa
            self.fg = self.a_gyr * gr + (1.0 - self.a_gyr) * self.fg
        fa, fg = self.fa, self.fg
        fnorm = float(np.linalg.norm(fa))

        roll, pitch = tilt(a)                             # TANG 7 (chua loc)
        froll, fpitch = tilt(fa)                          #        (da loc)
        tempC = tmp_raw / 333.87 + 21.0

        # ---- phat hien bat thuong, tren du lieu CHUA loc ----
        # Co y khong dung so da loc: EMA lam nhoe glitch ra ~10 mau, che mat dau vet.
        flags = []
        if abs(norm - G) > TH["norm"]:
            flags.append(f"NORM:{norm:.2f}")
        if gmax > TH["gyro"]:
            flags.append(f"GYRO:{gmax:.0f}")
        if st2 & 0x08:
            flags.append("MAGOVF")

        if self.prev is not None:
            da = araw - self.prev[0]
            dg = graw - self.prev[1]
            if np.abs(da).max() > TH["jump"]:
                k = int(np.abs(da).argmax())
                flags.append(f"JUMP:{'xyz'[k]}{da[k]:+.0f}")
                if near_multiple_256(da[k]):
                    flags.append(f"TEAR:{da[k]:+.0f}(~{round(da[k]/256)}x256)")
            if np.abs(dg).max() > TH["gjump"]:
                k = int(np.abs(dg).argmax())
                flags.append(f"GJUMP:{'xyz'[k]}{dg[k]:+.0f}")
                if near_multiple_256(dg[k]):
                    flags.append(f"GTEAR:{dg[k]:+.0f}")
            if np.array_equal(araw, self.prev[0]) and np.array_equal(graw, self.prev[1]):
                self.stale_n += 1
                if self.stale_n == TH["stale"]:
                    flags.append(f"STALE:{self.stale_n}")
            else:
                self.stale_n = 0
            if self.ring:
                dr = roll - self.ring[-1][I_ROLL]
                dp = pitch - self.ring[-1][I_PITCH]
                if max(abs(dr), abs(dp)) > TH["angle"]:
                    flags.append(f"ANGLE:dr={dr:+.1f},dp={dp:+.1f}")
        if len(self.dts) > 50:
            med = float(np.median(self.dts))
            if med > 0 and dt > TH["dt"] * med:
                flags.append(f"DT:{dt*1000:.0f}ms(med={med*1000:.1f})")
        self.prev = (araw, graw)

        row = [seg, self.i, round(t, 4), round(dt, 5),
               int(araw[0]), int(araw[1]), int(araw[2]),
               int(graw[0]), int(graw[1]), int(graw[2]), tmp_raw, st1, st2,
               round(a[0], 4), round(a[1], 4), round(a[2], 4), round(norm, 4),
               round(gd[0], 3), round(gd[1], 3), round(gd[2], 3),
               round(gr[0], 5), round(gr[1], 5), round(gr[2], 5),
               round(fa[0], 4), round(fa[1], 4), round(fa[2], 4), round(fnorm, 4),
               round(fg[0], 5), round(fg[1], 5), round(fg[2], 5),
               round(roll, 3), round(pitch, 3), round(froll, 3), round(fpitch, 3),
               round(tempC, 2),
               "" if exp[0] is None else exp[0], "" if exp[1] is None else exp[1],
               "|".join(flags)]
        self.ring.append(row)
        meas = dict(norm=norm, roll=roll, pitch=pitch, froll=froll, fpitch=fpitch,
                    gmax=gmax, tempC=tempC, fnorm=fnorm)
        return row, flags, meas


class EventWriter:
    """Ghi su kien kem 40 mau ngu canh truoc va sau."""

    def __init__(self, path):
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["event_id", "kind", "detail", "pos"] + COLS)
        self.pend = []
        self.n = 0
        self.counts = Counter()

    def feed(self, row):
        for p in self.pend[:]:
            p["after"].append(row)
            if len(p["after"]) >= CTX:
                self._flush(p)
                self.pend.remove(p)

    def add(self, flags, row, ring):
        if self.n >= MAX_EVENTS:
            return None
        for fl in flags:
            self.counts[fl.split(":")[0]] += 1
        eid = self.n
        self.pend.append({"id": eid, "kind": flags[0].split(":")[0],
                          "detail": "|".join(flags), "row": row,
                          "ctx": list(ring)[:-1], "after": []})
        self.n += 1
        return eid

    def io_error(self, e):
        self.counts["IOERR"] += 1
        self.w.writerow([self.n, "IOERR", repr(e)[:120], 0] + [""] * len(COLS))
        self.n += 1

    def _flush(self, p):
        for r in p["ctx"]:
            self.w.writerow([p["id"], p["kind"], p["detail"], "before"] + r)
        self.w.writerow([p["id"], p["kind"], p["detail"], "EVENT"] + p["row"])
        for r in p["after"]:
            self.w.writerow([p["id"], p["kind"], p["detail"], "after"] + r)
        self.f.flush()

    def close(self):
        for p in self.pend:
            self._flush(p)
        self.f.close()


def make_imu(dlpf):
    IMU = Q.QwiicIcm20948()
    if not IMU.connected:
        sys.exit("Khong tim thay IMU (i2cdetect -y 1 phai thay 0x69)")
    IMU.begin()
    IMU.setFullScaleRangeAccel(Q.gpm4)                    # TANG 1
    IMU.setFullScaleRangeGyro(Q.dps500)
    if DLPF[dlpf] is not None:                            # TANG 2
        ca, cg = DLPF[dlpf]
        IMU.setDLPFcfgAccel(ca)
        IMU.setDLPFcfgGyro(cg)
        IMU.enableDlpfAccel(True)
        IMU.enableDlpfGyro(True)
    return IMU


def banner(args, src, GS, a_acc, a_gyr, out, title, tail, calib_dlpf):
    fsr_g = "dps500" if abs(GS - 65.5) < 1 else "dps250" if abs(GS - 131) < 1 else "?"
    if calib_dlpf is None:
        note = "  (calib khong ghi DLPF - khong doi chieu duoc)"
    elif args.dlpf == calib_dlpf:
        note = "  (giong calib)"
    else:
        note = f"  (KHAC calib={calib_dlpf} - dang A/B)"
    print(f"\n{'='*78}")
    print(f" {title}")
    print(f" Calib : {src}")
    print(f" Tang 1 FSR       : gpm4 / {fsr_g} (sens={GS})")
    print(f" Tang 2 DLPF HW   : {args.dlpf}{note}")
    print(f" Tang 6 EMA alpha : accel={a_acc}  gyro={a_gyr}"
          + ("  (tat loc)" if a_acc >= 1.0 else "")
          + ("  [warm-start]" if args.warm_start else "  [khoi tao 0.0 nhu gyroacce.py]"))
    print(f" Tang 7 goc       : atan2 tu trong luc (KHONG Madgwick)")
    print(f" Ket qua -> {out}")
    print(f" {tail}")
    print(f"{'='*78}\n")


def read_sample(IMU):
    """Tra ve (araw, graw, tmp, st1, st2) hoac None neu chua co du lieu."""
    if not IMU.dataReady():
        return None
    IMU.getAgmt()
    return (np.array([IMU.axRaw, IMU.ayRaw, IMU.azRaw], float),
            np.array([IMU.gxRaw, IMU.gyRaw, IMU.gzRaw], float),
            IMU.tmpRaw, IMU.magStat1, IMU.magStat2)


def enter_pressed():
    return bool(select.select([sys.stdin], [], [], 0)[0])


def parse_angles(s):
    """'0,30' -> (0.0, 30.0);  ',30' -> (None, 30.0);  '-45,' -> (-45.0, None)"""
    parts = [p.strip() for p in s.replace(";", ",").split(",")]
    if len(parts) == 1:
        parts.append("")
    vals = []
    for p in parts[:2]:
        if p in ("", "-", "x", "X"):
            vals.append(None)
        else:
            vals.append(float(p))
    return vals[0], vals[1]


# ====================================================================== ANGLES
def mode_angles(args, pipe_args, out):
    SM, AB, GB, GS, src, a_acc, a_gyr, calib_dlpf = pipe_args
    IMU = make_imu(args.dlpf)
    banner(args, src, GS, a_acc, a_gyr, out,
           "DO SAI SO ROLL / PITCH THEO GOC THAT",
           f"Bo qua {args.settle:.1f}s dau moi doan cho on dinh.", calib_dlpf)

    if not sys.stdin.isatty():
        sys.exit("Che do --angles can terminal that (dung stdin). Bo --angles de chay soak.")

    pipe = Pipeline(SM, AB, GB, GS, a_acc, a_gyr, args.warm_start)
    ev = EventWriter(out / "events.csv")
    f_all = open(out / "all.csv", "w", newline="")
    w_all = csv.writer(f_all); w_all.writerow(COLS)
    f_seg = open(out / "segments.csv", "w", newline="")
    w_seg = csv.writer(f_seg)
    w_seg.writerow(["seg", "exp_roll", "exp_pitch", "n", "secs",
                    "roll_mean", "roll_err", "roll_std", "roll_p2p",
                    "pitch_mean", "pitch_err", "pitch_std", "pitch_p2p",
                    "norm_mean", "norm_std", "tempC", "n_events", "verdict"])

    segs = []
    cur = None
    t0 = time.monotonic()
    period = 1.0 / args.rate if args.rate > 0 else 0.0

    def close_seg():
        """Chot doan dang ghi -> segments.csv.
        Goi ca khi bam ENTER lan khi Ctrl+C, de khong mat du lieu da do."""
        nonlocal cur
        if cur is None:
            return
        c, cur = cur, None
        if not c["R"]:
            print("\r    (doan rong, bo qua)\n")
            return
        R, P, N = np.array(c["R"]), np.array(c["P"]), np.array(c["N"])
        er, ep, nev = c["er"], c["ep"], c["nev"]
        rm, pm = float(R.mean()), float(P.mean())
        re_ = None if er is None else rm - er
        pe_ = None if ep is None else pm - ep
        worst = max([abs(x) for x in (re_, pe_) if x is not None], default=0.0)
        verdict = "OK" if worst <= BAD_ANGLE and nev == 0 else \
                  ("LECH" if worst > BAD_ANGLE else "CO_SU_KIEN")
        rec = dict(seg=c["seg"], er=er, ep=ep, n=len(R),
                   secs=time.monotonic() - c["t_seg"],
                   rm=rm, re=re_, rs=float(R.std()), rp=float(np.ptp(R)),
                   pm=pm, pe=pe_, ps=float(P.std()), pp=float(np.ptp(P)),
                   nm=float(N.mean()), ns=float(N.std()),
                   tC=c["rows"][-1][COLS.index("tempC")], nev=nev, verdict=verdict)
        segs.append(rec)
        w_seg.writerow([rec["seg"], er if er is not None else "", ep if ep is not None else "",
                        rec["n"], round(rec["secs"], 2),
                        round(rm, 3), "" if re_ is None else round(re_, 3),
                        round(rec["rs"], 3), round(rec["rp"], 3),
                        round(pm, 3), "" if pe_ is None else round(pe_, 3),
                        round(rec["ps"], 3), round(rec["pp"], 3),
                        round(rec["nm"], 4), round(rec["ns"], 4),
                        rec["tC"], nev, verdict])
        f_seg.flush(); f_all.flush()

        mark = {"OK": "OK  ", "LECH": ">> LECH <<", "CO_SU_KIEN": "co su kien"}[verdict]
        print(f"\r    {mark}  n={rec['n']}  "
              + (f"roll {rm:+.2f} (dat {er:+.1f}, lech {re_:+.2f})  " if er is not None else "")
              + (f"pitch {pm:+.2f} (dat {ep:+.1f}, lech {pe_:+.2f})" if ep is not None else "")
              + f"\n         on dinh: roll ±{rec['rs']:.2f}  pitch ±{rec['ps']:.2f}  "
                f"|a|={rec['nm']:.3f}  T={rec['tC']}C\n")

    print("  Nhap goc ban DA DAT IMU vao, dang 'roll,pitch'.")
    print("  Vi du:  0,30   |   -45,0   |   ,30  (chi kiem pitch)   |   0,0")
    print("  Go 'q' roi ENTER de ket thuc phien.\n")

    try:
        while True:
            n = len(segs) + 1
            try:
                s = input(f"  [Doan {n}] Goc mong doi (roll,pitch) -> ENTER de BAT DAU GHI: ").strip()
            except EOFError:
                break
            if s.lower() in ("q", "quit", "exit"):
                break
            try:
                er, ep = parse_angles(s)
            except ValueError:
                print("    ! Sai dinh dang. Vi du hop le: 0,30  hoac  -45,0  hoac  ,30\n")
                continue
            if er is None and ep is None:
                print("    ! Phai nhap it nhat mot goc.\n")
                continue

            seg = f"s{n:02d}"
            lab = (f"roll={er:+.1f}" if er is not None else "roll=-") + \
                  ("  pitch=%+.1f" % ep if ep is not None else "  pitch=-")
            print(f"    Dang ghi doan {seg} ({lab}) ... nhan ENTER de KET THUC doan.")

            cur = dict(seg=seg, er=er, ep=ep, rows=[], R=[], P=[], N=[], nev=0,
                       t_seg=time.monotonic())
            rows, R, P, N = cur["rows"], cur["R"], cur["P"], cur["N"]
            t_seg = cur["t_seg"]
            t_prev = t_seg
            t_next = t_seg
            settled = False
            while True:
                if enter_pressed():
                    sys.stdin.readline()
                    break
                if period:
                    now = time.monotonic()
                    if now < t_next:
                        time.sleep(min(period / 4, t_next - now))
                        continue
                    t_next += period
                    if t_next < now:
                        t_next = now + period
                try:
                    smp = read_sample(IMU)
                except Exception as e:
                    ev.io_error(e)
                    continue
                if smp is None:
                    continue
                t = time.monotonic()
                dt, t_prev = t - t_prev, t
                row, flags, m = pipe.step(*smp, t - t0, dt, seg, (er, ep))
                w_all.writerow(row)
                ev.feed(row)
                if flags:
                    if ev.add(flags, row, pipe.ring) is not None:
                        cur["nev"] += 1
                        print(f"\n    !! {'|'.join(flags)}   |a|={m['norm']:.3f} "
                              f"roll={m['roll']:+.2f} pitch={m['pitch']:+.2f}")

                if not settled:
                    if t - t_seg < args.settle:
                        print(f"\r    on dinh... {args.settle-(t-t_seg):4.1f}s   "
                              f"roll={m['froll']:+7.2f}  pitch={m['fpitch']:+7.2f}",
                              end="", flush=True)
                        continue
                    settled = True
                    print("\r" + " " * 78, end="")

                rows.append(row); R.append(m["froll"]); P.append(m["fpitch"]); N.append(m["norm"])
                if len(R) % 10 == 0:
                    de = f"  d_roll={np.mean(R)-er:+6.2f}" if er is not None else ""
                    dp = f"  d_pitch={np.mean(P)-ep:+6.2f}" if ep is not None else ""
                    print(f"\r    n={len(R):5d}  roll={np.mean(R):+7.2f}  pitch={np.mean(P):+7.2f}"
                          f"{de}{dp}  |a|={np.mean(N):.3f}  loi={cur['nev']}", end="", flush=True)

            close_seg()

    except KeyboardInterrupt:
        print("\n\n  Ctrl+C - chot not doan dang ghi roi in bao cao.")
        try:
            close_seg()
        except Exception as e:
            print(f"  (khong chot duoc doan dang ghi: {e})")

    ev.close(); f_all.close(); f_seg.close()
    return report_angles(segs, ev, pipe, out, args, a_acc, a_gyr)


def report_angles(segs, ev, pipe, out, args, a_acc, a_gyr):
    L = ["=" * 96, " KET QUA DO GOC - ROLL & PITCH", "=" * 96,
         f" Cau hinh: DLPF={args.dlpf}  alpha_a={a_acc}  alpha_g={a_gyr}  "
         f"settle={args.settle}s  rate={args.rate or 'max'}Hz",
         f" Tong mau: {pipe.i}   Tong doan: {len(segs)}   Tong su kien: {ev.n}", ""]
    if segs:
        L.append(" {:>4} | {:>8} {:>9} {:>8} {:>6} | {:>8} {:>9} {:>8} {:>6} | {:>7} {:>4} {:>10}".format(
            "doan", "roll dat", "roll do", "LECH", "±std", "pitch dat", "pitch do",
            "LECH", "±std", "|a|", "loi", "ket luan"))
        L.append(" " + "-" * 94)
        for s in segs:
            L.append(" {:>4} | {:>8} {:>9.2f} {:>8} {:>6.2f} | {:>9} {:>8.2f} {:>8} {:>6.2f} | {:>7.3f} {:>4} {:>10}".format(
                s["seg"],
                f"{s['er']:+.1f}" if s["er"] is not None else "-", s["rm"],
                f"{s['re']:+.2f}" if s["re"] is not None else "-", s["rs"],
                f"{s['ep']:+.1f}" if s["ep"] is not None else "-", s["pm"],
                f"{s['pe']:+.2f}" if s["pe"] is not None else "-", s["ps"],
                s["nm"], s["nev"], s["verdict"]))
        bad = [s for s in segs if s["verdict"] == "LECH"]
        L += ["", f" So doan LECH qua {BAD_ANGLE}deg: {len(bad)} / {len(segs)}"]
        for s in bad:
            d = []
            if s["re"] is not None and abs(s["re"]) > BAD_ANGLE:
                d.append(f"roll lech {s['re']:+.2f}")
            if s["pe"] is not None and abs(s["pe"]) > BAD_ANGLE:
                d.append(f"pitch lech {s['pe']:+.2f}")
            L.append(f"   {s['seg']}: " + ", ".join(d) + f"   (|a|={s['nm']:.3f}, std_r={s['rs']:.2f}, std_p={s['ps']:.2f})")
        if bad:
            L += ["",
                  " GOI Y DOC KET QUA:",
                  "   - |a| lech xa 9.81  -> loi scale/FSR hoac calib accel.",
                  "   - |a| dung 9.81 ma goc van lech deu theo 1 huong",
                  "     -> lech co hoc: mat phang IMU khong song song mat ban chuan.",
                  "   - Lech tang dan theo goc lon -> ma tran SM (cross-axis) chua chuan,",
                  "     calib lai bang calib_accel_ellipsoid.py, lan them huong CANH/GOC",
                  "     cho do phu mat cau tang.",
                  "   - Lech ngau nhien, std lon, co su kien -> xem events.csv."]
    else:
        L.append(" (khong co doan nao)")
    if ev.counts:
        L += ["", " Phan loai su kien:"]
        for k, v in ev.counts.most_common():
            L.append(f"   {k:<10} {v:6d}   ({v/max(pipe.i,1)*100:.4f}% so mau)")
    L += ["", f" File: {out}", "=" * 96]
    return "\n".join(L)


# ======================================================================== SOAK
def mode_soak(args, pipe_args, out):
    SM, AB, GB, GS, src, a_acc, a_gyr, calib_dlpf = pipe_args
    IMU = make_imu(args.dlpf)
    banner(args, src, GS, a_acc, a_gyr, out,
           f"SOAK TEST 1 IMU - {args.minutes:.0f} phut @ {args.rate or 'max'} Hz",
           "DE IMU YEN O MOT GOC CO DINH. Ctrl+C de dung som.", calib_dlpf)

    pipe = Pipeline(SM, AB, GB, GS, a_acc, a_gyr, args.warm_start)
    ev = EventWriter(out / "events.csv")
    f_sm = open(out / "summary.csv", "w", newline="")
    w_sm = csv.writer(f_sm)
    w_sm.writerow(["t", "n", "hz", "norm_mean", "norm_min", "norm_max", "norm_std",
                   "roll_mean", "pitch_mean", "roll_p2p", "pitch_p2p",
                   "froll_mean", "fpitch_mean", "tempC", "gmax_dps", "n_events"])
    f_all = w_all = None
    if args.full:
        f_all = open(out / "all.csv", "w", newline="")
        w_all = csv.writer(f_all); w_all.writerow(COLS)

    sec = {"n": 0, "norm": [], "roll": [], "pitch": [], "froll": [], "fpitch": [],
           "gmax": 0.0, "t0": None, "ev": 0}
    period = 1.0 / args.rate if args.rate > 0 else 0.0
    t0 = time.monotonic()
    t_end = t0 + args.minutes * 60
    t_prev = t_next = t0
    io_err = 0

    try:
        while time.monotonic() < t_end:
            if period:
                now = time.monotonic()
                if now < t_next:
                    time.sleep(min(period / 4, t_next - now))
                    continue
                t_next += period
                if t_next < now:
                    t_next = now + period
            try:
                smp = read_sample(IMU)
            except Exception as e:
                io_err += 1
                ev.io_error(e)
                continue
            if smp is None:
                continue
            t = time.monotonic()
            dt, t_prev = t - t_prev, t
            row, flags, m = pipe.step(*smp, t - t0, dt)
            if w_all:
                w_all.writerow(row)
            ev.feed(row)
            if flags and ev.add(flags, row, pipe.ring) is not None:
                sec["ev"] += 1
                print(f"\n  !! #{ev.n-1:<3} t={t-t0:8.2f}s  {'|'.join(flags)}")
                print(f"     |a|={m['norm']:.3f} roll={m['roll']:+.2f} pitch={m['pitch']:+.2f}")

            if sec["t0"] is None:
                sec["t0"] = t
            sec["n"] += 1
            sec["norm"].append(m["norm"]); sec["roll"].append(m["roll"])
            sec["pitch"].append(m["pitch"]); sec["froll"].append(m["froll"])
            sec["fpitch"].append(m["fpitch"])
            sec["gmax"] = max(sec["gmax"], m["gmax"])
            if t - sec["t0"] >= 1.0:
                n_ = np.array(sec["norm"]); r_ = np.array(sec["roll"]); p_ = np.array(sec["pitch"])
                fr_ = np.array(sec["froll"]); fp_ = np.array(sec["fpitch"])
                el = t - t0
                w_sm.writerow([round(el, 2), sec["n"], round(sec["n"] / (t - sec["t0"]), 1),
                               round(float(n_.mean()), 4), round(float(n_.min()), 4),
                               round(float(n_.max()), 4), round(float(n_.std()), 4),
                               round(float(r_.mean()), 3), round(float(p_.mean()), 3),
                               round(float(np.ptp(r_)), 3), round(float(np.ptp(p_)), 3),
                               round(float(fr_.mean()), 3), round(float(fp_.mean()), 3),
                               round(m["tempC"], 2), round(sec["gmax"], 2), sec["ev"]])
                f_sm.flush()
                print(f"\r  t={el/60:6.2f}p  {sec['n']:3d}Hz  |a|={n_.mean():.3f}"
                      f"(±{n_.std():.3f})  roll={fr_.mean():+7.2f}  pitch={fp_.mean():+7.2f}"
                      f"  T={m['tempC']:.1f}C  su_kien={ev.n:<3}", end="", flush=True)
                sec = {"n": 0, "norm": [], "roll": [], "pitch": [], "froll": [],
                       "fpitch": [], "gmax": 0.0, "t0": t, "ev": 0}
    except KeyboardInterrupt:
        print("\n\n  Dung boi nguoi dung.")

    ev.close(); f_sm.close()
    if f_all:
        f_all.close()

    dur = time.monotonic() - t0
    L = ["=" * 70, " SOAK TEST KET THUC", "=" * 70,
         f" Thoi luong      : {dur/60:.2f} phut",
         f" Tong mau        : {pipe.i}   ({pipe.i/max(dur,1e-9):.1f} Hz trung binh)",
         f" Cau hinh        : DLPF={args.dlpf}  alpha_a={a_acc}  alpha_g={a_gyr}",
         f" Loi doc I2C     : {io_err}",
         f" Tong su kien    : {ev.n}" + ("  (DA TRAN MAX_EVENTS)" if ev.n >= MAX_EVENTS else ""),
         "", " Phan loai su kien:"]
    if ev.counts:
        for k, v in ev.counts.most_common():
            L.append(f"   {k:<10} {v:6d}   ({v/max(pipe.i,1)*100:.4f}% so mau)")
    else:
        L.append("   (khong co bat thuong nao)")
    L += ["", " Y NGHIA:",
          "   NORM   -> |a| sai chuan: scale/FSR sai hoac cu soc manh",
          "   TEAR   -> delta ~ boi so 256: TORN READ (byte cao/thap lech mau)",
          "   JUMP   -> nhay raw dot ngot khong do chuyen dong",
          "   STALE  -> thanh ghi khong doi: dataReady ket / doc lai mau cu",
          "   DT     -> bus I2C treo mot nhip",
          "   IOERR  -> readBlock tra thieu byte (getAgmt nem exception)",
          "   MAGOVF -> tu ke bao tran, I2C-master noi bo co van de",
          "",
          " Neu thay nhieu TEAR/JUMP -> thu lai voi:  --dlpf low",
          "", f" File: {out}", "=" * 70]
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angles", action="store_true",
                    help="che do do sai so roll/pitch theo goc ban tu nhap")
    ap.add_argument("--minutes", type=float, default=60, help="che do soak")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="che do angles: bo qua bao nhieu giay dau moi doan")
    ap.add_argument("--rate", type=float, default=50,
                    help="Hz. 50 = giong gyroacce.py. 0 = nhanh nhat")
    ap.add_argument("--alpha", type=float, default=0.2,
                    help="he so low-pass EMA. 0.2=gyroacce.py, 0.15=leg_server, 1.0=tat")
    ap.add_argument("--alpha-gyro", type=float, default=None,
                    help="rieng cho gyro (mac dinh = --alpha)")
    ap.add_argument("--dlpf", choices=list(DLPF), default="low",
                    help="DLPF phan cung. low = giong calib ellipsoid."
                         " Do tren log: off lam nhieu gyro gap ~15 lan"
                         " (std 2.5 vs 0.27 deg/s) - chi dung de A/B")
    ap.add_argument("--warm-start", action="store_true",
                    help="khoi tao EMA bang mau dau (gyroacce.py khoi tao bang 0.0)")
    ap.add_argument("--full", action="store_true", help="soak: ghi moi mau vao all.csv")
    ap.add_argument("--out", default=str(HERE / "logs"))
    args = ap.parse_args()

    SM, AB, GB, GS, src, calib_dlpf = load_calib()
    a_acc = args.alpha
    a_gyr = args.alpha if args.alpha_gyro is None else args.alpha_gyro
    pipe_args = (SM, AB, GB, GS, src, a_acc, a_gyr, calib_dlpf)

    tag = "angles" if args.angles else "soak"
    out = Path(args.out) / f"{tag}_{datetime.now():%Y%m%d_%H%M%S}"
    out.mkdir(parents=True, exist_ok=True)

    rep = (mode_angles if args.angles else mode_soak)(args, pipe_args, out)
    print("\n" + rep)
    (out / "report.txt").write_text(rep + "\n")


if __name__ == "__main__":
    main()
