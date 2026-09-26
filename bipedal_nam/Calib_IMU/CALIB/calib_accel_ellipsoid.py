# CALIB ACCEL BANG ELLIPSOID FIT
#
# Khac calib_accel.py o RANG BUOC:
#   calib_accel.py   khai HUONG trong luc  -> a = [0,0,g]     -> phu thuoc ga dat
#   file nay         khai DO LON trong luc -> ||a|| = 9.80665 -> khong phu thuoc
#
# Do lon trong luc khong doi theo tu the, nen ban nghieng / ga khong vuong
# deu khong anh huong. Doi lai, phuong phap nay KHONG xac dinh duoc huong
# he truc -> phan do giai o buoc calib lap dat tren robot.
#
# Xem docs/wonder.md (ly thuyet) va docs/analysis.md (vai tro tung file).
#
# CHAY:  python3 calib_accel_ellipsoid.py
#        Lan board qua ~30 tu the rai deu. Script tu phat hien luc dung yen
#        va tu lay mau. ENTER de dung som.

import argparse
import contextlib
import io
import json
import select
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import qwiic_icm20948 as Q

HERE = Path(__file__).parent
G = 9.80665

# DLPF phan cung: bat buoc trung voi luc DOC, neu khong he so khong khop.
# 'low' da duoc chung minh la khu duoc torn read (xem docs/analysis.md).
DLPF = {
    "off": None,
    "low": (Q.acc_d23bw9_n34bw4, Q.gyr_d23bw9_n35bw9),
    "mid": (Q.acc_d111bw4_n136bw, Q.gyr_d119bw5_n154bw3),
}

WIN = 25                # so mau trong cua so xet dung yen (0.5s @50Hz)
MIN_SEP_DEG = 15.0      # 2 tu the phai cach nhau it nhat bao nhieu do


def make_imu(dlpf):
    imu = Q.QwiicIcm20948()
    if not imu.connected:
        sys.exit("Khong tim thay IMU tren I2C.")
    imu.begin()
    imu.setFullScaleRangeAccel(Q.gpm4)
    imu.setFullScaleRangeGyro(Q.dps500)
    if DLPF[dlpf] is not None:
        a, g = DLPF[dlpf]
        imu.setDLPFcfgAccel(a)
        imu.setDLPFcfgGyro(g)
        imu.enableDlpfAccel(True)
        imu.enableDlpfGyro(True)
    return imu


def read_raw(imu):
    imu.getAgmt()
    return (np.array([imu.axRaw, imu.ayRaw, imu.azRaw], float),
            np.array([imu.gxRaw, imu.gyRaw, imu.gzRaw], float))


def enter_pressed():
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


def fit_ellipsoid(P):
    """P: (N,3) raw LSB. Tra ve (SM, bias) sao cho ||SM@raw - bias|| = G.

    Giai quadric  r'Qr - 2u'r + c = 0  bang SVD, roi doi sang (SM, bias).
    SM lay can bac hai DOI XUNG cua Q -> khu bot mo ho ve phep xoay.
    """
    s = float(np.linalg.norm(P, axis=1).mean())   # chuan hoa cho on dinh so
    R = P / s
    x, y, z = R.T
    D = np.column_stack([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, -2*x, -2*y, -2*z,
                         np.ones_like(x)])
    _, _, Vt = np.linalg.svd(D, full_matrices=False)
    th = Vt[-1]

    Qm = np.array([[th[0], th[3], th[4]],
                   [th[3], th[1], th[5]],
                   [th[4], th[5], th[2]]])
    u, c = th[6:9], th[9]
    if np.linalg.eigvalsh(Qm)[0] < 0:            # ep positive-definite
        Qm, u, c = -Qm, -u, -c
    if np.linalg.eigvalsh(Qm)[0] <= 0:
        raise RuntimeError("Fit that bai: du lieu chua phu du mat cau.")

    r0 = np.linalg.solve(Qm, u)                  # tam ellipsoid (LSB da chuan hoa)
    k = float(r0 @ Qm @ r0 - c)
    Qs = Qm * (G * G / k)

    w, V = np.linalg.eigh(Qs)                    # can bac hai doi xung
    A = V @ np.diag(np.sqrt(w)) @ V.T

    return A / s, A @ r0


def coverage(P, r0_lsb):
    """Do do phu tren mat cau: tri rieng nho nhat cua ma tran tan xa huong."""
    d = P - r0_lsb
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    return float(np.linalg.eigvalsh(d.T @ d / len(d))[0])


def describe(SM, bias, tag):
    U, sv, Vt = np.linalg.svd(SM)
    rot = U @ Vt
    S = Vt.T @ np.diag(sv) @ Vt
    ang = np.degrees(np.arccos(np.clip((np.trace(rot) - 1) / 2, -1, 1)))
    sc = np.diag(S)
    off = np.array([S[0, 1], S[0, 2], S[1, 2]])
    b_lsb = np.linalg.solve(SM, bias)
    print(f"  [{tag}]")
    print(f"    scale 3 truc (chuan hoa Z) = {np.round(sc / sc[2], 5)}")
    print(f"    chenh scale max            = {(sc.max()/sc.min()-1)*100:.3f} %")
    print(f"    cross-axis                 = {np.round(np.degrees(off/sc.mean()),3)} deg")
    print(f"    phan xoay bi nuot vao SM   = {ang:.3f} deg")
    print(f"    bias                       = {np.round(bias,4)} m/s2"
          f"  ({np.round(b_lsb/8192*1000,1)} mg)")



# ---------------------------------------------------------------- huong chuan
# 26 huong: 6 MAT (1 truc), 12 CANH (2 truc), 8 GOC (3 truc).
# Vector do duoc chi LEN TREN (accel do specific force), nen nhan "+Z" nghia la
# truc +Z dang huong len troi -> mat -Z ap xuong ban.

def _build_canon():
    out = []
    for x in (-1, 0, 1):
        for y in (-1, 0, 1):
            for z in (-1, 0, 1):
                if x == y == z == 0:
                    continue
                v = np.array([x, y, z], float)
                lab = "".join(f"{'+' if c > 0 else '-'}{ax}"
                              for c, ax in zip((x, y, z), "XYZ") if c)
                out.append((v / np.linalg.norm(v), lab, int(np.count_nonzero(v))))
    return out


CANON = _build_canon()
KIND = {1: "MAT ", 2: "CANH", 3: "GOC "}
NEED = {1: 6, 2: 12, 3: 8}


def opposite(lab):
    return lab.replace("+", "~").replace("-", "+").replace("~", "-")


def nearest_canon(n):
    """Chi so huong chuan gan nhat + goc lech (deg)."""
    d = np.array([c[0] for c in CANON]) @ n
    i = int(np.argmax(d))
    return i, float(np.degrees(np.arccos(np.clip(d[i], -1, 1))))


def suggest(n, hit, skipped):
    """Huong chua lay gan voi tu the hien tai nhat. Uu tien MAT+CANH truoc GOC."""
    for maxnz in (2, 3):
        cand = [i for i, (v, l, nz) in enumerate(CANON)
                if nz <= maxnz and i not in hit and i not in skipped]
        if cand:
            dots = np.array([CANON[i][0] for i in cand]) @ n
            j = int(np.argmax(dots))
            return cand[j], float(np.degrees(np.arccos(np.clip(dots[j], -1, 1))))
    return None, 0.0


def howto(i):
    v, lab, nz = CANON[i]
    o = opposite(lab)
    if nz == 1:
        return f"dat MAT {o} ap xuong ban"
    if nz == 2:
        return f"ke hop nam tren CANH {o} (canh do cham ban)"
    return f"dung hop tren GOC {o} (goc do cham ban)"


# ------------------------------------------------------------------- man hinh

class Screen:
    """Ve lai cung mot khoi H dong tai cho, khong cuon terminal."""

    def __init__(self, h):
        self.h = h
        self.first = True

    def draw(self, lines):
        lines = (list(lines) + [""] * self.h)[:self.h]
        buf = "" if self.first else f"\033[{self.h}A"
        self.first = False
        buf += "".join("\033[2K" + l + "\n" for l in lines)
        sys.stdout.write(buf)
        sys.stdout.flush()


def bar(frac, w=14):
    k = int(np.clip(frac, 0.0, 1.0) * w)
    return "[" + "#" * k + "." * (w - k) + "]"


def poll_key():
    """'' = ENTER (dung), 's' = bo qua huong goi y, None = chua bam gi."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


def live_coverage(poses):
    if len(poses) < 4:
        return None
    A = np.array(poses)
    return coverage(A, A.mean(0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", type=int, default=26, help="so tu the can lay")
    ap.add_argument("--per-pose", type=int, default=100, help="so mau moi tu the")
    ap.add_argument("--dlpf", choices=list(DLPF), default="low")
    ap.add_argument("--still-acc", type=float, default=40.0,
                    help="nguong std raw accel (LSB) coi la dung yen")
    ap.add_argument("--still-gyr", type=float, default=40.0,
                    help="nguong std raw gyro (LSB) coi la dung yen")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--out", default=str(HERE.parent / "READ" /
                                         "vinh_accel_calib_ellipsoid.json"))
    args = ap.parse_args()

    imu = make_imu(args.dlpf)
    period = 1.0 / args.rate

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    logdir = HERE / "logs" / f"ellipsoid_{stamp}"
    logdir.mkdir(parents=True, exist_ok=True)
    fcsv = open(logdir / "poses.csv", "w", buffering=1)
    fcsv.write("idx,label,kind,lech_deg,raw_x,raw_y,raw_z,std_max,n_mau,t_s\n")

    print(f" Log -> {logdir}")
    print(" ENTER = dung va tinh   |   s + ENTER = bo qua huong dang goi y\n")

    scr = Screen(24)
    poses, labels = [], []
    hit, skipped, hist = {}, set(), []
    wa, wg = [], []
    floor_a, floor_g = 1e9, 1e9        # std thap nhat quan sat duoc -> goi y nguong
    t0 = time.monotonic()
    stop = False

    def render(state, extra=""):
        cov = live_coverage(poses)
        cnt = {1: 0, 2: 0, 3: 0}
        for i in hit:
            cnt[CANON[i][2]] += 1
        L = ["=" * 72,
             f" CALIB ACCEL - ELLIPSOID FIT       DLPF={args.dlpf}"
             f"  FSR=gpm4/dps500",
             "=" * 72,
             f" DA CHOT  {len(poses):2d} / {args.poses}"
             f"        MAT {cnt[1]}/6   CANH {cnt[2]}/12   GOC {cnt[3]}/8",
             f" do phu   " + ("chua du mau" if cov is None else
                              f"{cov:.4f}   (can > 0.10 truoc khi dung)"),
             "-" * 72]
        if len(wa) == WIN:
            A, Gy = np.array(wa), np.array(wg)
            sa, sg = A.std(0).max(), Gy.std(0).max()
            m = A.mean(0)
            n = m / np.linalg.norm(m)
            sep = min((np.degrees(np.arccos(np.clip(n @ (p / np.linalg.norm(p)),
                                                    -1, 1))) for p in poses),
                      default=180.0)
            ci, cd = nearest_canon(n)
            L += [f" std accel {sa:6.1f} / {args.still_acc:<4.0f} "
                  f"{bar(sa / args.still_acc)}  "
                  f"{'OK' if sa < args.still_acc else 'QUA NGUONG'}",
                  f" std gyro  {sg:6.1f} / {args.still_gyr:<4.0f} "
                  f"{bar(sg / args.still_gyr)}  "
                  f"{'OK' if sg < args.still_gyr else 'QUA NGUONG'}",
                  f" cach cu   {sep:6.1f} / {MIN_SEP_DEG:<4.0f} "
                  f"{bar(sep / (2 * MIN_SEP_DEG))}  "
                  f"{'OK' if sep > MIN_SEP_DEG else 'TRUNG TU THE CU'}",
                  "-" * 72,
                  f" dang o huong : {KIND[CANON[ci][2]]} {CANON[ci][1]:<6s}"
                  f" (lech {cd:4.1f} deg so voi huong chuan)",
                  f" raw          : [{m[0]:7.0f} {m[1]:7.0f} {m[2]:7.0f} ]"]
            si, sd = suggest(n, hit, skipped)
        else:
            L += [" dang khoi dong bo dem...", "", "", "-" * 72, "", ""]
            si, sd = None, 0.0
        L.append("")
        if si is None:
            L += [" >>> DA LAY DU CAC HUONG - bam ENTER de tinh", "", ""]
        else:
            L += [f" >>> TIEP THEO : {KIND[CANON[si][2]]} {CANON[si][1]}"
                  f"      con cach {sd:.0f} deg",
                  f"     cach lam  : {howto(si)}", ""]
        L += ["-" * 72, f" {state}", f" {extra}", "-" * 72]
        for k in range(3):
            L.append(f" {'vua chot' if k == 0 else '        '} "
                     + (hist[k] if k < len(hist) else ""))
        L.append("-" * 72)
        return L

    while len(poses) < args.poses and not stop:
        t = time.monotonic()
        a, g = read_raw(imu)
        wa.append(a); wg.append(g)
        if len(wa) > WIN:
            wa.pop(0); wg.pop(0)

        state, extra = "khoi dong...", ""
        if len(wa) == WIN:
            A, Gy = np.array(wa), np.array(wg)
            sa, sg = A.std(0).max(), Gy.std(0).max()
            floor_a, floor_g = min(floor_a, sa), min(floor_g, sg)
            m = A.mean(0)
            n = m / np.linalg.norm(m)
            sep = min((np.degrees(np.arccos(np.clip(n @ (p / np.linalg.norm(p)),
                                                    -1, 1))) for p in poses),
                      default=180.0)
            still = sa < args.still_acc and sg < args.still_gyr

            if still and sep > MIN_SEP_DEG:
                buf, bad = [], False
                while len(buf) < args.per_pose:
                    buf.append(read_raw(imu)[0])
                    if len(buf) >= 10 and \
                            np.array(buf[-10:]).std(0).max() > args.still_acc * 1.5:
                        bad = True
                        break
                    if len(buf) % 5 == 0:
                        scr.draw(render(
                            f"DANG LAY MAU  {len(buf):3d}/{args.per_pose}"
                            f"  {bar(len(buf) / args.per_pose)}   GIU YEN!"))
                    time.sleep(period)
                if bad:
                    state = "HUY tu the - co dich chuyen giua chung. Dat lai."
                    wa, wg = [], []
                else:
                    mean = np.array(buf).mean(0)
                    u = mean / np.linalg.norm(mean)
                    ci, cd = nearest_canon(u)
                    poses.append(mean)
                    labels.append(CANON[ci][1])
                    hit.setdefault(ci, len(poses))
                    line = (f"[{len(poses):2d}] {KIND[CANON[ci][2]]}"
                            f" {CANON[ci][1]:<6s} lech {cd:4.1f}deg"
                            f"  raw=[{mean[0]:7.0f}{mean[1]:8.0f}{mean[2]:8.0f} ]")
                    hist.insert(0, line)
                    del hist[3:]
                    fcsv.write(f"{len(poses)},{CANON[ci][1]},{CANON[ci][2]},"
                               f"{cd:.2f},{mean[0]:.1f},{mean[1]:.1f},"
                               f"{mean[2]:.1f},{np.array(buf).std(0).max():.1f},"
                               f"{len(buf)},{time.monotonic()-t0:.1f}\n")
                    state = "DA CHOT - xoay sang huong tiep theo"
                    wa, wg = [], []
            elif not still:
                state = "DANG DI CHUYEN / rung  ->  dat xuong ban, giu yen"
                if time.monotonic() - t0 > 20 and floor_a > args.still_acc * 0.8:
                    extra = (f"goi y: nen bo dinh nhat la {floor_a:.0f} LSB,"
                             f" chay lai voi --still-acc {floor_a*1.5:.0f}")
            else:
                state = "DUNG YEN nhung TRUNG tu the cu -> xoay sang huong khac"

        scr.draw(render(state, extra))

        k = poll_key()
        if k == "":
            stop = True
        elif k == "s" and len(wa) == WIN:
            m = np.array(wa).mean(0)
            si, _ = suggest(m / np.linalg.norm(m), hit, skipped)
            if si is not None:
                skipped.add(si)

        d = period - (time.monotonic() - t)
        if d > 0:
            time.sleep(d)

    fcsv.close()
    print()
    if len(poses) < 9:
        sys.exit(f"\nChi co {len(poses)} tu the - can it nhat 9. Chay lai.")

    P = np.array(poses)
    SM, bias = fit_ellipsoid(P)

    r0 = np.linalg.solve(SM, bias)
    cov = coverage(P, r0)
    res = np.linalg.norm(P @ SM.T - bias, axis=1) - G

    rep = io.StringIO()
    with contextlib.redirect_stdout(rep):
        print("=" * 72)
        print(f" KET QUA  ({len(poses)} tu the)")
        print("=" * 72)
        print(f"  huong da lay    = {' '.join(sorted(set(labels)))}")
        print(f"  do phu mat cau  = {cov:.4f}   "
              f"({'DU' if cov > 0.10 else 'THIEU - lan them cac huong con thieu'})")
        print(f"  sai so ||a||    = mean {res.mean():+.5f}  std {res.std():.5f}"
              f"  max |.| {np.abs(res).max():.5f} m/s2"
              f"   ({np.abs(res).max()/G*100:.3f} %)")
        print()
        describe(SM, bias, "MOI - ellipsoid")

        old = HERE.parent / "READ" / "vinh_accel_calib.json"
        if old.exists():
            d = json.load(open(old))["accel"]
            print()
            describe(np.array(d["SM"]), np.array(d["bias"]), "CU - 6 mat")

    text = rep.getvalue()
    print(text)
    (logdir / "report.txt").write_text(text)

    out = Path(args.out)
    out.write_text(json.dumps({
        "accel": {"SM": SM.tolist(), "bias": bias.tolist()},
        "meta": {"method": "ellipsoid", "poses": len(poses),
                 "dlpf": args.dlpf, "fsr_accel": "gpm4",
                 "coverage": cov, "resid_max_pct": float(np.abs(res).max()/G*100),
                 "when": datetime.now().isoformat(timespec="seconds")},
    }, indent=2))
    print(f" Da ghi {out}")
    print(f" Log    {logdir}")


if __name__ == "__main__":
    main()
