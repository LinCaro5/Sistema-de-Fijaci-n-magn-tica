"""
Registro del Robotiq FT300 conectado al controlador UR (URCap), por Ethernet.
El URCap publica el stream del sensor en el puerto TCP 63351 como texto:
    ( Fx , Fy , Fz , Mx , My , Mz )   [N, N·m]  ~100 Hz

Uso (solo librerías estándar, no requiere pip):
  python ft300_ur_logger.py --ip 192.168.0.10 --dur 30 --tare
"""
import argparse
import csv
import os
import re
import socket
import sys
import time

PORT = 63351
NUM = r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"
FRAME = re.compile(r"\(\s*" + r"\s*,\s*".join([f"({NUM})"] * 6) + r"\s*\)")


def parse(buf: str):
    """Devuelve (lista de [6 floats], resto del buffer sin procesar)."""
    vals, last = [], 0
    for m in FRAME.finditer(buf):
        vals.append([float(x) for x in m.groups()])
        last = m.end()
    rest = buf[last:]
    if len(rest) > 500:          # basura sin tramas válidas: descartar
        rest = rest[-200:]
    return vals, rest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True, help="IP del controlador UR")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--dur", type=float, default=0, help="duración [s]; 0 = hasta Ctrl+C")
    ap.add_argument("--hz", type=float, default=0,
                    help="frecuencia de salida [Hz] promediando bloques; 0 = sin reducir (~100 Hz)")
    ap.add_argument("--tare", action="store_true", help="resta el promedio del primer segundo")
    ap.add_argument("--out", default=None, help="ruta del CSV (por defecto, junto al script)")
    a = ap.parse_args()
    if a.out is None:
        a.out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             f"ft300_{time.strftime('%Y%m%d_%H%M%S')}.csv")

    try:
        s = socket.create_connection((a.ip, a.port), timeout=3)
    except OSError as e:
        sys.exit(f"No se pudo conectar a {a.ip}:{a.port} -> {e}\n"
                 "Revisa la IP, el cable de red, que el PC esté en la misma subred "
                 "y que el URCap del FT300 esté activo.")
    s.settimeout(0.5)

    rows, buf = [], ""
    epoch0 = time.time()
    t0 = time.perf_counter()
    last_print = 0.0
    dur = a.dur if a.dur > 0 else float("inf")
    print(f"Conectado a {a.ip}:{a.port}. Registrando "
          f"{'hasta Ctrl+C' if a.dur <= 0 else f'{a.dur:g} s'}...")
    try:
        while time.perf_counter() - t0 < dur:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                print("El robot cerró la conexión.")
                break
            t = time.perf_counter() - t0
            vals, buf = parse(buf + chunk.decode("ascii", errors="ignore"))
            rows.extend([t] + v for v in vals)
            if rows and t - last_print > 1.0:      # monitor en vivo cada 1 s
                r = rows[-1]
                print(f"t={t:5.1f}s  Fx={r[1]:8.2f}  Fy={r[2]:8.2f}  Fz={r[3]:8.2f} N")
                last_print = t
    except KeyboardInterrupt:
        print("\nDetenido por el usuario.")
    finally:
        s.close()

    if not rows:
        sys.exit("Conectó pero no llegaron datos. Verifica que el sensor muestre "
                 "valores en el panel del URCap en el teach pendant.")

    if a.tare:
        n = max(1, sum(1 for r in rows if r[0] < 1.0))
        off = [sum(r[k] for r in rows[:n]) / n for k in range(1, 7)]
        rows = [[r[0]] + [r[k] - off[k - 1] for k in range(1, 7)] for r in rows]
        print("Tara aplicada (N, N·m):", [round(o, 3) for o in off])

    if a.hz > 0:  # promedio por bloques de 1/hz segundos (reduce ruido, no solo descarta)
        bins = {}
        for r in rows:
            bins.setdefault(int(r[0] * a.hz), []).append(r)
        rows = [[(k + 0.5) / a.hz] + [sum(r[j] for r in g) / len(g) for j in range(1, 7)]
                for k, g in sorted(bins.items())]

    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([f"# inicio_epoch={epoch0:.6f}", f"ip={a.ip}"])
        w.writerow(["t_s", "Fx_N", "Fy_N", "Fz_N", "Mx_Nm", "My_Nm", "Mz_Nm"])
        w.writerows([[f"{r[0]:.5f}"] + [f"{x:.3f}" for x in r[1:]] for r in rows])

    dur = rows[-1][0] - rows[0][0]
    print(f"{len(rows)} muestras en {dur:.1f} s  ->  {len(rows)/max(dur,1e-9):.1f} Hz")
    print(f"Guardado en {a.out}")


if __name__ == "__main__":
    main()
