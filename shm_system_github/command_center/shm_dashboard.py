#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SHM Command Center -- Structural Health Monitoring Dashboard
Front-end for: Arduino Uno + MPU-6050 + SSD1306 + HC-05 (SPP).

Telemetry @ 10 Hz:  <t_ms>,<tilt_x>,<tilt_y>,<vib_rms_g>,<state>
state: 0 = STABLE, 1 = THRESHOLD EXCEEDED

Install:  pip install -r requirements.txt
Run:      python shm_dashboard.py
"""

from __future__ import annotations

import queue
import threading
import time
import tkinter.font as tkfont
from collections import deque
from dataclasses import dataclass

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import customtkinter as ctk
import serial
import serial.tools.list_ports

# ------------------------------------------------------------------ config
POLL_MS       = 50      # GUI queue drain cadence
PLOT_EVERY    = 2       # draw every N poll cycles (~10 Hz)
LOG_CAP       = 400     # system log lines kept
BUFFER_POINTS = 6000    # ~10 min of telemetry @ 10 Hz

TILT_DEFAULT  = 2.50    # deg
VIB_DEFAULT   = 0.30    # g RMS
TILT_LIMITS   = (0.20, 45.0)
VIB_LIMITS    = (0.05, 5.00)
STALE_S       = 3.0     # no-data watchdog

# ------------------------------------------------------------------ palette
C_BG      = "#0B0E13"
C_PANEL   = "#12161D"
C_PANEL_2 = "#171C24"
C_BORDER  = "#232B36"
C_TEXT    = "#E6EAF0"
C_MUTED   = "#8B949E"
C_ACCENT  = "#3B82F6"
C_EMERALD = "#10B981"
C_CRIMSON = "#EF4444"
C_AMBER   = "#F59E0B"
C_VIB     = "#38BDF8"
C_PITCH   = "#60A5FA"
C_ROLL    = "#A78BFA"

LOG_COLORS = {"OK": "#10B981", "ERR": "#EF4444", "EVT": "#F59E0B",
              "INFO": "#8B949E", "RAW": "#8B949E"}

MONO_CANDIDATES = ("Consolas", "Menlo", "DejaVu Sans Mono",
                   "Courier New", "Courier")


@dataclass(frozen=True)
class Frame:
    t_ms: float
    pitch: float
    roll: float
    vib: float
    state: int


class TelemetryStore:
    """Fixed-capacity ring buffers feeding the scrolling plot."""

    def __init__(self, cap: int = BUFFER_POINTS) -> None:
        self.t: deque = deque(maxlen=cap)
        self.pitch: deque = deque(maxlen=cap)
        self.roll: deque = deque(maxlen=cap)
        self.vib: deque = deque(maxlen=cap)

    def append(self, f: Frame) -> None:
        self.t.append(f.t_ms / 1000.0)
        self.pitch.append(f.pitch)
        self.roll.append(f.roll)
        self.vib.append(f.vib)


class SerialWorker(threading.Thread):
    """Producer thread: owns the serial port, never touches Tk objects.

    Emits to the GUI through three queues:
      frames_q -> parsed telemetry frames
      logs_q   -> (level, message) log records
      events_q -> ("link", bool) connection state transitions
    """

    def __init__(self, port: str, baud: int,
                 frames_q: queue.Queue, logs_q: queue.Queue,
                 events_q: queue.Queue) -> None:
        super().__init__(daemon=True, name=f"SerialWorker[{port}]")
        self.port, self.baud = port, baud
        self._frames_q, self._logs_q, self._events_q = frames_q, logs_q, events_q
        self._stop_evt = threading.Event()
        self._tx: queue.Queue = queue.Queue()
        self._ser: serial.Serial | None = None
        self._fails = 0

    # ------------------------------------------------------------- API
    def send(self, text: str) -> None:
        self._tx.put(text if text.endswith("\n") else text + "\n")

    def stop(self) -> None:
        self._stop_evt.set()
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def active(self) -> bool:
        """True only while the serial port is actually open."""
        return self._ser is not None

    # -------------------------------------------------------- internals
    def _log(self, level: str, msg: str) -> None:
        self._logs_q.put((level, msg))

    def _link(self, up: bool) -> None:
        self._events_q.put(("link", up))

    def _open(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, self.baud,
                                      timeout=0.05, write_timeout=0.5)
        except serial.SerialException as exc:
            self._ser = None
            self._fails += 1
            if self._fails == 1 or self._fails % 10 == 0:
                self._log("ERR", f"PORT OPEN FAILED -- {self.port}: {exc}")
            self._link(False)
            return False
        self._fails = 0
        self._log("OK", f"LINK ESTABLISHED -- {self.port} @ {self.baud} BAUD (BT-SPP)")
        self._link(True)
        self.send("VER")
        self.send("PING")
        return True

    def _classify(self, text: str) -> None:
        if not text:
            return
        if text.startswith("#"):
            self._log("INFO", f"DEVICE BANNER -- {text[1:]}")
            return
        parts = text.split(",")
        if len(parts) == 5 and parts[0].isdigit():
            try:
                self._frames_q.put(Frame(float(parts[0]), float(parts[1]),
                                         float(parts[2]), float(parts[3]),
                                         int(parts[4])))
                return
            except ValueError:
                pass
        if text.startswith("OK:"):
            payload = text[3:]
            if payload == "CAL":
                self._log("OK", "ZERO-POINT BASELINE WRITTEN -- SENSOR REFERENCE SET")
            elif payload == "PING":
                self._log("OK", "HANDSHAKE ACKNOWLEDGED")
            else:
                self._log("OK", f"DEVICE ACK -- {payload}")
        elif text.startswith("ERR:"):
            self._log("ERR", f"DEVICE REJECT -- {text[4:]}")
        elif text.startswith("EVT:"):
            name = text[4:]
            if name == "ANOMALY":
                self._log("EVT", "DEVICE EVENT -- THRESHOLD EXCEEDED")
            elif name == "STABLE":
                self._log("EVT", "DEVICE EVENT -- PARAMETER RETURNED WITHIN LIMITS")
            else:
                self._log("INFO", f"DEVICE EVENT -- {name}")
        else:
            self._log("RAW", text)

    def _drop(self, reason: str) -> None:
        self._log("ERR", f"LINK LOST -- {reason}")
        self._link(False)
        self._ser = None

    # ------------------------------------------------------------- main
    def run(self) -> None:
        while not self._stop_evt.is_set():
            if self._ser is None:
                if not self._open():
                    self._stop_evt.wait(2.0)
                    continue
            ser = self._ser
            # ---- TX drain
            try:
                while True:
                    text = self._tx.get_nowait()
                    ser.write(text.encode("ascii"))
            except queue.Empty:
                pass
            except Exception as exc:
                self._drop(str(exc)); continue
            # ---- RX
            try:
                raw = ser.readline()
            except Exception as exc:
                self._drop(str(exc)); continue
            if raw:
                self._classify(raw.decode("ascii", "replace").strip())
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass


class MetricCard(ctk.CTkFrame):
    """Compact readout card in the top strip."""

    def __init__(self, master, title: str, mono: str) -> None:
        super().__init__(master, fg_color=C_PANEL, corner_radius=10)
        self.lbl_title = ctk.CTkLabel(self, text=title, text_color=C_MUTED,
                                      font=ctk.CTkFont(size=11, weight="bold"))
        self.lbl_title.pack(anchor="w", padx=12, pady=(10, 0))
        self.lbl_value = ctk.CTkLabel(
            self, text="--", text_color=C_TEXT,
            font=ctk.CTkFont(family=mono, size=22, weight="bold"))
        self.lbl_value.pack(anchor="w", padx=12, pady=(0, 10))

    def set(self, text: str, color: str = C_TEXT) -> None:
        self.lbl_value.configure(text=text, text_color=color)


class CommandCenter(ctk.CTk):
    """Main application window (observer pattern over poll loop)."""

    def __init__(self) -> None:
        super().__init__()
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")

        self.title("SHM COMMAND CENTER -- STRUCTURAL MONITORING")
        self.geometry("1200x760")
        self.minsize(1080, 640)
        self.configure(fg_color=C_BG)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # ---- application state
        self._mono = self._mono_family()
        self.q_frames: queue.Queue = queue.Queue()
        self.q_logs: queue.Queue = queue.Queue()
        self.q_events: queue.Queue = queue.Queue()
        self.worker: SerialWorker | None = None

        self.store = TelemetryStore()
        self.tilt_thr = TILT_DEFAULT
        self.vib_thr = VIB_DEFAULT
        self.online = False
        self.critical = False
        self.stale = False
        self.event_count = 0
        self.last_frame_t = 0.0
        self.view_window = 60.0
        self._v_ymax = 0.05
        self._t_ymax = 1.00
        self._tick = 0
        self._rate: deque = deque()
        self._t0 = time.time()

        self._build_sidebar()
        self._build_main()
        self.after(POLL_MS, self._poll)
        self.refresh_ports()

    # ------------------------------------------------------------ fonts
    @staticmethod
    def _mono_family() -> str:
        families = set(tkfont.families())
        for name in MONO_CANDIDATES:
            if name in families:
                return name
        return ""

    # =========================================================== SIDEBAR
    def _build_sidebar(self) -> None:
        side = ctk.CTkFrame(self, width=320, fg_color=C_PANEL, corner_radius=0)
        side.pack(side="left", fill="y")
        side.pack_propagate(False)

        pad = dict(padx=20)

        ctk.CTkLabel(side, text="SHM", font=ctk.CTkFont(size=28, weight="bold"),
                     text_color=C_TEXT).pack(anchor="w", pady=(24, 0), **pad)
        ctk.CTkLabel(side, text="STRUCTURAL HEALTH MONITORING",
                     font=ctk.CTkFont(size=11, weight="bold"),
                     text_color=C_MUTED).pack(anchor="w", **pad)
        ctk.CTkLabel(side, text="NODE 01 -- ESP32 / BT-SPP",
                     font=ctk.CTkFont(family=self._mono, size=10),
                     text_color=C_MUTED).pack(anchor="w", pady=(2, 14), **pad)

        def rule():
            ctk.CTkFrame(side, height=1, fg_color=C_BORDER)\
                .pack(fill="x", padx=20, pady=(0, 16))

        # ------------------------------------------------ link panel
        rule()
        ctk.CTkLabel(side, text="LINK", font=ctk.CTkFont(size=12, weight="bold"),
                     text_color=C_ACCENT).pack(anchor="w", pady=(0, 8), **pad)

        row = ctk.CTkFrame(side, fg_color="transparent")
        row.pack(fill="x", **pad)
        self.lbl_dot = ctk.CTkLabel(row, text="\u25cf", text_color=C_MUTED,
                                    font=ctk.CTkFont(size=16))
        self.lbl_dot.pack(side="left")
        self.lbl_link = ctk.CTkLabel(row, text="OFFLINE", text_color=C_MUTED,
                                     font=ctk.CTkFont(size=12, weight="bold"))
        self.lbl_link.pack(side="left", padx=(6, 0))
        self.lbl_rate = ctk.CTkLabel(row, text="", text_color=C_MUTED,
                                     font=ctk.CTkFont(family=self._mono, size=10))
        self.lbl_rate.pack(side="right")

        prow = ctk.CTkFrame(side, fg_color="transparent")
        prow.pack(fill="x", pady=(10, 0), **pad)
        self.port_var = ctk.StringVar()
        self.menu_port = ctk.CTkOptionMenu(
            prow, values=["--"], variable=self.port_var, width=210,
            fg_color=C_PANEL_2, button_color=C_BORDER, button_hover_color=C_BORDER,
            font=ctk.CTkFont(family=self._mono, size=11))
        self.menu_port.pack(side="left")
        ctk.CTkButton(prow, text="REFRESH", width=70, height=26,
                      fg_color=C_PANEL_2, hover_color=C_BORDER,
                      text_color=C_MUTED,
                      command=self.refresh_ports).pack(side="right")

        brow = ctk.CTkFrame(side, fg_color="transparent")
        brow.pack(fill="x", pady=(10, 0), **pad)
        self.baud_var = ctk.StringVar(value="9600")
        self.menu_baud = ctk.CTkOptionMenu(
            brow, values=["9600", "38400", "115200"], variable=self.baud_var,
            width=110, fg_color=C_PANEL_2, button_color=C_BORDER,
            font=ctk.CTkFont(family=self._mono, size=11))
        self.menu_baud.pack(side="left")
        self.btn_link = ctk.CTkButton(
            brow, text="CONNECT", width=140, height=26, corner_radius=6,
            fg_color=C_ACCENT, hover_color="#2563EB",
            font=ctk.CTkFont(size=12, weight="bold"),
            command=self._on_connect_toggle)
        self.btn_link.pack(side="right")

        note = ctk.CTkFrame(side, fg_color="transparent")
        note.pack(fill="x", pady=(10, 0), **pad)
        ctk.CTkFrame(note, height=1, fg_color=C_BORDER).pack(fill="x", pady=(0, 8))
        ctk.CTkLabel(note, text="BAUD SETTING IS IGNORED FOR BLUETOOTH SPP -- "
                                "SELECT THE OUTGOING SPP COM PORT",
                     text_color=C_MUTED, font=ctk.CTkFont(size=9),
                     justify="left").pack(anchor="w")

        # ------------------------------------------------ thresholds
        rule()
        ctk.CTkLabel(side, text="THRESHOLD PROFILE",
                     font=ctk.CTkFont(size=12, weight="bold"),
                     text_color=C_ACCENT).pack(anchor="w", pady=(0, 8), **pad)

        grid = ctk.CTkFrame(side, fg_color="transparent")
        grid.pack(fill="x", **pad)
        grid.columnconfigure(1, weight=1)

        ctk.CTkLabel(grid, text="TILT  (deg)", text_color=C_MUTED,
                     font=ctk.CTkFont(size=11))\
            .grid(row=0, column=0, sticky="w", pady=3)
        self.ent_tilt = ctk.CTkEntry(grid, width=90, fg_color=C_PANEL_2,
                                     font=ctk.CTkFont(family=self._mono, size=12))
        self.ent_tilt.insert(0, f"{TILT_DEFAULT:.2f}")
        self.ent_tilt.grid(row=0, column=1, sticky="e", padx=(10, 0), pady=3)

        ctk.CTkLabel(grid, text="VIB   (g RMS)", text_color=C_MUTED,
                     font=ctk.CTkFont(size=11))\
            .grid(row=1, column=0, sticky="w", pady=3)
        self.ent_vib = ctk.CTkEntry(grid, width=90, fg_color=C_PANEL_2,
                                    font=ctk.CTkFont(family=self._mono, size=12))
        self.ent_vib.insert(0, f"{VIB_DEFAULT:.2f}")
        self.ent_vib.grid(row=1, column=1, sticky="e", padx=(10, 0), pady=3)

        self.btn_apply = ctk.CTkButton(
            side, text="APPLY PROFILE", height=30, corner_radius=6,
            fg_color=C_PANEL_2, hover_color=C_BORDER, border_width=1,
            border_color=C_BORDER, font=ctk.CTkFont(size=12, weight="bold"),
            command=self._on_apply)
        self.btn_apply.pack(fill="x", pady=(10, 0), **pad)

        # ------------------------------------------------ system
        rule()
        ctk.CTkLabel(side, text="SYSTEM", font=ctk.CTkFont(size=12, weight="bold"),
                     text_color=C_ACCENT).pack(anchor="w", pady=(0, 8), **pad)
        self.btn_cal = ctk.CTkButton(
            side, text="CALIBRATE ZERO-POINT", height=30, corner_radius=6,
            fg_color=C_EMERALD, hover_color="#059669",
            font=ctk.CTkFont(size=12, weight="bold"), command=self._on_calibrate)
        self.btn_cal.pack(fill="x", **pad)
        ctk.CTkLabel(side, text="STRUCTURE AT REST REQUIRED -- T = 1.0 s",
                     text_color=C_MUTED,
                     font=ctk.CTkFont(size=10)).pack(anchor="w", pady=(4, 0), **pad)

        # ------------------------------------------------ session stats
        rule()
        self.lbl_session = ctk.CTkLabel(
            side, text="SESSION  --  NO DATA", text_color=C_MUTED,
            font=ctk.CTkFont(family=self._mono, size=10))
        self.lbl_session.pack(anchor="w", pady=(0, 20), **pad)

    # ============================================================== MAIN
    def _build_main(self) -> None:
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.pack(side="right", fill="both", expand=True, padx=(10, 14),
                  pady=(14, 14))

        # ---- top strip: health badge + metric cards
        top = ctk.CTkFrame(main, fg_color="transparent", height=92)
        top.pack(fill="x")

        badge = ctk.CTkFrame(top, fg_color=C_PANEL, corner_radius=10)
        badge.pack(side="left", fill="y", padx=(0, 10))
        ctk.CTkLabel(badge, text="SYSTEM HEALTH", text_color=C_MUTED,
                     font=ctk.CTkFont(size=10, weight="bold"))\
            .pack(anchor="w", padx=16, pady=(10, 2))
        self.lbl_badge = ctk.CTkLabel(
            badge, text="STANDBY", text_color=C_MUTED,
            fg_color="transparent", corner_radius=8,
            font=ctk.CTkFont(size=20, weight="bold"))
        self.lbl_badge.pack(anchor="w", padx=16, pady=(0, 4))
        self.lbl_badge_note = ctk.CTkLabel(
            badge, text="AWAITING LINK", text_color=C_MUTED,
            font=ctk.CTkFont(size=10))
        self.lbl_badge_note.pack(anchor="w", padx=16, pady=(0, 10))

        cards = [("TILT X", "deg"), ("TILT Y", "deg"), ("VIB RMS", "g")]
        self.cards = []
        for title, _unit in cards:
            card = MetricCard(top, title, self._mono)
            card.pack(side="left", fill="y", expand=True, padx=5)
            self.cards.append(card)

        # ---- plot panel
        panel = ctk.CTkFrame(main, fg_color=C_PANEL, corner_radius=10)
        panel.pack(fill="both", expand=True, pady=(10, 0))

        head = ctk.CTkFrame(panel, fg_color="transparent")
        head.pack(fill="x", padx=14, pady=(10, 0))
        ctk.CTkLabel(head, text="STRUCTURAL RESPONSE -- REAL-TIME",
                     font=ctk.CTkFont(size=12, weight="bold"),
                     text_color=C_TEXT).pack(side="left")
        self.view_var = ctk.StringVar(value="60 s")
        self.menu_view = ctk.CTkOptionMenu(
            head, values=["30 s", "60 s", "120 s", "300 s"],
            variable=self.view_var, width=92, height=24,
            fg_color=C_PANEL_2, button_color=C_BORDER,
            font=ctk.CTkFont(family=self._mono, size=11),
            command=self._on_view_change)
        self.menu_view.pack(side="right")
        ctk.CTkLabel(head, text="WINDOW", text_color=C_MUTED,
                     font=ctk.CTkFont(size=10, weight="bold"))\
            .pack(side="right", padx=(0, 6))

        holder = ctk.CTkFrame(panel, fg_color=C_PANEL, corner_radius=10)
        holder.pack(fill="both", expand=True, padx=10, pady=(2, 10))
        self._build_plot(holder)

        # ---- system log
        logf = ctk.CTkFrame(main, fg_color=C_PANEL, corner_radius=10, height=185)
        logf.pack(fill="x", pady=(10, 0))
        ctk.CTkLabel(logf, text="SYSTEM LOG", text_color=C_MUTED,
                     font=ctk.CTkFont(size=10, weight="bold"))\
            .pack(anchor="w", padx=14, pady=(8, 0))
        self.log_box = ctk.CTkTextbox(
            logf, height=145, fg_color=C_PANEL_2, text_color=C_TEXT,
            font=ctk.CTkFont(family=self._mono, size=11), wrap="none")
        self.log_box.pack(fill="both", expand=True, padx=10, pady=(4, 10))
        self.log_box.configure(state="disabled")
        self._log_tags()

    # ------------------------------------------------------------- plot
    def _build_plot(self, parent) -> None:
        self.fig = Figure(facecolor=C_PANEL, dpi=100)
        self.ax_vib, self.ax_tilt = self.fig.subplots(
            2, 1, sharex=True, gridspec_kw={"height_ratios": [1.25, 1.0]})
        self.fig.subplots_adjust(left=0.075, right=0.985, top=0.94,
                                 bottom=0.10, hspace=0.30)

        for ax in (self.ax_vib, self.ax_tilt):
            ax.set_facecolor("#0E1218")
            for s in ax.spines.values():
                s.set_color(C_BORDER)
            ax.tick_params(colors=C_MUTED, labelsize=8)
            ax.grid(True, color=C_BORDER, lw=0.6, alpha=0.7)

        self.ax_vib.set_ylabel("VIB RMS (g)", color=C_MUTED, fontsize=8)
        self.ax_tilt.set_ylabel("TILT (deg)", color=C_MUTED, fontsize=8)
        self.ax_tilt.set_xlabel("TIME (s, scrolling)", color=C_MUTED, fontsize=8)

        self.line_vib, = self.ax_vib.plot([], [], color=C_VIB, lw=1.5,
                                          label="VIB RMS")
        self.thr_vib = self.ax_vib.axhline(VIB_DEFAULT, ls="--", lw=1.0,
                                           color=C_CRIMSON, alpha=0.85,
                                           label="THRESHOLD")
        self.line_pitch, = self.ax_tilt.plot([], [], color=C_PITCH, lw=1.3,
                                             label="TILT X")
        self.line_roll, = self.ax_tilt.plot([], [], color=C_ROLL, lw=1.3,
                                            label="TILT Y")
        self.ax_tilt.axhline(0, color=C_BORDER, lw=0.8)
        self.thr_hi = self.ax_tilt.axhline(TILT_DEFAULT, ls="--", lw=1.0,
                                           color=C_CRIMSON, alpha=0.85)
        self.thr_lo = self.ax_tilt.axhline(-TILT_DEFAULT, ls="--", lw=1.0,
                                           color=C_CRIMSON, alpha=0.85)

        self.ax_vib.legend(loc="upper right", fontsize=7, ncol=2, frameon=False,
                           labelcolor=C_MUTED)
        self.ax_tilt.legend(loc="upper right", fontsize=7, ncol=2, frameon=False,
                            labelcolor=C_MUTED)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ============================================================ events
    def refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.menu_port.configure(values=ports or ["-- NO PORTS --"])
        if ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("-- NO PORTS --")

    def _on_connect_toggle(self) -> None:
        if self.worker is None:
            port = self.port_var.get()
            if not port or port.startswith("--"):
                self.q_logs.put(("ERR", "NO PORT SELECTED"))
                return
            try:
                self.worker = SerialWorker(port, int(self.baud_var.get()),
                                           self.q_frames, self.q_logs, self.q_events)
                self.worker.start()
            except serial.SerialException as exc:
                self.q_logs.put(("ERR", f"PORT OPEN FAILED -- {port}: {exc}"))
                self.worker = None
                return
            except Exception as exc:
                self.q_logs.put(("ERR", f"CONNECT ERROR -- {port}: {exc}"))
                self.worker = None
                return
            self.btn_link.configure(text="DISCONNECT", fg_color=C_PANEL_2,
                                    hover_color=C_BORDER)
        else:
            self.worker.stop()
            self.worker = None
            self.q_logs.put(("INFO", "LINK TERMINATED -- OPERATOR REQUEST"))
            self._set_link(False)
            self.btn_link.configure(text="CONNECT", fg_color=C_ACCENT,
                                    hover_color="#2563EB")

    def _on_view_change(self, _value: str) -> None:
        self.view_window = float(self.view_var.get().split()[0])

    def _on_apply(self) -> None:
        raw_t = self.ent_tilt.get().strip().replace(",", ".")
        raw_v = self.ent_vib.get().strip().replace(",", ".")
        try:
            t = float(raw_t)
            v = float(raw_v)
        except ValueError:
            self.q_logs.put(("ERR", "INVALID THRESHOLD FORMAT"))
            return
        t = max(TILT_LIMITS[0], min(TILT_LIMITS[1], t))
        v = max(VIB_LIMITS[0], min(VIB_LIMITS[1], v))
        self.tilt_thr, self.vib_thr = t, v
        self.ent_tilt.delete(0, "end"); self.ent_tilt.insert(0, f"{t:.2f}")
        self.ent_vib.delete(0, "end");  self.ent_vib.insert(0, f"{v:.2f}")
        self.thr_vib.set_ydata([v, v])
        self.thr_hi.set_ydata([t, t])
        self.thr_lo.set_ydata([-t, -t])
        if self.worker is not None:
            self.worker.send(f"SET:TILT:{t:.2f}")
            self.worker.send(f"SET:VIB:{v:.2f}")
        self.q_logs.put(("INFO", f"THRESHOLD PROFILE UPDATED -- TILT +-{t:.2f} deg | VIB {v:.2f} g"))

    def _on_calibrate(self) -> None:
        if self.worker is None:
            self.q_logs.put(("ERR", "CALIBRATION REQUIRES AN ACTIVE LINK"))
            return
        self.worker.send("CAL")
        self.btn_cal.configure(state="disabled")
        self.after(3000, lambda: self.btn_cal.configure(state="normal"))
        self.q_logs.put(("INFO", "BASELINE CAPTURE REQUESTED -- HOLD STRUCTURE AT REST (T = 1.0 s)"))

    # ======================================================== GUI states
    def _set_link(self, up: bool) -> None:
        self.online = up
        if up:
            self._t0 = time.time()
            self.lbl_session.configure(text="SESSION  --  UPT 00:00:00  |  EVENTS 0")
        self.lbl_dot.configure(text_color=C_EMERALD if up else C_MUTED)
        self.lbl_link.configure(text="ONLINE" if up else "OFFLINE",
                                text_color=C_TEXT if up else C_MUTED)
        state = "normal" if up else "disabled"
        self.btn_apply.configure(state=state)
        self.btn_cal.configure(state=state)
        if not up:
            self.lbl_rate.configure(text="")
            self.lbl_badge.configure(text="STANDBY", text_color=C_MUTED)
            self.lbl_badge_note.configure(text="AWAITING LINK", text_color=C_MUTED)
            self.critical = False
            self.stale = False

    def _set_badge(self, critical: bool) -> None:
        self.critical = critical
        if critical:
            self.lbl_badge.configure(text="THRESHOLD EXCEEDED",
                                     text_color="#FFFFFF", fg_color=C_CRIMSON)
            self.lbl_badge_note.configure(
                text="STRUCTURAL ANOMALY DETECTED", text_color=C_CRIMSON)
        else:
            self.lbl_badge.configure(text="STABLE", text_color=C_EMERALD,
                                     fg_color="transparent")
            self.lbl_badge_note.configure(
                text="ALL PARAMETERS WITHIN LIMITS", text_color=C_MUTED)

    # ============================================================= poll
    def _poll(self) -> None:
        # ---- telemetry drain
        try:
            while True:
                frame = self.q_frames.get_nowait()
                self._on_frame(frame)
        except queue.Empty:
            pass
        # ---- log drain
        try:
            while True:
                level, msg = self.q_logs.get_nowait()
                self._append_log(level, msg)
        except queue.Empty:
            pass
        # ---- control events
        try:
            while True:
                kind, payload = self.q_events.get_nowait()
                if kind == "link":
                    self._set_link(bool(payload))
        except queue.Empty:
            pass

        # ---- auto-reconnect: if the ESP32 rebooted, powered off, or the
        # BT SPP link dropped and the worker port closed itself, spin up a
        # fresh worker on the same port so the UI doesn't go dark.
        if (self.worker is None or not self.worker.active()) and self.online:
            self.q_logs.put(("ERR", "LINK LOST -- ATTEMPTING RECONNECT"))
            port = self.port_var.get()
            if port and not port.startswith("--"):
                try:
                    self.worker = SerialWorker(port, int(self.baud_var.get()),
                                                 self.q_frames, self.q_logs,
                                                 self.q_events)
                    self.worker.start()
                except Exception as exc:
                    self.q_logs.put(("ERR", f"RECONNECT FAILED -- {exc}"))
                    self._set_link(False)
            else:
                self._set_link(False)

        # ---- staleness watchdog
        if self.online and self.last_frame_t > 0:
            stale = (time.time() - self.last_frame_t) > STALE_S
            if stale != self.stale:
                self.stale = stale
                self.lbl_rate.configure(
                    text="NO DATA" if stale else "")
                self.lbl_rate.configure(
                    text_color=C_AMBER if stale else C_MUTED)

        # ---- draw
        self._tick += 1
        if self._tick % PLOT_EVERY == 0:
            self._draw_plot()
        self.after(POLL_MS, self._poll)

    def _on_frame(self, f: Frame) -> None:
        self.store.append(f)
        self.last_frame_t = time.time()
        if self.stale:
            self.stale = False
            self.lbl_rate.configure(text="", text_color=C_MUTED)

        # client-side threshold confirmation (device state is authoritative,
        # local check covers pathological frames)
        crit = (f.state == 1 or abs(f.pitch) >= self.tilt_thr
                or abs(f.roll) >= self.tilt_thr or f.vib >= self.vib_thr)
        if crit != self.critical:
            self._set_badge(crit)
            if crit:
                self.event_count += 1
                self._append_log("EVT", f"STRUCTURAL ANOMALY DETECTED -- "
                                        f"TILT T={f.pitch:+.2f},{f.roll:+.2f} deg | "
                                        f"VIB={f.vib:.3f} g")
            else:
                self._append_log("EVT", "PARAMETERS RETURNED WITHIN LIMITS -- "
                                        "ANOMALY CLEARED")

        self.cards[0].set(f"{f.pitch:+.2f}",
                          C_CRIMSON if abs(f.pitch) >= self.tilt_thr else C_TEXT)
        self.cards[1].set(f"{f.roll:+.2f}",
                          C_CRIMSON if abs(f.roll) >= self.tilt_thr else C_TEXT)
        self.cards[2].set(f"{f.vib:.3f} g",
                          C_CRIMSON if f.vib >= self.vib_thr else C_TEXT)

        # frame-rate + session counters
        now = time.time()
        self._rate.append(now)
        while self._rate and (now - self._rate[0]) > 1.0:
            self._rate.popleft()
        self.lbl_rate.configure(text=f"{len(self._rate)} f/s")
        self.lbl_session.configure(
            text=f"SESSION  --  UPT {time.strftime('%H:%M:%S', time.gmtime(now - self._t0))}"
                 f"  |  EVENTS {self.event_count}")

    # ============================================================== log
    def _log_tags(self) -> None:
        try:
            box = self.log_box._textbox  # underlying tk.Text
            for level, color in LOG_COLORS.items():
                box.tag_configure(level, foreground=color)
        except Exception:
            pass

    def _append_log(self, level: str, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log_box.configure(state="normal")
        tag = level if level in LOG_COLORS else "INFO"
        try:
            self.log_box.insert("end", f"[{ts}] {msg}\n", tag)
        except Exception:
            self.log_box.insert("end", f"[{ts}] {msg}\n")
        # cap history
        lines = int(self.log_box.index("end-1c").split(".")[0])
        if lines > LOG_CAP:
            self.log_box.delete("1.0", f"{lines - LOG_CAP}.0")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    # ============================================================= plot
    def _draw_plot(self) -> None:
        st = self.store
        if len(st.t) < 2:
            return
        t = np.asarray(st.t)
        t_rel = t - t[-1]
        w = self.view_window
        m = t_rel >= -w

        self.line_vib.set_data(t_rel[m], np.asarray(st.vib)[m])
        self.line_pitch.set_data(t_rel[m], np.asarray(st.pitch)[m])
        self.line_roll.set_data(t_rel[m], np.asarray(st.roll)[m])
        self.ax_vib.set_xlim(-w, 0.0)

        vm = np.asarray(st.vib)[m]
        tm = np.maximum(np.abs(np.asarray(st.pitch)[m]),
                        np.abs(np.asarray(st.roll)[m]))
        vmax = max((vm.max() * 1.25 if vm.size else 0.0),
                   self.vib_thr * 1.15, 0.05)
        tmax = max((tm.max() * 1.25 if tm.size else 0.0),
                   self.tilt_thr * 1.15, 1.00)
        # asymmetric autoscale: fast up, slow down
        self._v_ymax = vmax if vmax > self._v_ymax else self._v_ymax * 0.92
        self._t_ymax = tmax if tmax > self._t_ymax else self._t_ymax * 0.92
        self.ax_vib.set_ylim(0.0, self._v_ymax)
        self.ax_tilt.set_ylim(-self._t_ymax, self._t_ymax)

        self.canvas.draw_idle()

    # ============================================================ close
    def _on_close(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker.join(0.8)
        self.destroy()


def main() -> None:
    app = CommandCenter()
    app.mainloop()


if __name__ == "__main__":
    main()
