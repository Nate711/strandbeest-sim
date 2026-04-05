from __future__ import annotations

import time
import tkinter as tk
from tkinter import ttk

from .geometry import GeometryError, LENGTH_NAMES, LinkageParams, Pose, foot_path, solve_pose

WINDOW_SIZE = "1420x920"
CANVAS_BG = "#f8f5ef"
TRACE_COLOR = "#d5d0c7"
FRAME_COLOR = "#666666"
LINK_COLOR = "#1f4b99"
CRANK_COLOR = "#d95f02"
NODE_COLOR = "#111111"
LABEL_COLOR = "#333333"

SLIDER_RANGES: dict[str, tuple[float, float]] = {
    "a": (10.0, 90.0),
    "b": (15.0, 110.0),
    "c": (15.0, 110.0),
    "d": (15.0, 110.0),
    "e": (20.0, 130.0),
    "f": (15.0, 110.0),
    "g": (15.0, 110.0),
    "h": (20.0, 150.0),
    "i": (20.0, 120.0),
    "j": (20.0, 120.0),
    "k": (20.0, 140.0),
    "l": (0.0, 40.0),
    "m": (5.0, 45.0),
}

LINKS = (
    ("ground", "upper_joint"),
    ("ground", "lower_joint"),
    ("ground", "left_joint"),
    ("crank_center", "crank_end"),
    ("crank_end", "upper_joint"),
    ("crank_end", "lower_joint"),
    ("upper_joint", "left_joint"),
    ("left_joint", "knee_joint"),
    ("lower_joint", "knee_joint"),
    ("lower_joint", "foot"),
    ("knee_joint", "foot"),
)

SEGMENT_LABELS = {
    "b": ("ground", "upper_joint"),
    "c": ("ground", "lower_joint"),
    "d": ("ground", "left_joint"),
    "e": ("left_joint", "upper_joint"),
    "f": ("left_joint", "knee_joint"),
    "g": ("lower_joint", "knee_joint"),
    "h": ("knee_joint", "foot"),
    "i": ("lower_joint", "foot"),
    "j": ("crank_end", "upper_joint"),
    "k": ("crank_end", "lower_joint"),
    "m": ("crank_center", "crank_end"),
}

JOINT_LABELS = {
    "ground": "O2",
    "crank_center": "O4",
    "crank_end": "X",
    "upper_joint": "W",
    "lower_joint": "U",
    "left_joint": "V",
    "knee_joint": "T",
    "foot": "S",
}


class StrandbeestApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Strandbeest Linkage Simulator")
        self.geometry(WINDOW_SIZE)
        self.minsize(1200, 760)

        self.length_vars = {
            name: tk.DoubleVar(value=getattr(LinkageParams(), name))
            for name in LENGTH_NAMES
        }
        self.angle_var = tk.DoubleVar(value=0.0)
        self.speed_var = tk.DoubleVar(value=48.0)
        self.trace_samples_var = tk.IntVar(value=220)
        self.show_trace_var = tk.BooleanVar(value=True)

        self.status_var = tk.StringVar(value="Ready")
        self._playing = False
        self._last_tick = time.perf_counter()
        self._pose: Pose | None = None
        self._trace_cache_key: tuple[tuple[float, ...], int] | None = None
        self._trace_cache: list[tuple[float, float]] = []
        self._viewport_cache_key: tuple[float, ...] | None = None
        self._viewport_points: list[tuple[float, float]] = []

        self._build_ui()
        self.after(16, self._tick)
        self.after(1, self.redraw)

    def _build_ui(self) -> None:
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        controls = ttk.Frame(self, padding=12)
        controls.grid(row=0, column=0, sticky="nsw")
        controls.columnconfigure(0, weight=1)

        canvas_frame = ttk.Frame(self, padding=(0, 12, 12, 12))
        canvas_frame.grid(row=0, column=1, sticky="nsew")
        canvas_frame.columnconfigure(0, weight=1)
        canvas_frame.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, background=CANVAS_BG, highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        status = ttk.Label(canvas_frame, textvariable=self.status_var, anchor="w")
        status.grid(row=1, column=0, sticky="ew", pady=(8, 0))

        ttk.Label(controls, text="Motion", font=("SF Pro Display", 14, "bold")).grid(
            row=0, column=0, sticky="w", pady=(0, 8)
        )

        buttons = ttk.Frame(controls)
        buttons.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(buttons, text="Play / Pause", command=self.toggle_play).grid(row=0, column=0, sticky="ew")
        ttk.Button(buttons, text="Reset Defaults", command=self.reset_defaults).grid(
            row=0, column=1, sticky="ew", padx=(8, 0)
        )
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)

        self._make_slider(controls, "Crank Angle", self.angle_var, 0.0, 360.0, row=2, resolution=0.1)
        self._make_slider(controls, "Speed (deg/s)", self.speed_var, -240.0, 240.0, row=3, resolution=1.0)

        ttk.Checkbutton(
            controls,
            text="Show Foot Trace",
            variable=self.show_trace_var,
            command=self.redraw,
        ).grid(row=4, column=0, sticky="w", pady=(4, 0))

        self._make_slider(
            controls,
            "Trace Samples",
            self.trace_samples_var,
            60,
            420,
            row=5,
            resolution=10,
        )

        ttk.Label(controls, text="Lengths", font=("SF Pro Display", 14, "bold")).grid(
            row=6, column=0, sticky="w", pady=(14, 8)
        )

        row = 7
        for name in LENGTH_NAMES:
            low, high = SLIDER_RANGES[name]
            self._make_slider(controls, f"{name} = {getattr(LinkageParams(), name):.1f}", self.length_vars[name], low, high, row)
            row += 1

        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def _make_slider(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.Variable,
        low: float,
        high: float,
        row: int,
        resolution: float = 0.1,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky="ew", pady=2)
        frame.columnconfigure(0, weight=1)
        value_label = ttk.Label(frame, text=label)
        value_label.grid(row=0, column=0, sticky="w")

        def on_change(_value: str) -> None:
            raw_value = float(variable.get())
            if isinstance(variable, tk.IntVar):
                value_label.config(text=f"{label.split('=')[0].strip()}: {int(raw_value)}")
            elif label.startswith(tuple(LENGTH_NAMES)):
                key = label[0]
                value_label.config(text=f"{key}: {raw_value:.1f}")
            else:
                value_label.config(text=f"{label.split(':')[0]}: {raw_value:.1f}")
            self.redraw()

        scale = tk.Scale(
            frame,
            variable=variable,
            from_=low,
            to=high,
            orient="horizontal",
            resolution=resolution,
            showvalue=False,
            command=on_change,
            length=260,
        )
        scale.grid(row=1, column=0, sticky="ew")
        on_change(str(variable.get()))

    def current_params(self) -> LinkageParams:
        return LinkageParams(**{name: float(var.get()) for name, var in self.length_vars.items()})

    def reset_defaults(self) -> None:
        defaults = LinkageParams()
        for name in LENGTH_NAMES:
            self.length_vars[name].set(getattr(defaults, name))
        self.angle_var.set(0.0)
        self.speed_var.set(48.0)
        self.trace_samples_var.set(220)
        self.show_trace_var.set(True)
        self._pose = None
        self._trace_cache_key = None
        self._viewport_cache_key = None
        self.redraw()

    def toggle_play(self) -> None:
        self._playing = not self._playing
        self._last_tick = time.perf_counter()

    def _tick(self) -> None:
        now = time.perf_counter()
        dt = now - self._last_tick
        self._last_tick = now
        if self._playing:
            angle = (float(self.angle_var.get()) + float(self.speed_var.get()) * dt) % 360.0
            self.angle_var.set(angle)
            self.redraw()
        self.after(16, self._tick)

    def _trace_path(self, params: LinkageParams) -> list[tuple[float, float]]:
        samples = int(self.trace_samples_var.get())
        key = (params.as_key(), samples)
        if key != self._trace_cache_key:
            self._trace_cache = foot_path(params, samples=samples)
            self._trace_cache_key = key
        return self._trace_cache

    def _viewport_points_for_params(self, params: LinkageParams) -> list[tuple[float, float]]:
        key = params.as_key()
        if key != self._viewport_cache_key:
            sweep_points: list[tuple[float, float]] = []
            previous: Pose | None = None
            for index in range(180):
                angle = 360.0 * index / 180.0
                previous = solve_pose(params, angle, previous)
                sweep_points.extend(getattr(previous, name) for name in JOINT_LABELS)
            self._viewport_points = sweep_points
            self._viewport_cache_key = key
        return self._viewport_points

    def redraw(self) -> None:
        self.canvas.delete("all")

        params = self.current_params()
        angle = float(self.angle_var.get())

        try:
            pose = solve_pose(params, angle, self._pose)
            self._pose = pose
            trace = self._trace_path(params) if self.show_trace_var.get() else []
            self.status_var.set(
                f"Angle {angle:6.1f} deg   Foot ({pose.foot[0]:6.2f}, {pose.foot[1]:6.2f})   Speed {self.speed_var.get():6.1f} deg/s"
            )
            self._draw_scene(params, pose, trace)
        except GeometryError as exc:
            self._pose = None
            self.status_var.set(f"Infeasible geometry: {exc}")
            self._draw_reference(params)

    def _draw_scene(self, params: LinkageParams, pose: Pose, trace: list[tuple[float, float]]) -> None:
        world_points = self._viewport_points_for_params(params)
        transform = self._transform(world_points)

        if trace:
            trace_points: list[float] = []
            for point in trace:
                sx, sy = transform(point)
                trace_points.extend((sx, sy))
            self.canvas.create_line(*trace_points, fill=TRACE_COLOR, width=2, smooth=True)

        self._draw_fixed_offsets(transform, pose)

        for start, end in LINKS:
            sx, sy = transform(getattr(pose, start))
            ex, ey = transform(getattr(pose, end))
            color = CRANK_COLOR if {"crank_center", "crank_end"} == {start, end} else LINK_COLOR
            width = 4 if color == CRANK_COLOR else 3
            self.canvas.create_line(sx, sy, ex, ey, fill=color, width=width, capstyle="round")

        for label, (start, end) in SEGMENT_LABELS.items():
            self._draw_segment_label(transform, getattr(pose, start), getattr(pose, end), label)

        for name, label in JOINT_LABELS.items():
            x, y = transform(getattr(pose, name))
            radius = 5 if name != "foot" else 6
            fill = CRANK_COLOR if name in {"crank_center", "crank_end"} else NODE_COLOR
            self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=fill, outline="")
            self.canvas.create_text(x + 14, y - 14, text=label, fill=LABEL_COLOR, font=("SF Pro Text", 11, "bold"))

        self.canvas.create_text(
            22,
            22,
            anchor="nw",
            text="Theo Jansen linkage",
            fill=LABEL_COLOR,
            font=("SF Pro Display", 18, "bold"),
        )

    def _draw_reference(self, params: LinkageParams) -> None:
        points = [(0.0, 0.0), (params.a, params.l), (params.a + params.m, params.l)]
        transform = self._transform(self._viewport_points_for_params(params))
        reference_pose = Pose(
            ground=points[0],
            crank_center=points[1],
            crank_end=points[2],
            upper_joint=points[0],
            lower_joint=points[0],
            left_joint=points[0],
            knee_joint=points[0],
            foot=points[0],
        )
        self._draw_fixed_offsets(transform, reference_pose)
        crank_center = transform(points[1])
        crank_tip = transform(points[2])
        self.canvas.create_line(*crank_center, *crank_tip, fill=CRANK_COLOR, width=4)
        self._draw_segment_label(transform, points[1], points[2], "m")

    def _draw_fixed_offsets(self, transform, pose: Pose) -> None:
        ground = pose.ground
        crank_center = pose.crank_center
        corner = (crank_center[0], ground[1])

        ground_xy = transform(ground)
        corner_xy = transform(corner)
        crank_center_xy = transform(crank_center)

        self.canvas.create_line(*ground_xy, *corner_xy, fill=FRAME_COLOR, width=2, dash=(6, 4))
        self.canvas.create_line(*corner_xy, *crank_center_xy, fill=FRAME_COLOR, width=2, dash=(6, 4))

        self._draw_segment_label(transform, ground, corner, "a", dy=-14)
        self._draw_segment_label(transform, corner, crank_center, "l", dx=14)

    def _draw_segment_label(
        self,
        transform,
        start: tuple[float, float],
        end: tuple[float, float],
        label: str,
        dx: float = 0.0,
        dy: float = 0.0,
    ) -> None:
        sx, sy = transform(start)
        ex, ey = transform(end)
        mx = (sx + ex) / 2.0
        my = (sy + ey) / 2.0

        if dx == 0.0 and dy == 0.0:
            vx = ex - sx
            vy = ey - sy
            length = max((vx * vx + vy * vy) ** 0.5, 1.0)
            dx = -vy / length * 12.0
            dy = vx / length * 12.0

        self.canvas.create_text(
            mx + dx,
            my + dy,
            text=label,
            fill=LABEL_COLOR,
            font=("SF Pro Text", 11, "bold"),
        )

    def _transform(self, world_points: list[tuple[float, float]]):
        width = max(self.canvas.winfo_width(), 100)
        height = max(self.canvas.winfo_height(), 100)
        margin = 60.0

        xs = [point[0] for point in world_points]
        ys = [point[1] for point in world_points]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)

        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)
        scale *= 0.9

        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0

        def transform(point: tuple[float, float]) -> tuple[float, float]:
            x = width / 2.0 + (point[0] - center_x) * scale
            y = height / 2.0 - (point[1] - center_y) * scale
            return x, y

        return transform


def main() -> None:
    app = StrandbeestApp()
    app.mainloop()
