from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.ticker import AutoMinorLocator, MaxNLocator, MultipleLocator
from scipy.signal import find_peaks, windows


# ============================================================
# Gera figuras limpas para relatorio/TCC a partir dos CSVs.
#
# Entrada esperada: CSV com colunas tempo,x,y,z, em segundos e g.
# Saidas por coleta:
# - aceleracao no tempo;
# - FFT principal 0-400 Hz com picos anotados;
# - FFT geral 0-1000 Hz para visao ampla.
#
# Ajuste pelo terminal:
#   python gerar_figuras_tcc_enfoque.py --pasta coletas --saida figuras
# ============================================================

G_TO_MS2 = 9.80665
AXES = ("x", "y", "z")
AXIS_LABELS = {"x": "X", "y": "Y", "z": "Z"}
AXIS_COLORS = {"x": "#1f77b4", "y": "#ff7f0e", "z": "#2ca02c"}


@dataclass
class Collection:
    name: str
    path: Path
    time_s: np.ndarray
    signals_g: dict[str, np.ndarray]


def path_near_script(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return Path(__file__).resolve().parent / path


def normalize_time_seconds(time_raw: np.ndarray) -> np.ndarray:
    finite = time_raw[np.isfinite(time_raw)]
    if finite.size < 2:
        return time_raw.astype(float)

    diffs = np.diff(finite)
    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
    if diffs.size == 0:
        return time_raw.astype(float)

    dt = float(np.median(diffs))
    max_time = float(np.nanmax(finite))
    if max_time > 1000 or (dt > 0.01 and max_time > 100):
        return time_raw.astype(float) / 1000.0
    return time_raw.astype(float)


def load_collection(csv_path: Path, window_seconds: float | None) -> Collection:
    df = pd.read_csv(csv_path, on_bad_lines="skip", engine="python")
    df.columns = df.columns.str.strip().str.lower()

    required = {"tempo", "x", "y", "z"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"{csv_path.name}: colunas ausentes: {', '.join(sorted(missing))}")

    df = df[["tempo", "x", "y", "z"]].apply(pd.to_numeric, errors="coerce")
    time_s = normalize_time_seconds(df["tempo"].to_numpy(dtype=float))
    finite_time = time_s[np.isfinite(time_s)]
    if finite_time.size < 2:
        raise ValueError(f"{csv_path.name}: tempo insuficiente.")

    time_s = time_s - float(finite_time[0])
    if window_seconds is not None:
        mask = np.isfinite(time_s) & (time_s <= window_seconds)
        df = df.loc[mask].reset_index(drop=True)
        time_s = time_s[mask]

    signals_g = {axis: df[axis].to_numpy(dtype=float) for axis in AXES}
    return Collection(csv_path.stem, csv_path, time_s, signals_g)


def sample_rate_hz(time_s: np.ndarray) -> float:
    diffs = np.diff(time_s)
    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
    if diffs.size == 0:
        raise ValueError("Nao foi possivel calcular a taxa de amostragem.")
    return float(1.0 / np.median(diffs))


def centered_signal(signal_g: np.ndarray) -> np.ndarray:
    return signal_g - np.nanmean(signal_g)


def interpolate_missing(time_s: np.ndarray, signal_g: np.ndarray) -> np.ndarray:
    finite = np.isfinite(time_s) & np.isfinite(signal_g)
    if np.count_nonzero(finite) < 2:
        raise ValueError("Sinal com menos de duas amostras validas.")
    if np.all(finite):
        return signal_g.astype(float)
    return np.interp(time_s, time_s[finite], signal_g[finite])


def compute_fft(signal_g: np.ndarray, fs_hz: float) -> tuple[np.ndarray, np.ndarray]:
    signal_ms2 = np.asarray(signal_g, dtype=float) * G_TO_MS2
    signal_ms2 = signal_ms2 - float(np.mean(signal_ms2))
    window = windows.hann(signal_ms2.size, sym=False)
    freqs = np.fft.rfftfreq(signal_ms2.size, d=1.0 / fs_hz)
    spectrum = np.fft.rfft(signal_ms2 * window)
    amplitude = np.abs(spectrum) * 2.0 / np.sum(window)
    if amplitude.size:
        amplitude[0] = 0.0
    return freqs, amplitude


def detect_peak_rows(
    freqs: np.ndarray,
    amplitude: np.ndarray,
    limit_hz: float,
    peak_count: int,
    min_spacing_hz: float = 2.0,
) -> list[tuple[float, float, float]]:
    mask = (freqs > 0) & (freqs <= limit_hz) & np.isfinite(amplitude)
    f = freqs[mask]
    a = amplitude[mask]
    if f.size < 3:
        return []

    df = float(np.median(np.diff(f)))
    distance_bins = max(1, int(round(min_spacing_hz / max(df, 1e-12))))
    floor = float(np.median(a))
    prominence = max(float(np.max(a)) * 0.04, floor * 5.0, 1e-12)
    peaks, props = find_peaks(a, prominence=prominence, distance=distance_bins)

    if peaks.size == 0:
        peaks = np.asarray([int(np.argmax(a))])
        prominences = np.asarray([max(float(np.max(a) - floor), 0.0)])
    else:
        prominences = props.get("prominences", np.zeros(peaks.size))

    rows = [(float(f[idx]), float(a[idx]), float(prom)) for idx, prom in zip(peaks, prominences)]
    rows.sort(key=lambda item: item[1], reverse=True)
    return rows[:peak_count]


def decimation_slice(length: int, max_points: int) -> slice:
    if length <= max_points:
        return slice(None)
    return slice(None, None, int(math.ceil(length / max_points)))


def setup_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#111827",
            "axes.labelcolor": "#111827",
            "axes.titlesize": 14,
            "axes.labelsize": 11,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "font.size": 10,
            "savefig.facecolor": "white",
            "savefig.bbox": "tight",
        }
    )


def apply_axes_style(ax: plt.Axes) -> None:
    ax.grid(True, which="major", color="#9ca3af", alpha=0.24, linewidth=0.75)
    ax.grid(True, which="minor", color="#d1d5db", alpha=0.16, linewidth=0.45)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(axis="both", which="major", length=4, width=0.8)
    ax.tick_params(axis="both", which="minor", length=2.5, width=0.55)


def set_time_ticks(ax: plt.Axes, duration_s: float) -> None:
    step = 1.0 if duration_s <= 12.0 else 2.0
    ax.xaxis.set_major_locator(MultipleLocator(step))
    ax.xaxis.set_minor_locator(MultipleLocator(step / 2.0))


def set_frequency_ticks(ax: plt.Axes, limit_hz: float) -> None:
    if limit_hz <= 400:
        major_step, minor_step = 50.0, 10.0
    elif limit_hz <= 1000:
        major_step, minor_step = 100.0, 25.0
    else:
        major_step, minor_step = 200.0, 50.0

    ax.xaxis.set_major_locator(MultipleLocator(major_step))
    ax.xaxis.set_minor_locator(MultipleLocator(minor_step))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))


def collection_duration_s(collection: Collection) -> float:
    if collection.time_s.size < 2:
        return 0.0
    return float(collection.time_s[-1] - collection.time_s[0])


def duration_label(value: float) -> str:
    return f"{value:.1f}".replace(".", "p")


def save_time_figure(collection: Collection, output_dir: Path, scale_g: int, max_points: int, dpi: int) -> Path:
    duration_s = collection_duration_s(collection)
    output_path = output_dir / f"{collection.name}_01_aceleracao_tempo_{duration_label(duration_s)}s.png"
    ds = decimation_slice(collection.time_s.size, max_points)
    ac_by_axis = {axis: centered_signal(collection.signals_g[axis]) for axis in AXES}

    y_limit = 0.0
    for signal in ac_by_axis.values():
        finite = signal[np.isfinite(signal)]
        if finite.size:
            y_limit = max(y_limit, float(np.nanmax(np.abs(finite))))
    y_limit = max(y_limit * 1.06, 0.25)

    fig, axes = plt.subplots(3, 1, figsize=(12.8, 7.0), sharex=True)
    fig.suptitle(f"Aceleracao no tempo - {collection.name}", fontsize=16, fontweight="bold")

    for ax, axis in zip(axes, AXES):
        signal = ac_by_axis[axis]
        ax.plot(collection.time_s[ds], signal[ds], color=AXIS_COLORS[axis], linewidth=0.52, alpha=0.90)
        ax.axhline(0.0, color="#111827", linewidth=0.65, alpha=0.42)
        ax.set_ylabel(f"{AXIS_LABELS[axis]} (g)")
        ax.set_ylim(-y_limit, y_limit)
        apply_axes_style(ax)

    axes[-1].set_xlabel("Tempo (s)")
    axes[-1].set_xlim(float(collection.time_s[0]), float(collection.time_s[-1]))
    set_time_ticks(axes[-1], duration_s)
    fig.text(
        0.01,
        0.012,
        f"Sinal AC com media removida. Janela: {duration_s:.1f} s; "
        f"fs={sample_rate_hz(collection.time_s):.0f} Hz; faixa nominal: +/-{scale_g} g.",
        fontsize=9,
        color="#4b5563",
    )
    fig.tight_layout(rect=(0, 0.045, 1, 0.955))
    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)
    return output_path


def fft_data(collection: Collection) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    fs = sample_rate_hz(collection.time_s)
    return {
        axis: compute_fft(interpolate_missing(collection.time_s, collection.signals_g[axis]), fs)
        for axis in AXES
    }


def save_fft_main(
    collection: Collection,
    output_dir: Path,
    limit_hz: float,
    peak_count: int,
    dpi: int,
) -> tuple[Path, list[dict[str, float | str]]]:
    output_path = output_dir / f"{collection.name}_02_fft_0_{int(limit_hz)}hz.png"
    spectra = fft_data(collection)
    peak_rows: list[dict[str, float | str]] = []

    fig, axes = plt.subplots(3, 1, figsize=(12.8, 7.0), sharex=True)
    fig.suptitle(f"FFT da aceleracao - {collection.name}", fontsize=16, fontweight="bold")

    for ax, axis in zip(axes, AXES):
        freqs, amplitude = spectra[axis]
        mask = freqs <= limit_hz
        ax.plot(freqs[mask], amplitude[mask], color=AXIS_COLORS[axis], linewidth=0.88)

        for rank, (freq, value, prominence) in enumerate(detect_peak_rows(freqs, amplitude, limit_hz, peak_count), start=1):
            peak_rows.append(
                {
                    "coleta": collection.name,
                    "eixo": AXIS_LABELS[axis],
                    "rank": rank,
                    "frequencia_hz": freq,
                    "amplitude_m_s2": value,
                    "prominencia": prominence,
                    "rpm_se_1x": freq * 60.0,
                    "rpm_se_2x": freq * 30.0,
                }
            )
            ax.scatter([freq], [value], color="#111827", s=22, zorder=4)
            ax.annotate(
                f"{freq:.1f} Hz",
                xy=(freq, value),
                xytext=(6, 10),
                textcoords="offset points",
                fontsize=8.5,
                color="#111827",
                bbox={"boxstyle": "round,pad=0.18", "facecolor": "white", "edgecolor": "#d1d5db", "alpha": 0.92},
            )

        if np.any(mask):
            y_max = float(np.nanmax(amplitude[mask]))
            if y_max > 0:
                ax.set_ylim(0.0, y_max * 1.24)
        ax.set_ylabel(f"{AXIS_LABELS[axis]} (m/s2)")
        ax.set_xlim(0.0, limit_hz)
        set_frequency_ticks(ax, limit_hz)
        apply_axes_style(ax)

    axes[-1].set_xlabel("Frequencia (Hz)")
    fig.text(
        0.01,
        0.012,
        f"FFT com media removida, janela Hann e amplitude corrigida. Faixa exibida: 0-{limit_hz:.0f} Hz.",
        fontsize=9,
        color="#4b5563",
    )
    fig.tight_layout(rect=(0, 0.045, 1, 0.955))
    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)
    return output_path, peak_rows


def save_fft_overview(collection: Collection, output_dir: Path, limit_hz: float, dpi: int) -> Path:
    output_path = output_dir / f"{collection.name}_03_fft_0_{int(limit_hz)}hz.png"
    spectra = fft_data(collection)

    fig, axes = plt.subplots(3, 1, figsize=(12.8, 7.0), sharex=True)
    fig.suptitle(f"FFT geral da aceleracao - {collection.name}", fontsize=16, fontweight="bold")

    for ax, axis in zip(axes, AXES):
        freqs, amplitude = spectra[axis]
        mask = freqs <= limit_hz
        ax.plot(freqs[mask], amplitude[mask], color=AXIS_COLORS[axis], linewidth=0.72)

        if np.any(mask):
            y_max = float(np.nanmax(amplitude[mask]))
            if y_max > 0:
                ax.set_ylim(0.0, y_max * 1.12)
        ax.set_ylabel(f"{AXIS_LABELS[axis]} (m/s2)")
        ax.set_xlim(0.0, limit_hz)
        set_frequency_ticks(ax, limit_hz)
        apply_axes_style(ax)

    axes[-1].set_xlabel("Frequencia (Hz)")
    fig.text(
        0.01,
        0.012,
        f"Visao geral da FFT ate {limit_hz:.0f} Hz. Use a FFT 0-400 Hz para destacar os picos principais.",
        fontsize=9,
        color="#4b5563",
    )
    fig.tight_layout(rect=(0, 0.045, 1, 0.955))
    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)
    return output_path


def normalize_collection_names(selected_names: str | None) -> set[str] | None:
    if not selected_names:
        return None

    result = set()
    for item in (part.strip() for part in selected_names.split(",")):
        if not item:
            continue
        if item.startswith("coleta_"):
            result.add(Path(item).stem)
        elif item.isdigit():
            result.add(f"coleta_{int(item):05d}")
        else:
            result.add(Path(item).stem)
    return result


def select_files(input_dir: Path, selected_names: str | None) -> list[Path]:
    files = sorted(input_dir.glob("coleta_*.csv"))
    wanted = normalize_collection_names(selected_names)
    if wanted is None:
        return files
    return [path for path in files if path.stem in wanted]


def write_index(output_dir: Path, generated: list[Path], peaks: pd.DataFrame) -> Path:
    path = output_dir / "indice_figuras.md"
    lines = ["# Figuras geradas", "", "## Arquivos", ""]
    lines.extend(f"- {item.name}" for item in generated)

    if not peaks.empty:
        lines.extend(
            [
                "",
                "## Picos anotados na FFT 0-400 Hz",
                "",
                "| coleta | eixo | rank | frequencia_hz | amplitude_m_s2 | rpm_se_2x |",
                "|---|---|---:|---:|---:|---:|",
            ]
        )
        for _, row in peaks.iterrows():
            lines.append(
                f"| {row['coleta']} | {row['eixo']} | {int(row['rank'])} | "
                f"{float(row['frequencia_hz']):.2f} | {float(row['amplitude_m_s2']):.3f} | {float(row['rpm_se_2x']):.1f} |"
            )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def process(args: argparse.Namespace) -> None:
    setup_style()
    input_dir = path_near_script(args.pasta)
    output_dir = path_near_script(args.saida)
    output_dir.mkdir(parents=True, exist_ok=True)
    window = None if args.janela <= 0 else args.janela

    csv_files = select_files(input_dir, args.coletas)
    if not csv_files:
        raise SystemExit(f"Nenhum CSV encontrado em {input_dir}.")

    generated: list[Path] = []
    peak_rows: list[dict[str, float | str]] = []
    for csv_path in csv_files:
        collection = load_collection(csv_path, window)
        generated.append(save_time_figure(collection, output_dir, args.escala_g, args.max_pontos_tempo, args.dpi))
        fft_path, peaks = save_fft_main(collection, output_dir, args.fft_limite, args.picos, args.dpi)
        generated.append(fft_path)
        peak_rows.extend(peaks)
        if args.fft_geral_limite > args.fft_limite:
            generated.append(save_fft_overview(collection, output_dir, args.fft_geral_limite, args.dpi))

    peaks = pd.DataFrame(peak_rows)
    peaks.to_csv(output_dir / "picos_fft.csv", index=False)
    generated.append(write_index(output_dir, generated, peaks))

    print("Figuras geradas:")
    for item in generated:
        print(f"  {item}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Gera figuras de aceleracao e FFT a partir das coletas CSV.")
    parser.add_argument("--pasta", default="coletas", help="Pasta com CSVs. Padrao: coletas")
    parser.add_argument("--saida", default="figuras", help="Pasta de saida. Padrao: figuras")
    parser.add_argument("--coletas", help="Opcional. Ex.: 1,2 ou coleta_00001,coleta_00002")
    parser.add_argument("--janela", type=float, default=10.0, help="Janela em segundos. Use 0 para arquivo completo.")
    parser.add_argument("--escala-g", type=int, default=8, choices=[2, 4, 8, 16], help="Faixa configurada no LIS3DH.")
    parser.add_argument("--fft-limite", type=float, default=400.0, help="Limite da FFT principal.")
    parser.add_argument("--fft-geral-limite", type=float, default=1000.0, help="Limite da FFT geral.")
    parser.add_argument("--picos", type=int, default=2, help="Picos anotados por eixo na FFT principal.")
    parser.add_argument("--dpi", type=int, default=260, help="Resolucao das figuras.")
    parser.add_argument("--max-pontos-tempo", type=int, default=8000, help="Maximo de pontos no grafico de tempo.")
    process(parser.parse_args())


if __name__ == "__main__":
    main()
