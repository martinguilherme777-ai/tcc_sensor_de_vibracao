from __future__ import annotations

import argparse
import csv
import math
import re
import struct
import time
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from xml.sax.saxutils import escape


# ============================================================
# Salva no computador os dados enviados pelo ESP32 receptor.
# O receptor envia frames binarios; este script reconstrui os
# pacotes, salva CSV/XLSX e marca pacotes ausentes como NaN.
#
# Ajuste por argumentos de linha de comando:
#   python salvar_coleta_10s.py --porta auto --saida coletas --escala-g 8
# Use --listar-portas para ver as portas seriais disponiveis.
# ============================================================

SERIAL_BAUD = 2_000_000
WINDOW_SECONDS = 10.0
DEFAULT_TIMEOUT_SECONDS = 30.0
EXPECTED_SAMPLE_RATE_HZ = 2000
EXPECTED_SAMPLES_PER_AXIS = int(EXPECTED_SAMPLE_RATE_HZ * WINDOW_SECONDS)
DEFAULT_ACCEL_RANGE_G = 8
RAW_TO_G_BY_SCALE = {
    2: 0.016,
    4: 0.032,
    8: 0.064,
    16: 0.192,
}
ACCEL_RANGE_G = DEFAULT_ACCEL_RANGE_G
RAW_TO_G = RAW_TO_G_BY_SCALE[DEFAULT_ACCEL_RANGE_G]
DATA_MAGIC = 0x31415845
SERIAL_FRAME_MAGIC = 0x5AA5
PACKET_HEADER = struct.Struct("<IHHHIHHIH")
FRAME_HEADER = struct.Struct("<HH")
FRAME_CHECKSUM = struct.Struct("<H")
PACKET_HEADER_SIZE = PACKET_HEADER.size
SAMPLE_SIZE = 3
RECEPTOR_ID = "ESP_RECEPTOR"
COLLECTION_NAME_RE = re.compile(r"^coleta_(\d+)$")
PORT_CACHE_NAME = ".porta_receptor_cache.txt"
KNOWN_USB_SERIAL_IDS = {
    (0x10C4, 0xEA60),  # CP210x
    (0x1A86, 0x7523),  # CH340
    (0x1A86, 0x55D4),  # CH910x
    (0x0403, 0x6001),  # FTDI
    (0x303A, 0x1001),  # ESP32 USB/JTAG em algumas placas
}
USB_SERIAL_KEYWORDS = (
    "esp",
    "cp210",
    "ch340",
    "ch910",
    "silicon labs",
    "usb serial",
    "usb-serial",
    "uart",
    "wch",
    "ftdi",
)


def raw_to_g_for_scale(scale_g: int) -> float:
    try:
        return RAW_TO_G_BY_SCALE[scale_g]
    except KeyError as exc:
        valid = ", ".join(str(value) for value in sorted(RAW_TO_G_BY_SCALE))
        raise SystemExit(f"Escala invalida: {scale_g}. Use uma destas: {valid} g.") from exc


def accel_range_text(scale_g: int) -> str:
    return f"+/-{scale_g} g"


def path_near_script(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return Path(__file__).resolve().parent / path


def port_text(port_info: object) -> str:
    parts = [
        getattr(port_info, "device", ""),
        getattr(port_info, "description", ""),
        getattr(port_info, "manufacturer", ""),
        getattr(port_info, "hwid", ""),
    ]
    return " | ".join(str(part) for part in parts if part)


def port_score(port_info: object) -> int:
    text = port_text(port_info).lower()
    score = 0

    if "bluetooth" in text:
        return -100

    if any(keyword in text for keyword in USB_SERIAL_KEYWORDS):
        score += 10

    vid = getattr(port_info, "vid", None)
    pid = getattr(port_info, "pid", None)
    if vid is not None and pid is not None:
        score += 3
        if (int(vid), int(pid)) in KNOWN_USB_SERIAL_IDS:
            score += 20

    device = str(getattr(port_info, "device", "")).upper()
    if device.startswith("COM"):
        score += 1

    return score


def sorted_serial_ports(ports: list[object]) -> list[object]:
    def com_number(port_info: object) -> int:
        device = str(getattr(port_info, "device", "")).upper()
        if device.startswith("COM") and device[3:].isdigit():
            return int(device[3:])
        return 9999

    return sorted(ports, key=lambda item: (-port_score(item), com_number(item), str(getattr(item, "device", ""))))


def print_available_ports(ports: list[object]) -> None:
    print("Portas encontradas:")
    for index, item in enumerate(sorted_serial_ports(ports), start=1):
        print(f"  {index}) {port_text(item)}")


def cache_path() -> Path:
    return Path(__file__).resolve().parent / PORT_CACHE_NAME


def read_cached_port() -> str | None:
    path = cache_path()
    if not path.exists():
        return None

    cached = path.read_text(encoding="utf-8").strip()
    return cached or None


def write_cached_port(port: str) -> None:
    cache_path().write_text(f"{port}\n", encoding="utf-8")


def port_is_available(port: str, ports: list[object]) -> bool:
    wanted = port.upper()
    return any(str(getattr(item, "device", "")).upper() == wanted for item in ports)


def find_port_info(port: str, ports: list[object]) -> object | None:
    wanted = port.upper()
    for item in ports:
        if str(getattr(item, "device", "")).upper() == wanted:
            return item
    return None


def detect_receptor_port(ports: list[object], baud: int) -> str | None:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Instale o pyserial: pip install pyserial") from exc

    for item in sorted_serial_ports(ports):
        device = str(getattr(item, "device", ""))
        try:
            with serial.Serial(device, baudrate=baud, timeout=0.2, write_timeout=0.5) as esp32:
                # Ao abrir a porta, muitos ESP32 reiniciam.
                time.sleep(2.0)
                esp32.reset_input_buffer()

                for _ in range(3):
                    esp32.write(b"ID?\n")
                    esp32.flush()

                    deadline = time.monotonic() + 0.8
                    while time.monotonic() < deadline:
                        raw_line = esp32.readline()
                        if not raw_line:
                            continue

                        line = raw_line.decode("utf-8", errors="replace").strip()
                        if line == RECEPTOR_ID or line == "aguardando dados":
                            return device
        except Exception:
            continue

    return None


def choose_from_list(ports: list[object]) -> str:
    ordered_ports = sorted_serial_ports(ports)
    print_available_ports(ordered_ports)

    while True:
        answer = input("Digite o numero da porta do ESP32 receptor: ").strip()
        if answer.isdigit():
            index = int(answer)
            if 1 <= index <= len(ordered_ports):
                return str(getattr(ordered_ports[index - 1], "device"))
        print("Opcao invalida.")


def choose_serial_port(port: str | None, baud: int, list_only: bool = False) -> str:
    if port and port.lower() != "auto":
        write_cached_port(port)
        return port

    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise SystemExit("Instale o pyserial: pip install pyserial") from exc

    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("Nenhuma porta serial encontrada. Conecte o ESP32 receptor ao PC.")

    if list_only:
        print_available_ports(ports)
        raise SystemExit(0)

    cached_port = read_cached_port()
    cached_info = find_port_info(cached_port, ports) if cached_port else None
    if cached_port and cached_info is not None and port_score(cached_info) > 0:
        print(f"Usando porta salva anteriormente: {cached_port}")
        return cached_port

    detected = detect_receptor_port(ports, baud)
    if detected:
        print(f"Porta do ESP32 receptor detectada automaticamente: {detected}")
        write_cached_port(detected)
        return detected

    likely_ports = [item for item in ports if port_score(item) > 0]
    if len(likely_ports) == 1:
        device = str(getattr(likely_ports[0], "device"))
        print(f"Porta serial escolhida automaticamente: {device}")
        write_cached_port(device)
        return device

    if not likely_ports:
        print_available_ports(ports)
        raise SystemExit(
            "Nenhuma porta USB do ESP32 foi encontrada. Conecte o ESP32 receptor pelo cabo USB, "
            "instale o driver CP210x/CH340 se necessario e feche o Serial Monitor da Arduino IDE."
        )

    print("Nao consegui identificar sozinho qual porta e o receptor.")
    selected = choose_from_list(likely_ports)
    write_cached_port(selected)
    return selected


def next_collection_paths(output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)

    last_index = 0
    for path in output_dir.glob("coleta_*.*"):
        if path.suffix.lower() not in {".csv", ".xlsx"}:
            continue

        match = COLLECTION_NAME_RE.match(path.stem)
        if match:
            last_index = max(last_index, int(match.group(1)))

    index = last_index + 1
    while True:
        csv_path = output_dir / f"coleta_{index:05d}.csv"
        xlsx_path = output_dir / f"coleta_{index:05d}.xlsx"
        if not csv_path.exists() and not xlsx_path.exists():
            return csv_path, xlsx_path
        index += 1


def parse_data_line(line: str) -> tuple[float, float, float, float] | None:
    parts = line.split(",")
    if len(parts) != 4:
        return None

    try:
        tempo = float(parts[0])
        x = float(parts[1])
        y = float(parts[2])
        z = float(parts[3])
    except ValueError:
        return None

    return tempo, x, y, z


def checksum16(data: bytes) -> int:
    return sum(data) & 0xFFFF


def signed_int8(value: int) -> int:
    return value - 256 if value > 127 else value


def create_empty_rows(total_samples: int, sample_rate_hz: int) -> list[tuple[float, float, float, float]]:
    missing = float("nan")
    return [(sample_index / sample_rate_hz, missing, missing, missing) for sample_index in range(total_samples)]


def decode_data_packet(payload: bytes) -> dict[str, object] | None:
    if len(payload) < PACKET_HEADER_SIZE:
        return None

    (
        magic,
        block_id,
        packet_index,
        total_packets,
        first_sample,
        sample_count,
        sample_rate_hz,
        total_samples,
        duration_seconds,
    ) = PACKET_HEADER.unpack_from(payload, 0)

    expected_len = PACKET_HEADER_SIZE + sample_count * SAMPLE_SIZE
    if len(payload) != expected_len:
        return None

    if (
        magic != DATA_MAGIC
        or sample_rate_hz != EXPECTED_SAMPLE_RATE_HZ
        or duration_seconds != int(WINDOW_SECONDS)
        or total_samples != EXPECTED_SAMPLES_PER_AXIS
        or sample_count <= 0
        or first_sample + sample_count > total_samples
        or packet_index >= total_packets
    ):
        return None

    samples: list[tuple[float, float, float]] = []
    offset = PACKET_HEADER_SIZE
    for _ in range(sample_count):
        x_raw = signed_int8(payload[offset])
        y_raw = signed_int8(payload[offset + 1])
        z_raw = signed_int8(payload[offset + 2])
        offset += SAMPLE_SIZE
        samples.append((x_raw * RAW_TO_G, y_raw * RAW_TO_G, z_raw * RAW_TO_G))

    return {
        "block_id": block_id,
        "packet_index": packet_index,
        "total_packets": total_packets,
        "first_sample": first_sample,
        "sample_count": sample_count,
        "sample_rate_hz": sample_rate_hz,
        "total_samples": total_samples,
        "samples": samples,
    }


def store_packet_rows(rows: list[tuple[float, float, float, float]], packet: dict[str, object]) -> None:
    first_sample = int(packet["first_sample"])
    samples = packet["samples"]
    if not isinstance(samples, list):
        return

    for offset, values in enumerate(samples):
        sample_index = first_sample + offset
        if sample_index >= len(rows):
            break

        tempo = rows[sample_index][0]
        x, y, z = values
        rows[sample_index] = (tempo, float(x), float(y), float(z))


def parse_serial_frames(buffer: bytearray) -> list[bytes]:
    frames: list[bytes] = []
    magic_bytes = struct.pack("<H", SERIAL_FRAME_MAGIC)

    while True:
        start = buffer.find(magic_bytes)
        if start < 0:
            if len(buffer) > 4096:
                del buffer[:-1]
            break

        if start > 0:
            del buffer[:start]

        if len(buffer) < FRAME_HEADER.size:
            break

        magic, length = FRAME_HEADER.unpack_from(buffer, 0)
        if magic != SERIAL_FRAME_MAGIC or length < PACKET_HEADER_SIZE or length > 512:
            del buffer[0]
            continue

        frame_size = FRAME_HEADER.size + length + FRAME_CHECKSUM.size
        if len(buffer) < frame_size:
            break

        payload_start = FRAME_HEADER.size
        payload_end = payload_start + length
        payload = bytes(buffer[payload_start:payload_end])
        received_checksum = FRAME_CHECKSUM.unpack_from(buffer, payload_end)[0]
        del buffer[:frame_size]

        if checksum16(payload) == received_checksum:
            frames.append(payload)

    return frames


def extract_text_lines_before_frame(buffer: bytearray) -> list[str]:
    lines: list[str] = []
    magic_bytes = struct.pack("<H", SERIAL_FRAME_MAGIC)

    while True:
        magic_pos = buffer.find(magic_bytes)
        search_end = len(buffer) if magic_pos < 0 else magic_pos
        newline_pos = buffer.find(b"\n", 0, search_end)

        if newline_pos < 0:
            break

        raw_line = bytes(buffer[: newline_pos + 1])
        del buffer[: newline_pos + 1]
        line = raw_line.decode("utf-8", errors="replace").strip()
        if line:
            lines.append(line)

    return lines


def capture_binary_rows(esp32: object, timeout_s: float) -> list[tuple[float, float, float, float]]:
    buffer = bytearray()
    rows: list[tuple[float, float, float, float]] = []
    legacy_rows: list[tuple[float, float, float, float]] = []
    legacy_csv_mode = False
    seen_packets: set[int] = set()
    active_block_id: int | None = None
    total_packets: int | None = None
    last_packet_time = time.monotonic()
    start_wait = last_packet_time

    print("Aguardando pacotes binarios...")

    while True:
        waiting = getattr(esp32, "in_waiting", 0)
        chunk = esp32.read(waiting if waiting else 1)
        now = time.monotonic()

        if chunk:
            buffer.extend(chunk)

        for line in extract_text_lines_before_frame(buffer):
            if line == "CSV_BEGIN":
                legacy_rows = []
                legacy_csv_mode = True
                print("Modo CSV antigo detectado. Carregue o receptor atualizado para reduzir perdas.")
                continue

            if line == "CSV_END" and legacy_csv_mode:
                return legacy_rows

            if legacy_csv_mode:
                if line.startswith("tempo"):
                    continue

                parsed = parse_data_line(line)
                if parsed is not None:
                    legacy_rows.append(parsed)
                    if len(legacy_rows) % 5000 == 0:
                        print(f"Amostras CSV recebidas: {len(legacy_rows)}")
                continue

            if line in {"aguardando dados", "buffer recebido", "erro de pacote", RECEPTOR_ID}:
                print(line)

        for payload in parse_serial_frames(buffer):
            packet = decode_data_packet(payload)
            if packet is None:
                continue

            block_id = int(packet["block_id"])
            if active_block_id is None or block_id != active_block_id:
                active_block_id = block_id
                total_packets = int(packet["total_packets"])
                total_samples = int(packet["total_samples"])
                sample_rate_hz = int(packet["sample_rate_hz"])
                rows = create_empty_rows(total_samples, sample_rate_hz)
                seen_packets.clear()
                print(f"Recebendo bloco {active_block_id}...")

            packet_index = int(packet["packet_index"])
            if packet_index in seen_packets:
                continue

            seen_packets.add(packet_index)
            store_packet_rows(rows, packet)
            last_packet_time = now

            if total_packets and len(seen_packets) % 50 == 0:
                print(f"Pacotes recebidos: {len(seen_packets)}/{total_packets}")

            if total_packets and len(seen_packets) >= total_packets:
                print(f"Pacotes recebidos: {len(seen_packets)}/{total_packets}")
                return rows

        if not rows and now - start_wait > timeout_s:
            if legacy_rows:
                print("Aviso: timeout no modo CSV antigo. Salvando o que foi recebido.")
                return legacy_rows
            print("Aviso: nenhum pacote binario recebido dentro do timeout.")
            return rows

        if rows and now - last_packet_time > timeout_s:
            if total_packets:
                print(f"Aviso: timeout. Pacotes recebidos: {len(seen_packets)}/{total_packets}")
            return rows


def save_rows(csv_path: Path, rows: list[tuple[float, float, float, float]]) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["tempo", "x", "y", "z"])
        for tempo, x, y, z in rows:
            writer.writerow([f"{tempo:.6f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])


def column_name(index: int) -> str:
    name = ""
    while index:
        index, remainder = divmod(index - 1, 26)
        name = chr(65 + remainder) + name
    return name


def text_cell(row: int, column: int, value: object, style: int = 0) -> str:
    ref = f"{column_name(column)}{row}"
    text = escape(str(value))
    return f'<c r="{ref}" t="inlineStr" s="{style}"><is><t>{text}</t></is></c>'


def number_cell(row: int, column: int, value: object, style: int = 2) -> str:
    ref = f"{column_name(column)}{row}"
    try:
        numeric_value = float(value)
    except (TypeError, ValueError):
        return text_cell(row, column, value)

    if math.isnan(numeric_value) or math.isinf(numeric_value):
        return text_cell(row, column, "NaN")

    return f'<c r="{ref}" s="{style}"><v>{value}</v></c>'


def row_has_missing_axes(row: tuple[float, float, float, float]) -> bool:
    return any(math.isnan(value) or math.isinf(value) for value in row[1:])


def worksheet_xml(sheet_rows: list[str], dimension: str, auto_filter: str | None = None) -> str:
    auto_filter_xml = f'<autoFilter ref="{auto_filter}"/>' if auto_filter else ""
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        f'<dimension ref="{dimension}"/>'
        '<sheetViews><sheetView workbookViewId="0">'
        '<pane ySplit="1" topLeftCell="A2" activePane="bottomLeft" state="frozen"/>'
        '</sheetView></sheetViews>'
        '<sheetFormatPr defaultRowHeight="15"/>'
        '<cols>'
        '<col min="1" max="1" width="14" customWidth="1"/>'
        '<col min="2" max="4" width="13" customWidth="1"/>'
        '<col min="5" max="5" width="18" customWidth="1"/>'
        '</cols>'
        f'<sheetData>{"".join(sheet_rows)}</sheetData>'
        f"{auto_filter_xml}"
        "</worksheet>"
    )


def build_data_sheet(rows: list[tuple[float, float, float, float]]) -> str:
    sheet_rows = [
        "<row r=\"1\">"
        f"{text_cell(1, 1, 'tempo_s', 1)}"
        f"{text_cell(1, 2, 'x_g', 1)}"
        f"{text_cell(1, 3, 'y_g', 1)}"
        f"{text_cell(1, 4, 'z_g', 1)}"
        f"{text_cell(1, 5, 'status', 1)}"
        "</row>"
    ]

    for index, (tempo, x, y, z) in enumerate(rows, start=2):
        status = "pacote_perdido" if row_has_missing_axes((tempo, x, y, z)) else "ok"
        sheet_rows.append(
            f'<row r="{index}">'
            f"{number_cell(index, 1, f'{tempo:.6f}')}"
            f"{number_cell(index, 2, f'{x:.6f}')}"
            f"{number_cell(index, 3, f'{y:.6f}')}"
            f"{number_cell(index, 4, f'{z:.6f}')}"
            f"{text_cell(index, 5, status)}"
            "</row>"
        )

    last_row = len(rows) + 1
    return worksheet_xml(sheet_rows, f"A1:E{last_row}", f"A1:E{last_row}")


def collection_stats(rows: list[tuple[float, float, float, float]]) -> tuple[int, int, int, float, float]:
    sample_count = len(rows)
    missing_count = sum(1 for row in rows if row_has_missing_axes(row))
    valid_count = sample_count - missing_count

    if sample_count >= 2:
        duration = rows[-1][0] - rows[0][0]
        mean_fs = (sample_count - 1) / duration if duration > 0 else 0.0
    else:
        duration = 0.0
        mean_fs = 0.0

    return sample_count, valid_count, missing_count, duration, mean_fs


def build_summary_sheet(rows: list[tuple[float, float, float, float]]) -> str:
    sample_count, valid_count, missing_count, duration, mean_fs = collection_stats(rows)
    if sample_count == EXPECTED_SAMPLES_PER_AXIS and missing_count == 0:
        status = "OK"
    elif sample_count == EXPECTED_SAMPLES_PER_AXIS:
        status = "com perdas"
    else:
        status = "verificar coleta"

    summary_rows: list[tuple[str, object]] = [
        ("linhas salvas", sample_count),
        ("amostras validas por eixo", valid_count),
        ("amostras perdidas por eixo", missing_count),
        ("amostras esperadas por eixo", EXPECTED_SAMPLES_PER_AXIS),
        ("duracao nominal s", WINDOW_SECONDS),
        ("duracao calculada s", round(duration, 6)),
        ("frequencia nominal hz", EXPECTED_SAMPLE_RATE_HZ),
        ("frequencia media calculada hz", round(mean_fs, 3)),
        ("escala acelerometro g", ACCEL_RANGE_G),
        ("faixa nominal do acelerometro", accel_range_text(ACCEL_RANGE_G)),
        ("conversao g por contagem", RAW_TO_G),
        ("status", status),
    ]

    sheet_rows = [
        "<row r=\"1\">"
        f"{text_cell(1, 1, 'parametro', 1)}"
        f"{text_cell(1, 2, 'valor', 1)}"
        "</row>"
    ]

    for row_number, (label, value) in enumerate(summary_rows, start=2):
        if isinstance(value, (int, float)):
            value_cell = number_cell(row_number, 2, value)
        else:
            value_cell = text_cell(row_number, 2, value)

        sheet_rows.append(
            f'<row r="{row_number}">'
            f"{text_cell(row_number, 1, label)}"
            f"{value_cell}"
            "</row>"
        )

    return worksheet_xml(sheet_rows, f"A1:B{len(summary_rows) + 1}", f"A1:B{len(summary_rows) + 1}")


def save_excel(xlsx_path: Path, rows: list[tuple[float, float, float, float]]) -> None:
    timestamp = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
    data_sheet = build_data_sheet(rows)
    summary_sheet = build_summary_sheet(rows)

    workbook_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        '<sheets>'
        '<sheet name="Dados" sheetId="1" r:id="rId1"/>'
        '<sheet name="Resumo" sheetId="2" r:id="rId2"/>'
        '</sheets>'
        '</workbook>'
    )

    workbook_rels = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>'
        '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet2.xml"/>'
        '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>'
        '</Relationships>'
    )

    root_rels = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>'
        '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>'
        '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>'
        '</Relationships>'
    )

    content_types = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        '<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
        '<Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
        '<Override PartName="/xl/worksheets/sheet2.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
        '<Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>'
        '<Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>'
        '<Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>'
        '</Types>'
    )

    styles_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        '<numFmts count="1"><numFmt numFmtId="164" formatCode="0.000000"/></numFmts>'
        '<fonts count="2"><font><sz val="11"/><name val="Calibri"/></font><font><b/><sz val="11"/><name val="Calibri"/></font></fonts>'
        '<fills count="2"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill></fills>'
        '<borders count="1"><border><left/><right/><top/><bottom/><diagonal/></border></borders>'
        '<cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>'
        '<cellXfs count="3">'
        '<xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/>'
        '<xf numFmtId="0" fontId="1" fillId="0" borderId="0" xfId="0" applyFont="1"/>'
        '<xf numFmtId="164" fontId="0" fillId="0" borderId="0" xfId="0" applyNumberFormat="1"/>'
        '</cellXfs>'
        '<cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles>'
        '</styleSheet>'
    )

    core_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
        'xmlns:dc="http://purl.org/dc/elements/1.1/" '
        'xmlns:dcterms="http://purl.org/dc/terms/" '
        'xmlns:dcmitype="http://purl.org/dc/dcmitype/" '
        'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">'
        '<dc:creator>salvar_coleta_10s.py</dc:creator>'
        '<cp:lastModifiedBy>salvar_coleta_10s.py</cp:lastModifiedBy>'
        f'<dcterms:created xsi:type="dcterms:W3CDTF">{timestamp}</dcterms:created>'
        f'<dcterms:modified xsi:type="dcterms:W3CDTF">{timestamp}</dcterms:modified>'
        '</cp:coreProperties>'
    )

    app_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" '
        'xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">'
        '<Application>Python</Application>'
        '</Properties>'
    )

    with zipfile.ZipFile(xlsx_path, "w", compression=zipfile.ZIP_DEFLATED) as workbook:
        workbook.writestr("[Content_Types].xml", content_types)
        workbook.writestr("_rels/.rels", root_rels)
        workbook.writestr("docProps/core.xml", core_xml)
        workbook.writestr("docProps/app.xml", app_xml)
        workbook.writestr("xl/workbook.xml", workbook_xml)
        workbook.writestr("xl/_rels/workbook.xml.rels", workbook_rels)
        workbook.writestr("xl/styles.xml", styles_xml)
        workbook.writestr("xl/worksheets/sheet1.xml", data_sheet)
        workbook.writestr("xl/worksheets/sheet2.xml", summary_sheet)


def print_summary(rows: list[tuple[float, float, float, float]]) -> None:
    sample_count, valid_count, missing_count, duration, mean_fs = collection_stats(rows)

    print(f"Quantidade de linhas salvas: {sample_count}")
    print(f"Amostras validas por eixo: {valid_count}")
    print(f"Amostras perdidas por eixo: {missing_count}")
    print(f"Amostras esperadas por eixo: {EXPECTED_SAMPLES_PER_AXIS}")
    print(f"Duracao da coleta: {duration:.6f} s")
    print(f"Frequencia media real: {mean_fs:.3f} Hz")
    print(f"Escala do acelerometro: {accel_range_text(ACCEL_RANGE_G)}")
    print(f"Conversao aplicada: {RAW_TO_G:.6f} g/contagem")
    if sample_count != EXPECTED_SAMPLES_PER_AXIS:
        print(f"Aviso: a quantidade salva ficou diferente de {EXPECTED_SAMPLES_PER_AXIS} amostras por eixo.")
    if missing_count:
        print("Aviso: existem linhas NaN porque alguns pacotes nao chegaram ao receptor.")


def capture_collection(port: str, baud: int, output_dir: Path, timeout_s: float) -> Path:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Instale o pyserial: pip install pyserial") from exc

    csv_path, xlsx_path = next_collection_paths(output_dir)

    print(f"Usando porta serial: {port}")

    with serial.Serial(port, baudrate=baud, timeout=0.05) as esp32:
        # Ao abrir a porta, muitos ESP32 reiniciam. Esta pausa evita perder o boot.
        time.sleep(2.0)
        esp32.reset_input_buffer()

        input(f"Pressione ENTER para iniciar a gravacao de {int(WINDOW_SECONDS)} segundos...")
        esp32.write(b"START\n")
        esp32.flush()

        rows = capture_binary_rows(esp32, timeout_s)

    if not rows:
        raise SystemExit(
            "Nenhuma amostra foi recebida. Verifique: carregou o receptor_espnow_serial.ino atualizado, "
            "o master esta ligado, a porta COM e do receptor e o Serial Monitor da Arduino IDE esta fechado."
        )

    save_rows(csv_path, rows)
    save_excel(xlsx_path, rows)
    print(f"CSV salvo: {csv_path}")
    print(f"Excel salvo: {xlsx_path}")
    print_summary(rows)
    return csv_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Salva dados de vibracao recebidos do ESP32 via Serial.")
    parser.add_argument("--porta", help="Porta serial do ESP32 receptor. Use auto para detectar. Ex.: COM5 ou /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help="Baud rate. Padrao: 2000000")
    parser.add_argument("--saida", default="coletas", help="Pasta de saida dos CSVs. Padrao: coletas")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_SECONDS, help="Timeout sem dados em segundos. Padrao: 30")
    parser.add_argument(
        "--escala-g",
        type=int,
        choices=sorted(RAW_TO_G_BY_SCALE),
        default=DEFAULT_ACCEL_RANGE_G,
        help="Escala configurada no LIS3DH. Deve bater com ACCEL_RANGE_G no master. Padrao: 8",
    )
    parser.add_argument("--listar-portas", action="store_true", help="Mostra as portas seriais encontradas e sai.")
    args = parser.parse_args()

    global ACCEL_RANGE_G, RAW_TO_G
    ACCEL_RANGE_G = args.escala_g
    RAW_TO_G = raw_to_g_for_scale(args.escala_g)

    port = choose_serial_port(args.porta, args.baud, args.listar_portas)
    output_dir = path_near_script(args.saida)
    capture_collection(port, args.baud, output_dir, args.timeout)


if __name__ == "__main__":
    main()
