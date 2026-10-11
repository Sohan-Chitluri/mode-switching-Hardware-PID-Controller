#!/usr/bin/env python3
"""
export_report_charts.py
Extracts all embedded SVG charts from docs/report.html, cleans and normalizes XML entities,
saves standalone SVG files into docs/charts/, and renders high-resolution PNG images via rsvg-convert.
"""

import os
import re
import html
import subprocess
import struct

CHART_NAMES = [
    "01_pid_cascaded_signal_flow",
    "02_flight_mode_fsm",
    "03_dynamic_range_resolution",
    "04_cell_area_comparison",
    "05_power_breakdown_comparison",
    "06_cell_count_comparison",
    "07_throughput_comparison",
    "08_verification_tiers_matrix",
    "09_closed_loop_step_response",
    "10_mode_switch_transient",
    "11_pnr_runtime_comparison"
]

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HTML_PATH = os.path.join(REPO_ROOT, "docs", "report.html")
OUTPUT_DIR = os.path.join(REPO_ROOT, "docs", "charts")

def get_png_dimensions(file_path):
    try:
        with open(file_path, "rb") as f:
            data = f.read(24)
            if len(data) >= 24 and data[:8] == b"\x89PNG\r\n\x1a\n":
                w, h = struct.unpack(">LL", data[16:24])
                return f"{w}x{h}"
    except Exception:
        pass
    return "N/A"

def clean_svg_xml(svg_raw):
    # Preserve standard XML entities before unescaping HTML5 entities
    text = svg_raw.replace("&lt;", "__XML_LT__")
    text = text.replace("&gt;", "__XML_GT__")
    text = text.replace("&amp;", "__XML_AMP__")
    text = text.replace("&quot;", "__XML_QUOT__")
    text = text.replace("&apos;", "__XML_APOS__")

    # Unescape HTML5 named entities (like &alpha;, &Delta;, &le;, &minus;) into native unicode
    unescaped = html.unescape(text)

    # Restore XML entities
    restored = unescaped.replace("__XML_LT__", "&lt;")
    restored = restored.replace("__XML_GT__", "&gt;")
    restored = restored.replace("__XML_AMP__", "&amp;")
    restored = restored.replace("__XML_QUOT__", "&quot;")
    restored = restored.replace("__XML_APOS__", "&apos;")

    # Re-escape any bare '&' characters that aren't valid XML entities
    xml_clean = re.sub(r"&(?!(amp|lt|gt|quot|apos);)", "&amp;", restored)

    # Ensure xmlns is present
    if "xmlns=" not in xml_clean:
        xml_clean = re.sub(r"<svg", r'<svg xmlns="http://www.w3.org/2000/svg"', xml_clean, count=1)

    return '<?xml version="1.0" encoding="UTF-8"?>\n' + xml_clean

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    with open(HTML_PATH, "r", encoding="utf-8") as f:
        html_content = f.read()

    svg_pattern = re.compile(r"<svg[\s\S]*?</svg>", re.IGNORECASE)
    matches = list(svg_pattern.finditer(html_content))

    if len(matches) != len(CHART_NAMES):
        print(f"Warning: Expected {len(CHART_NAMES)} SVGs, but found {len(matches)}!")

    print(f"Extracting and rendering {len(matches)} SVG charts to '{OUTPUT_DIR}'...\n")

    results = []

    for idx, m in enumerate(matches):
        name = CHART_NAMES[idx] if idx < len(CHART_NAMES) else f"chart_{idx+1:02d}"
        svg_raw = m.group(0)

        standalone_svg = clean_svg_xml(svg_raw)

        svg_file = os.path.join(OUTPUT_DIR, f"{name}.svg")
        png_file = os.path.join(OUTPUT_DIR, f"{name}.png")

        with open(svg_file, "w", encoding="utf-8") as sf:
            sf.write(standalone_svg)

        # Convert to PNG using rsvg-convert (2x zoom factor, crisp white background)
        cmd = ["rsvg-convert", "-z", "2", "-b", "white", svg_file, "-o", png_file]
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except (FileNotFoundError, subprocess.CalledProcessError):
            cmd_nix = ["nix-shell", "-p", "librsvg", "--run", f"rsvg-convert -z 2 -b white '{svg_file}' -o '{png_file}'"]
            subprocess.run(cmd_nix, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        svg_size = os.path.getsize(svg_file)
        png_size = os.path.getsize(png_file)
        dims = get_png_dimensions(png_file)

        results.append((name, svg_size, png_size, dims))
        print(f"[{idx+1:02d}/11] Processed {name}.svg ({svg_size:,} bytes) -> {name}.png ({png_size:,} bytes, {dims})")

    print("\n" + "="*80)
    print(f"{'Chart Name':<35} {'SVG Size':<12} {'PNG Size':<12} {'Dimensions'}")
    print("="*80)
    for name, svg_sz, png_sz, dims in results:
        print(f"{name:<35} {svg_sz:<12} {png_sz:<12} {dims}")
    print("="*80)
    print(f"\nSuccessfully generated all {len(results)} SVG & PNG charts in: {OUTPUT_DIR}")

if __name__ == "__main__":
    main()
