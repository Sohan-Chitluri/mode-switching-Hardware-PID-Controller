#!/usr/bin/env python3
"""
generate_report_md.py
Converts docs/report.html into a comprehensive, beautifully structured docs/report.md
markdown document, embedding the generated PNG charts from docs/charts/ and screenshots
from docs/images/.
"""

import os
import re
import html

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HTML_PATH = os.path.join(REPO_ROOT, "docs", "report.html")
MD_PATH = os.path.join(REPO_ROOT, "docs", "report.md")

CHART_MAP = [
    ("01_pid_cascaded_signal_flow", "PID Cascaded Control Architecture — Signal Flow"),
    ("02_flight_mode_fsm", "4-State Flight Mode FSM"),
    ("03_dynamic_range_resolution", "Numeric Dynamic Range and LSB Resolution Comparison"),
    ("04_cell_area_comparison", "Cell Area Comparison (16-bit Shared vs. Parallel)"),
    ("05_power_breakdown_comparison", "Power Breakdown Comparison"),
    ("06_cell_count_comparison", "Cell Count Comparison"),
    ("07_throughput_comparison", "Sampling Throughput Comparison"),
    ("08_verification_tiers_matrix", "Verification Tiers Status Matrix"),
    ("09_closed_loop_step_response", "Closed-Loop Step Response — Roll Axis (6-DOF Co-Simulation)"),
    ("10_mode_switch_transient", "Bumpless Mode-Switch Transient Distribution (1,000 Transitions)"),
    ("11_pnr_runtime_comparison", "Place & Route Runtimes Before and After Optimization")
]

def convert_html_table(table_html):
    # Extract headers
    headers = []
    th_matches = re.findall(r"<th[^>]*>(.*?)</th>", table_html, re.DOTALL)
    for th in th_matches:
        clean_th = re.sub(r"<[^>]+>", "", th).strip()
        clean_th = html.unescape(clean_th)
        headers.append(clean_th)

    # Extract rows
    rows = []
    tr_matches = re.findall(r"<tr>(.*?)</tr>", table_html, re.DOTALL)
    for tr in tr_matches:
        tds = re.findall(r"<td[^>]*>(.*?)</td>", tr, re.DOTALL)
        if tds:
            row = []
            for td in tds:
                clean_td = re.sub(r"<br\s*/?>", " ", td)
                clean_td = re.sub(r"<code>(.*?)</code>", r"`\1`", clean_td)
                clean_td = re.sub(r"<strong>(.*?)</strong>", r"**\1**", clean_td)
                clean_td = re.sub(r"<[^>]+>", "", clean_td).strip()
                clean_td = html.unescape(clean_td)
                clean_td = clean_td.replace("\n", " ").replace("|", "\\|")
                row.append(clean_td)
            rows.append(row)

    if not headers and rows:
        headers = [f"Col {i+1}" for i in range(len(rows[0]))]

    if not headers:
        return ""

    md_lines = []
    md_lines.append("| " + " | ".join(headers) + " |")
    md_lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for r in rows:
        # pad if needed
        while len(r) < len(headers):
            r.append("")
        md_lines.append("| " + " | ".join(r[:len(headers)]) + " |")

    return "\n" + "\n".join(md_lines) + "\n"

def main():
    with open(HTML_PATH, "r", encoding="utf-8") as f:
        content = f.read()

    # Extract main body
    body_match = re.search(r"<body>(.*?)</body>", content, re.DOTALL)
    body = body_match.group(1) if body_match else content

    # Strip everything before Section 1 (the first <h2> tag)
    h2_pos = body.find("<h2")
    if h2_pos != -1:
        body = body[h2_pos:]

    # 1. Replace SVGs with PNG images
    svg_matches = list(re.finditer(r"<div class=\"chart-(?:container|wrap)\">\s*<svg[\s\S]*?</svg>(?:\s*<div class=\"caption\">(.*?)</div>)?\s*</div>", body))
    if not svg_matches:
        svg_matches = list(re.finditer(r"<svg[\s\S]*?</svg>", body))

    # Replace SVG blocks sequentially with corresponding chart PNGs
    def svg_replacer(match):
        nonlocal svg_idx
        if svg_idx < len(CHART_MAP):
            cname, ctitle = CHART_MAP[svg_idx]
            svg_idx += 1
            return f"\n\n![{ctitle}](./charts/{cname}.png)\n*Figure: {ctitle}*\n\n"
        svg_idx += 1
        return ""

    svg_idx = 0
    body = re.sub(r"<div class=\"chart-(?:container|wrap)\">\s*<svg[\s\S]*?</svg>(?:\s*<div class=\"caption\">(.*?)</div>)?\s*</div>", svg_replacer, body)
    if svg_idx < len(CHART_MAP):
        body = re.sub(r"<svg[\s\S]*?</svg>", svg_replacer, body)

    # 2. Convert arch-box blocks to fenced code blocks
    body = re.sub(r"<div class=\"arch-box\">(.*?)</div>", r"\n```text\n\1\n```\n", body, flags=re.DOTALL)

    # 3. Convert math-block to blockquotes / math
    def math_replacer(m):
        raw = m.group(1)
        raw = re.sub(r"<br\s*/?>", "\n", raw)
        raw = re.sub(r"&nbsp;", " ", raw)
        raw = html.unescape(raw)
        raw = re.sub(r"<[^>]+>", "", raw).strip()
        lines = [f"> {line}" for line in raw.splitlines() if line.strip()]
        return "\n" + "\n".join(lines) + "\n"

    body = re.sub(r"<div class=\"math-block\">(.*?)</div>", math_replacer, body, flags=re.DOTALL)

    # 4. Convert callouts
    def callout_replacer(m):
        raw = m.group(1)
        raw = re.sub(r"<br\s*/?>", "\n> ", raw)
        raw = re.sub(r"<code>(.*?)</code>", r"`\1`", raw)
        raw = re.sub(r"<strong>(.*?)</strong>", r"**\1**", raw)
        raw = re.sub(r"&bull;", "•", raw)
        raw = html.unescape(raw)
        raw = re.sub(r"<[^>]+>", "", raw).strip()
        return f"\n> {raw}\n"

    body = re.sub(r"<div class=\"callout(?:-green)?\">(.*?)</div>", callout_replacer, body, flags=re.DOTALL)

    # 5. Convert img containers
    def img_replacer(m):
        src = m.group(1)
        alt = m.group(2) if m.group(2) else "Figure"
        return f"\n\n![{alt}]({src})\n\n"

    body = re.sub(r"<img[^>]*src=[\"']([^\"']+)[\"'][^>]*alt=[\"']([^\"']*)[\"'][^>]*>", img_replacer, body)
    body = re.sub(r"<img[^>]*src=[\"']([^\"']+)[\"'][^>]*>", r"\n\n![](\1)\n\n", body)

    # 6. Convert tables
    def table_replacer(m):
        tbl = m.group(0)
        return convert_html_table(tbl)

    body = re.sub(r"<table[\s\S]*?</table>", table_replacer, body)

    # 7. Convert headings
    body = re.sub(r"<h2>1\.\s*Executive Summary &amp; Problem Statement</h2>", "## 1. Problem Statement & Overview", body)
    body = re.sub(r"<h2>(.*?)</h2>", r"\n## \1\n", body)
    body = re.sub(r"<h3>(.*?)</h3>", r"\n### \1\n", body)
    body = re.sub(r"<h4>(.*?)</h4>", r"\n#### \1\n", body)

    # 8. Convert formatting: code, bold, em, caption
    body = re.sub(r"<div class=\"caption\">(.*?)</div>", r"*\1*\n", body, flags=re.DOTALL)
    body = re.sub(r"<code>(.*?)</code>", r"`\1`", body)
    body = re.sub(r"<strong>(.*?)</strong>", r"**\1**", body)
    body = re.sub(r"<em>(.*?)</em>", r"*\1*", body)
    body = re.sub(r"<li>(.*?)</li>", r"- \1\n", body)

    # 9. Clean remaining HTML tags and boilerplate
    body = re.sub(r"</?(?:div|p|ul|ol|header-box|badge|meta-grid|meta-item|subtitle|img-container|img-two-col|chart-two-col|chart-container|col|page-break|span)[^>]*>", "\n", body)
    body = re.sub(r"<hr[^>]*>", "\n---\n", body)

    # Decode HTML entities
    body = html.unescape(body)

    # Consolidate excessive blank lines
    body = re.sub(r"\n{3,}", "\n\n", body)

    # Master Markdown Header
    header = """# PS-02: Mode-Switching Hardware PID Flight Controller
## Comprehensive Technical Design & Sign-Off Report

> **Sudarshana Semiconductors • VLSI Hackathon 2026 • VIT Chennai**  
> **Problem Statement PS-02:** Deterministic, Bumpless, Fault-Aware Hardware Coprocessor in TSMC 180nm CMOS  
> **Student Designers:** **Sohan Chitluri** (24BVD1104) & **Prasiddh Saha** (24BVD1103)  
> **Date:** October 11, 2026

---

"""
    full_md = header + body.strip() + "\n"

    with open(MD_PATH, "w", encoding="utf-8") as f:
        f.write(full_md)

    size = os.path.getsize(MD_PATH)
    lines = len(full_md.splitlines())
    print(f"Generated '{MD_PATH}' successfully ({size:,} bytes, {lines:,} lines).")

if __name__ == "__main__":
    main()
