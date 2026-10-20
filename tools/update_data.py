#!/usr/bin/env python3
"""Update BRouter RD5 segment files, baseline fixtures, and test assertions."""

import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SEGMENTS_DIR = REPO_ROOT / "data" / "segments4"
FIXTURES_DIR = REPO_ROOT / "fixtures" / "brouter_de"
ROUTING_TESTS_PATH = REPO_ROOT / "tests" / "routing_tests.rs"
RD5_TESTS_PATH = REPO_ROOT / "tests" / "rd5_tests.rs"

DEFAULT_SEGMENTS = ["E0_N50", "W5_N50"]
SEGMENTS_URL = "https://brouter.de/brouter/segments4"
ROUTING_URL = "https://brouter.de/brouter"

ROUTES = {
    "clapham_hackney": "-0.143613,51.462414|-0.055296,51.544998",
    "london_brighton": "-0.127818,51.507392|-0.136913,50.822431",
    "london_oxford": "-0.127818,51.507392|-1.257704,51.751988",
    "london_cardiff": "-0.127818,51.507392|-3.179096,51.481607",
    "richmond_oxford_multi_point": "-0.3060,51.4590|-0.6060,51.4810|-0.7740,51.5710|-1.257704,51.751988",
}

FIXTURES = [
    # Clapham -> Hackney
    ("clapham_hackney_fastbike.json", "clapham_hackney", "fastbike", None),
    ("clapham_hackney_lowtraffic.json", "clapham_hackney", "fastbike-lowtraffic", None),
    ("clapham_hackney_verylowtraffic.json", "clapham_hackney", "fastbike-verylowtraffic", None),
    ("clapham_hackney_gravel.json", "clapham_hackney", "gravel", None),
    ("clapham_hackney_trekking.json", "clapham_hackney", "trekking", None),

    # London -> Brighton
    ("london_brighton_fastbike.json", "london_brighton", "fastbike", None),
    ("london_brighton_lowtraffic.json", "london_brighton", "fastbike-lowtraffic", None),
    ("london_brighton_fastbike-lowtraffic_alt0.json", "london_brighton", "fastbike-lowtraffic", 0),
    ("london_brighton_verylowtraffic.json", "london_brighton", "fastbike-verylowtraffic", None),
    ("london_brighton_gravel.json", "london_brighton", "gravel", None),
    ("london_brighton_trekking.json", "london_brighton", "trekking", None),
    ("london_brighton_fastbike-lowtraffic_alt1.json", "london_brighton", "fastbike-lowtraffic", 1),
    ("london_brighton_fastbike-lowtraffic_alt2.json", "london_brighton", "fastbike-lowtraffic", 2),
    ("london_brighton_fastbike-lowtraffic_alt3.json", "london_brighton", "fastbike-lowtraffic", 3),

    # London -> Oxford
    ("london_oxford_fastbike.json", "london_oxford", "fastbike", None),
    ("london_oxford_lowtraffic.json", "london_oxford", "fastbike-lowtraffic", None),
    ("london_oxford_verylowtraffic.json", "london_oxford", "fastbike-verylowtraffic", None),
    ("london_oxford_gravel.json", "london_oxford", "gravel", None),
    ("london_oxford_trekking.json", "london_oxford", "trekking", None),

    # London -> Cardiff
    ("london_cardiff_fastbike.json", "london_cardiff", "fastbike", None),
    ("london_cardiff_lowtraffic.json", "london_cardiff", "fastbike-lowtraffic", None),
    ("london_cardiff_verylowtraffic.json", "london_cardiff", "fastbike-verylowtraffic", None),
    ("london_cardiff_gravel.json", "london_cardiff", "gravel", None),
    ("london_cardiff_trekking.json", "london_cardiff", "trekking", None),

    # Richmond -> Oxford
    ("richmond_oxford_multi_point_fastbike.json", "richmond_oxford_multi_point", "fastbike", None),
    ("richmond_oxford_multi_point_lowtraffic.json", "richmond_oxford_multi_point", "fastbike-lowtraffic", None),
    ("richmond_oxford_multi_point_verylowtraffic.json", "richmond_oxford_multi_point", "fastbike-verylowtraffic", None),
]


def download_file(url: str, dest_path: Path):
    """Download a URL to dest_path atomically using a temporary file."""
    tmp_path = dest_path.with_suffix(dest_path.suffix + ".tmp")
    req = urllib.request.Request(url, headers={"User-Agent": "rsrouter-data-updater"})

    with urllib.request.urlopen(req, timeout=120) as resp, open(tmp_path, "wb") as out_file:
        total = int(resp.headers.get("Content-Length", 0))
        downloaded = 0
        chunk_size = 64 * 1024
        while True:
            chunk = resp.read(chunk_size)
            if not chunk:
                break
            out_file.write(chunk)
            downloaded += len(chunk)
            if total > 0:
                percent = (downloaded / total) * 100
                mb = downloaded / (1024 * 1024)
                sys.stdout.write(f"\r  {mb:.1f} MB / {total / (1024 * 1024):.1f} MB ({percent:.0f}%)")
            else:
                mb = downloaded / (1024 * 1024)
                sys.stdout.write(f"\r  {mb:.1f} MB")
            sys.stdout.flush()

    sys.stdout.write("\n")
    tmp_path.rename(dest_path)


def update_segments(segments):
    """Download specified RD5 segments to data/segments4/."""
    SEGMENTS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Updating {len(segments)} RD5 segment(s) in {SEGMENTS_DIR}...")

    for seg in segments:
        filename = f"{seg}.rd5" if not seg.endswith(".rd5") else seg
        url = f"{SEGMENTS_URL}/{filename}"
        dest = SEGMENTS_DIR / filename
        print(f"Downloading {filename}...")
        download_file(url, dest)
        print(f"Saved {dest} ({dest.stat().st_size / (1024 * 1024):.1f} MB)")


def update_fixtures():
    """Fetch all 27 GeoJSON baseline fixtures from brouter.de."""
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Updating {len(FIXTURES)} fixture(s) in {FIXTURES_DIR}...")

    for filename, route_key, profile, alt in FIXTURES:
        lonlats = ROUTES[route_key]
        url = f"{ROUTING_URL}?lonlats={lonlats}&profile={profile}&format=geojson"
        if alt is not None:
            url += f"&alternativeidx={alt}"

        dest = FIXTURES_DIR / filename
        tmp_dest = dest.with_suffix(".tmp")
        req = urllib.request.Request(url, headers={"User-Agent": "rsrouter-fixture-updater"})

        for attempt in range(6):
            try:
                with urllib.request.urlopen(req, timeout=90) as resp:
                    raw = resp.read()
                    data = json.loads(raw.decode("utf-8"))
                    if data.get("type") != "FeatureCollection" or not data.get("features"):
                        raise ValueError(f"Invalid GeoJSON received for {filename}")
                    with open(tmp_dest, "w") as f:
                        json.dump(data, f, indent=2)
                    tmp_dest.rename(dest)
                    props = data["features"][0].get("properties", {})
                    dist = props.get("track-length")
                    cost = props.get("cost")
                    print(f"  {filename}: dist={dist}m cost={cost}")
                    break
            except Exception as e:
                if attempt == 5:
                    raise
                wait_s = 3 * (attempt + 1)
                print(f"  Attempt {attempt + 1} failed for {filename}: {e}. Retrying in {wait_s}s...")
                time.sleep(wait_s)

        time.sleep(0.5)

    print("All fixtures updated.")


def update_test_assertions():
    """Update hardcoded test constants in tests/."""
    print("Updating test assertions in tests/...")

    # 1. Update routing_tests.rs constants from fixture files
    if ROUTING_TESTS_PATH.exists():
        content = ROUTING_TESTS_PATH.read_text()

        cases = [
            ("test_clapham_hackney_fastbike", "clapham_hackney_fastbike.json"),
            ("test_clapham_hackney_lowtraffic", "clapham_hackney_lowtraffic.json"),
            ("test_clapham_hackney_verylowtraffic", "clapham_hackney_verylowtraffic.json"),
        ]

        for fn_name, fixture_name in cases:
            fixture_file = FIXTURES_DIR / fixture_name
            if not fixture_file.exists():
                continue
            with open(fixture_file) as f:
                data = json.load(f)
            props = data["features"][0]["properties"]
            coords = data["features"][0]["geometry"]["coordinates"]

            dist = float(props["track-length"])
            cost = int(props["cost"])
            pts = len(coords)

            pattern = re.compile(
                rf"(fn {fn_name}\(\)[\s\S]*?route\.total_distance_m - )(\d+(?:\.\d+)?)\)\.abs\(\);([\s\S]*?route\.total_cost, )(\d+)(,[\s\S]*?route\.points\.len\(\),\s*)(\d+)(,)"
            )
            repl = rf"\g<1>{dist:.1f}).abs();\g<3>{cost}\g<5>{pts}\g<7>"
            content = pattern.sub(repl, content)

        ROUTING_TESTS_PATH.write_text(content)
        print("Updated tests/routing_tests.rs assertions.")

    # 2. Update rd5_tests.rs node and link counts by running test output
    if RD5_TESTS_PATH.exists():
        proc = subprocess.run(
            ["cargo", "test", "--test", "rd5_tests", "--", "test_rd5_slice_in_memory_parity", "--nocapture"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        match = re.search(r"Rd5Slice decoded \d+ cells with (\d+) nodes, (\d+) links", proc.stdout)
        if match:
            nodes, links = match.group(1), match.group(2)
            content = RD5_TESTS_PATH.read_text()
            content = re.sub(r"assert_eq!\(total_nodes, \d+\);", f"assert_eq!(total_nodes, {nodes});", content)
            content = re.sub(r"assert_eq!\(total_links, \d+\);", f"assert_eq!(total_links, {links});", content)
            RD5_TESTS_PATH.write_text(content)
            print(f"Updated tests/rd5_tests.rs: nodes={nodes}, links={links}.")


def run_checks():
    """Run cargo test and compare_routes to verify parity."""
    print("Running cargo test...")
    proc_test = subprocess.run(["cargo", "test"], cwd=REPO_ROOT)
    if proc_test.returncode != 0:
        print("ERROR: cargo test failed.")
        sys.exit(proc_test.returncode)

    print("Running compare_routes...")
    proc_bench = subprocess.run(
        ["cargo", "run", "--release", "--example", "compare_routes"],
        cwd=REPO_ROOT,
    )
    if proc_bench.returncode != 0:
        print("ERROR: compare_routes failed.")
        sys.exit(proc_bench.returncode)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--segments",
        nargs="*",
        default=None,
        help="RD5 segment names to download (default: E0_N50 W5_N50).",
    )
    parser.add_argument(
        "--rd5-only",
        action="store_true",
        help="Update only RD5 segments.",
    )
    parser.add_argument(
        "--fixtures-only",
        action="store_true",
        help="Update only GeoJSON baseline fixtures.",
    )
    parser.add_argument(
        "--update-tests",
        action="store_true",
        help="Update hardcoded test assertions in tests/ to match new data.",
    )
    parser.add_argument(
        "--tests-only",
        action="store_true",
        help="Update only hardcoded test assertions without downloading files.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Run cargo test and compare_routes after updating.",
    )

    args = parser.parse_args()

    if args.tests_only:
        update_test_assertions()
        if args.check:
            run_checks()
        print("Done.")
        return

    do_segments = not args.fixtures_only
    do_fixtures = not args.rd5_only

    if do_segments:
        segs = args.segments if args.segments is not None else DEFAULT_SEGMENTS
        update_segments(segs)

    if do_fixtures:
        update_fixtures()

    if args.update_tests:
        update_test_assertions()

    if args.check:
        run_checks()

    print("Done.")


if __name__ == "__main__":
    main()
