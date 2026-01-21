# Please note, taht each bagfile / series should be in its dedicated folder.

import os
import argparse
import subprocess
import socket
import time
import urllib.request
from pathlib import Path

# --- Notification Utility (No pip dependencies) ---
def send_ntfy(topic, message, title="Batch Converter", priority="default", tags=None):
    if not topic:
        return

    url = f"https://ntfy.sh/{topic}"
    hostname = socket.gethostname()
    
    headers = {
        "Title": title,
        "Priority": priority,
        "Tags": ",".join(tags) if tags else ""
    }
    
    # Add hostname context to message
    full_message = f"[{hostname}] {message}"
    
    try:
        req = urllib.request.Request(
            url, 
            data=full_message.encode('utf-8'), 
            headers=headers, 
            method='POST'
        )
        with urllib.request.urlopen(req) as response:
            pass # Request sent
    except Exception as e:
        print(f"[WARN] Failed to send ntfy notification: {e}")

# --- Core Logic ---
def run_batch_conversion(src_root, dst_root, converter_cmd, ntfy_topic=None, dry_run=False, with_plugins=False):
    src_root = Path(src_root).resolve()
    dst_root = Path(dst_root).resolve()
    
    # NOTE: We do NOT resolve converter_cmd because it is likely a command in PATH
    
    if not src_root.exists():
        print(f"[Error] Source directory not found: {src_root}")
        return

    # 1. Scan Phase
    print("=" * 60)
    print(f"BATCH CONVERSION START")
    print(f"Source:  {src_root}")
    print(f"Target:  {dst_root}")
    print(f"Plugins: {with_plugins}")
    print(f"Command: {converter_cmd}")
    print(f"Ntfy:    {ntfy_topic if ntfy_topic else 'Disabled'}")
    print("-" * 60)
    print("Scanning for bag folders...")

    tasks = []
    for dirpath, dirnames, filenames in os.walk(src_root):
        # Check if folder contains .bag files
        if any(f.endswith('.bag') for f in filenames):
            tasks.append(Path(dirpath))

    total_tasks = len(tasks)
    print(f"Found {total_tasks} folders to process.")
    print("=" * 60)

    # Notify Start
    if not dry_run:
        send_ntfy(ntfy_topic, f"Started batch conversion of {total_tasks} folders.", title="Batch Started", tags=["rocket"])

    start_time = time.time()
    success_count = 0
    fail_count = 0
    skipped_count = 0

    for index, current_input_path in enumerate(tasks, 1):
        relative_path = current_input_path.relative_to(src_root)
        target_output_folder = dst_root / relative_path
        
        # Define expected metadata file to check for completion
        is_done = (target_output_folder / "metadata.yaml").exists()

        if is_done:
            print(f"[{index}/{total_tasks}] [SKIP] {relative_path}")
            skipped_count += 1
            continue

        print(f"\n[{index}/{total_tasks}] [Processing] {relative_path}")
        
        cmd = [
            converter_cmd,
            str(current_input_path),
            "--series",
            "--out-dir", str(target_output_folder)
        ]

        if with_plugins:
            cmd.append("--with-plugins")

        if dry_run:
            print(f"  [Dry Run] CMD: {' '.join(cmd)}")
            success_count += 1 
        else:
            try:
                # Create parent structure
                target_output_folder.parent.mkdir(parents=True, exist_ok=True)
                
                t0 = time.time()
                subprocess.run(cmd, check=True)
                duration = time.time() - t0
                
                success_count += 1
                
                msg = f"Completed {index}/{total_tasks}: {relative_path} ({duration:.1f}s)"
                send_ntfy(ntfy_topic, msg, title="Conversion Progress", priority="low", tags=["white_check_mark"])

            except subprocess.CalledProcessError as e:
                fail_count += 1
                error_msg = f"Failed {relative_path}\nExit Code: {e.returncode}"
                print(f"  [ERROR] {error_msg}")
                send_ntfy(ntfy_topic, error_msg, title="Conversion Error", priority="high", tags=["warning", "x"])
                
            except Exception as e:
                fail_count += 1
                error_msg = f"Critical Error on {relative_path}\n{str(e)}"
                print(f"  [ERROR] {error_msg}")
                send_ntfy(ntfy_topic, error_msg, title="Critical Failure", priority="urgent", tags=["skull"])

    # Final Summary
    total_duration = time.time() - start_time
    summary = (
        f"Batch Finished in {total_duration/60:.1f} min.\n"
        f"Total: {total_tasks}\n"
        f"Success: {success_count}\n"
        f"Skipped: {skipped_count}\n"
        f"Failed: {fail_count}"
    )
    
    print("\n" + "=" * 60)
    print(summary)
    print("=" * 60)

    if not dry_run:
        tag = "tada" if fail_count == 0 else "warning"
        send_ntfy(ntfy_topic, summary, title="Batch Complete", priority="default", tags=[tag])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Smart Batch Convert ROS1 -> MCAP with Notifications")
    parser.add_argument("--input", required=True, help="Root folder containing ROS1 bag folders")
    parser.add_argument("--output", required=True, help="Root folder for MCAP output")
    
    # CHANGED: Default is now just the system command "convert_bag"
    parser.add_argument("--converter", default="convert_bag", help="Command to run converter (default: convert_bag)")
    
    parser.add_argument("--with-plugins", action="store_true", help="Enable plugins")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without running")
    parser.add_argument("--ntfy", default=None, help="ntfy.sh topic (e.g., 'my_secret_topic_123')")

    args = parser.parse_args()

    run_batch_conversion(
        args.input, 
        args.output, 
        args.converter, 
        ntfy_topic=args.ntfy,
        dry_run=args.dry_run, 
        with_plugins=args.with_plugins
    )