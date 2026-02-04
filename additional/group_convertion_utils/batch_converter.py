import os
import argparse
import subprocess
import socket
import time
import urllib.request
import yaml 
from pathlib import Path

# --- Notification Utility ---
def send_ntfy(topic, message, title="Batch Converter", priority="default", tags=None):
    if not topic: return
    try:
        url = f"https://ntfy.sh/{topic}"
        headers = {"Title": title, "Priority": priority, "Tags": ",".join(tags) if tags else ""}
        data = f"[{socket.gethostname()}] {message}".encode('utf-8')
        req = urllib.request.Request(url, data=data, headers=headers, method='POST')
        with urllib.request.urlopen(req): pass 
    except Exception as e:
        print(f"[WARN] Failed to send ntfy: {e}")

def get_folder_bag_stats(folder_path):
    total = sum(f.stat().st_size for f in folder_path.glob("*.bag"))
    count = len(list(folder_path.glob("*.bag")))
    for unit in ['B', 'KB', 'MB', 'GB']:
        if total < 1024.0: break
        total /= 1024.0
    return count, f"{total:.2f} {unit}"

# --- Core Logic ---
def run_batch_conversion(src_root, dst_root, converter_cmd, ntfy_topic=None, **kwargs):
    dry_run = kwargs.get('dry_run', False)
    src_root = Path(src_root).resolve()
    dst_root = Path(dst_root).resolve()
    
    if not src_root.exists():
        print(f"[Error] Source directory not found: {src_root}")
        return

    tasks = []
    for dirpath, _, filenames in os.walk(src_root):
        if any(f.endswith('.bag') for f in filenames):
            tasks.append(Path(dirpath))

    total_tasks = len(tasks)
    print(f"Found {total_tasks} folders to process.")

    mode_prefix = "[DRY RUN] " if dry_run else ""
    send_ntfy(ntfy_topic, f"{mode_prefix}Start batch: {total_tasks} folders.", title=f"{mode_prefix}Batch Started", tags=["rocket"])

    start_time = time.time()
    success_count, fail_count, skipped_count = 0, 0, 0

    try:
        for index, current_input_path in enumerate(tasks, 1):
            relative_path = current_input_path.relative_to(src_root)
            target_output_folder = dst_root / relative_path
            
            # --- Skip Logic ---
            should_skip = False
            summary_file = target_output_folder / "conversion_summary.yaml"
            metadata_file = target_output_folder / "metadata.yaml"
            
            if summary_file.exists():
                try:
                    with open(summary_file, 'r') as f:
                        if yaml.safe_load(f).get("status") == "SUCCESS": should_skip = True
                except: pass
            elif metadata_file.exists():
                should_skip = True

            if should_skip:
                print(f"[{index}/{total_tasks}] [SKIP] {relative_path}")
                skipped_count += 1
                continue

            print(f"\n[{index}/{total_tasks}] [Processing] {relative_path}")
            
            # --- Command Construction ---
            cmd = [converter_cmd, str(current_input_path)]
            
            if kwargs.get('series'): cmd.append("--series")
            if kwargs.get('with_plugins'): cmd.append("--with-plugins")
            if kwargs.get('split_size'): cmd.extend(["--split-size", kwargs['split_size']])
            if kwargs.get('distro'): cmd.extend(["--distro", kwargs['distro']])
            
            if kwargs.get('skip_topics'):
                cmd.append("--skip-topics")
                cmd.extend(kwargs['skip_topics']) 

            if dry_run: cmd.append("--dry-run")
            
            cmd.extend(["--out-dir", str(target_output_folder)])

            print(f"  [EXEC] {' '.join(cmd)}")

            if dry_run:
                success_count += 1 
            else:
                try:
                    # Create parent of output explicitly
                    target_output_folder.parent.mkdir(parents=True, exist_ok=True)
                    
                    t0 = time.time()
                    
                    # [FIXED] Removed stdout/stderr capture. 
                    # Output (including progress bars) now streams directly to your terminal.
                    subprocess.run(cmd, check=True)
                    
                    duration = time.time() - t0
                    success_count += 1
                    send_ntfy(ntfy_topic, f"Converted: {relative_path}\n{duration:.1f}s", title=f"Success {index}/{total_tasks}", tags=["white_check_mark"])
                    
                except subprocess.CalledProcessError as e:
                    if e.returncode == 130: break 
                    fail_count += 1
                    
                    print(f"  [ERROR] Process failed with exit code {e.returncode}")
                    
                    # Since we aren't capturing output, we tell the user to check the screen.
                    send_ntfy(ntfy_topic, f"Failed: {relative_path}\nCheck console logs.", title="Error", priority="high", tags=["x"])
                except Exception as e:
                    fail_count += 1
                    print(f"  [CRITICAL] {e}")

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")

    summary = f"Finished.\nSuccess: {success_count}\nFail: {fail_count}\nSkipped: {skipped_count}"
    print(f"\n{summary}")
    send_ntfy(ntfy_topic, summary, title=f"{mode_prefix}Batch Complete", tags=["tada"])

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--converter", default="convert_bag")
    parser.add_argument("--ntfy", default=None)
    parser.add_argument("--dry-run", action="store_true")
    
    parser.add_argument("--series", action="store_true")
    parser.add_argument("--split-size", default=None)
    parser.add_argument("--distro", default="humble")
    parser.add_argument("--with-plugins", action="store_true")
    parser.add_argument("--skip-topics", nargs='+', default=[])

    args = parser.parse_args()
    
    run_batch_conversion(args.input, args.output, args.converter, ntfy_topic=args.ntfy, **vars(args))