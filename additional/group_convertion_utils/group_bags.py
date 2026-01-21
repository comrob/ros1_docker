import os
import re
import shutil
import argparse

def group_bagfiles(root_dir, dry_run=False):
    """
    Groups ROS1 bagfiles into subfolders based on iterative increasing postfix.
    """
    # Regex to capture the base name and the index (e.g., name_TIMESTAMP_0.bag)
    # Group 1: Base name (everything before the final _N)
    # Group 2: The index N
    pattern = re.compile(r'^(.*)_(\d+)\.bag$')

    for dirpath, dirnames, filenames in os.walk(root_dir):
        # Filter for .bag files only
        bag_files = sorted([f for f in filenames if f.endswith('.bag')])
        
        if not bag_files:
            continue

        print(f"Processing: {dirpath}")
        
        clusters = []
        current_cluster = []
        last_index = -1

        for bag_file in bag_files:
            match = pattern.search(bag_file)
            
            if match:
                current_index = int(match.group(2))
                
                # Logic to decide if we continue the current series or start a new one
                # 1. We have a current cluster
                # 2. The index is exactly one higher than the last one
                # 3. The index is NOT 0 (0 always indicates a start of a new split series)
                if current_cluster and (current_index == last_index + 1) and (current_index != 0):
                    current_cluster.append(bag_file)
                    last_index = current_index
                else:
                    # Close previous cluster if exists
                    if current_cluster:
                        clusters.append(current_cluster)
                    # Start new cluster
                    current_cluster = [bag_file]
                    last_index = current_index
            else:
                # File does not match pattern (e.g., no _N suffix)
                # Treat as a standalone file (own cluster)
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                
                clusters.append([bag_file])
                last_index = -1

        # Append the final cluster remaining after the loop
        if current_cluster:
            clusters.append(current_cluster)

        # Process the calculated clusters
        for cluster in clusters:
            if not cluster:
                continue

            # Determine folder name based on the first file in the cluster
            first_file = cluster[0]
            match = pattern.search(first_file)
            
            if match:
                # Use the base name (captured in regex group 1)
                # e.g., "turning_downhill_top_2025-07-10-14-22-04_0.bag" -> "turning_downhill_top_2025-07-10-14-22-04"
                folder_name = match.group(1)
            else:
                # Fallback for non-indexed files: use filename without extension
                folder_name = os.path.splitext(first_file)[0]

            # Construct full path for the new subfolder
            target_folder_path = os.path.join(dirpath, folder_name)

            # Avoid moving if the file is already in a folder with that name (sanity check)
            if os.path.basename(dirpath) == folder_name:
                print(f"  [Skip] Files are already in correct folder: {folder_name}")
                continue

            if not dry_run:
                try:
                    os.makedirs(target_folder_path, exist_ok=True)
                except OSError as e:
                    print(f"  [Error] Could not create folder {target_folder_path}: {e}")
                    continue

            # Move files
            for file_name in cluster:
                src = os.path.join(dirpath, file_name)
                dst = os.path.join(target_folder_path, file_name)
                
                if dry_run:
                    print(f"  [Dry Run] Would move '{file_name}' -> '{folder_name}/'")
                else:
                    try:
                        shutil.move(src, dst)
                        print(f"  Moved: {file_name}")
                    except Exception as e:
                        print(f"  [Error] Failed to move {file_name}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Group ROS1 bagfiles into series folders.")
    parser.add_argument("path", nargs='?', default=".", help="Root directory to scan (default: current dir)")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without moving files")
    
    args = parser.parse_args()
    
    print(f"Scanning directory: {os.path.abspath(args.path)}")
    if args.dry_run:
        print("--- DRY RUN MODE ---")
        
    group_bagfiles(args.path, dry_run=args.dry_run)
    
    print("\nDone.")