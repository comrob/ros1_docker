import sys
import os
import re
import argparse
import signal
import time
from pathlib import Path
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores, get_types_from_msg
from mcap.writer import Writer as McapWriter

# --- Global State for Signal Handling ---
KEEP_RUNNING = True

def signal_handler(signum, frame):
    global KEEP_RUNNING
    print(f"\n[SIGNAL] 🛑 Caught signal {signum}. Stopping gracefully after current file...")
    KEEP_RUNNING = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# --- ROS 2 QoS Definitions ---
# This YAML string tells ros2 bag play to treat the topic as "Transient Local" (Latched).
# Durability 1 = Transient Local
# Reliability 1 = Reliable
LATCHED_QOS_PROFILE = """
- history: 1
  depth: 1
  reliability: 1
  durability: 1
  deadline:
    sec: 2147483647
    nsec: 4294967295
  lifespan:
    sec: 2147483647
    nsec: 4294967295
  liveliness: 1
  liveliness_lease_duration:
    sec: 2147483647
    nsec: 4294967295
  avoid_ros_namespace_conventions: false
""".strip()

def get_ros2_type_name(ros1_type: str) -> str:
    """Normalizes pkg/Type to pkg/msg/Type if needed."""
    parts = ros1_type.split('/')
    if len(parts) == 2:
        return f"{parts[0]}/msg/{parts[1]}"
    return ros1_type

class BatchConverter:
    def __init__(self):
        self.typestore = get_typestore(Stores.LATEST)
        # Memory to store latched messages across split files
        # Structure: {topic: (ros2_type_name, cdr_data)}
        self.latched_msgs = {}

    def convert_batch(self, file_list, output_root_dir):
        """
        Processes a list of bag files as a single continuous sequence.
        """
        # 1. Determine Batch Name (Directory Name)
        first_file = Path(file_list[0])
        batch_name = re.sub(r'_\d+\.bag$', '', first_file.name)
        if batch_name == first_file.name:
            batch_name = first_file.stem
            
        # 2. Create Output Directory
        batch_dir = output_root_dir / batch_name
        batch_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"\n" + "="*60)
        print(f"[JOB START] Batch Name: {batch_name}")
        print(f"            Output Dir: {batch_dir}")
        print(f"            Files:      {len(file_list)}")
        print(f"="*60)

        total_msgs = 0
        
        # 3. Process Sequence
        for i, bag_path in enumerate(file_list):
            if not KEEP_RUNNING: 
                print("[STOP] User interruption detected. Aborting batch.")
                break
            
            bag_path = Path(bag_path)
            mcap_name = f"{bag_path.stem}.mcap"
            output_mcap = batch_dir / mcap_name
            
            print(f"\n[FILE {i+1}/{len(file_list)}] Processing: {bag_path.name}")
            
            try:
                msgs_written = self._convert_single_file(bag_path, output_mcap, part_index=i)
                total_msgs += msgs_written
            except Exception as e:
                print(f"[ERROR] Failed to convert file {bag_path.name}: {e}")
            
        print(f"\n[JOB END] Batch finished. Total messages saved: {total_msgs}")

    def _convert_single_file(self, input_path, output_path, part_index):
        count = 0
        start_time = time.time()
        last_log = time.time()

        print(f"    [INIT] Output: {output_path}")

        with Reader(input_path) as reader:
            with open(output_path, "wb") as f:
                writer = McapWriter(f)
                writer.start(profile="ros2", library="ros1_to_mcap_converter")
                
                type_map = {}    # ros2_type -> schema_id
                topic_map = {}   # topic -> channel_id
                conn_map = {}    # conn_id -> channel_id

                # --- A. Inject Context (Latched Messages) ---
                if self.latched_msgs and part_index > 0:
                    print(f"    [CTX]  Injecting {len(self.latched_msgs)} latched topics from previous files...")
                    for topic, (ros2_type, data) in self.latched_msgs.items():
                        s_id = self._get_schema_id(writer, ros2_type, type_map)
                        c_id = self._get_channel_id(writer, topic, s_id, topic_map)
                        
                        writer.add_message(channel_id=c_id, log_time=0, data=data, publish_time=0)

                # --- B. Register Connections & Types ---
                for conn in reader.connections:
                    ros2_type = get_ros2_type_name(conn.msgtype)
                    
                    # 1. Register Type Definition if new
                    if ros2_type not in self.typestore.types:
                        try:
                            new_types = get_types_from_msg(conn.msgdef, ros2_type)
                            self.typestore.register({k:v for k,v in new_types.items() if k not in self.typestore.types})
                        except Exception as e:
                            print(f"    [WARN] Type Registration Failed: {ros2_type} (Topic: {conn.topic})")
                            continue

                    # 2. Prepare Writer Schema/Channel
                    try:
                        s_id = self._get_schema_id(writer, ros2_type, type_map)
                        c_id = self._get_channel_id(writer, conn.topic, s_id, topic_map)
                        conn_map[conn.id] = c_id
                    except Exception as e:
                         print(f"    [WARN] Schema/Channel Reg Failed: {conn.topic} ({ros2_type}) -> {e}")

                # --- C. Write Loop ---
                print(f"    [RUN]  Starting message conversion loop...")
                
                for conn, timestamp, rawdata in reader.messages():
                    if not KEEP_RUNNING: break
                    if conn.id not in conn_map: continue
                    
                    try:
                        cdr = self.typestore.ros1_to_cdr(rawdata, conn.msgtype)
                        ros2_type = get_ros2_type_name(conn.msgtype) # Needed for tracking? Not really, mostly for debugging
                        
                        # Capture Latching (e.g., tf_static) for NEXT files
                        if conn.topic == '/tf_static':
                            # Save type info so we can reconstruct later
                            self.latched_msgs[conn.topic] = (get_ros2_type_name(conn.msgtype), cdr)
                        
                        writer.add_message(
                            channel_id=conn_map[conn.id],
                            log_time=timestamp,
                            data=cdr,
                            publish_time=timestamp
                        )
                        count += 1
                        
                        # Logging
                        now = time.time()
                        if now - last_log > 1.0:
                            elapsed = now - start_time
                            rate = count / elapsed if elapsed > 0 else 0
                            print(f"           Msg: {count} | Rate: {rate:.0f} hz", end="\r")
                            last_log = now

                    except Exception as e:
                        pass
                
                # --- D. Finalize ---
                print(f"           Msg: {count} | Rate: {(count/(time.time()-start_time)):.0f} hz [COMPLETE]")
                print(f"    [FIN]  Building index and flushing to disk... (Do not interrupt)")
                writer.finish()
                print(f"    [DONE] File closed.")
                return count

    def _get_schema_id(self, writer, ros2_type, type_map):
        if ros2_type not in type_map:
            type_map[ros2_type] = writer.register_schema(name=ros2_type, encoding="ros2msg", data=b"")
        return type_map[ros2_type]

    def _get_channel_id(self, writer, topic, schema_id, topic_map):
        if topic not in topic_map:
            # --- QOS FIX APPLIED HERE ---
            # If the topic is static TF, we attach the ROS 2 Latched QoS profile metadata
            metadata = {}
            if topic == '/tf_static':
                metadata = {"offered_qos_profiles": LATCHED_QOS_PROFILE}

            topic_map[topic] = writer.register_channel(
                topic=topic, 
                message_encoding="cdr", 
                schema_id=schema_id,
                metadata=metadata  # <--- Metadata injection
            )
        return topic_map[topic]


def scan_folder_for_batches(folder_path):
    folder = Path(folder_path)
    all_bags = sorted([f for f in folder.iterdir() if f.suffix == '.bag' and f.is_file()])
    batches = {}
    pattern = re.compile(r"^(.*)(_\d+)?\.bag$")
    
    for bag in all_bags:
        match = pattern.match(bag.name)
        if match:
            batch_id = match.group(1) if match.group(2) else bag.stem
        else:
            batch_id = bag.stem

        if batch_id not in batches:
            batches[batch_id] = []
        batches[batch_id].append(bag)

    for bid in batches:
        batches[bid].sort()
    return list(batches.values())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS1 bags to MCAP batches")
    parser.add_argument("inputs", nargs="+", help="Input file(s) or folder")
    parser.add_argument("--batch", action="store_true", help="Force treating explicit list of files as one single batch")
    args = parser.parse_args()

    converter = BatchConverter()
    jobs = [] 
    input_path = Path(args.inputs[0])
    
    if input_path.is_dir():
        print(f"[INIT] Mode: FOLDER SCAN -> {input_path}")
        jobs = scan_folder_for_batches(input_path)
        print(f"[INIT] Detected {len(jobs)} unique recording sequence(s).")
    elif len(args.inputs) > 1:
        if args.batch:
            print(f"[INIT] Mode: EXPLICIT BATCH")
            jobs = [args.inputs]
        else:
            print(f"[INIT] Mode: INDEPENDENT FILES")
            jobs = [[f] for f in args.inputs]
    else:
        print(f"[INIT] Mode: SINGLE FILE")
        jobs = [[input_path]]

    for job_files in jobs:
        if not KEEP_RUNNING: break
        first_file = Path(job_files[0])
        output_dir = first_file.parent
        converter.latched_msgs.clear()
        converter.convert_batch(job_files, output_dir)