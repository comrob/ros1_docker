import sys
import signal
import os
import time
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores, get_types_from_msg
from mcap.writer import Writer as McapWriter

# Toggle this to True if you want to see every single message type being registered
DEBUG_MODE = False

def convert_ros1_to_mcap(input_bag: str, output_mcap: str = None):
    # --- Auto-generate output filename ---
    if output_mcap is None:
        base, _ = os.path.splitext(input_bag)
        output_mcap = f"{base}.mcap"

    if os.path.abspath(input_bag) == os.path.abspath(output_mcap):
        print(f"[ERROR] Input and Output cannot be the same file.")
        sys.exit(1)
        
    print(f"[INFO] Opening: {input_bag}")
    print(f"[INFO] Target:  {output_mcap}")
    print(f"[INFO] Press Ctrl+C ONCE to stop gracefully (save data).")
    print(f"[INFO] Press Ctrl+C TWICE to force quit immediately (corrupt data).")
    
    # --- Signal Handling ---
    state = {
        'keep_running': True, 
        'sig_count': 0,
        'current_action': 'Initializing'
    }
    
    def handler(signum, frame):
        state['sig_count'] += 1
        if state['sig_count'] == 1:
            print(f"\n\n[SIGNAL] Caught Ctrl+C. Stopping after current message...")
            print(f"         Current State: {state['current_action']}")
            print(f"         (Press Ctrl+C again to FORCE KILL)")
            state['keep_running'] = False
        else:
            print(f"\n[SIGNAL] Force Kill received. Exiting immediately!")
            sys.exit(1)
            
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    # --- Type Store Setup ---
    typestore = get_typestore(Stores.LATEST)
    
    with Reader(input_bag) as reader:
        with open(output_mcap, "wb") as f:
            writer = McapWriter(f)
            writer.start(profile="ros2", library="ros1_to_mcap_converter")

            try:
                print(f"[INFO] Scanning connections...")
                state['current_action'] = "Registering Types"

                conn_id_to_channel_id = {}
                type_name_to_schema_id = {}
                topic_to_channel_id = {}

                # 1. REGISTER SCHEMAS & CHANNELS
                for conn in reader.connections:
                    if not state['keep_running']: break

                    ros1_type = conn.msgtype
                    parts = ros1_type.split('/')
                    if len(parts) == 2:
                        ros2_type = f"{parts[0]}/msg/{parts[1]}"
                    else:
                        ros2_type = ros1_type

                    # Register Type if missing
                    if ros2_type not in typestore.types:
                        try:
                            msg_text = conn.msgdef
                            new_types = get_types_from_msg(msg_text, ros2_type)
                            types_to_register = {k: v for k, v in new_types.items() if k not in typestore.types}
                            if types_to_register:
                                typestore.register(types_to_register)
                                if DEBUG_MODE: print(f"[DEBUG] Registered: {ros2_type}")
                        except Exception as e:
                            print(f"[WARN] Failed to register {ros2_type}: {e}")
                            continue

                    # Register Schema
                    if ros2_type not in type_name_to_schema_id:
                        schema_id = writer.register_schema(name=ros2_type, encoding="ros2msg", data=b"")
                        type_name_to_schema_id[ros2_type] = schema_id
                    schema_id = type_name_to_schema_id[ros2_type]

                    # Register Channel
                    if conn.topic not in topic_to_channel_id:
                        channel_id = writer.register_channel(topic=conn.topic, message_encoding="cdr", schema_id=schema_id)
                        topic_to_channel_id[conn.topic] = channel_id
                    
                    conn_id_to_channel_id[conn.id] = topic_to_channel_id[conn.topic]

                # 2. CONVERT MESSAGES
                print(f"[INFO] Starting conversion loop...")
                count = 0
                start_time = time.time()
                last_log = time.time()
                
                state['current_action'] = "Reading/Writing Messages"
                
                for connection, timestamp, rawdata in reader.messages():
                    if not state['keep_running']: 
                        print("\n[INFO] Loop break triggered by user.")
                        break
                    
                    if connection.id not in conn_id_to_channel_id:
                        continue 

                    channel_id = conn_id_to_channel_id[connection.id]
                    
                    try:
                        cdr_data = typestore.ros1_to_cdr(rawdata, connection.msgtype)
                        writer.add_message(channel_id=channel_id, log_time=timestamp, data=cdr_data, publish_time=timestamp)
                        count += 1
                        
                        # Time-based logging (every 1.0 second) to avoid spamming I/O
                        now = time.time()
                        if now - last_log > 1.0:
                            elapsed = now - start_time
                            rate = count / elapsed if elapsed > 0 else 0
                            print(f"       Processed {count} msgs | Time: {elapsed:.1f}s | Rate: {rate:.0f} msg/s", end="\r")
                            last_log = now
                            
                    except Exception as e:
                        pass

            except Exception as e:
                print(f"\n[FATAL] Error during conversion: {e}")
            
            finally:
                # 3. CLOSING CEREMONY
                print(f"\n\n[STATUS] Conversion loop finished.")
                if count > 0:
                    print(f"[STATUS] 🛑 STARTING WRITER FINISH. DO NOT INTERRUPT unless you want a broken file.")
                    print(f"[STATUS] This step builds the index and flushes buffers to disk.")
                    state['current_action'] = "Writer Finishing (Indexing & Flushing)"
                    
                    t_fin_start = time.time()
                    writer.finish()
                    t_fin_end = time.time()
                    
                    print(f"[STATUS] Writer finish took {t_fin_end - t_fin_start:.2f} seconds.")
                else:
                    print("[STATUS] No messages written, skipping finish.")
                    
                print(f"[DONE] Saved to: {output_mcap}")
                print(f"       Total messages: {count}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 ros1_to_mcap.py <input.bag> [output.mcap]")
        sys.exit(1)
    
    out_file = sys.argv[2] if len(sys.argv) > 2 else None
    convert_ros1_to_mcap(sys.argv[1], out_file)