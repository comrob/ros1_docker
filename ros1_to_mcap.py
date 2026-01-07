import sys
import signal
import os
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores, get_types_from_msg
from mcap.writer import Writer as McapWriter

def convert_ros1_to_mcap(input_bag: str, output_mcap: str):
    if os.path.abspath(input_bag) == os.path.abspath(output_mcap):
        print(f"\n[ERROR] Input and Output cannot be the same file!")
        print(f"        You are trying to overwrite: {input_bag}")
        print(f"        Aborting to protect your data.")
        sys.exit(1)
    print(f"🚀 Opening {input_bag}...")
    
    # --- Signal Handling ---
    state = {'keep_running': True}
    def handler(signum, frame):
        print(f"\n🛑 Signal {signum} received. Stopping loop and saving...")
        state['keep_running'] = False
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    # --- Type Store Setup ---
    typestore = get_typestore(Stores.LATEST)
    
    with Reader(input_bag) as reader:
        with open(output_mcap, "wb") as f:
            writer = McapWriter(f)
            writer.start(profile="ros2", library="ros1_to_mcap_converter")

            try:
                print(f"🔍 Analyzing {len(reader.connections)} connections...")

                conn_id_to_channel_id = {}
                type_name_to_schema_id = {}
                topic_to_channel_id = {}

                # 1. REGISTER SCHEMAS & CHANNELS
                for conn in reader.connections:
                    if not state['keep_running']: break

                    ros1_type = conn.msgtype
                    
                    # Convert names: "pkg/Type" -> "pkg/msg/Type"
                    parts = ros1_type.split('/')
                    if len(parts) == 2:
                        ros2_type = f"{parts[0]}/msg/{parts[1]}"
                    else:
                        ros2_type = ros1_type

                    # --- FIX: Register Custom Types with Conflict Filtering ---
                    if ros2_type not in typestore.types:
                        try:
                            msg_text = conn.msgdef
                            new_types = get_types_from_msg(msg_text, ros2_type)
                            
                            # FILTER STEP: Only register types that don't already exist
                            # This drops 'std_msgs/msg/Header' from the new set, preventing the error
                            types_to_register = {
                                k: v for k, v in new_types.items() 
                                if k not in typestore.types
                            }
                            
                            if types_to_register:
                                typestore.register(types_to_register)
                                # print(f"✨ Registered custom type: {ros2_type}")
                                
                        except Exception as e:
                            print(f"⚠️  [SKIP] {conn.topic}: Could not register type '{ros2_type}'. Reason: {e}")
                            continue

                    if ros2_type not in typestore.types:
                         print(f"⚠️  [SKIP] {conn.topic}: Type '{ros2_type}' still not found.")
                         continue

                    # A. Register Schema (Deduplicated)
                    if ros2_type not in type_name_to_schema_id:
                        schema_id = writer.register_schema(
                            name=ros2_type,
                            encoding="ros2msg",
                            data=b"" 
                        )
                        type_name_to_schema_id[ros2_type] = schema_id
                    schema_id = type_name_to_schema_id[ros2_type]

                    # B. Register Channel (Deduplicated)
                    if conn.topic in topic_to_channel_id:
                        channel_id = topic_to_channel_id[conn.topic]
                    else:
                        channel_id = writer.register_channel(
                            topic=conn.topic,
                            message_encoding="cdr",
                            schema_id=schema_id
                        )
                        topic_to_channel_id[conn.topic] = channel_id

                    conn_id_to_channel_id[conn.id] = channel_id

                # 2. CONVERT MESSAGES
                print(f"✅ Converting messages...")
                count = 0
                
                for connection, timestamp, rawdata in reader.messages():
                    if not state['keep_running']: break
                    
                    if connection.id not in conn_id_to_channel_id:
                        continue 

                    channel_id = conn_id_to_channel_id[connection.id]
                    
                    try:
                        cdr_data = typestore.ros1_to_cdr(rawdata, connection.msgtype)
                        
                        writer.add_message(
                            channel_id=channel_id,
                            log_time=timestamp,
                            data=cdr_data,
                            publish_time=timestamp
                        )
                        count += 1
                        if count % 2000 == 0:
                            print(f"   Converted {count} messages...", end="\r")
                            
                    except Exception as e:
                        # Occasional conversion errors might happen if data is corrupt
                        pass

            except Exception as e:
                print(f"\n❌ Critical Error: {e}")
            
            finally:
                writer.finish()
                print(f"\n\n🎉 Done! Saved to: {output_mcap}")
                print(f"   Total messages written: {count}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 ros1_to_mcap_v6.py <input.bag> <output.mcap>")
        sys.exit(1)
    
    convert_ros1_to_mcap(sys.argv[1], sys.argv[2])