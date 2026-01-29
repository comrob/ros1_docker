import re
from pathlib import Path
from typing import Optional, List
from mcap.writer import Writer as McapWriter

def parse_size(size_str: Optional[str]) -> int:
    """Converts human readable size (3G, 500M) to bytes. Returns 0 if None."""
    if not size_str:
        return 0
    units = {"K": 1024, "M": 1024**2, "G": 1024**3, "T": 1024**4}
    match = re.match(r'^(\d+)([KMGT]?)$', size_str.upper())
    if match:
        val, unit = match.groups()
        return int(val) * units.get(unit, 1)
    raise ValueError(f"Invalid size format: {size_str}")

class RollingMcapWriter:
    def __init__(self, base_path: Path, split_size: int, profile: str):
        self.base_path = base_path
        self.split_size = split_size
        self.profile = profile
        self.part_num = 0
        
        # Track all generated file segments for final renaming
        self.generated_segments: List[Path] = []
        
        # State preservation for rotation
        self.schemas = {}
        self.channels = {}
        
        # Current active writer state
        self.writer = None
        self._f = None
        self.current_file_path = None # This will be the .part path
        
        # Signal that a rotation happened
        self.just_rotated = False

        self._open_new_segment()
        
        # Reset flag after initial open
        self.just_rotated = False

    def _open_new_segment(self):
        """Closes current and opens next file segment."""
        if self.writer:
            self.writer.finish()
            self._f.close()

        # Determine the target "final" name (e.g. "data.mcap" or "data_1.mcap")
        if self.part_num == 0:
            final_name = self.base_path
        else:
            stem = self.base_path.stem
            final_name = self.base_path.parent / f"{stem}_{self.part_num}.mcap"
        
        # ALWAYS write to a .part file initially
        # e.g., data.mcap -> data.mcap.part
        self.current_file_path = final_name.with_suffix(final_name.suffix + ".part")
        self.generated_segments.append(self.current_file_path)

        self._f = open(self.current_file_path, "wb")
        self.writer = McapWriter(self._f)
        self.writer.start(profile=self.profile)
        
        # Re-register known state
        for name, (enc, data) in self.schemas.items():
            self.writer.register_schema(name=name, encoding=enc, data=data)
            
        for topic, (schema_id, enc, meta) in self.channels.items():
            self.writer.register_channel(
                topic=topic,
                schema_id=schema_id, 
                message_encoding=enc,
                metadata=meta
            )
            
        self.part_num += 1
        self.just_rotated = True

    def register_schema(self, name, encoding, data):
        if name not in self.schemas:
            self.schemas[name] = (encoding, data)
        return self.writer.register_schema(name=name, encoding=encoding, data=data)

    def register_channel(self, topic, message_encoding, schema_id, metadata):
        if topic not in self.channels:
            self.channels[topic] = (schema_id, message_encoding, metadata)
        return self.writer.register_channel(topic=topic, message_encoding=message_encoding, schema_id=schema_id, metadata=metadata)

    def add_message(self, channel_id, log_time, publish_time, data):
        if self.split_size > 0 and self._f.tell() > self.split_size:
            self._open_new_segment()
            
        self.writer.add_message(
            channel_id=channel_id, 
            log_time=log_time, 
            data=data, 
            publish_time=publish_time
        )

    def finish(self):
        if self.writer:
            self.writer.finish()
            self._f.close()

    def finalize_filenames(self, success: bool = True) -> List[Path]:
        """
        Renames all generated .part files to their final state.
        If success=True:  .mcap.part -> .mcap
        If success=False: .mcap.part -> .mcap.incomplete
        Returns list of final paths.
        """
        # Ensure everything is closed first
        self.finish()

        final_paths = []
        
        for temp_path in self.generated_segments:
            if not temp_path.exists():
                continue
                
            # temp_path is like "data.mcap.part"
            # Remove ".part" to get the base "data.mcap"
            base_name = temp_path.with_suffix("") 
            
            if success:
                new_path = base_name # "data.mcap"
            else:
                # Add .incomplete suffix to the base name -> "data.mcap.incomplete"
                new_path = base_name.with_suffix(base_name.suffix + ".incomplete")

            try:
                temp_path.rename(new_path)
                final_paths.append(new_path)
            except OSError as e:
                print(f"[ERR] Failed to rename {temp_path.name}: {e}")
        
        return final_paths