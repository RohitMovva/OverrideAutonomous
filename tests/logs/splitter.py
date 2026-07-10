#!/usr/bin/env python3
"""
Log File Splitter - Splits large log files based on "Robot initialized" pattern
Usage: python log_splitter.py <input_file> [output_directory]
"""

import re
import os
import sys
from pathlib import Path

def split_log_file(input_file, output_dir=None):
    """
    Split a log file based on the "Robot initialized" pattern.
    
    Args:
        input_file (str): Path to the input log file
        output_dir (str): Directory to save split files (optional)
    """
    
    # Set default output directory
    if output_dir is None:
        output_dir = f"{Path(input_file).stem}_split"
    
    # Create output directory
    Path(output_dir).mkdir(exist_ok=True)
    
    # Pattern to match the Robot initialized line
    # Matches: [HH:MM:SS.mmm] [LOG] Robot initialized
    pattern = re.compile(r'^\[\d{2}:\d{2}:\d{2}\.\d{3}] \[LOG] Robot initialized')
    
    current_file = None
    file_count = 0
    line_count = 0
    
    try:
        with open(input_file, 'r', encoding='utf-8') as infile:
            print(f"Processing: {input_file}")
            
            for line_num, line in enumerate(infile, 1):
                # Check if this line matches the Robot initialized pattern
                if pattern.match(line.strip()):
                    # Close previous file if open
                    if current_file:
                        current_file.close()
                        print(f"  Closed file {file_count}: {line_count} lines")
                    
                    # Create new output file
                    file_count += 1
                    output_filename = os.path.join(output_dir, f"session_{file_count:03d}.log")
                    current_file = open(output_filename, 'w', encoding='utf-8')
                    line_count = 0
                    print(f"  Started file {file_count}: {output_filename}")
                
                # Write line to current file (if we have one)
                if current_file:
                    current_file.write(line)
                    line_count += 1
                else:
                    # Lines before first "Robot initialized" - save to preamble file
                    if file_count == 0:
                        preamble_file = os.path.join(output_dir, "preamble.log")
                        with open(preamble_file, 'a', encoding='utf-8') as preamble:
                            preamble.write(line)
            
            # Close the last file
            if current_file:
                current_file.close()
                print(f"  Closed file {file_count}: {line_count} lines")
            
            print(f"\nSplit complete!")
            print(f"  Total sessions found: {file_count}")
            print(f"  Output directory: {output_dir}")
            
            if file_count == 0:
                print("  Warning: No 'Robot initialized' patterns found in the file")
    
    except FileNotFoundError:
        print(f"Error: File '{input_file}' not found")
        return False
    except Exception as e:
        print(f"Error processing file: {e}")
        if current_file:
            current_file.close()
        return False
    
    return True

def main():
    """Main function to handle command line arguments"""
    if len(sys.argv) < 2:
        print("Usage: python log_splitter.py <input_file> [output_directory]")
        print("\nExample:")
        print("  python log_splitter.py robot.log")
        print("  python log_splitter.py robot.log ./split_logs")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None
    
    # Validate input file exists
    if not os.path.isfile(input_file):
        print(f"Error: Input file '{input_file}' does not exist")
        sys.exit(1)
    
    # Split the file
    success = split_log_file(input_file, output_dir)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()