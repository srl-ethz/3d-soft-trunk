import xacro
import sys

"""
usage: python3 xacro2urdf.py myFile.urdf.xacro myFile.urdf
need to `pip3 install xacro`
"""

input_filename = sys.argv[1]
output_filename = sys.argv[2]
print(f"converting XACRO: {input_filename} to URDF: {output_filename}")

output_xml = xacro.process_file(input_filename)

with open(output_filename, "w") as f:
    f.write(output_xml.toprettyxml())
