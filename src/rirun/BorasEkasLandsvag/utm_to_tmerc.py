#!/usr/bin/env python3
"""
Convert PROJ string from UTM to TMERC (Transverse Mercator) format.
UTM zones use specific parameters that can be expressed as general TMERC projections.
"""

import re
import sys


def utm_to_tmerc(proj_string):
    """
    Convert UTM PROJ string to TMERC format.
    
    Args:
        proj_string: PROJ string with UTM projection
        
    Returns:
        PROJ string with TMERC projection
    """
    # Parse the PROJ string
    params = {}
    for match in re.finditer(r'\+(\w+)(?:=([^\s]+))?', proj_string):
        key = match.group(1)
        value = match.group(2) if match.group(2) else True
        params[key] = value
    
    # Check if it's a UTM projection
    if params.get('proj') != 'utm':
        raise ValueError("Input projection must be UTM (+proj=utm)")
    
    # Get UTM zone
    zone = params.get('zone')
    if not zone:
        raise ValueError("UTM zone not specified (+zone=N)")
    
    zone = int(zone)
    if zone < 1 or zone > 60:
        raise ValueError(f"Invalid UTM zone: {zone}. Must be between 1 and 60.")
    
    # Determine if southern hemisphere
    is_south = 'south' in params
    
    # Calculate central meridian for the zone
    # UTM zones: central meridian = (zone * 6) - 183
    lon_0 = (zone * 6) - 183
    
    # UTM parameters
    lat_0 = 0  # Latitude of origin
    k_0 = 0.9996  # Scale factor
    x_0 = 500000  # False easting
    y_0 = 10000000 if is_south else 0  # False northing (10M for south)
    
    # Build TMERC PROJ string
    tmerc_params = [
        '+proj=tmerc',
        f'+lat_0={lat_0}',
        f'+lon_0={lon_0}',
        f'+k_0={k_0}',
        f'+x_0={x_0}',
        f'+y_0={y_0}'
    ]
    
    # Copy over datum, ellipsoid, and units
    for key in ['datum', 'ellps', 'units', 'towgs84', 'nadgrids']:
        if key in params:
            tmerc_params.append(f'+{key}={params[key]}')
    
    # Add no_defs if present
    if 'no_defs' in params:
        tmerc_params.append('+no_defs')
    
    return ' '.join(tmerc_params)


def main():
    """Main function for command-line usage."""
    if len(sys.argv) < 2:
        print("Usage: python utm_to_tmerc.py '<PROJ_STRING>'")
        print("   or: python utm_to_tmerc.py +proj=utm +zone=33 +datum=WGS84 +units=m +no_defs")
        print("\nExample with quotes:")
        print("  python utm_to_tmerc.py '+proj=utm +zone=33 +datum=WGS84 +units=m +no_defs'")
        print("\nExample without quotes:")
        print("  python utm_to_tmerc.py +proj=utm +zone=33 +datum=WGS84 +units=m +no_defs")
        print("\nOr use as a module:")
        print("  from utm_to_tmerc import utm_to_tmerc")
        print("  result = utm_to_tmerc('+proj=utm +zone=33 +datum=WGS84')")
        sys.exit(1)
    
    # Join all arguments (in case user didn't quote the string)
    proj_string = ' '.join(sys.argv[1:])
    
    try:
        result = utm_to_tmerc(proj_string)
        print("Input (UTM):")
        print(f"  {proj_string}")
        print("\nOutput (TMERC):")
        print(f"  {result}")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) == 1:
        # Show examples when no arguments provided
        examples = [
            "+proj=utm +zone=33 +datum=WGS84 +units=m +no_defs",
            "+proj=utm +zone=10 +south +ellps=WGS84 +units=m",
            "+proj=utm +zone=17 +datum=NAD83 +units=m +no_defs",
        ]
        
        print("UTM to TMERC Converter - Examples:\n")
        for ex in examples:
            print(f"Input:  {ex}")
            print(f"Output: {utm_to_tmerc(ex)}\n")
    else:
        main()