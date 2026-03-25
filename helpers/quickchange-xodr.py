import xml.etree.ElementTree as ET
import argparse

def process_xodr_file(input_file_path, output_file_path, shift_x=0.0, shift_y=0.0, width_increase=None, center=False, symmetric_widening=False):
    """
    Parses an OpenDRIVE (XODR) file to find the minimum x and y coordinates,
    then optionally shifts all geometry coordinates to be relative to this new origin,
    and saves the result to a new file.

    This version correctly handles XML namespaces.

    Args:
        input_file_path (str): The path to the input .xodr file.
        output_file_path (str): The path where the modified .xodr file will be saved.
        width_increase (float, optional): If provided, changes the width of road lanes. Defaults to None.
        symmetric_widening (bool, optional): If True, treats width_increase as the total road width increase, 
            distributes it evenly among lanes, and adds a <laneOffset> so the road widens symmetrically from its center.
    """
    try:
        tree = ET.parse(input_file_path)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing XML file: {e}")
        return
    except FileNotFoundError:
        print(f"Error: Input file not found at '{input_file_path}'")
        return

    # The namespace is defined in the root element. We need to use it to find other elements.
    namespace = ''
    if '}' in root.tag:
        namespace = root.tag.split('}')[0][1:]
    
    if not namespace:
        print("Could not determine XML namespace from root element.")
        return

    ns = {'od': namespace}
    ET.register_namespace('', namespace) # For cleaner output file

    min_x = float('inf')
    min_y = float('inf')

    # First pass: find the minimum x and y values from geometry elements
    geometries = root.findall('.//od:geometry', ns)
    if not geometries:
        print("Warning: No <geometry> tags found. The file might not be modified.")

    for geometry in geometries:
        try:
            x = float(geometry.get('x'))
            y = float(geometry.get('y'))
            if x < min_x:
                min_x = x
            if y < min_y:
                min_y = y
        except (ValueError, TypeError):
            continue
            
    # Also consider the header coordinates which define the bounding box
    header = root.find('od:header', ns)
    if header is not None:
        try:
            west = float(header.get('west'))
            south = float(header.get('south'))
            if west < min_x:
                min_x = west
            if south < min_y:
                min_y = south
        except (ValueError, TypeError):
            print("Warning: Could not parse header's west/south attributes.")
            pass

    if min_x == float('inf') or min_y == float('inf'):
        print("Could not find any valid coordinates to determine minimums.")
        return

    print(f"Minimum X found: {min_x}")
    print(f"Minimum Y found: {min_y}")

    if center == True:
        # Second pass: subtract the minimum values from all geometry coordinates
        for geometry in geometries:
            try:
                original_x = float(geometry.get('x'))
                original_y = float(geometry.get('y'))
                
                geometry.set('x', f"{original_x - min_x + shift_x}")
                geometry.set('y', f"{original_y - min_y + shift_y}")
            except (ValueError, TypeError):
                continue

        # Update header with new shifted coordinates and add offset tag
        if header is not None:
            # Remove existing offset tag if present
            existing_offset = header.find('od:offset', ns)
            if existing_offset is not None:
                header.remove(existing_offset)

            geo_ref = header.find('od:geoReference', ns)
            if geo_ref is not None:
                # Find index of geoReference to insert after it
                header_children = list(header)
                try:
                    geo_ref_index = header_children.index(geo_ref)
                    offset_element = ET.Element('offset')
                    offset_element.set('x', str(min_x))
                    offset_element.set('y', str(min_y))
                    offset_element.set('z', "-0.00")
                    offset_element.set('hdg', "0")
                    header.insert(geo_ref_index + 1, offset_element)
                except ValueError:
                    print("Warning: Could not find geoReference tag to insert offset tag after.")
            
            try:
                # We assume header attributes exist if west and south were found before
                north = float(header.get('north'))
                east = float(header.get('east'))
                
                header.set('north', f"{north - min_y:.4f}")
                header.set('south', "0.0000") # it's the new min y
                header.set('east', f"{east - min_x:.4f}")
                header.set('west', "0.0000") # it's the new min x
            except (ValueError, TypeError, AttributeError) as e:
                print(f"Warning: Could not update all header attributes. {e}")
            

    # Pass to update road widths if a new width is provided
    if width_increase is not None:
        if not symmetric_widening:
            lanes = root.findall('.//od:width', ns)
            if not lanes:
                print("Warning: No <width> tags found. Cannot apply new width.")
            else:
                for width in lanes:
                    try:
                        width.set('a', str(float(width.get('a')) + width_increase))
                        width.set('b', "0")
                        width.set('c', "0")
                        width.set('d', "0")
                    except (ValueError, TypeError) as e:
                        print(f"Warning: Could not set new width for a lane. {e}")
                print(f"Changed all lane widths by {width_increase} meters.")
        else:
            roads = root.findall('.//od:road', ns)
            lanes_processed = 0
            for road in roads:
                lanes_element = road.find('od:lanes', ns)
                if lanes_element is None:
                    continue
                
                lane_sections = lanes_element.findall('od:laneSection', ns)
                if not lane_sections:
                    continue
                    
                first_ls_idx = list(lanes_element).index(lane_sections[0])
                insert_idx = first_ls_idx
                
                for ls in lane_sections:
                    s_val = ls.get('s', '0.0')
                    
                    left_elem = ls.find('od:left', ns)
                    right_elem = ls.find('od:right', ns)
                    
                    left_lanes = []
                    if left_elem is not None:
                        left_lanes = [l for l in left_elem.findall('od:lane', ns) if l.find('od:width', ns) is not None and l.get('type') != 'none']
                        
                    right_lanes = []
                    if right_elem is not None:
                        right_lanes = [l for l in right_elem.findall('od:lane', ns) if l.find('od:width', ns) is not None and l.get('type') != 'none']
                    
                    n_left = len(left_lanes)
                    n_right = len(right_lanes)
                    n_total = n_left + n_right
                    
                    if n_total == 0:
                        continue
                        
                    lane_inc = width_increase / n_total
                    
                    for l in left_lanes + right_lanes:
                        width = l.find('od:width', ns)
                        if width is not None:
                            try:
                                width.set('a', str(float(width.get('a')) + lane_inc))
                                width.set('b', "0")
                                width.set('c', "0")
                                width.set('d', "0")
                                lanes_processed += 1
                            except (ValueError, TypeError) as e:
                                print(f"Warning: Could not set new width for a lane. {e}")
                    
                    # Add laneOffset shift to keep road centered
                    # Positive delta means shift road left. Left lanes expand left, right lanes expand right.
                    delta_offset = - (n_left - n_right) * lane_inc / 2.0
                    
                    if delta_offset != 0:
                        # Find existing laneOffset at this s_val
                        existing_offsets = lanes_element.findall('od:laneOffset', ns)
                        offset_at_s = None
                        for off in existing_offsets:
                            if off.get('s') == s_val:
                                offset_at_s = off
                                break
                        
                        if offset_at_s is not None:
                            try:
                                old_a = float(offset_at_s.get('a', '0.0'))
                                offset_at_s.set('a', str(old_a + delta_offset))
                                offset_at_s.set('b', "0")
                                offset_at_s.set('c', "0")
                                offset_at_s.set('d', "0")
                            except (ValueError, TypeError):
                                print("Warning: Could not parse old laneOffset 'a' value.")
                        else:
                            # Create new laneOffset
                            off_tag = f"{{{namespace}}}laneOffset" if namespace else 'laneOffset'
                            new_offset = ET.Element(off_tag)
                            new_offset.set('s', s_val)
                            new_offset.set('a', str(delta_offset))
                            new_offset.set('b', "0")
                            new_offset.set('c', "0")
                            new_offset.set('d', "0")
                            new_offset.tail = "\n      " # Basic formatting
                            lanes_element.insert(insert_idx, new_offset)
                            insert_idx += 1
            print(f"Symmetrically widened {lanes_processed} lanes by total road width {width_increase} meters.")

    # Write the modified tree to a new file
    try:
        tree.write(output_file_path, encoding='UTF-8', xml_declaration=True)
        print(f"Successfully processed file and saved to '{output_file_path}'")
    except IOError as e:
        print(f"Error writing to output file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Modify OpenDRIVE (XODR) geometry coordinates and lane widths.")
    parser.add_argument("input_file", help="Path to the input .xodr file.")
    parser.add_argument("output_file", help="Path where the modified .xodr file will be saved.")
    parser.add_argument("--shift-x", type=float, default=0.0, help="Shift all X coordinates by this amount (default: 0.0).")
    parser.add_argument("--shift-y", type=float, default=0.0, help="Shift all Y coordinates by this amount (default: 0.0).")
    parser.add_argument("--width-increase", type=float, default=None, help="Change the width of road lanes by this amount.")
    parser.add_argument("--center", action="store_true", help="Center the geometry by moving the minimum coordinates to the origin.")
    parser.add_argument("--symmetric-widening", action="store_true", help="Treat width_increase as total road width increase and distribute symmetrically.")

    args = parser.parse_args()

    process_xodr_file(
        args.input_file,
        args.output_file,
        shift_x=args.shift_x,
        shift_y=args.shift_y,
        width_increase=args.width_increase,
        center=args.center,
        symmetric_widening=args.symmetric_widening
    )
