import time
import os


def rirun(duration=2):
    """Animate 2-line ASCII cars driving left→right, revealing the BIBUN text in their wake.

    Updates:
      * Cars move at the same speed.
      * Staggered start order: Car 1 enters first, then Car 3, then Car 2.
    """

    ascii_art = [
        "    ██████╗ ██╗██████╗ ██╗   ██╗███╗   ██╗",
        "    ██╔══██╗██║██╔══██╗██║   ██║████╗  ██║",
        "    ██████╔╝██║██████╔╝██║   ██║██╔██╗ ██║",
        "    ██╔══██╗██║██╔══██╗██║   ██║██║╚██╗██║",
        "    ██║  ██║██║██║  ██║╚██████╔╝██║ ╚████║",
        "    ╚═╝  ╚═╝╚═╝╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═══╝",
    ]

    # Normalize width so all rows have equal length
    cols = max(len(line) for line in ascii_art)
    ascii_art = [line.ljust(cols) for line in ascii_art]
    rows = len(ascii_art)

    # Define three 2-line cars, each with its own band of rows
    car_arts = [
        ["  __  ", "-o-o\> "],  # Car 1 (rows 0-1)
        ["  __  ", "-o-o\> "],  # Car 2 (rows 2-3)
        ["  __  ", "-o-o\> "],  # Car 3 (rows 4-5)
    ]

    cars = []
    bands = [(0, 1), (2, 3), (4, 5)]
    speed = 1  # all cars same speed

    # staggered start offsets (negative start positions so they enter later)
    # order: Car1 starts first, then Car3, then Car2
    start_offsets = [0, -20, -10]

    for i in range(3):
        art_top, art_bottom = car_arts[i]
        width = max(len(art_top), len(art_bottom))
        art_top = art_top.ljust(width)
        art_bottom = art_bottom.ljust(width)
        start_pos = -width + start_offsets[i]
        cars.append(
            {
                "rows": [bands[i][0], bands[i][1]],
                "pos": start_pos,
                "speed": speed,
                "art": [art_top, art_bottom],
                "width": width,
            }
        )

    # Track how far each row has been revealed (frontier is left edge of the car minus 1)
    frontier = [-1] * rows

    # Estimate frames to distribute the total duration into a smooth delay
    total_frames_est = (
        cols + max(car["width"] - off for car, off in zip(cars, start_offsets)) + 10
    )
    delay = max(duration / total_frames_est, 0.008)

    frame = 0
    while True:
        # Update positions & reveal frontiers
        for car in cars:
            if frame % car["speed"] == 0:
                car["pos"] += 1
            # Reveal everything left of the car (both rows it covers)
            for r in car["rows"]:
                frontier[r] = max(frontier[r], car["pos"] - 1)

        # Render frame
        os.system("cls" if os.name == "nt" else "clear")
        for r, line in enumerate(ascii_art):
            row_chars = list(" " * cols)

            # Draw revealed portion of the ASCII art up to the frontier
            f = frontier[r]
            if f >= 0:
                upto = min(f + 1, cols)
                for c in range(upto):
                    row_chars[c] = line[c]

            # Overlay cars on top (masking underlying text where the car sits)
            for car in cars:
                if r in car["rows"]:
                    idx = 0 if r == car["rows"][0] else 1
                    art_line = car["art"][idx]
                    for i, ch in enumerate(art_line):
                        col = car["pos"] + i
                        if 0 <= col < cols:
                            row_chars[col] = ch if ch != "\n" else " "

            print("".join(row_chars))

        # Stop when all rows fully revealed
        if all(f >= cols - 1 for f in frontier):
            break

        frame += 1
        time.sleep(delay)
