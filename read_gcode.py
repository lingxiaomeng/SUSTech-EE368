def read_gcode(file_name, z_free, z_press, x_offset, y_offset, scale=1.0):
    f = open(file_name)
    text = f.readlines()

    ### Skip header lines
    start = 0
    for data in text:
        start += 1
        if 'G-code START' in data:
            break
    points = []
    ### Start Generate Trajectory
    z = z_free
    for i in range(start + 3, len(text)):
        if 'G-code END' in text[i]:
            break
        speed = 0.02
        if 'G' in text[i]:
            if 'G0' in text[i]:
                speed = 0.18 # G0 refer to higher speed
            point = text[i].split(" ")
            x = point[1]
            y = point[2]
            x = float(x[1:]) / 100 * scale + x_offset
            y = float(y[1:]) / 100 * scale + y_offset
            if 'M3' in text[i + 1]: ## M3 means robot arm move down
                points.append((x, y, z_free, speed))
                points.append((x, y, z_press, 0.01))
                z = z_press
            elif 'M5' in text[i + 1]: ## M3 means robot arm move up
                points.append((x, y, z_press, speed))
                points.append((x, y, z_free, 0.18))
                z = z_free
            else:
                points.append((x, y, z, speed))
    f.close()
    return points