def read_gcode(file_name, z_free, z_press, x_offset, y_offset, scale=1.0):
    f = open(file_name)
    text = f.readlines()

    start = 0
    for data in text:
        start += 1
        if 'G-code START' in data:
            break
    # print(start)
    points = []
    for i in range(start + 3, len(text), 2):
        point = text[i].split(" ")
        if 'G-code END' in text[i]:
            break
        x = point[1]
        y = point[2]
        x = float(x[1:]) / 100 * scale + x_offset
        y = float(y[1:]) / 100 * scale + y_offset
        if 'M3' in text[i + 1]:
            points.append((x, y, z_free))
            points.append((x, y, z_press))
        if 'M5' in text[i + 1]:
            points.append((x, y, z_press))
            points.append((x, y, z_free))
    f.close()

    return points


def convet2slop(points):
    new_points = []
    
    pass