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

    z = z_free
    for i in range(start + 3, len(text)):

        # print(text[i])
        if 'G-code END' in text[i]:
            break
        speed = 0.1
        if 'G' in text[i]:

            if 'G0' in text[i]:
                speed = 0.18
            point = text[i].split(" ")
            x = point[1]
            y = point[2]
            x = float(x[1:]) / 100 * scale + x_offset
            y = float(y[1:]) / 100 * scale + y_offset
            if 'M3' in text[i + 1]:
                points.append((x, y, z_free, speed))
                points.append((x, y, z_press, 0.1))
                z = z_press
                # print('M3')
            elif 'M5' in text[i + 1]:
                points.append((x, y, z_press, speed))
                points.append((x, y, z_free, 0.18))
                z = z_free
                # print('M5')
            else:
                points.append((x, y, z, speed))
    f.close()
    return points


def convet2slop(points):
    new_points = []

    pass
