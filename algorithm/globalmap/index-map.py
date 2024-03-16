import numpy as np
from PIL import Image

# All RGB values not here are "Solid Lines" which is number 0 -> 아래 색 이외는 전부 실선 취급
color_mapping = {
    (37, 37, 0, 255): 1,  
    (32, 32, 0, 255): 1,
    (35, 35, 0, 255): 1,
    (10, 10, 0, 255): 1,
    (18, 18, 0, 255): 1,
    (36, 36, 0, 255): 1,
    (4, 4, 0, 255): 1,
    (9, 9, 0, 255): 1,
    (40, 40, 0, 255): 1,
    (47, 47, 0, 255): 1,
    (41, 41, 0, 255): 1,     # Stop Line (정지선)
    (255, 110, 255, 255): 2,    # Crosswalk (횡단보도)
    (109, 109, 236, 255): 3,
    (218, 218, 255, 255): 3,    # Dotted Line (점선)
    (0, 145, 0, 255): 4,        # Parking Lot (주차구역)
    (0, 109, 109, 255): 5,      # Roundabout (로터리)
    (255, 146, 146, 255): 6,    # StartLine (시작구역)
    (0, 0, 0, 255): 7,          # Floor (차도)
    (26, 60, 94, 255): 8,       # Bus Line (버스전용차선)
    (172, 161, 63, 255): 9      # Sidewalk (인도 -> 노란색)
}

def map_colors_to_numbers(image_path, output_file):
    img = Image.open(image_path)
    img_data = np.array(img)

    result = np.zeros(img_data.shape[:2], dtype=int)

    for y in range(img_data.shape[0]):
        for x in range(img_data.shape[1]):
            pixel = tuple(img_data[y, x])
            result[y, x] = color_mapping.get(pixel, 0)

    with open(output_file, 'w') as file:
        for row in result:
            file.write(' '.join(map(str, row)) + '\n')

image_path = "track07.png"
output_file = "map-index.txt"
map_colors_to_numbers(image_path, output_file)

