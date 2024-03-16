import numpy as np
import matplotlib.pyplot as plt

# 색상 매핑
number_to_color = {
    0: (1.0, 1.0, 1.0),  # Solid Lines (White)
    1: (37/255, 37/255, 0),  # Stop Line
    2: (255/255, 110/255, 255/255),  # Crosswalk
    3: (218/255, 218/255, 255/255),  # Dotted Line
    4: (0, 145/255, 0),  # Parking Lot
    5: (0, 109/255, 109/255),  # Roundabout
    6: (255/255, 146/255, 146/255),  # StartLine
    7: (0, 0, 0),  # Floor
    8: (26/255, 60/255, 94/255),  # Bus Line
    9: (172/255, 161/255, 63/255)  # Sidewalk
}

def plot_map_from_file(file_path):
    # 파일에서 맵 데이터 읽기
    with open(file_path, 'r') as file:
        map_data = [[int(num) for num in line.split()] for line in file]

    # numpy 배열로 변환
    map_array = np.array(map_data)

    # 색상 배열 생성
    color_array = np.zeros((map_array.shape[0], map_array.shape[1], 3))

    # 각 숫자에 대해 색상 할당
    for number, color in number_to_color.items():
        color_array[map_array == number] = color

    # 맵 시각화
    plt.imshow(color_array, interpolation='nearest')
    plt.axis('off')
    plt.show()


# 파일 경로
file_path = "map-index.txt"
plot_map_from_file(file_path)
