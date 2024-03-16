import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

def load_graph(graph_path):
    return nx.read_graphml(graph_path)

def load_map_data(file_path):
    with open(file_path, 'r') as file:
        return [[int(num) for num in line.split()] for line in file]

def visualize_map(map_data, color_mapping, ax):
    map_array = np.array(map_data)
    color_array = np.zeros((map_array.shape[0], map_array.shape[1], 3))
    for number, color in color_mapping.items():
        color_array[map_array == number] = color
    ax.imshow(color_array, interpolation='nearest')

def plot_graph_nodes(graph, ax, resolution):
    for node_id in graph.nodes:
        node_data = graph.nodes[node_id]
        x = node_data.get("x", 0)*resolution + 65
        y = node_data.get("y", 0)*resolution + 45
        ax.scatter(x, y, color="blue", s=10)
        ax.text(x, y, node_id, color="white", fontsize=6)

def update_animation(frame, graph, node_list, red_dot, path_line, resolution):
    node_id = node_list[frame]
    x, y = graph.nodes[node_id]['x']*resolution + 65, graph.nodes[node_id]['y']*resolution + 45
    red_dot.set_data(x, y)
    if frame > 0:
        prev_x, prev_y = path_line.get_data()
        new_x, new_y = list(prev_x) + [x], list(prev_y) + [y]
        path_line.set_data(new_x, new_y)
    return red_dot, path_line

def create_animation(graph, node_list, ax, resolution):
    red_dot, = ax.plot([], [], 'ro')
    path_line, = ax.plot([], [], 'r-')
    return FuncAnimation(ax.get_figure(), update_animation, fargs=(graph, node_list, red_dot, path_line, resolution), frames=len(node_list))

def main():
    graph_path = 'Competition_track_graph.graphml'
    map_file_path = "map-index.txt"
    waypoints_file_path = 'waypoints.txt'
    resolution = 86.85

    graph = load_graph(graph_path)
    map_data = load_map_data(map_file_path)

    fig, ax = plt.subplots()
    visualize_map(map_data, number_to_color, ax)
    plot_graph_nodes(graph, ax, resolution)

    with open(waypoints_file_path, 'r') as file:
        node_list = [line.strip() for line in file]
    
    ani = create_animation(graph, node_list, ax, resolution)
    plt.axis('off')
    plt.show()

if __name__ == "__main__":
    main()
