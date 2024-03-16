import xml.etree.ElementTree as ET

'''
To translate graphml file to txt so that cpp can parse
'''

# Load graphml file
tree = ET.parse('Competition_track_graph.graphml')
root = tree.getroot()

# Save nodes info in txt format
with open('parsinggraphinfo.txt', 'w') as file:
    for node in root.findall('.//{http://graphml.graphdrawing.org/xmlns}node'):
        node_id = node.get('id')
        x = float(node.find('./{http://graphml.graphdrawing.org/xmlns}data[@key="d0"]').text)
        y = float(node.find('./{http://graphml.graphdrawing.org/xmlns}data[@key="d1"]').text)
        
        # Calculate resolution
        x_calculated = int(x * 86.85 + 65)
        y_calculated = int(y * 86.85 + 45)

        file.write(f'{node_id}, {x_calculated}, {y_calculated}\n')
