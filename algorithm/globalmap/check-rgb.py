from PIL import Image, ImageTk
import tkinter as tk

'''
To check accurate RGB data for map
If you click the map, RGB status will shown in terminal
'''

def get_rgb(event):
    x = canvas.canvasx(event.x)
    y = canvas.canvasy(event.y)
    rgb = image.getpixel((x, y))
    print(f"RGB at ({x}, {y}): {rgb}")
    # print(f"({x}, {y}), \\")

image_path = "track07.png"

image = Image.open(image_path)

root = tk.Tk()
root.title("Image Viewer")

canvas = tk.Canvas(root)
canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

scrollbar = tk.Scrollbar(root, command=canvas.yview)
scrollbar.pack(side=tk.LEFT, fill=tk.Y)

canvas.configure(yscrollcommand=scrollbar.set)
canvas.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

tk_img = ImageTk.PhotoImage(image)
canvas.create_image(0, 0, anchor='nw', image=tk_img)

canvas.bind("<Button-1>", get_rgb)

root.mainloop()
