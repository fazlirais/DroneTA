import cv2
import pygame
import KeyPressModule as kp
import numpy as np
from time import sleep
import math
import tkinter as tk
from tkinter import filedialog
from pyproj import Transformer

######## PARAMETERS FOR POINT DRAW ###########

aSpeed = 360 / 15  # Angular Speed Degrees/s (36d/s)
Uspeed = 1
interval = 1
Fovx = 54
Fovy = 41
altitude = 30  # Mapping altitude
flight_altitude = 1.5  # Drone flight altitude
cam_degree = 16
state = False
Start_lat = -6.9138541
Start_lon = 107.8522157
Start_yaw = 114.42460188209286

# Create a transformer for coordinate conversion
transformer = Transformer.from_crs('epsg:4326', 'epsg:3857', always_xy=True)

# Precompute shared values
cam_degree_rad = math.radians(cam_degree)
Fovx_rad = math.radians(Fovx)
Fovx_half_rad = Fovx_rad / 2
denominator = float(
    math.tan(cam_degree_rad + Fovx_half_rad) -
    math.tan(cam_degree_rad - Fovx_half_rad)
)
Start_yaw_rad = math.radians(Start_yaw)
x0_real, y0_real = transformer.transform(Start_lon, Start_lat)
print(x0_real)
print(y0_real)
y0_real = y0_real + altitude * (math.tan(math.radians(cam_degree_rad - Fovx_half_rad)) + denominator / 2)* math.cos(math.radians(Start_yaw + 270))
aInterval = aSpeed * interval
UInterval = Uspeed * interval
fSpeed = 2 * flight_altitude * math.tan(math.radians(Fovy / 2)) / interval  # Forward Speed in m/s (m/s)
dInterval = fSpeed * interval
###############################################
kp.init()

def get_image_path():
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(
        filetypes=[("Image files", "*.jpg;*.jpeg;*.png;*.bmp;*.tiff;*.gif")],
        title="Select an Image File"
    )
    return file_path

def calculateBox(x, y, yaw, altitude, flight_altitude, img_width, img_height,
                 cam_degree_rad, Fovx_half_rad, denominator):
    x_offset_max = (flight_altitude / altitude) * (img_width * math.tan(cam_degree_rad + Fovx_half_rad)) / denominator
    x_offset_min = (flight_altitude / altitude) * (img_width * math.tan(cam_degree_rad - Fovx_half_rad)) / denominator

    y_max = y + int(img_height * flight_altitude / (2 * altitude))
    y_min = y - int(img_height * flight_altitude / (2 * altitude))
    x_max = x - int(x_offset_max)
    x_min = x - int(x_offset_min)

    corners = np.array([
        [x_min, y_min],
        [x_max, y_min],
        [x_max, y_max],
        [x_min, y_max]
    ])

    theta = math.radians(yaw)
    rotation_matrix = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)]
    ])
    rotated_corners = np.dot(corners - np.array([x, y]), rotation_matrix.T) + np.array([x, y])
    return rotated_corners.astype(int)

def drawBox(img, corners, draw_text=True):
    for i in range(4):
        pt1 = tuple(corners[i])
        pt2 = tuple(corners[(i + 1) % 4])
        cv2.line(img, pt1, pt2, (0, 0, 255), 2)  # Red color
    if draw_text:  # Only draw text if specified
        cv2.putText(img, 'Mapping Area',
                    (corners[0][0], corners[0][1] - 10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)  # Red color
def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    global x, y, yaw, a, flight_altitude, x0_real, y0_real
    d = 0
    key_pressed = False
    if kp.getKey("LEFT"):
        key_pressed = True
        lr = -fSpeed
        d = dInterval
        a = yaw-180
    elif kp.getKey("RIGHT"):
        key_pressed = True
        lr = fSpeed
        d = -dInterval
        a = yaw+180
    if kp.getKey("UP"):
        key_pressed = True
        fb = fSpeed
        d = dInterval
        a = yaw+270
    elif kp.getKey("DOWN"):
        key_pressed = True
        fb = -fSpeed
        d = -dInterval
        a = yaw-90
    if kp.getKey("a"):
        key_pressed = True
        #yv = -aSpeed
        yaw -= aInterval
        a -= aInterval
    elif kp.getKey("d"):
        key_pressed = True
        #yv = aSpeed
        yaw += aInterval
        a += aInterval
    if kp.getKey("w"):
        key_pressed = True
        ud = Uspeed
        flight_altitude += UInterval
    elif kp.getKey("s"):
        key_pressed = True
        ud = -Uspeed
        flight_altitude -= UInterval
    sleep(0.25)
    x0_real += d * math.cos(math.radians(math.radians(Start_yaw + a)))
    y0_real += d * math.sin(math.radians(math.radians(Start_yaw + a)))
    x += int(d * math.cos(math.radians(a)) * img_width / (altitude * denominator))
    y += int(d * math.sin(math.radians(a)) * img_height / (2 * altitude * math.tan(math.radians(Fovy / 2))))
    return [lr, fb, ud, yaw, x, y, x0_real, y0_real, a], key_pressed

def drawPoints(img, points, altitude, flight_altitude, cam_degree_rad, Fovx_half_rad, denominator, img_width, img_height):
    # Draw the points as before
    current_point = points[-1]
    current_point = tuple(map(int, current_point))

    if state:
        for i in range(1, len(points)):
            pt1 = tuple(map(int, points[i - 1]))
            pt2 = tuple(map(int, points[i]))
            cv2.line(img, pt1, pt2, (0, 255, 0), 2)  # Green line for the path


    triangle_center = current_point
    triangle_size = 30
    triangle_angle = yaw
    triangle_color = (255, 0, 0)

    # Draw the triangle for the current position (the drone icon)
    theta = np.radians(triangle_angle)
    c, s = np.cos(theta), np.sin(theta)
    rotation_matrix = np.array([[c, -s], [s, c]])

    triangle_vertices = np.array([[-triangle_size / 2, triangle_size / 2],
                                   [triangle_size / 2, triangle_size / 2],
                                   [0, -triangle_size]]).T

    rotated_vertices = np.dot(rotation_matrix, triangle_vertices).T
    rotated_vertices += np.array(triangle_center)
    cv2.fillPoly(img, [np.array(rotated_vertices, np.int32)], triangle_color)

    # Always draw the current bounding box (with text)
    corners = calculateBox(
        x=current_point[0], y=current_point[1], yaw=yaw,
        altitude=altitude, flight_altitude=flight_altitude,
        img_width=img_width, img_height=img_height,
        cam_degree_rad=cam_degree_rad, Fovx_half_rad=Fovx_half_rad, denominator=denominator
    )
    drawBox(img, corners)  # Draw current box with text

    # Only store the bounding box when `state` is True
    if state:
        stored_boxes.append(corners)  # Store the new box for future frames

    # Draw all previously stored boxes without text if `state` is True
    if state:
        for corners in stored_boxes:
            drawBox(img, corners, draw_text=False)

def render_text(surface, flight_altitude):
    """Renders control instructions and flight altitude on the Pygame window."""
    # Instructions text
    instructions = [
        "Controls:",
        "Arrow Keys: Move",
        "'a'/'d': Rotate",
        "'w'/'s': Adjust Altitude",
        "SPACE: Start Mapping",
        "ESC: Quit",
    ]

    # Draw each instruction line
    y_offset = desired_height - 150  # Starting position for the text
    for line in instructions:
        text_surface = font.render(line, True, (0, 0, 255))  # White text
        surface.blit(text_surface, (10, y_offset))
        y_offset += 20  # Line spacing

    # Display flight altitude
    altitude_text = f"Flight Altitude: {flight_altitude:.2f} m"
    altitude_surface = font.render(altitude_text, True, (0, 0, 255))
    surface.blit(altitude_surface, (10, y_offset))  # Offset for altitude

# Load image and initialize variables
img_path = get_image_path()
img = cv2.imread(img_path)
img_height, img_width, _ = img.shape
desired_height = 600
aspect_ratio = img_width / img_height
window_width = int(desired_height * aspect_ratio)

x, y = img_width / 2, img_height / 2
points = [(x, y)]
stored_boxes = []
a, yaw = 90, 90

# Initialize Pygame
pygame.init()
win = pygame.display.set_mode((window_width, desired_height))
pygame.display.set_caption("Drone Path Mapping")
# Initialize Pygame font
pygame.font.init()
font = pygame.font.SysFont("Arial", 20)  # Choose font and size

clock = pygame.time.Clock()
running = True

vals_list = []

# Main loop
with open('vals_data.txt', 'w') as f:
    while running:
        img = cv2.imread(img_path)
        vals, key_pressed = getKeyboardInput()
        if kp.getKey("SPACE"):
            state = True
            points = [points[-1], points[-1]]
        if state and key_pressed:
            f.write(f"{vals[:9]}\n")
            vals_list.append(vals)
        if points[-1][0] != vals[4] or points[-1][1] != vals[5]:
            points.append((vals[4], vals[5]))
            print(vals)

        drawPoints(img, points, altitude, flight_altitude, cam_degree_rad, Fovx_half_rad, denominator, img_width,
                   img_height)

        img_resized = cv2.resize(img, (window_width, desired_height))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        img_pygame = pygame.image.frombuffer(img_rgb.tobytes(), img_rgb.shape[1::-1], 'RGB')
        win.fill((0, 0, 0))
        win.blit(img_pygame, (0, 0))

        # Render text for controls and flight altitude
        render_text(win, flight_altitude)

        pygame.display.update()
        clock.tick(60)

        if kp.getKey("ESCAPE"):
            running = False
            break
cv2.imwrite('Drone Path Map.jpg', img)
print("Last image saved as Drone Path Map.jpg")
pygame.quit()
cv2.destroyAllWindows()










