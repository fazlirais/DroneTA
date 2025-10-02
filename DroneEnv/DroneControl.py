import time
import cv2
import numpy as np
from ultralytics import YOLO
import subprocess
import paho.mqtt.client as mqtt
import ast
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import math
from pyproj import Transformer
import json

# Parameters and constants
interval = 1  # Time interval between sending velocity commands
LOW_VOLTAGE_THRESHOLD = 10.8  # Voltage threshold to check if battery is low
MQTT_BROKER = "127.0.0.1"  # Change this to your broker address
MQTT_TOPIC = "drone/machine_learning"  # Topic to publish machine learning results
Fovx = 54
Fovy = 41
cam_degree = 16
H_Plant = 0.1
img_width = 2592
img_height = 1944
START_YAW = 34
vx_speed = 85
vy_speed = 97
resized_width = 640
resized_height = 480
START_YAW = 114.42460188209286
xmin, xmax = 11250587, 11250621
ymin, ymax = 3949597, 3949620

# MQTT Client setup
mqtt_client = mqtt.Client()
mqtt_client.connect("localhost", 1883, 60)  # Connect to MQTT broker (default port 1883)
mqtt_client.loop_start()

# Load YOLO model
model = YOLO(r"/home/fazli123/best (2).pt")

terrain_image_path = r"/home/fazli123/terrain.jpg"

# Collect data for plotting
disease_data = {"all": []}# Dictionary to store coordinates per disease

# List of diseases
disease = ["Chlorosis", "Gulma", "Leaf Spot", "Necrosis", "Pest"]
for d in disease:
    disease_data[d] = []

# Starting coordinates
START_LON = 107.8522082
START_LAT = -6.9137975

# Define the folder path
save_folder = "processed_images"
plot_folder = "plots"

# New dictionaries for storing processed time and heatmap data
elapsed_time_data = {d: [] for d in disease}
heatmap_data = {d: np.zeros((20, 20)) for d in disease}

def clear_processed_images():
    folder = r"/home/fazli123/processed_images"

    if os.path.exists(folder):
        for filename in os.listdir(folder):
            file_path = os.path.join(folder, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.remove(file_path)  # remove file
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)  # remove folder if accidentally exists
            except Exception as e:
                print(f'Failed to delete {file_path}. Reason: {e}')
    else:
        os.makedirs(folder, exist_ok=True)  # if images folder not exist, create it

# Delete the whole folder if exists
if os.path.exists(save_folder):
    shutil.rmtree(save_folder)

# Recreate empty folder
os.makedirs(save_folder)

if not os.path.exists(plot_folder):
    os.makedirs(plot_folder)

positions = []
def merged_picture(positions, save_folder, resized_width, resized_height):
    vx_speed = 85
    vy_speed = 97

    x_list, y_list = zip(*positions)
    min_x, max_x = min(x_list), max(x_list)
    min_y, max_y = min(y_list), max(y_list)

    canvas_width = int(((max_x - min_x) / vx_speed + 1) * resized_width)
    canvas_height = int(((max_y - min_y) / vy_speed + 1) * resized_height)

    merged = Image.new('RGB', (canvas_width, canvas_height), (255, 255, 255))

    image_files = sorted([f for f in os.listdir(save_folder) if f.endswith('.jpg')])

    for idx, (px, py) in enumerate(positions):
        image_path = os.path.join(save_folder, image_files[idx])  # berdasarkan urutan waktu
        img = Image.open(image_path)
        img = img.resize((resized_width, resized_height))

        pos_x = int((px - min_x)/vx_speed * resized_width)
        pos_y = int((py - min_y)/vy_speed * resized_height)

        merged.paste(img, (pos_x, pos_y))

    # Simpan hasil
    merged.save(" merged_images_map.jpg", quality=85)
def find_coordinate(x, y, x_drone, y_drone, Height, Fovx, Fovy, yaw, H_plant):
    dx = Height * (x * math.tan(math.radians(cam_degree + Fovx / 2)) +
                   (img_width - x) * math.tan(math.radians(cam_degree - Fovx / 2))) / img_width
    dy = Height * (2 * y - img_height) * math.tan(math.radians(Fovy / 2)) / img_height
    Dx = dy * math.cos(math.radians(yaw)) - dx * math.sin(math.radians(yaw))
    Dy = dy * math.sin(math.radians(yaw)) + dx * math.cos(math.radians(yaw))
    D = math.sqrt(dx ** 2 + dy ** 2)
    d = H_plant * D / Height
    Dxc = Dx * (D - d) / D
    Dyc = Dy * (D - d) / D
    x2 = x_drone + Dxc
    y2 = y_drone + Dyc
    x2 = x2* math.cos(math.radians(START_YAW-90)) - y2*math.sin(math.radians(START_YAW-90))
    y2 = x2* math.sin(math.radians(START_YAW-90)) + y2*math.cos(math.radians(START_YAW-90))
    return x2, y2

# Connect to drone
def connect_drone():
    print("Connecting to drone...")
    vehicle = connect('/dev/serial0', baud=57600, wait_ready=True)
    print("Connected to drone!")
    return vehicle

# Arm and takeoff to target altitude
def arm_and_takeoff(vehicle, target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Set the drone to a specific GPS location
def set_starting_location(vehicle, lat, lon, alt):
    print(f"Setting starting location to Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
    target_location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target_location)

    # Wait until the drone reaches the starting location
    while True:
        current_location = vehicle.location.global_relative_frame
        if abs(current_location.lat - lat) < 0.0001 and abs(current_location.lon - lon) < 0.0001:
            print("Drone reached starting location.")
            break
        time.sleep(1)

def upload_images_to_github():
    # Mapping image name to local path
    images = {
        "drone_path_map": "Drone Path Map.jpg",
        "merged_output": "merged_output.jpg",
        "scatter_Pest": "plots/scatter_Pest.png",
        "scatter_Necrosis": "plots/scatter_Necrosis.png",
        "scatter_Leaf_Spot": "plots/scatter_Leaf Spot.png",
        "scatter_Gulma": "plots/scatter_Gulma.png",
        "scatter_Chlorosis": "plots/scatter_Chlorosis.png",
        "scatter_All": "plots/scatter_all.png"
    }

    # Path to your local github repo
    repo_path = r"/home/fazli123/droneEnv"
    images_folder = os.path.join(repo_path, 'images')

    # Auto create images/ if not exists
    os.makedirs(images_folder, exist_ok=True)

    # Copy all images into repo/images/
    for name, path in images.items():
        if os.path.exists(path):
            dest = os.path.join(images_folder, os.path.basename(path))
            shutil.copy2(path, dest)
            print(f"[OK] Copy {path} -> {dest}")
        else:
            print(f"[WARNING] {path} not found!")

    # Git commands to push changes
    commands = [
        f'cd {repo_path}',
        'git add .',
        'git commit -m "auto update images"',
        'git push origin main'
    ]

    subprocess.call(' && '.join(commands), shell=True)
    print("[INFO] Images uploaded to GitHub successfully!")

# Send velocity command to drone
def send_velocity_command(vehicle, vx, vy, vz, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Coordinate frame
        0b0000111111000111,  # Type mask (only velocities enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw rate in rad/s, yaw not used
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# Capture image using libcamera
def capture_image():
    libcamera_command = ['libcamera-still', '--output', 'captured_image.jpg', '--width', img_width, '--height',
                         img_height]  # For 5MP
    subprocess.run(libcamera_command)

    # After capturing, read the image using OpenCV
    frame = cv2.imread('captured_image.jpg')
    return frame


# Send MQTT message with machine learning results
def send_mqtt_message(class_name, confidence, x, y, xavg, yavg, x_drone, y_drone, elapsed_time, yaw, lat_drone, lon_drone):
    payload = {
        "class": class_name,
        "confidence": confidence,
        "x": x,
        "y": y,
        "xavg": xavg,
        "yavg": yavg,
        "elapsed_time": elapsed_time,
        "x_drone": x_drone,
        "y_drone": y_drone,
        "yaw": yaw,
        "lat_drone": lat_drone,
        "lon_drone": lon_drone
    }
    mqtt_client.publish(MQTT_TOPIC, json.dumps(payload))
    # Store elapsed time
    if class_name in elapsed_time_data:
        elapsed_time_data[class_name].append(elapsed_time)

    # Update heatmap
    update_heatmap(x, y, class_name, confidence)
    try:
        with open("payload.txt", "a") as file:
            file.write(json.dumps(payload) + "\n")
    except Exception as e:
        print(f"Error writing to file: {e}")

def check_battery(vehicle):
    battery = vehicle.battery
    if battery.voltage < LOW_VOLTAGE_THRESHOLD:
        print(f"Battery voltage is low ({battery.voltage}V). Returning home.")
        vehicle.mode = VehicleMode("RTL")  # Return to launch
        while vehicle.mode != VehicleMode("LAND"):
            time.sleep(1)
        return False
    return True

def save_scatter_plots():
    terrain_img = Image.open(terrain_image_path)

    # Define colors (avoiding green and brown)
    color_map = {
        "Chlorosis": "red",
        "Gulma": "blue",
        "Leaf Spot": "magenta",
        "Necrosis": "cyan",
        "Pest": "orange"
    }

    for category, points in disease_data.items():
        if not points:
            continue
        x_vals, y_vals = zip(*points)
        plt.figure(figsize=(8, 8))
        plt.imshow(terrain_img, extent=[xmin, xmax, ymin, ymax])

        if category == "all":
            for disease_class, color in color_map.items():
                class_points = disease_data[disease_class]
                if class_points:
                    x_class, y_class = zip(*class_points)
                    plt.scatter(x_class, y_class, c=color, alpha=0.6, s=5, label=disease_class)
            plt.legend()
        else:
            plt.scatter(x_vals, y_vals, c=color_map.get(category, "black"), alpha=0.6, s=5)

        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        plt.axis('off')
        plt.savefig(os.path.join(plot_folder, f"scatter_{category}.png"), bbox_inches='tight', pad_inches=0)
        plt.close()
        print(f"Saved scatter plot for {category}")

def save_processed_time_plots():
    for disease_class, times in elapsed_time_data.items():
        if not times:
            continue

        avg_time = sum(times) / len(times)  # Calculate average time
        std_dev = statistics.stdev(times) if len(times) > 1 else 0  # Calculate standard deviation

        plt.figure(figsize=(8, 4))
        plt.plot(times, marker='o', linestyle='-', color='b', label="Processing Time")
        plt.axhline(y=avg_time, color='r', linestyle='--', label=f"Avg: {avg_time:.2f}s")
        plt.fill_between(range(len(times)), avg_time - std_dev, avg_time + std_dev, color='r', alpha=0.2,
                         label=f"Std Dev: {std_dev:.2f}")

        plt.xlabel("Detection Index")
        plt.ylabel("Elapsed Time (s)")
        plt.title(f"Processed Time for {disease_class}")
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(plot_folder, f"processed_time_{disease_class}.png"), bbox_inches='tight')
        plt.close()

        print(f"Saved processed time plot for {disease_class} with Avg: {avg_time:.2f}s and Std Dev: {std_dev:.2f}s")


def save_heatmaps():
    terrain_img = Image.open(terrain_image_path)

    for disease_class, heatmap in heatmap_data.items():
        plt.figure(figsize=(8, 8))

        # Plot the terrain image as background
        plt.imshow(terrain_img, extent=[xmin, xmax, ymin, ymax], zorder=-1)

        # Create the heatmap with transparency
        sns.heatmap(heatmap, cmap="coolwarm", alpha=0.6, xticklabels=False, yticklabels=False, cbar=False)

        plt.title(f"Heatmap for {disease_class}")
        plt.savefig(os.path.join(plot_folder, f"heatmap_{disease_class}.png"), bbox_inches='tight', pad_inches=0)
        plt.close()
        print(f"Saved heatmap for {disease_class}")

def update_heatmap(x, y, class_name, confidence):
    if class_name in heatmap_data:
        # Convert x, y into a 20x20 matrix index
        grid_x = int(((x - xmin) / (xmax - xmin)) * 20)
        grid_y = int(((y - ymin) / (ymax - ymin)) * 20)
        grid_x = max(0, min(grid_x, 19))
        grid_y = max(0, min(grid_y, 19))

        # Increase heatmap intensity by confidence value
        heatmap_data[class_name][grid_y, grid_x] += confidence

def main():
    with open("payload.txt", "w") as file:
        file.write("")  # Clears the file content
    vehicle = connect_drone()
    # Take off to a specified altitude (e.g., 10 meters)
    arm_and_takeoff(vehicle, 5)
    # Set starting location for the drone (before starting the path)
    set_starting_location(vehicle, START_LAT, START_LON, START_ALT)

    try:
        # Read vals_data.txt for predetermined path
        file_path = 'vals_data.txt'
        with open(file_path, 'r') as f:
            while True:
                if not check_battery(vehicle):
                    break

                line = f.readline()
                if not line:
                    print("End of file reached.")
                    break

                    start_time = time.time()
                    send_velocity_command(vehicle, 0, 0, 0, interval)
                    time.sleep(0.8)

                    # Capture image while hovering
                    frame = capture_image()

                    results = model(frame)  # Predict on the frame
                    result = results[0]

                    vals = ast.literal_eval(line.strip())
                    vz = vals[2]
                    yaw0 = vals[3] + START_YAW - 90
                    yaw = vals[8] + START_YAW - 90
                    x_pixel = vals[4]
                    y_pixel = vals[5]
                    positions.append((x_pixel, y_pixel))

                    # x_drone1 = (vals[4]/2592)*float(33.8068618582614)
                    # y_drone1 = (vals[5]/1944)*float(22.4330807690883)

                    lat_drone, lon_drone = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon
                    x_drone, y_drone = Transformer.from_crs('epsg:4326', 'epsg:3857', always_xy=True).transform(
                        lon_drone, lat_drone)

                    vx = - vals[0] * math.sin(math.radians(yaw0)) + vals[1] * math.cos(math.radians(yaw0))
                    vy = vals[0] * math.cos(math.radians(yaw0)) + vals[1] * math.sin(math.radians(yaw0))

                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        class_id = int(box.cls[0].item())
                        class_name = disease[class_id]
                        conf = box.conf[0].item()
                        if conf >= 0.5:
                            xavg = int((x1 + x2) / 2)
                            yavg = int((y1 + y2) / 2)
                            # x_drone, y_drone = Transformer.from_crs('epsg:4326', 'epsg:3857', always_xy=True).transform(lon_drone, lat_drone)
                            x_final, y_final = find_coordinate(xavg, yavg, x_drone, y_drone, Drone_ALT, Fovx, Fovy,
                                                               yaw0, H_Plant)
                            # lat_final, lon_final = Transformer.from_crs('epsg:3857','epsg:4326', always_xy=True).transform(
                            # y_final, x_final)
                            disease_data["all"].append((x_final, y_final))
                            disease_data[class_name].append((x_final, y_final))
                            send_mqtt_message(class_name, conf, x_final, y_final, xavg, yavg, x_drone, y_drone,
                                              time.time() - start_time, yaw, lat_drone, lon_drone)
                            # Draw bounding box and label on the image
                            color = (0, 0, 255)  # Yellow color for boxes
                            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                            label = f"{class_name} {conf:0.2f}"
                            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, color, 2)
                    # Generate a unique filename and save it in the folder
                    save_path = os.path.join(save_folder, f"output_{int(time.time())}.jpg")
                    cv2.imwrite(save_path, image)
                    print(f"Saved image to {save_path}")
                    send_velocity_command(vehicle, vx, vy, vz, interval)
                    # time.sleep(interval)

                except FileNotFoundError:
                print(f"File {file_path} not found.")
            except KeyboardInterrupt:
            print("Exiting...")

        finally:
        mqtt_client.loop_stop()  # Stop the background thread
        mqtt_client.disconnect()
        set_starting_location(vehicle, START_LAT, START_LON, START_ALT)
        vehicle.mode = VehicleMode("LAND")
        while vehicle.armed:
            print(" Waiting for drone to land...")
            time.sleep(1)
        print("drone landed succesfully")
        vehicle.close()
        merged_picture(positions, save_folder, resized_width, resized_height)
        save_scatter_plots()
        save_processed_time_plots()
        save_heatmaps()
        upload_images_to_github()
        mqtt_client.publish('drone/machine_learning/plot', "refresh")


if __name__ == "__main__":
    main()



