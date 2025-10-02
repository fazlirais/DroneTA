from dronekit import connect, VehicleMode
import time
import subprocess
import argparse
import json
import math

# Parse connection string argument
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/serial0')  # Default is UART
args = parser.parse_args()

# Connect to the vehicle
print(f"Connecting to vehicle on: {args.connect}")
vehicle = connect(args.connect, baud=57600, wait_ready=True)


# Function to arm and take off to a target altitude
def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# Function to capture an image and record the drone's location and yaw
def capture_terrain_image(filename="terrain.jpg", log_file="location_log.json"):
    print("Capturing terrain image...")
    try:
        # Capture image using libcamera
        subprocess.run(["libcamera-still", "-o", filename, "-t", "1000"], check=True)
        print(f"Image saved as {filename}")

        # Record drone location and yaw
        location = vehicle.location.global_frame
        attitude = vehicle.attitude  # Get attitude (yaw, pitch, roll)

        # Convert yaw from radians to degrees
        yaw = math.degrees(attitude.yaw)

        location_data = {
            "latitude": location.lat,
            "longitude": location.lon,
            "altitude": location.alt,
            "yaw": yaw,  # Add yaw to location data
            "image_file": filename,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        print(f"Location and yaw at capture: {location_data}")

        # Append location and yaw data to a JSON log file
        with open(log_file, "a") as log:
            log.write(json.dumps(location_data) + "\n")
        print(f"Location and yaw data logged to {log_file}")

    except subprocess.CalledProcessError as e:
        print(f"Failed to capture image: {e}")


# Main execution
try:
    # Step 1: Take off to 30 meters
    arm_and_takeoff(30)

    # Step 2: Hover for 5 seconds
    print("Hovering at target altitude for 5 seconds...")
    time.sleep(5)

    # Step 3: Capture a terrain image and log the location and yaw
    capture_terrain_image("terrain.jpg", "location_log.json")

    # Step 4: Land the drone
    print("Landing the drone...")
    vehicle.mode = VehicleMode("LAND")

    # Wait until the drone has landed
    while vehicle.armed:
        print(" Waiting for drone to land...")
        time.sleep(1)

    print("Drone has landed successfully")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Close the vehicle connection
    print("Closing vehicle connection")
    vehicle.close()

