# import serial

# # Change this based on your system
# LIDAR_PORT = "/dev/ttyUSB0"  # Linux/macOS
# BAUD_RATE = 115200  # Change according to your LiDAR

# # def read_lidar():
# #     try:
# #         ser = serial.Serial(LIDAR_PORT, BAUD_RATE, timeout=1)
# #         print(f"Connected to LiDAR on {LIDAR_PORT} at {BAUD_RATE} baud")

# #         while True:
# #             data = ser.read(32)  # Read 32 bytes (adjust as needed)
# #             if data:
# #                 print(f"Raw Data: {data.hex()}")  # Print hex representation
# #     except serial.SerialException as e:
# #         print(f"Error: {e}")
# #     except KeyboardInterrupt:
# #         print("\nExiting...")
# #     finally:
# #         ser.close()

# def parse_data(data):
#     # Assuming each pair of bytes is a distance measurement (you'll need to adjust this for your specific format)
#     distances = []
#     for i in range(0, len(data), 2):  # assuming every 2 bytes represent a distance
#         distance = int(data[i:i+2], 16)  # Convert the hex pair to an integer
#         distances.append(distance)
#     return distances

# def read_lidar():
#     try:
#         ser = serial.Serial(LIDAR_PORT, BAUD_RATE, timeout=1)
#         print(f"Connected to LiDAR on {LIDAR_PORT} at {BAUD_RATE} baud")

#         while True:
#             data = ser.read(32)  # Read 32 bytes (adjust as needed)
#             if data:
#                 print(f"Raw Data: {data.hex()}")
#                 distances = parse_data(data.hex())  # Parse the data into meaningful measurements
#                 print(f"Distances: {distances}")  # Or visualize this data if needed
#     except serial.SerialException as e:
#         print(f"Error: {e}")
#     except KeyboardInterrupt:
#         print("\nExiting...")
#     finally:
#         ser.close()


# if __name__ == "__main__":
#     read_lidar()



import serial
import matplotlib.pyplot as plt
import numpy as np

LIDAR_PORT = "/dev/ttyUSB0"  # Linux/macOS
BAUD_RATE = 115200  # Change according to your LiDAR

# Example parse function - you'll need to adjust this for your LiDAR data format
def parse_data(data):
    # Let's assume the LiDAR data consists of 180 distance readings (1 byte per reading)
    distances = []
    for i in range(0, len(data), 2):  # Adjust this depending on your LiDAR data
        distance = int(data[i:i+2], 16)  # Convert hex pair to integer
        distances.append(distance)
    return distances

def read_lidar():
    try:
        ser = serial.Serial(LIDAR_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to LiDAR on {LIDAR_PORT} at {BAUD_RATE} baud")

        while True:
            data = ser.read(32)  # Read 32 bytes (adjust as needed)
            if data:
                print(f"Raw Data: {data.hex()}")
                distances = parse_data(data.hex())  # Parse the raw data
                
                # Assuming the data represents 180 distance readings for a full 360° scan
                angles = np.linspace(0, 360, len(distances), endpoint=False)
                
                # Convert to polar coordinates for plotting
                plt.clf()  # Clear previous plot
                plt.polar(np.radians(angles), distances, marker='o', linestyle='None')

                plt.title("LiDAR Scan")
                plt.draw()
                plt.pause(0.1)
                
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    plt.ion()  # Interactive mode
    read_lidar()
