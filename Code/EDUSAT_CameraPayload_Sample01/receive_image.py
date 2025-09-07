import socket
import struct
from PIL import Image

ESP32_IP = '192.168.59.199'  # Replace with your ESP32's IP address
PORT = 5000

# Helper function to receive an exact number of bytes from the socket
def recv_exact(sock, size):
    data = b''
    while len(data) < size:
        part = sock.recv(size - len(data))
        if not part:
            raise ConnectionError('Socket closed before expected data was received')
        data += part
    return data

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ESP32_IP, PORT))
        print('Connected to ESP32!')

        # Receive 4 bytes for the image length (little-endian uint32)
        raw_len = recv_exact(s, 4)
        img_len = struct.unpack('<I', raw_len)[0]
        print(f'Image length: {img_len} bytes')

        # Receive the JPEG image data
        img_data = recv_exact(s, img_len)
        with open('output.jpg', 'wb') as f:
            f.write(img_data)
        print('Image saved as output.jpg')

        # Rotate the image 90 degrees clockwise and save
        img = Image.open('output.jpg')
        rotated = img.rotate(-90, expand=True)  # -90 for clockwise rotation
        rotated.save('output_rotated.jpg')
        print('Rotated image saved as output_rotated.jpg')

if __name__ == '__main__':
    main()
