import socket
import numpy as np
import matplotlib.pyplot as plt
import time

# Server configuration
HOST = '0.0.0.0'  # Listen on all available network interfaces
PORT = 1234  # Port number to listen on
BUFFER_SIZE = 4096  # Size of the receive buffer

def save_sound_data(data):
    # Save the received sound data to a file
    with open('sound_data.bin', 'ab') as file:
        file.write(data)

def visualize_sound_data(data):
    # Convert the received data to numpy array
    sound_data = np.frombuffer(data, dtype=np.int16)

    # Create a time array based on the sample rate
    sample_rate = 44100  # Adjust if necessary
    time_array = np.arange(len(sound_data)) / sample_rate

    # Plot the time domain waveform
    plt.figure()
    plt.plot(time_array, sound_data)
    plt.title('Time Domain')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.show()

    # Perform FFT to obtain the frequency domain representation
    fft_data = np.fft.fft(sound_data)
    frequency_array = np.fft.fftfreq(len(sound_data), 1 / sample_rate)

    # Plot the frequency domain spectrum
    plt.figure()
    plt.plot(frequency_array, np.abs(fft_data))
    plt.title('Frequency Domain')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.show()

def start_server():
    # Create a TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the host and port
    server_socket.bind((HOST, PORT))

    # Listen for incoming connections
    server_socket.listen(1)
    print('Server listening on {}:{}'.format(HOST, PORT))

    while True:
        # Accept a client connection
        client_socket, client_address = server_socket.accept()
        print('Connected to client:', client_address)

        # Receive and save the sound data
        while True:
            data = client_socket.recv(BUFFER_SIZE)
            if not data:
                break
            save_sound_data(data)
            visualize_sound_data(data)

        # Close the client connection
        client_socket.close()
        print('Client disconnected')

    # Close the server socket
    server_socket.close()

if __name__ == '__main__':
    start_server()