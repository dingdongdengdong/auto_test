#!/usr/bin/env python3
"""
UDP Port Forwarder
Forwards UDP packets from port 2369 to port 2368 for Velodyne simulation
"""

import socket
import threading

def udp_forwarder(source_port, dest_port):
    # Create source socket (receives from simulator)
    source_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    source_sock.bind(('127.0.0.1', source_port))
    
    # Create destination socket (sends to velodyne driver)
    dest_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"UDP Forwarder: {source_port} -> {dest_port}")
    print("Waiting for packets...")
    
    try:
        while True:
            # Receive packet from simulator
            data, addr = source_sock.recvfrom(4096)
            print(f"Received {len(data)} bytes from {addr}")
            
            # Forward to velodyne driver
            dest_sock.sendto(data, ('127.0.0.1', dest_port))
            print(f"Forwarded to port {dest_port}")
            
    except KeyboardInterrupt:
        print("\nForwarder stopped")
    finally:
        source_sock.close()
        dest_sock.close()

if __name__ == "__main__":
    # Forward from 2369 to 2368
    udp_forwarder(2369, 2368)
