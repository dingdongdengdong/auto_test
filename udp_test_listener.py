#!/usr/bin/env python3
"""
Simple UDP listener to test if packets are arriving at port 2368
"""

import socket
import time

def udp_listener(port):
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', port))
    sock.settimeout(1.0)  # 1 second timeout
    
    print(f"UDP Listener started on 127.0.0.1:{port}")
    print("Waiting for packets... (Press Ctrl+C to stop)")
    
    packet_count = 0
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                packet_count += 1
                print(f"[{packet_count}] Received {len(data)} bytes from {addr}")
                
                # Show first few bytes as hex
                hex_data = ' '.join(f'{b:02x}' for b in data[:16])
                print(f"    First 16 bytes: {hex_data}")
                
            except socket.timeout:
                print(".", end="", flush=True)  # Show we're still waiting
                
    except KeyboardInterrupt:
        print(f"\nStopped. Total packets received: {packet_count}")
    finally:
        sock.close()

if __name__ == "__main__":
    udp_listener(2368)
