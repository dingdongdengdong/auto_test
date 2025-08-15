#!/usr/bin/env python3
"""
Raw UDP Listener for MORAI debugging
Listen for any UDP packets on port 2368
"""

import socket
import time

def listen_udp(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', port))
    sock.settimeout(2.0)
    
    print(f"🔍 Listening for UDP packets on 127.0.0.1:{port}")
    print("📊 MORAI 시뮬레이터에서 Lidar 데이터 전송을 시작해주세요...")
    print("⏹️  Ctrl+C로 중지")
    
    packet_count = 0
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                packet_count += 1
                
                print(f"\n✅ 패킷 #{packet_count} 수신!")
                print(f"   📤 송신자: {addr}")
                print(f"   📦 크기: {len(data)} bytes")
                print(f"   🔢 첫 16바이트: {' '.join(f'{b:02x}' for b in data[:16])}")
                
                if packet_count == 1:
                    print("\n🎉 MORAI 데이터 수신 성공! 이제 ROS 브리지를 실행할 수 있습니다.")
                
            except socket.timeout:
                print(".", end="", flush=True)
                
    except KeyboardInterrupt:
        print(f"\n\n📊 총 {packet_count}개 패킷 수신됨")
        if packet_count > 0:
            print("✅ MORAI 연결 성공")
        else:
            print("❌ MORAI 데이터 없음 - 시뮬레이터 설정을 확인하세요")
    finally:
        sock.close()

if __name__ == "__main__":
    listen_udp(2368)
