#!/usr/bin/env python3
"""
주문 데이터를 지속적으로 모니터링하고 새 주문이 들어오면 명령어를 실행하는 스크립트
사용법: python3 order_executor.py
"""

import requests
import subprocess
import json
import sys
import time
from datetime import datetime

# 서버 설정
SERVER_URL = "http://155.230.12.187:6027"
DATA_ENDPOINT = f"{SERVER_URL}/data"

# 모니터링 설정
CHECK_INTERVAL = 3  # 3초마다 확인
MAX_ORDERS = 0  # 처리된 최대 주문 수 (새 주문 감지용)

def get_order_data():
    """서버에서 주문 데이터를 가져옵니다."""
    try:
        response = requests.get(DATA_ENDPOINT, timeout=10)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"❌ 서버 연결 오류: {e}")
        return None

def generate_command(order_data):
    """주문 데이터를 기반으로 명령어를 생성합니다."""
    if not order_data or not order_data.get('orders'):
        print("❌ 주문 데이터가 없습니다.")
        return None
    
    # 최신 주문 가져오기
    latest_order = order_data['orders'][0]  # 이미 최신순으로 정렬됨
    
    # 주문 정보 추출
    order_number = latest_order.get('order_number', 'N/A')
    method = latest_order.get('method', 'button')
    bread = latest_order.get('bread', 'N/A')
    ingredients = [ingredient.lower() for ingredient in latest_order.get('ingredients', [])]
    
    # 명령어 생성 (예시 - 여기서 개조하세요!)
    command_parts = []
    
    # 기본 정보 출력
    command_parts.append(f'echo "=== 주문 처리 시작 ==="')
    command_parts.append(f'echo "주문번호: {order_number}"')
    command_parts.append(f'echo "주문 방법: {method}"')
    command_parts.append(f'echo "빵: {bread}"')
    command_parts.append(f'echo "재료: {", ".join(ingredients)}"')
    orders = ["dish", "bread"]
    orders.extend(ingredients)
    orders.append("bread")
    
    # 명령어 문장 생성
    command_sentences = []
    for i, order in enumerate(orders):
        if i > 0:  # 첫 번째 항목이 아닌 경우에만
            command_sentences.append(f"pick the {order}, place it on {orders[i-1]}.")
    
    cmd = f"echo '{' '.join(command_sentences)}'"
    return cmd

def execute_command(command):
    """명령어를 실행합니다."""
    try:
        print(f"🚀 명령어 실행: {command}")
        print("=" * 50)
        
        result = subprocess.run(
            command,
            shell=True,
            capture_output=False,  # 실시간 출력을 위해 False
            text=True,
            timeout=60
        )
        
        print("=" * 50)
        print(f"✅ 명령어 실행 완료 (종료 코드: {result.returncode})")
        return True
        
    except subprocess.TimeoutExpired:
        print("❌ 명령어 실행 시간 초과 (60초)")
        return False
        
    except Exception as e:
        print(f"❌ 명령어 실행 오류: {e}")
        return False

def check_for_new_orders():
    """새 주문이 있는지 확인하고 처리합니다."""
    global MAX_ORDERS
    
    # 주문 데이터 가져오기
    order_data = get_order_data()
    
    if not order_data:
        print("❌ 서버 연결 실패")
        return False
    
    total_orders = order_data.get('total_orders', 0)
    
    # 새 주문이 있는지 확인
    if total_orders > MAX_ORDERS:
        new_orders_count = total_orders - MAX_ORDERS
        print(f"🆕 새 주문 {new_orders_count}개 발견!")
        
        # 최신 주문 처리
        latest_order = order_data['orders'][0]
        order_number = latest_order.get('order_number', 'N/A')
        print(f"📦 처리할 주문: {order_number}")
        
        # 명령어 생성 및 실행
        command = generate_command(order_data)
        if command:
            print("⚡ 새 주문 처리 중...")
            success = execute_command(command)
            if success:
                print(f"✅ 주문 {order_number} 처리 완료!")
            else:
                print(f"❌ 주문 {order_number} 처리 실패!")
        
        # 처리된 주문 수 업데이트
        MAX_ORDERS = total_orders
        return True
    
    return False

def main():
    """메인 함수 - 지속적으로 주문을 모니터링"""
    global MAX_ORDERS
    
    print("🥪 주문 데이터 모니터링 시작")
    print("=" * 50)
    print(f"📡 서버: {SERVER_URL}")
    print(f"⏰ 확인 간격: {CHECK_INTERVAL}초")
    print("🔄 지속적으로 새 주문을 모니터링합니다...")
    print("=" * 50)
    
    # 초기 주문 수 확인
    initial_data = get_order_data()
    if initial_data:
        MAX_ORDERS = initial_data.get('total_orders', 0)
        print(f"📊 현재 주문 수: {MAX_ORDERS}")
    
    print("🎯 새 주문 대기 중... (Ctrl+C로 종료)")
    print("-" * 50)
    
    try:
        while True:
            # 새 주문 확인
            has_new_order = check_for_new_orders()
            
            if not has_new_order:
                # 새 주문이 없으면 대기
                current_time = datetime.now().strftime("%H:%M:%S")
                print(f"[{current_time}] 새 주문 대기 중...")
            
            # 다음 확인까지 대기
            time.sleep(CHECK_INTERVAL)
            
    except KeyboardInterrupt:
        print("\n🛑 모니터링을 종료합니다.")
        print("👋 안녕히 가세요!")
    except Exception as e:
        print(f"\n❌ 예상치 못한 오류: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
