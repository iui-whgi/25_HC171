from flask import Flask, request, jsonify, render_template_string
from datetime import datetime
import pytz
import json
import os
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

app = Flask(__name__)

# JSON 인코딩 설정
app.config['JSON_AS_ASCII'] = False
app.config['JSON_SORT_KEYS'] = False

# 주문 데이터를 저장할 리스트
orders = []

# HTML 파일들을 별도로 관리하여 코드 간결화

@app.route('/')
def main_page():
    """메인 키오스크 페이지"""
    with open('/home/son/kiosk/main.html', 'r', encoding='utf-8') as f:
        return f.read()

@app.route('/button')
def button_order():
    """버튼 주문 페이지"""
    with open('/home/son/kiosk/button_order.html', 'r', encoding='utf-8') as f:
        return f.read()

@app.route('/voice')
def voice_order():
    """음성 주문 페이지"""
    with open('/home/son/kiosk/voice_order.html', 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 환경 변수에서 Agent ID 가져오기
    agent_id = os.getenv('AGENT_ID', 'agent_4101k7h0tqkvebbap2b5c52m2ccy')
    
    # HTML에서 Agent ID를 환경 변수 값으로 교체
    content = content.replace('value="agent_4101k7h0tqkvebbap2b5c52m2ccy"', f'value="{agent_id}"')
    
    # 음성 주문 완료 시 주문 데이터를 서버로 전송하는 함수 추가
    voice_order_script = """
    <script>
        // 음성 주문 완료 시 호출되는 함수
        function handleVoiceOrderComplete(orderItems) {
            const orderData = {
                method: 'voice',
                items: orderItems.map(item => ({ name: item }))
            };

            fetch('/api/order', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(orderData)
            })
            .then(response => response.json())
            .then(result => {
                if (result.success) {
                    alert(`음성 주문 완료!\\n주문번호: ${result.order_number}`);
                    window.location.href = '/complete';
                }
            })
            .catch(e => {
                alert('주문 처리 중 오류가 발생했습니다.');
            });
        }

        // 음성 주문에서 주문 완료 시 자동으로 호출되는 함수 (ElevenLabs 연동용)
        function onVoiceOrderSuccess(transcript, orderItems) {
            console.log('음성 주문 성공:', transcript, orderItems);
            
            // 주문 데이터 구성
            const orderData = {
                method: 'voice',
                items: orderItems.map(item => ({ name: item }))
            };

            // 서버로 주문 전송
            fetch('/api/order', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(orderData)
            })
            .then(response => response.json())
            .then(result => {
                if (result.success) {
                    alert(`음성 주문 완료!\\n주문번호: ${result.order_number}\\n주문 내용: ${orderItems.join(', ')}`);
                    window.location.href = '/complete';
                }
            })
            .catch(e => {
                alert('주문 처리 중 오류가 발생했습니다.');
            });
        }

        // ElevenLabs Agent 응답에서 주문 정보 추출하는 함수
        function extractOrderFromAgentResponse(agentResponse) {
            console.log('Agent 응답 분석:', agentResponse);
            
            // 주문 완료 키워드 확인
            const orderKeywords = ['감사합니다', '주문 완료', '주문이 완료', '주문해드릴게요', '주문하겠습니다', '주문 접수'];
            const isOrderComplete = orderKeywords.some(keyword => 
                agentResponse.toLowerCase().includes(keyword.toLowerCase())
            );
            
            if (isOrderComplete) {
                // 기본 주문 아이템 (실제로는 Agent가 더 구체적으로 말할 수 있음)
                const defaultOrder = ['화이트 빵', '양상추', '토마토'];
                
                // Agent 응답에서 빵 종류 추출 시도
                let bread = '화이트 빵';
                if (agentResponse.includes('호밀')) bread = '호밀 빵';
                if (agentResponse.includes('통밀')) bread = '통밀 빵';
                
                // Agent 응답에서 재료 추출 시도
                const ingredients = [];
                if (agentResponse.includes('양상추')) ingredients.push('양상추');
                if (agentResponse.includes('토마토')) ingredients.push('토마토');
                if (agentResponse.includes('치즈')) ingredients.push('치즈');
                
                const orderItems = [bread, ...ingredients];
                console.log('추출된 주문:', orderItems);
                
                // 주문 데이터 전송
                setTimeout(() => {
                    onVoiceOrderSuccess(agentResponse, orderItems);
                }, 1000); // 1초 후 주문 처리
            }
        }

        // 전역 함수로 등록하여 ElevenLabs에서 호출 가능하도록 함
        window.handleVoiceOrderComplete = handleVoiceOrderComplete;
        window.onVoiceOrderSuccess = onVoiceOrderSuccess;
        window.extractOrderFromAgentResponse = extractOrderFromAgentResponse;

        // 개발용: 음성 주문 테스트 함수 (콘솔에서 호출 가능)
        window.testVoiceOrder = function() {
            const testItems = ['화이트 빵', '양상추', '토마토'];
            onVoiceOrderSuccess('테스트 음성 주문', testItems);
        };
        
        console.log('음성 주문 함수 준비 완료!');
        console.log('테스트: testVoiceOrder() 실행');
    </script>
    """
    
    # 원래 HTML에 스크립트만 주입
    content = content.replace('</body>', voice_order_script + '</body>')
    return content

@app.route('/complete')
def order_complete():
    """주문 완료 페이지"""
    with open('/home/son/kiosk/complete.html', 'r', encoding='utf-8') as f:
        return f.read()

@app.route('/data')
def show_orders():
    """주문 내역을 JSON 형식으로 반환"""
    # 주문을 최신순으로 정렬
    sorted_orders = sorted(orders, key=lambda x: x.get('timestamp', 0), reverse=True)
    
    # 한글을 영어로 변환하는 함수
    def convert_to_english(korean_name):
        korean_to_english = {
            '화이트 빵': 'White Bread',
            '호밀 빵': 'Rye Bread',
            '통밀 빵': 'Whole Wheat Bread',
            '양상추': 'Lettuce',
            '토마토': 'Tomato',
            '치즈': 'Cheese'
        }
        return korean_to_english.get(korean_name, korean_name)
    
    # 가격 정보를 제거한 주문 데이터
    orders_data = []
    for order in sorted_orders:
        order_data = {
            'order_number': order.get('order_number', 'N/A'),
            'order_time': order.get('order_time', 'N/A'),
            'method': order.get('method', 'button'),
            'method_name': order.get('method_name', '버튼 주문'),
            'bread': '',
            'ingredients': []
        }
        
        # 빵과 재료를 구분해서 분류 (영어로 변환)
        for item in order.get('items', []):
            item_name = item.get('name', 'N/A')
            english_name = convert_to_english(item_name)
            
            # 빵인지 확인
            if '빵' in item_name:
                order_data['bread'] = english_name
            else:
                # 재료로 분류
                order_data['ingredients'].append(english_name)
        
        orders_data.append(order_data)
    
    response = jsonify({
        'orders': orders_data,
        'total_orders': len(orders)
    })
    response.headers['Content-Type'] = 'application/json; charset=utf-8'
    return response

@app.route('/api/order', methods=['POST'])
def receive_order():
    """키오스크에서 주문 데이터를 받는 API"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({'success': False, 'error': 'JSON 데이터가 필요합니다.'}), 400
        
        # 한국 시간대 설정
        korea_tz = pytz.timezone('Asia/Seoul')
        now = datetime.now(korea_tz)
        
        # 주문 번호 생성 (타임스탬프 기반)
        order_number = f"ORD{int(now.timestamp())}"
        
        # 주문 데이터 구성
        order_data = {
            'order_number': order_number,
            'order_time': now.strftime('%Y-%m-%d %H:%M:%S'),
            'order_date': now.strftime('%Y-%m-%d'),
            'timestamp': now.timestamp(),
            'method': data.get('method', 'button'),  # 'voice' or 'button'
            'method_name': '음성 주문' if data.get('method') == 'voice' else '버튼 주문',
            'items': data.get('items', [])
        }
        
        # 주문 데이터 저장
        orders.append(order_data)
        
        # 콘솔에 로그 출력
        print(f"📦 새 주문 접수: {order_number}")
        print(f"   주문 방법: {order_data['method_name']}")
        print(f"   주문 시간: {order_data['order_time']}")
        print(f"   주문 아이템: {[item.get('name', 'N/A') for item in order_data['items']]}")
        
        return jsonify({
            'success': True,
            'order_number': order_number,
            'message': '주문이 성공적으로 접수되었습니다.'
        })
        
    except Exception as e:
        print(f"❌ 주문 처리 오류: {str(e)}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 400

@app.route('/api/orders')
def get_orders():
    """주문 데이터를 JSON으로 반환하는 API"""
    return jsonify({
        'orders': orders,
        'total_orders': len(orders)
    })

@app.route('/api/clear')
def clear_orders():
    """주문 데이터 초기화 (개발용)"""
    global orders
    orders.clear()
    return jsonify({'message': '주문 데이터가 초기화되었습니다.'})

if __name__ == '__main__':
    # 환경 변수에서 설정값 가져오기
    server_host = os.getenv('SERVER_HOST', '0.0.0.0')
    server_port = int(os.getenv('SERVER_PORT', 6027))
    server_ip = os.getenv('SERVER_IP', 'localhost')
    debug_mode = os.getenv('DEBUG', 'True').lower() == 'true'
    agent_id = os.getenv('AGENT_ID', 'agent_4101k7h0tqkvebbap2b5c52m2ccy')
    
    base_url = f"http://{server_ip}:{server_port}"
    
    print("🔧 서버 설정:")
    print(f"   호스트: {server_host}")
    print(f"   포트: {server_port}")
    print(f"   외부 IP: {server_ip}")
    print(f"   디버그 모드: {debug_mode}")
    print(f"   기본 URL: {base_url}")
    print(f"   Agent ID: {agent_id}")
    print("=" * 50)
    
    print("🥪 키오스크 통합 서버가 시작됩니다...")
    print(f"📍 메인 키오스크: {base_url}")
    print(f"📍 주문 내역 보기: {base_url}/data")
    print(f"📍 API 엔드포인트: {base_url}/api/order")
    print(f"📍 주문 데이터 조회: {base_url}/api/orders")
    print(f"📍 데이터 초기화: {base_url}/api/clear")
    print("⏰ 자동 새로고침: 5초마다")
    print("=" * 50)
    
    app.run(host=server_host, port=server_port, debug=debug_mode)
