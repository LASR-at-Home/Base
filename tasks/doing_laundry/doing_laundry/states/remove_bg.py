from rembg import remove
from PIL import Image

# 1. 파일 경로 설정
input_path = '/home/hayeonglee/test_img/basket.jpeg'       # 원본 이미지
output_path = 'basket.png' # 배경이 지워진 결과 (투명 배경)

try:
    # 2. 이미지 불러오기
    input_image = Image.open(input_path)

    print("배경 제거 중... (첫 실행 시 모델을 로드하느라 몇 초 정도 걸릴 수 있습니다)")
    
    # 3. 배경 제거 (이 함수가 ~/.u2net/u2net.onnx 파일을 사용합니다)
    output_image = remove(input_image)

    # 4. 결과 저장 (PNG 형식이어야 투명 배경이 유지됩니다)
    output_image.save(output_path)
    
    print(f"완료! '{output_path}' 파일이 생성되었습니다.")

except FileNotFoundError:
    print(f"에러: '{input_path}' 파일을 찾을 수 없습니다. 경로를 확인해주세요.")
except Exception as e:
    print(f"에러 발생: {e}")