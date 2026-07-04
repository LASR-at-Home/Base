import cv2
import numpy as np

# 설정 값
TABLE_HEIGHT = 0.44  # 테이블 높이

def get_fold_points(image, depth_map):
    # 1. 간단한 세그멘테이션 (티셔츠 마스크 가정)
    # 실제로는 RMBG나 DINO 결과를 사용
    mask = (image[:,:,0] > 200) # 파란색 계열 티셔츠 예시
    
    # 2. 깊이값 추출 및 Base Depth 적용
    segmented_depths = depth_map[mask > 0]
    valid_depths = segmented_depths[segmented_depths > 0]
    
    # 테이블 높이 기준 안전 장치 적용
    z_height = np.median(valid_depths) if len(valid_depths) > 0 else (TABLE_HEIGHT + 0.001)
    
    # 3. 소매점 추출 (마스크의 무게중심 기준 좌우 끝점 찾기)
    coords = np.column_stack(np.where(mask > 0))
    left_sleeve = coords[coords[:, 1].argmin()]
    right_sleeve = coords[coords[:, 1].argmax()]
    
    # 4. 접기 포인트 계산 (3등분)
    fold_point = left_sleeve + (1/3) * (right_sleeve - left_sleeve)
    
    return fold_point, z_height

# 테스트 실행
img = cv2.imread('/home/robocup/test_img/tshirt.png')
depth = np.load('/home/robocup/test_img/depth.npy')
f_point, z = get_fold_points(img, depth)

print(f"테스트 결과: 접기 포인트={f_point}, Z 높이={z}")