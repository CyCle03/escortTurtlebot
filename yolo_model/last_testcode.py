import cv2
from ultralytics import YOLO

# 1. 모델 설정
model_path = 'last.pt'
model = YOLO(model_path)

# 2. 웹캠 연결
cap = cv2.VideoCapture(0)

print("진단 모드 시작! 종료하려면 'q'를 누르세요.")

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # 좌우 반전 (거울 모드)
    frame = cv2.flip(frame, 1)

    # 3. 모델 감지
    results = model(frame, conf=0.4)

    # 기본 상태는 STOP
    status = "STOP"
    display_color = (0, 0, 255)  # 빨간색 (BGR)

    # 4. 결과 분석 로직
    for r in results:
        # 화면에 박스와 라벨을 그린 프레임 가져오기
        annotated_frame = r.plot()

        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name = model.names[cls_id]

            # 터미널에 실시간 로그 출력 (디버깅용)
            print(f"🔎 감지됨: {name} | 확신도: {conf:.2f}")

            # 클래스 ID 확인 (0: Phone, 1: Hand, 2: Mask_VIP)
            # 마스크(2번)가 감지되었을 때만 MOVE로 변경
            if cls_id == 2:
                status = "MOVE"
                display_color = (0, 255, 0)  # 초록색 (BGR)

    # 5. 화면에 현재 상태 표시
    cv2.putText(annotated_frame, f"STATUS: {status}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, display_color, 3)

    # 6. 최종 화면 출력
    cv2.imshow("Turtlebot Control Diagnosis", annotated_frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
