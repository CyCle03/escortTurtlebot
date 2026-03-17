# 🐢 Escort Turtlebot YOLO Model

This repository contains the YOLOv8 models used as the 'eyes' of the leader-follower robot project, specifically trained to detect people and their states.

## 📌 Model Overview
The model is designed to facilitate interaction between humans and TurtleBots by identifying VIP status, safety gear, and specific gestures.

## 🏷️ Class Definitions
Use these class indices when parsing `results.box.cls`:

| Index | Class Name | Description |
| :--- | :--- | :--- |
| **0** | **Phone** | Person holding or using a smartphone. |
| **1** | **Hands Up** | Hand gesture (e.g., for signaling the robot). |
| **2** | **Mask VIP** | Identified VIP user wearing a mask. |

## 🛠️ Artifacts & Models
- `best.pt`: Initial training weights for mask detection (3 classes: with_mask, without_mask, incorrect).
- `last.pt`: Final comprehensive model (Phone, HandsUp, Mask_VIP).
- `last_openvino_model/`: Optimized model for Intel hardware (contains `.bin`, `.xml`, `metadata.yaml`).

---

# 한국어 안내

이 저장소는 에스코트 터틀봇 프로젝트의 시각 인식을 담당하는 YOLOv8 모델을 포함합니다. 이 모델은 로봇 주변의 사람을 감지하고, 상태(VIP 여부, 기기 사용 등)를 파악하기 위해 학습되었습니다.

## 📌 주요 클래스 정의
코드에서 `result.box.cls`로 반환되는 인덱스 및 상세 설명입니다:

| 인덱스 | 클래스명 | 설명 |
| :--- | :--- | :--- |
| **0** | **Phone** | 스마트폰을 손에 들고 있거나 사용 중인 상태. |
| **1** | **Hands Up** | 손을 머리 위로 든 상태 (로봇 호출 등의 제스처). |
| **2** | **Mask VIP** | 마스크를 착용한 보호 대상자(VIP). |

## 🛠️ 포함된 모델 파일
- `last.pt`: Phone, HandsUp, Mask_VIP를 모두 포함하는 최종 통합 학습 모델.
- `last_openvino_model/`: 인텔 CPU/GPU 환경에서 추론 속도를 높이기 위해 최적화된 OpenVINO 형식 모델.
- `best.pt`: 초기 단계의 마스크 착용 여부 판별 모델 (with_mask, without_mask 등).
