# Gimbal Designation Loop

NI-DAQmx와 C를 이용한 짐벌 위치 제어 실험 코드입니다.  
자이로 센서와 포텐쇼미터 신호를 읽고, PD 제어기와 저역통과필터를 적용하여 모터 명령을 출력하고 실험 데이터를 저장합니다.

## 파일 구성

| 파일 | 역할 |
|---|---|
| `main.c` | 프로그램 실행 순서와 메인 루프 |
| `mygimbal_runexp.c` | DAQ 입출력, 센서 변환, PD 제어, LPF, 데이터 저장 |
| `mygimbal_runexp.h` | 실험 상수와 함수 선언 |
| `mygimbal_designation_step.c` | Step 실험 명령과 실험 정보 반환 |
| `mygimbal_designation_step.h` | 샘플링 주파수, 실험 시간, 출력 파일명 정의 |

## 주요 설정

- Sampling frequency: 200 Hz
- Target angle: 20 deg
- Controller: PD controller
- Gyro LPF cutoff frequency: 10 Hz
- Emergency stop key: `f`

## DAQ 채널

- `Dev4/ai2`: Gyro voltage
- `Dev4/ai3`: Potentiometer voltage
- `Dev4/ao0`: Switch command
- `Dev4/ao1`: Motor command

## 실행 환경

이 프로젝트는 다음 환경을 전제로 합니다.

- Windows
- Visual Studio C project
- NI-DAQmx driver and C development support
- `NIDAQmx.h`를 찾을 수 있도록 설정된 include/library 경로
- NI DAQ 장치 이름이 `Dev4`로 설정된 환경

## 실행 전 주의사항

1. 실제 DAQ 채널과 코드의 `Dev4` 채널 설정이 일치하는지 확인합니다.
2. 짐벌을 정지한 상태에서 자이로 오프셋 계산을 수행합니다.
3. 비상 정지는 키보드의 `f` 키를 사용합니다.
4. 장비가 연결되지 않은 일반 컴퓨터에서는 그대로 실행되지 않습니다.

## 출력 데이터

실험 결과는 기본적으로 다음 파일에 저장됩니다.

```text
Step_DesignationLoop.txt
```

저장 열:

```text
time Vcmd Vc omega_h omega_h_filter Vpoten psi_h
```
