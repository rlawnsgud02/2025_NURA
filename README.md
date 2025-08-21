# 🚀 TEAM ASEC 2025 NURA Avionics

<div align="center">

<img src="images/Launch.gif" width="400" />

</div>

## Introduction
본 저장소는 **건국대학교 항공우주동아리 ASEC 로켓팀**이 2025년도 NURA 대회를 위해 개발한 **Avionics 시스템 코드**입니다.  

이번 대회의 목표는 **카나드를 통한 롤 제어**이며, 발사 후 로켓을 **90° 롤 회전 후 안정적으로 유지**함으로써 제어 성능을 입증하고자 합니다.

### System Goals
- 기본적인 **로켓 발사 및 회수 기능** 수행  
- 건국대학교 연구 목표 충족 및 검증

### Key Requirements
1. 전원 인가 후 최소 **30분 이상 안정 동작**  
2. 지상 준비 과정에서는 **안전핀(Safety Pin)** 으로 낙하산 사출 방지  
3. 센서 데이터의 **SD 카드 저장 & 지상국 실시간 전송**  
4. 지정된 조건 충족 시 **낙하산 반드시 사출**  
5. 지상국과 **최소 400m 이상 통신 거리 확보**
   
***

## Hardware Configuration

### Bill of Materials (BOM)

<div align="center">

| Main Category         | Sub Category         | Component / Model                  |
|-----------------------|----------------------|------------------------------------|
| **MCU & Core**        | MCU                  | Arduino Nano ESP32                 |
| **Sensors**           | IMU                  | EBIMU-9DOFV5-R3                    |
|                       | Barometric           | DFRobot Fermion BMP390L            |
|                       | GPS Module           | SparkFun GPS Breakout - NEO-M9N, SMA |
|                       | GPS Antenna          | AKA150                             |
| **RF & Communication**| RF Module            | NMT-UM434R1-C SEVB (G2)            |
|                       | RF Antenna           | NMT-SA434G2                        |
|                       | GCS Antenna          | YG-447                             |
| **Storage & Power**   | SD Socket            | SZH-EKBZ-005                       |
|                       | 3.3V LDO             | LM1117T-3.3/NOPB                   |
|                       | 5V LDO               | L4940V5                            |
| **Actuators**         | Canard Servo         | AFRC-D3519HB-S                     |
|                       | Parachute Servo      | S2300M                             |

</div>

### PCB Design

<div align="center">
  
<p float="left">
  <img src="images/pcb1.jpg" width="300" />
  <img src="images/pcb2.jpg" width="300" />
  <img src="images/avionics_module.jpg" width="300" />
</p>

</div>


본 PCB는 **KiCad 9.0**을 이용하여 설계되었습니다. 공간 효율성을 극대화하기 위해 보드를 **4개의 파트로 분할 설계**하고, 이를 **적층(Stacked) 구조**로 구성하였습니다.  

각 PCB 층은 몰렉스 커넥터를 통해 연결되며, 핀이 90° 꺾인 **몰렉스 5268**을 PCB에 납땜하고, **몰렉스 5264**를 이용하여 상호 연결하도록 구현하였습니다.  

📂 PCB 설계 파일은 아래 링크에서 확인하실 수 있습니다:  
[Google Drive - PCB Files](https://drive.google.com/file/d/1G7LwpJqrYb3B2x5KDOTYG1Fmn34BIEfY/view?usp=drive_link)

***

## Code Description

***
## Contributors

### Team Members

**스마트운행체공학과 김준형**  
Avionics 총괄. 센서 처리(IMU, GPS), RTOS, PID 제어, RF 통신 및 SD 로깅, PCB 설계 및 하드웨어

**항공우주모빌리티공학과 김랑현**  
센서 처리(Barometric), 낙하산 사출, RF 통신 및 SD 로깅

**전기전자공학부 김용진**  
지상국(GCS) 시각화 총괄, RF 통신 packet 최적화  
[Konkuk Univ. NURA GCS GitHub](https://github.com/kywls405/NURA2025_GCS)

**전기전자공학부 신승민**  
RF 통신, PCB 설계 및 하드웨어 제작

### Honorable Mention

**전기전자공학부 주현빈**  
RF 통신 하드웨어 검수  
