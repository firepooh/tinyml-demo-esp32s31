# TinyML Demo — ESP32-S31-Korvo-1 + SC101IOT

[ozanoner/tinyml-demo-esp32s31](https://github.com/ozanoner/tinyml-demo-esp32s31)을 가져와 수정한 저장소다.
SC101IOT 카메라를 단 ESP32-S31-Korvo-1 보드에서 ESP-IDF v6.1로 빌드할 수 있게 고쳤다.

"Hi, ESP"라고 말한 뒤 "cheese"라고 하면 사진을 찍는다.
YOLO11n이 사진 속 물체를 찾아 LCD에 박스와 이름으로 표시한다.
모든 처리는 보드 안에서 이루어진다.

원본 설명은 [docs/README.upstream.md](docs/README.upstream.md)에 있다.

## 하드웨어

| 항목 | 내용 |
|---|---|
| 보드 | ESP32-S31-Korvo-1 (16 MB flash, 16 MB PSRAM) |
| 카메라 | SC101IOT (DVP, 1280×720 YUV422) |
| 화면 | 800×480 RGB LCD, GT1151 터치 |
| 개발 환경 | ESP-IDF v6.1 (Windows) |

## 원본과 달라진 점

- **IDF 버전**: `main/idf_component.yml`의 요구 버전을 `>=6.2`에서 `>=6.1`로 낮췄다.
- **컴파일 옵션**: `-Wno-error=non-c-typedef-for-linkage`는 GCC 16 이상에서만 붙인다.
  v6.1의 GCC 15는 이 옵션을 모른다.
- **카메라**: 원본 Korvo-1 설정은 OV3660(240×240 RGB565)을 쓴다.
  여기서는 SC101IOT를 켜고, 1280×720 UYVY 프레임을 소프트웨어로 4:1 축소한다.
  검출기와 LCD 모두 320×180 이미지를 받는다.
  관련 파일은 `devkits/korvo1/sdkconfig.defaults`, `devkits/korvo1/camera_korvo1.cpp`다.
- **마이크 코덱 클럭**: BSP(`esp32_s31_korvo_1` 1.0.1)는 ES8389를 기본 옵션(`no_dac_ref=false`)으로 만든다.
  이 모드에서 esp_codec_dev 1.5.9는 BCLK를 16 kHz × 16 bit × 4 = 1.024 MHz로 보고 분주비를 정한다.
  그런데 BSP의 I2S는 2슬롯 STD라서 실제 BCLK는 512 kHz다.
  그 결과 코덱 ADC가 8 kHz로 동작해 모든 샘플이 두 번씩 반복됐다.
  WakeNet은 이 소리로는 "Hi, ESP"를 인식하지 못했다.
  `devkits/korvo1/audio_korvo1.cpp`에서 코덱을 직접 만들고 `no_dac_ref=true`를 주어 고쳤다.
  이 앱은 DAC 레퍼런스 신호를 쓰지 않으므로(AFE `ref=0`) 부작용은 없다.

## 빌드와 플래시

ESP-IDF v6.1 환경을 불러온 PowerShell에서 실행한다.

```powershell
idf.py --preset korvo1 build
idf.py --preset korvo1 -p COM4 flash monitor
```

결과물은 `build/korvo1/`에 생긴다. 포트 번호는 PC에 맞게 바꾼다.

### 플래시 속도

esptool v5.6에는 ESP32-S31용 플래시 스텁이 아직 없다.
그래서 IDF는 `--no-stub`으로 ROM 부트로더가 직접 쓰게 하고, 압축 전송도 하지 않는다.
기본값인 460800 baud에서는 앱(약 4 MB)만 써도 140초쯤 걸린다.
Korvo-1의 CP210x USB-UART 포트에서 2 Mbaud로 쓰면 70초 정도로 줄어든다.

```powershell
idf.py --preset korvo1 -p COM4 -b 2000000 app-flash
```

`app-flash`는 앱 파티션만 다시 쓴다.
음성 모델(`model`)과 YOLO 모델(`coco_det`) 파티션이 바뀌지 않았다면 이것으로 충분하다.

## 알려진 제약

- SC101IOT 모듈의 장착 방향은 아직 보드에서 확인하지 않았다.
  화면이 뒤집혀 나오면 `camera_korvo1.cpp`의 `kFlipRows`나 `kMirrorCols`를 `true`로 바꾼다.
- 원본의 DVP 안정화 패치(`devkits/korvo1/patches/`)는 IDF v6.1 릴리스에 적용되지 않는다.
  빌드는 경고만 내고 계속 진행된다.
  부팅 후 첫 캡처가 실패할 수 있지만, 500 ms 뒤 실패로 처리되고 앱은 멈추지 않는다.
- 클럭을 고친 뒤 마이크 입력이 작다(말할 때 피크 5000 안팎).
  코덱 아날로그 게인은 이미 최대인 36.5 dB다.
  그래서 "Hi, ESP"와 "cheese"를 놓칠 때가 많다. 보드 가까이에서 또렷하게 말한다.
- Korvo-1 사용자 가이드는 ADC 데이터를 TDM으로 받는 구성을 전제한다.
  위의 클럭 수정은 BSP의 2슬롯 STD 설정에 코덱을 맞춘 우회책이다.
  I2S를 TDM 4슬롯으로 바꾸면 마이크 2개와 에코 제거용 레퍼런스를 함께 받을 수 있다.
  이 방식은 아직 시도하지 않았다.

## 라이선스

MIT. 원본 저작권 표시는 [LICENSE](LICENSE)에 있다.
