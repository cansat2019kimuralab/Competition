# Competition
本番で使用したプログラムを保管  

## プログラム一覧
### Arliss
- Arliss.py : 本番のメインプログラム
- CheckArliss.py : 本番直前の確認用プログラム
- ReceiveArliss.py : 
- ReceiveArliss_Distance.py : 
- ReceiveArliss_fast.py : 

### Noshiro
- Noshiro.py : 能代での本番のメインプログラム
- RecieveNoshiro.py : 能代での地上局側プログラム

## ライブラリ一覧
### [SensorModuleTest](https://github.com/cansat2019kimuralab/SensorModuleTest)
センサ，モジュール類のライブラリ  
- [BME280.py](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/BME280/BME280.py)  
気圧センサ用BME280用ライブラリ  
仕様書:[BME280.md](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/BME280/BME280.md)
- [BMX055.py](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/BMX055/BMX055.py)  
9軸センサBMX055用ライブラリ  
仕様書:[bmx055.md](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/BMX055/bmx055.md)
- [Capture.py](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/Camera/Capture.py)  
Raspberry pi用ライブラリ  
仕様書:[Camera.md](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/Camera/Camera.md)
- [GPS.py](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/GPS/GPS.py)  
GPS用ライブラリ  
仕様書:[GPS.md](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/GPS/GPS.md)
- [IM920.py](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/IM920/IM920.py)  
通信モジュールIM920用ライブラリ  
仕様書:[im920.md](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/IM920/im920.md)
- [Melting.py](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/Melting/Melting.py)  
溶断モジュール用ライブラリ  
仕様書:[Melting.md](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/Melting/Melting.md)
- [Motor.py](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/Motor/Motor.py)  
モータ用ライブラリ  
仕様書:[Motor.md](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/Motor/Motor.md)  
- [TSL2561.py](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/TSL2561/TSL2561.py)  
照度センサTSL2561用ライブラリ  
仕様書:[TSL2561.md](https://github.com/cansat2019kimuralab/SensorModuleTest/blob/master/TSL2561/TSL2561.md)

### [Detection](https://github.com/cansat2019kimuralab/Detection)
判定系プログラム用ライブラリ  
放出，着地，パラシュート，ゴール判定に使用  
- [goal_detection.py](https://github.com/cansat2019kimuralab/Detection/blob/master/GoalDetection/goal_detection.py) 
ゴール検出用ライブラリ  
仕様書:[GoalDetection.md](https://github.com/cansat2019kimuralab/Detection/blob/master/GoalDetection/GoalDetection.md)
- [ParaDetection.py](https://github.com/cansat2019kimuralab/Detection/blob/master/ParachuteDetection/ParaDetection.py)  
パラシュート検出用プログラム  
仕様書:[ParaDetection.md](https://github.com/cansat2019kimuralab/Detection/blob/master/ParachuteDetection/ParaDetection.py)
- [Release.py](https://github.com/cansat2019kimuralab/Detection/blob/master/ReleaseAndLandingDetection/Release.py)  
放出判定用ライブラリ  
仕様書:[ReleaseAndLanding.md](https://github.com/cansat2019kimuralab/Detection/blob/master/ReleaseAndLandingDetection/ReleaseAndLanding.md)
- [Land.py](https://github.com/cansat2019kimuralab/Detection/blob/master/ReleaseAndLandingDetection/Land.py)  
着地判定用ライブラリ  
仕様書:[ReleaseAndLanding.md](https://github.com/cansat2019kimuralab/Detection/blob/master/ReleaseAndLandingDetection/ReleaseAndLanding.md)

### [IntegratedProgram](https://github.com/cansat2019kimuralab/IntegratedProgram)
複数のモジュールを組み合わせたプログラム，ライブラリ  
- [Calibration.py](https://github.com/cansat2019kimuralab/IntegratedProgram/blob/master/Calibration/Calibration.py)  
地磁気センサ(BMX055)のキャリブレーション用ライブラリ  
仕様書:[Calibration.py](https://github.com/cansat2019kimuralab/IntegratedProgram/blob/master/Calibration/Calibration.md)
- [pidControl.py](https://github.com/cansat2019kimuralab/IntegratedProgram/blob/master/Control/pidControl.py)  
回転速度をPID制御で調整するライブラリ，キャリブレーションで使用  
仕様書:なし
- [RunningGPS.py](https://github.com/cansat2019kimuralab/IntegratedProgram/blob/master/Running/RunningGPS.py)
GPSを用いてゴールまで走行するプログラムで使用するライブラリ  
仕様書:[Running.md](https://github.com/cansat2019kimuralab/IntegratedProgram/blob/master/Running/Running.md)
- [Stuck.py](https://github.com/cansat2019kimuralab/IntegratedProgram/blob/master/Stuck/Stuck.py)  
スタック回避関係のライブラリ  
仕様書:なし
- [stuckDetection.py](https://github.com/cansat2019kimuralab/IntegratedProgram/blob/master/Stuck/stuckDetection.py)  
スタック検出関係のライブラリ  
仕様書:なし

### Mission
- [sendPhoto.py](https://github.com/cansat2019kimuralab/Mission/blob/master/sendPhoto.py)  
画像伝送用ライブラリ  
仕様書:なし

### Other
- [Other.py]()
その他の便利プログラムの詰め合わせ
仕様書:[Other.md](https://github.com/cansat2019kimuralab/Other/blob/master/Other.md)

## ブランチ一覧
- NoshiroFirst : 能代打ち上げ第1回
- NoshiroSecond : 能代打ち上げ第2回
- ArlissFirst : ARLISS打ち上げ第1回
- ArlissSecond : ARLISS打ち上げ第2回
