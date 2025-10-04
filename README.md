# Robocon 2025 A Team Master Node

## UART通信
Netetra作のプロトコルSBTPを使用します。  https://github.com/Netetra/sbtp

### NRCC-2025 (Niihama Robot Control Commands 2025)
#### 特徴
- 最初の1byteはコマンド
- ビックデンディアンを使用
- こちらのプロトコルも含みます https://github.com/NiiRoboKen/robocon-2025-a-esp32-ps4

#### 受信成功
```
| 0x00 |
```

#### ping
```
| 0x01 |
```

#### 受信失敗
```
| 0x02 | error code (uint_8)|
```

--- 

#### 独立ステアリング目標値送信(raspi -> esp)
```
| 0x10 | x (uint32_t) | y (uint32_t) | theta x 100 (uint32_t) | 
```
- x, y の単位はmm
- x, y にはマイナスの値も入る
- theta の単位はdegree 角度の100の値を入れる

#### 独立ステアリング現在値送信(esp -> raspi)
```
| 0x20 | x (uint32_t) | y (uint32_t) | theta x 100 (uint32_t) | 
```
- x, y の単位はmm
- x, y にはマイナスの値も入る
- theta の単位はdegree 角度の100の値を入れる

