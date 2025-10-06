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

---

## CAN通信

### Sootta自作
#### 特徴
- 29bitの拡張IDにコマンド、送り元のID、送り先のIDが含まれます
```
| command (13bit) | sender id (8bit) | receiver id (8bit) |
```

#### Command
- `STOP`        `0x0000`
- `RESET`      `0x0001`
- `CONTROL`  `0x0002`
- `PING`        `0x0003`
- `PONG`        `0x0005`

以下のものは`Commnad`が `CONTROL` の時の8byteのデータを示します。

#### 独立ステアリング(0x01 0x02 0x03 0x04)
```
| 0x01 | theta | dir | duty | state |
```
- `theta`には0から180の値が入る
- `dir`には0か1が入る
- `duty`には0から100の値が入る
- `state`には0か1が入る

#### ロジャー展開 (0x10)
```
| duty | is_up |
```
- `duty`には0から100の値が入る
- `is_up`には0か1が入る(上昇なら`1`)

#### 右アーム動作 (0x31)
```
| is_open | is_open_move | is_fold | is_fold_move |
```
- `is_open`には1か0が入ります(展開なら1)
- `is_open_move`には1か0が入ります(動くなら1)
- `is_fold`には1か0が入ります(掴むなら1)
- `is_fold_move`には1か0が入ります(動くなら1)

#### 左アーム動作 (0x32)
```
| is_open | is_open_move | is_fold | is_fold_move |
```
- `is_open`には1か0が入ります(展開なら1)
- `is_open_move`には1か0が入ります(動くなら1)
- `is_fold`には1か0が入ります(掴むなら1)
- `is_fold_move`には1か0が入ります(動くなら1)

#### 右アーム昇降 (0x41)
``` 
| duty | is_up | 
```
- `duty`には0から100が入ります
- `is_up`には1か0が入ります(上昇なら1)

#### 左アーム昇降 (0x42)
```
| duty | is_up | 
```
- `duty`には0から100が入ります
- `is_up`には1か0が入ります(上昇なら1)


