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

#### pong
```
| 0x03 |
```

#### 非常停止
```
| 0x0A |
```
- 全ての動きを止めます

--- 
### 独立ステアリング

#### 独立ステアリング現在値送信(esp -> raspi)
```
| 0x20 | x (uint32_t) | y (uint32_t) | theta x 100 (uint32_t) | 
```
- x, y の単位はmm
- x, y にはマイナスの値も入る
- theta の単位はdegree 角度の100の値を入れる


#### 独立ステアリング目標値送信(raspi -> esp)
```
| 0x10 | x (uint32_t) | y (uint32_t) | theta x 100 (uint32_t) | 
```
- x, y の単位はmm
- x, y にはマイナスの値も入る
- theta の単位はdegree 角度の100の値を入れる

### Side アーム

#### Side アーム展開 (All)
```
| 0x30 |
```

#### Side アーム展開 (Right `0x31` Left `0x32`)
```
| command |
```

#### Side アーム開く(最大まで)
```
| 0x33 |
```

#### Side アーム収納 (All)
```
| 0x34 |
```

#### Side アーム収納 (Right `0x35` Left `0x36`)
```
| command |
```

### 共有アーム

#### 共有アームストップ (All)
```
| 0x50 |
```
- 共有アームの非常停止スイッチ

#### 共有アームストップ (Right `0x51` Left `0x52`)
```
| command |
```
- 共有アームの非常停止スイッチ

#### 共有アーム天井上げる
```
| 0x53 |
```
- 共有ボックスを上にかけるときの姿勢

#### 共有アーム回収モード
```
| 0x54 |
```
- ボックスを回収する時の姿勢

#### 共有アーム畳む 上 (All)
```
| 0x55 |
```
- 上側に畳む

#### 共有アーム収納 前 (All)
```
| 0x56 |
```
- 下側に収納

#### 共有アーム収納 前 (Right `0x57` Left `0x58`)
```
| command |
```
- 下側に収納

#### 共有アームスタート (All)
```
| 0x5A |
```
- 本番最初の一回だけ送る
- これを送らないとPID制御が始まらない

#### 共有アームスタート (Right `0x5B` Left `0x5C`)
```
| command |
```
- 本番最初の一回だけ送る
- これを送らないとPID制御が始まらない


#### 共有アーム吸引
```
| 0x60 | is_on (uint8_t) |
```
- `is_on`には0か1が入る(onなら1)

#### 共有アーム吸引 (Right `0x61` Left `0x62`)
```
| command | is_on (uint8_t) |
```
- `is_on`には0か1が入る(onなら1)

---


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

#### 独立ステアリング `0x01 0x02 0x03 0x04`
```
| 0x01 | theta | dir | duty | state |
```
- `theta`には0から180の値が入る
- `dir`には0か1が入る
- `duty`には0から100の値が入る
- `state`には0か1が入る

#### ロジャー展開 (All `0x10` Right `0x11` Left `0x12`)
```
| duty | is_up |
```
- `duty`には0から100の値が入る
- `is_up`には0か1が入る(上昇なら`1`)

#### 右アーム動作 `0x31`
```
| is_open | is_open_move | is_fold | is_fold_move |
```
- `is_open`には1か0が入ります(展開なら1)
- `is_open_move`には1か0が入ります(動くなら1)
- `is_fold`には1か0が入ります(掴むなら1)
- `is_fold_move`には1か0が入ります(動くなら1)

#### 左アーム動作 `0x32`
```
| is_open | is_open_move | is_fold | is_fold_move |
```
- `is_open`には1か0が入ります(展開なら1)
- `is_open_move`には1か0が入ります(動くなら1)
- `is_fold`には1か0が入ります(掴むなら1)
- `is_fold_move`には1か0が入ります(動くなら1)

#### 右アーム昇降 `0x41`
``` 
| duty | is_up | 
```
- `duty`には0から100が入ります
- `is_up`には1か0が入ります(上昇なら1)

#### 左アーム昇降 `0x42`
```
| duty | is_up | 
```
- `duty`には0から100が入ります
- `is_up`には1か0が入ります(上昇なら1)

#### 共有アーム姿勢(right:`0x51` left:`0x52`)
```
| 0x00 | number |
```
- `number`には各ボックスの番号が入る

#### 共有アーム姿勢角度(right:`0x51` left:`0x52`)
```
| 0x01 | theta1[deg] | isNegative1 | theta2[deg] | isNegative2 |
```
- `theta1`にはアームの肘にあたる角度が入る(0から180)
- `theta2`にはアームの手首にあたる角度が入る(0から180)
- `isNegative1`と`isNegative2`には`theta1` `theta2`それぞれの値が正か負かを入れる(正なら1)

#### 共有アーム姿勢duty(right:`0x51` left:`0x52`)
```
| 0x02 | duty1 | dir1 | duty2 | dir2 | 
```
- `duty1`にはアームの肘にあたるduty比が入る(0から100)
- `duty2`にはアームの手首にあたるduty比が入る(0から100)

#### 共有アームスタート(right:`0x51` left:`0x52`)
```
| 0x03 |
```
- 本番最初の一回だけ送る
- これを送らないとPID制御が始まらない

#### 共有アーム吸引 (right:`0x61` left:`0x62`)
```
| is_on |
```
- `is_on`には0か1が入る(onなら1)

