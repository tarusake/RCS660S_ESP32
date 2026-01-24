# RCS660S_ESP32

このライブラリは、ESP32マイコンで[RC-S660/S](https://www.sony.co.jp/Products/felica/business/products/reader/RC-S660S.html)を制御するためのArduinoライブラリです。FeliCaカードの検出や通信を簡単に実装することができます。

## 機能

- RC-S660/Sリーダーの初期化
- FeliCaカードのポーリング（検出）
- カードIDm（識別情報）とPMm（製造情報）の取得
- カードへのコマンド送信と応答受信

## インストール方法

### PlatformIOの場合

`platformio.ini`ファイルに以下を追加：

```ini
lib_deps =
    tarusake/RCS660S_ESP32
```

### 手動インストール

1. このリポジトリをクローンまたはダウンロード
2. Arduinoのライブラリフォルダに配置
   - Windows: `Documents\Arduino\libraries\`
   - Mac: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`

## 使用方法

基本的な使用例：

```cpp
#include <Arduino.h>
#include "RCS660S_ESP32.h"

// UARTピン定義（例：M5 Core2 PORT-A）
#define UART_RX 33
#define UART_TX 32

// RCS660Sインスタンスの作成
RCS660S nfc;

void setup() {
  // デバッグ用シリアル初期化
  Serial.begin(115200);

  // RCS660S通信用シリアル初期化
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  // RCS660Sの初期化
  nfc.initDevice();

  Serial.println("NFCリーダー初期化完了");
}

void loop() {
  // FeliCaカードのポーリング
  if (nfc.polling() == 1) {
    // カード検出時の処理
    Serial.print("カード検出: IDm = ");

    // IDmの表示
    for (int i = 0; i < 8; i++) {
      if (nfc.idm[i] < 0x10) Serial.print('0');
      Serial.print(nfc.idm[i], HEX);
    }
    Serial.println();

    delay(1000);
  }

  delay(100);
}
```

より詳細な使用例は[examplesフォルダ](examples/)を参照してください。

### Suica残高取得（Core2例）

`examples/Core2/Core2.ino` にはSuica残高取得のサンプルを含めています。  
Suica以外のFeliCaカードを使用する場合は、サービスコードをカード仕様に合わせて変更してください。

## API リファレンス

### 主要なメソッド

- `RCS660S(Stream &serial = Serial1)` - コンストラクタ
- `int initDevice(void)` - デバイスの初期化
- `int polling(uint16_t systemCode = 0xFFFF)` - カードのポーリング
- `int cardCommand(const uint8_t *command, uint8_t commandLen, uint8_t response[], uint8_t *responseLen)` - カードへのコマンド送信

### 主要なプロパティ

- `uint8_t idm[8]` - カードのIDm（識別情報）
- `uint8_t pmm[8]` - カードのPMm（製造情報）
- `unsigned long timeout` - 通信タイムアウト（ミリ秒）
- `LogLevel logLevel` - ログレベル設定

## ライセンス

このライブラリはMITライセンスの下で公開されています。詳細は[LICENSE](LICENSE)ファイルを参照してください。
