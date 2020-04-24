# stm32f3discovery-quaternion_visualization
stm32f3discovery上のl3gd20で角速度，lsm303dlhcで加速度と磁力を計測し，これらの値から，Madgwick Filterというアルゴリズムを用いてボードの姿勢推定を行う．

Quaternionというデータ表現形式を用い，ITMというプロトコルを用いてデータを送信する，

# Dependency
 
```toml
[dependencies]
cast = { version = "0.2.3", default-features = false }
byteorder = { version = "1.3.4", default-features = false }
cobs = { version = "0.1.4", default-features = false }
cortex-m = "0.6.2"
cortex-m-rt = "0.6.12"
embedded-hal = "0.2.3"
panic-semihosting = "0.5.3"
madgwick = "0.1.1"
nb =  "0.1.2"
aligned = "0.3.2"

[dependencies.f3]
features = ["rt"]
version = "0.6.1"
```
 
# Usage
 
pc側で，itmdumpを用いて受信したデータをビジュアライザにパイプする．
ビジュアライザのコードはここにある．↓

https://github.com/japaric/f3/tree/v0.5.3/viz

コマンドの例を以下に示す．
```bash
# 設定
stty -F /dev/ttyUSB0 raw 2000000 -echo
itmdump -f /dev/ttyUSB0 > data.txt

# 実行
itmdump -f /dev/ttyUSB0 | ./viz
```