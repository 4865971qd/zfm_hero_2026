 1. 安装依赖（一次性）

  sudo apt update
  sudo apt install -y ffmpeg pkg-config libxkbcommon-dev
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

  2. 编译

  cd ~/RM_zidinyi/rm-native-viewer
  cargo build --release

  3. 运行
ji
  cargo run --release

  就这一条命令。默认参数就是你要的：
  - MQTT Broker: 192.168.12.1:3333（裁判系统地址）
  - Topic: CustomByteBlock
  - 输入格式: h264
  - Client ID: 101


