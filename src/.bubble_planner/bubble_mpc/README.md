# 在xavier上執行gen_model.py

```bash
# 進入acados_env環境
source /opt/acados_env/bin/activate

cd ~/rover2_ws/src/bubble_planner/bubble_mpc/bubble_mpc_model/

# 刪除舊檔
rm -rf c_generated_code/ __pycache__/ acados_ocp.json 

# 建立新檔
python3 gen_model.py

# 退出acados_env環境
deactivate
```

# 在xavier上裝acados

## 安裝acados

```bash
# 下載acados
cd /opt
sudo git clone https://github.com/acados/acados.git
cd acados
sudo git submodule update --init --recursive

# 編譯 acados
mkdir -p build
cd build

# 我們直接用您系統現有的 cmake (3.27)
# 參數保持針對 Xavier (ARMv8) 的優化
sudo cmake -DACADOS_WITH_QPOASES=ON \
      -DACADOS_INSTALL_DIR=/opt/acados \
      -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57 \
      ..

sudo make install -j4
```

## 建立 Python 隔離環境 (關鍵！保護系統 Python)

```bash
# 1. 建立虛擬環境
sudo python3 -m venv /opt/acados_env

# 2. 啟動虛擬環境 (注意：之後每次要生成代碼都要先做這行)
source /opt/acados_env/bin/activate

# 3. 安裝 Python 依賴
pip install --upgrade setuptools wheel
pip install -e /opt/acados/interfaces/acados_template
```

## 下載生成器 (Tera Renderer)

```bash
# 1. 下載源碼到暫存區
cd /tmp
git clone https://github.com/acados/tera_renderer.git
cd tera_renderer

# 2. 切換到支援 Ubuntu 20.04 的舊版本
git checkout v0.0.34

# 3. 降級依賴套件 (必須在 tera_renderer 目錄下執行)
## 解決 pest 版本過新問題
cargo update -p pest_derive --precise 2.7.15
cargo update -p pest_generator --precise 2.7.15
cargo update -p pest_meta --precise 2.7.15
cargo update -p pest --precise 2.7.15

## 解決 backtrace 版本過新問題
cargo update -p backtrace --precise 0.3.71

# 4. 開始編譯 (Release 模式)
cargo build --verbose --release

# 5. 安裝編譯好的檔案
## 建立存放目錄 (若不存在)
sudo mkdir -p /opt/acados/bin

## 複製檔案
sudo cp target/release/t_renderer /opt/acados/bin/t_renderer

## 賦予權限
sudo chmod +x /opt/acados/bin/t_renderer

# 6. 驗證 (成功應顯示 "t_renderer 0.0.34" 且無報錯)
/opt/acados/bin/t_renderer --version
```

## 設定環境變數

```bash
echo 'export ACADOS_SOURCE_DIR=/opt/acados' >> ~/.bashrc
echo 'export ACADOS_INSTALL_DIR=/opt/acados' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/acados/lib' >> ~/.bashrc
source ~/.bashrc
```
