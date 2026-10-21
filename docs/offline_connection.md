# 离线无网络环境连接 Jetson 调车指南

> 适用：比赛/调试现场没有路由器、没有外网，只有笔记本电脑 + Jetson（Xavier，Ubuntu 20.04 + ROS2 Foxy）。
> 目标：SSH 登录调车 + 浏览器看检测框网页（http://192.168.1.212:8081/）。

Jetson 网络现状：`eth0` 为 **DHCP 动态 IP**（NetworkManager 管理，连接名 `Wired connection 1`）。
无路由器时 DHCP 拿不到地址，**必须先配置静态 IP**（一次性配置）。

---

## 方案一：网线直连（最推荐，稳定可靠）

### 1.1 Jetson 配置静态 IP（一次性，联网或离线状态都可执行）

```bash
# 查看当前连接名（确认是 "Wired connection 1"）
nmcli con show

# 设置静态 IP 192.168.1.212
sudo nmcli con mod "Wired connection 1" \
  ipv4.method manual ipv4.addresses 192.168.1.212/24

# 立即生效
sudo nmcli con up "Wired connection 1"

# 验证
ip -4 addr show eth0   # 应显示 192.168.1.212/24
```

> 以后如需恢复 DHCP（回到路由器环境）：`sudo nmcli con mod "Wired connection 1" ipv4.method auto && sudo nmcli con up "Wired connection 1"`

### 1.2 Windows 笔记本配置静态 IP（每次离线调车时设置）

1. 用网线连接 **笔记本网口 ↔ Jetson 网口**（千兆网线，直通线即可，现代网卡自动翻转）。
2. Windows：设置 → 网络和 Internet → 以太网 → 找到你的有线网卡 → IP 分配改为手动：
   - **IP 地址**：`192.168.1.100`
   - **子网掩码**：`255.255.255.0`
   - 网关、DNS：留空
3. 测试连通：`ping 192.168.1.212`。

> 若笔记本有多块网卡/虚拟网卡（VMware、VirtualBox 等），先禁用虚拟网卡，
> 或在「网络连接」里确认启用的就是物理以太网卡，否则路由可能走错。

### 1.3 连接与调车

| 用途 | 命令 / 地址 |
|---|---|
| SSH 登录 | `ssh kami@192.168.1.212`（密码 666666） |
| 检测框网页 | 浏览器打开 `http://192.168.1.212:8081/` |
| 相机监控网页 | `http://192.168.1.212:8080/` |
| 传输文件 | `scp 文件 kami@192.168.1.212:~/` |

Windows 已配好 SSH 免密别名：直接 `ssh jetson`（本机 `~/.ssh/config` 已配置 192.168.1.212）。

---

## 方案二：USB 共享网络（rndis，无需网线）

Jetson 的 USB-C 口支持 rndis 虚拟网卡（系统已有 `rndis0` 接口）。用一根 USB 线连笔记本即可组网。

### 2.1 Jetson 配置 rndis 静态 IP（一次性）

```bash
sudo nmcli con add type ethernet ifname rndis0 con-name rndis-shared \
  ipv4.method manual ipv4.addresses 192.168.55.1/24
sudo nmcli con up rndis-shared
```

### 2.2 Windows 侧

1. USB 线连笔记本 ↔ Jetson 的 USB-C（OTG）口，设备管理器会出现网络适配器（Linux USB Ethernet/RNDIS Gadget，驱动自动装）。
2. 给该适配器设静态 IP：`192.168.55.100`，掩码 `255.255.255.0`。
3. `ping 192.168.55.1` 验证，然后 `ssh kami@192.168.55.1`。

> 注意：Jetson 的 USB-C 口若同时用于供电与 OTG 可能冲突；若 USB 线插上后 rndis0 不出现，
> 换另一根 USB 线或检查 BIOS/设备树是否启用 OTG。

---

## 方案三：手机热点（临时最方便）

手机开热点（不消耗流量），Jetson 与笔记本都连热点，即在同一局域网。

```bash
# Jetson 连热点（一次性，密码会记住）
sudo nmcli dev wifi connect "热点名" password "热点密码"

# 查看拿到的新 IP
ip -4 addr show wlan0
# 笔记本同样连热点后，用该 IP 访问（如 ssh kami@192.168.43.xxx）
```

> 热点分配的 IP 每次可能变，用 `ip -4 addr show wlan0` 确认。

---

## 常见问题

| 现象 | 处理 |
|---|---|
| 网线直连 ping 不通 | 检查双方 IP 是否同网段（192.168.1.x）；Windows 只留物理网卡静态 IP；换网线 |
| `ssh` 超时 | Jetson 是否开机完成（等 30s）；`ping 192.168.1.212` 先通；防火墙（Jetson 默认无） |
| 网页打不开（SSH 通） | 确认自启动服务在跑：`systemctl status rm_vision`；`ss -tlnp | grep 8081` |
| 现场路由器网段不是 192.168.1.x | 用方案三（热点）或临时改 Jetson 静态 IP 到现场网段 |
| USB 线插上无 rndis 网卡 | 换线 / 换 USB 口 / 用方案一网线 |
