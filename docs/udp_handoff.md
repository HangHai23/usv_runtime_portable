# Mac 视觉数据到 Jetson Orin 的 UDP 接口

版本：air2s.vision v1，2026年9月28日。本文和 tools/jetson_vision_receiver.py 可直接交给嵌入式端 GPT。此接口发送观测，不发送推进器、舵机、自动驾驶或路径控制指令。实地控制前必须由船端实现安全状态机。

## 1 启动和文件

Mac项目根目录为 `/Users/hai/Documents/Codex/2026-08-24/ton`。

Mac发送端为 `tools/vision_udp.py`，已经接入 `tools/h264_receiver.py` 及旧版入口 `tools/h264_receiver_original.py`。接收参考为 `tools/jetson_vision_receiver.py`，仅需 Python 3 标准库，无需ROS、OpenCV或GPU。测试为 `tools/test_vision_udp.py`。

Jetson先运行（将 MAC_IP 换成Mac真实IPv4地址）：

```bash
python3 jetson_vision_receiver.py --mac-ip MAC_IP --port 5700
```

Mac运行（将 JETSON_IP 换成Jetson真实IPv4地址）：

```bash
cd "/Users/hai/Documents/Codex/2026-08-24/ton"
python3 tools/h264_receiver_original.py --udp-host JETSON_IP --udp-port 5700 --udp-hz 10 --udp-ttl-ms 500
```

iPhone仍向Mac的TCP 5600传视频；Mac向Jetson的UDP 5700发送检测。不要把Jetson地址填成iPhone视频接收地址。两机必须网络可达，Jetson防火墙允许指定Mac来源的UDP 5700。发送源端口由系统分配，不是固定5700。Mac不指定 --udp-host 就不启用UDP；指定后自动开启球、船、拿球和场地检测。--no-preview不能与UDP检测并用。

Ctrl+C或关闭接收器退出会关闭发送线程；按C重标定会清除待发数据、变更坐标系ID。端口5700可修改，不占用已有UDP 5601网络测试端口。

## 2 封装与更新规则

每个UDP数据报是一个独立UTF-8 JSON对象，没有换行帧分隔、没有二进制长度头。一个包携带完整最新快照，不能把两个包拼接解析。最大1400字节，意图减少常见1500 MTU局域网的IP分片；VPN/隧道等较小MTU仍需另行验证。包过大或含非有限数字时，发送fresh=false失效包，不截断JSON、不静默删船。多船数量增长可能触发此上限，需要以后升级协议或分消息设计。

默认10Hz定时发送，允许1至30Hz；不缓存历史帧、不补发旧结果、不等待确认。发送线程只保留最新检测快照，不阻塞视觉循环。发送成功不代表Jetson收到；UDP会丢包、乱序、重复。画面暂时无更新时可以多次发送同一frame，但seq仍递增，这不代表新增视觉观测。

## 3 完整示例

以下数值仅为格式示例，不是现场指令：

```json
{"protocol":"air2s.vision","version":1,"session":"0123456789abcdef0123456789abcdef","seq":123,"sent_unix_ms":1790000000100,"ttl_ms":500,"age_ms":36.2,"fresh":true,"observation":{"frame":88,"coord":"abcdef0123456789abcdef0123456789","field_valid":true,"field_status":"locked","size_m":[20.0,11.0],"goals":[{"side":"left","ends_m":[[0.0,4.2],[0.0,6.9]]},{"side":"right","ends_m":[[20.0,4.3],[20.0,6.8]]}],"ball":{"state":"detected","xy_m":[10.1,5.2],"score":0.86},"boats":[{"id":0,"xy_m":[9.0,5.0],"tag_heading_deg":12.3,"possession":"free"}],"processed_unix_ms":1790000000080}}
```

无画面或超时示例：

```json
{"protocol":"air2s.vision","version":1,"session":"0123456789abcdef0123456789abcdef","seq":124,"sent_unix_ms":1790000000600,"ttl_ms":500,"age_ms":536.2,"fresh":false,"observation":null,"coord":"abcdef0123456789abcdef0123456789"}
```

启动时age_ms可为null。reason可选，snapshot_too_large_or_nonfinite表示快照过大或数字非法。未知扩展字段可忽略，但未知主版本应拒绝。

## 4 字段定义

|字段|类型|意义|
|---|---|---|
|protocol / version|string / integer|固定air2s.vision / 1|
|session|string|每次发送进程启动生成的32字符UUID十六进制值|
|seq|integer|会话内递增包序号，重复frame也递增|
|sent_unix_ms|integer|Mac发送打包时的Unix毫秒，用于日志，不自动保证两机时钟一致|
|ttl_ms|integer|解码完成起算的有效期；默认500，参数允许100至2000|
|age_ms|number或null|Mac单调时钟测得的距该帧解码完成时间，包含检测及等待发送时间|
|fresh|boolean|该帧是否未超过有效期；不代表球、船或场地检测成功|
|observation|object或null|完整快照；失效包必须为null|
|frame|integer|Mac预览实际处理的帧计数，不是飞机原始曝光帧序号|
|coord|string|坐标系身份；重启或重新标定改变，必须清空船端轨迹与旧路径|
|processed_unix_ms|integer|快照构建时刻，约为检测结束后；不是相机曝光时间|
|field_valid|boolean|当帧有可用像素到场地映射|
|field_status|string|calibrated、locked、unknown、camera_unverified或disabled|
|size_m|[number,number]或null|主体估计宽、高；当前20×11米，不包含突出球门|
|goals|array|当帧通过局部可见性核验的球门；缺项就是不可用，不沿用旧项|
|goals.side|string|left或right；固定场地左右，不是船自身左右|
|goals.ends_m|[[x,y],[x,y]]|门线低Y、高Y端点；中心可取均值，长度可取端点欧氏距离|
|ball.state|string|detected实测、predicted短时预测、lost丢失|
|ball.xy_m|[x,y]或null|仅detected且field_valid时给坐标；预测点不作为导航坐标发布|
|ball.score|number|启发式质量分，不能解释为概率；预测时是衰减分，不是实测可信度|
|boats|array|当帧成功识别的AprilTag船只；空数组不是船在原点|
|boats.id|integer|AprilTag ID；船端必须选定自己的ID|
|boats.xy_m|[x,y]或null|标签中心场地位置，不是船体几何中心|
|boats.tag_heading_deg|number或null|场地X轴向右为0度，逆时针向上为90度；范围约[-180,180]。从标签中心指向角点0和1的边中点，不直接等同于船头|
|boats.possession|string|held、candidate、free、unknown；视觉拿球状态，不是机械夹持确认|

主体边界采用规则矩形近似，可由size_m重建四角：(0,0)、(宽,0)、(宽,高)、(0,高)。X向右、Y向上、单位估计米；不是GPS经纬度，也不是ENU或机体坐标。原始像素、相机PnP距离、预测球位置、无人机遥测、速度、船宽和导航路径暂不在v1包中，避免带宽和未经标定的数据混用。球的实际边界弯曲不通过此矩形表达。

## 5 时间安全和有效性

age_ms起点是Mac解码完成，不包含飞机至iPhone、iPhone至Mac以及解码之前的缓存。它不是曝光年龄，不是端到端延迟。两机单调时钟不可相减。未同步Unix时钟时，不能用Jetson当前Unix时间减sent_unix_ms当作可靠网络延迟。

示例接收器采用本地到达时刻加 min(500ms, ttl_ms-age_ms) 作为截止时间。它能处理Mac停流、断线和大部分旧数据，但不感知包到达之前的网络排队时间。自主控制前须建立已验证的时钟同步/往返时延机制，设置网络延迟上限，拒绝迟到包；仅使用当前示例不足以保证安全控制。

必须同时满足：fresh、field_valid、自己的船ID存在且位置方向有效，以及当前任务所需目标有效。追球要求ball.state为detected且位置非null；入门要求目标门线存在且本船定位有效。拿球unknown不能自动理解为free或held。失效包立即清空有效观测；通信中断超过本地看门狗期限同样清空。不要累积接收队列按历史包依次驱动电机。

## 6 Jetson端GPT实现任务

1. 使用IPv4 UDP监听5700，接收缓冲区至少65535，限制合法协议包为1400字节；按UTF-8 JSON解析，拒绝NaN、Infinity、类型错误和未知版本。
2. 限定来源Mac IP。按session识别重启、按seq拒绝重复和旧包。坐标系coord变化时撤销旧路径、清空速度估计，等待重新确认。不要仅依赖frame判断包序。
3. 参考脚本只维护最新观测并打印，不控制电机。将ReceiverState.current的有效性检查接入本地独立控制定时器，不能仅在收到包时检查超时。自己的ID未出现时立即将本船定位标为失效。
4. 标定标签的0/1边方向与船头的角度偏置，测量标签中心到船体中心的平移偏置；然后换算真实船体位置和航向。记录船长、船宽、转弯半径及定位裕量。
5. 追球、确认拿球、门内对准、穿门等任务状态机由船端负责。接近点必须留足船体尺寸和门柱裕量。Mac不选择目标球门、不下发航速，不提供水面其他障碍物检测。
6. 设置手动接管、独立急停和通信/视觉失效动作；先架空推进器或仿真验证，再低速封闭场地测试。严禁将示例里的坐标直接映射为PWM。
7. 记录原始UDP包、到达单调时间、任务状态和实际执行输出，先离线验证丢包、重复、乱序、重启、重标定、标签遮挡、过期及错误数据。

示例接收器检查核心类型和非有限坐标，并非完整生产级JSON Schema验证。它不含密码学认证、时钟同步、抗恶意包、任务规划和电机保护。来源IP可伪造，仅适合受控局域网；实际部署应隔离网络，并按威胁模型加入消息认证。session切换应在生产系统中结合已配对发送者及人工/安全握手确认。

## 7 本地验证

```bash
cd "/Users/hai/Documents/Codex/2026-08-24/ton"
python3 tools/test_vision_udp.py
```

测试只使用127.0.0.1临时端口并自动关闭。验证启动失效包、完整数据包、过期、重复拒绝、非法JSON/NaN、坐标重置及包尺寸限制。尚未验证真实Mac到Jetson无线网络，也没有做水面自主控制试验。
