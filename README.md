# M100A_STM32 串口透传固件发布日志

## Non-LoRaWAN
2016/11/16 v1.0.0 

* 模块支持透传指定格式的串口数据用LoRa射频发送。
* 使用说明和串口协议格式请查看《M100A 非LoRaWAN串口数据透传模块使用说明》。
* 该版本支持的配置参数如下表描述，使用配置工具进行相应配置，使用说明请查看《M100A非LoRaWAN配置工具使用说明》：

|参数|备注/描述|
|:---:|:---:|
|DevEUI|8个字节的设备EUI地址(大端模式)|
|RF频率|通信频率：137～525MHz，需要配合相应的天线|
|SyncWord|同步字|
|PreambleLength|前导码长度|
|Tx Power|射频发射功率，值范围：[20, 14, 11, 8, 5, 2]|
|Datarate|射频扩频因子，值范围：[DR_SF12, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7H, DR_FSK]|
|BandWidth|射频带宽|
|HeaderMode|消息头模式|
|Coding Rate|消息编码率|
|CRC Enable|CRC校验|
|RX IQ signal|RX IQ 翻转|
|TX IQ signal|TX IQ 翻转|
|Debug Out Switch|Debug 打印输出开关[ON:1, OFF:0]|
|Baud|串口波特率|
|Data Bits|串口数据位|
|Stop Bits|串口停止位|
|Parity Bits|串口校验位|

---

## LoRaWAN
2016/11/16 v1.0.0 

* 该版本使用LoRaWAN 1.0.1协议 Class A 。
* 模块支持透传指定格式的串口数据到服务器。
* 使用说明和串口协议格式请查看《M100A 串口数据透传模块使用说明》。
* 该版本支持的配置参数如下表描述，使用配置工具进行相应配置，使用说明请查看《M100A配置工具使用说明》：

|参数|备注/描述|
|:---:|:---:|
|DevEUI|8个字节的设备EUI地址(大端模式)|
|激活方式|Over the Air 或 Personalization|
|AppEUI|8个字节的App EUI(大端模式)|
|AppKey|16个字节|
|NwkSKey|16个字节|
|AppSKey|16个字节|
|DevAddr|服务器分配的4个字节设备地址|
|Channel 0～15|信道0～15的配置|
|网络类型|public or private|
|Rx2 Channel|接收窗口2 的信道配置|
|Channel mask|屏蔽信道配置|
|Transmission Redundancy|Number of uplink messages repetitions [1:15] (unconfirmed messages only)|
|Receive Delay 1|默认1000000us|
|Receive Delay 2|默认2000000us|
|Join Accept Delay 1|默认5000000us|
|Join Accept Delay 2|默认6000000us|
|App Tx Dutycycle|模块入网后自行上报数据的时间间隔，默认60000000us|
|App Tx Dutycycle Random|时间间隔随机数，模式1000000us|
|Channel Datarate|不使用ADR时的信道数据速率索引，值范围：[DR_SF12, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7H, DR_FSK]|
|Channel Tx Power|信道发送功率索引，值范围：[20, 14, 11, 8, 5, 2]|
|ADR Enable|ADR开关[ON:1, OFF:0]|
|Application mode|无法配置，仅串口透传模式和配置模式。|
|设备 LoRaWAN Class 类型|仅支持A与C|
|Debug printf Enable|Debug 打印输出开关[ON:1, OFF:0]|

---