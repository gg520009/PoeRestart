PA0 T2P
PA1 PWGD
PB0 LED
PB6 USART1_TX
PA6 relay // 置高负载加上
PA7 dcdc en

1、PA0、PA1用作ADC采样，分别采样LTC4294的T2P、PWGD信号
2、PB0接led用作信号指示
3、PB3用作USART1_TX发送日志
4、PA6和PA7配置为输出，开漏输出，PA6默认高电平，PA7默认为低电平；
5、10us内设置一个状态机器：状态1，led 1秒闪一次，持续检测PA1电平，若持续高于1.5V，维持200ms以上切换为状态2；状态2，led 2s闪一次，通过T2P判定功率等级，若为71W，切换到状态3，否则切换到状态4；状态3，led 3秒闪一次，PA6置高，PA7置高；状态4，led关，PA6置低，PA7置低

