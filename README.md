# 工程机器人 C 板控制工程

本工程对应 DT7/DR16 遥控器、四轮差速底盘、双后腿升降和四轴机械臂。基础外设由本机 STM32CubeMX 6.18.1 生成，使用 STM32CubeF4 1.28.3、FreeRTOS CMSIS-RTOS v2 和 GCC，面向 CLion/CMake。

当前上电默认进入底盘调试模式，可控制四轮和左右后腿联动升降。

本次已加入四轮底盘功率限制，详见 [功率限制说明](功率限制说明.md)。按工程机器人65W规则配置，裁判模块接CAN2、0x31、DLC4、大端。默认留5W余量，四轮模型预算为60W；升降及机械臂不参与计算和缩放。当前台架配置在无裁判反馈时采用45W四轮模型预算。比赛前将 `ROBOT_POWER_REQUIRE_REFEREE` 改回 1。

## 第一次使用

1. 用 CLion 打开本目录的 `CMakeLists.txt`，采用提供的 `CMakePresets.json` 中 Debug 配置。工具链为 ARM GCC，不使用桌面 MinGW 生成固件。默认工具路径对应本机已有的 CubeCLT。
2. 也可以在 PowerShell 执行 `powershell -ExecutionPolicy Bypass -File .\Build.ps1`。Release 使用 `-Configuration Release`。更换工具安装路径时，传入 `-CubeCltRoot`，并相应修改 CMakePresets 的工具路径。
3. 固件位于 `build/Debug/engineer_robot.elf`、`.hex`、`.bin`。SWD 可使用 ST-Link；`openocd.cfg` 提供连接配置，需按自己的调试器接线选择复位方式。
4. 接好 DR16 与 C 板专用 DBUS 接口，打开遥控器。在 CLion 观察 `g_robot_debug.input.rc`，核对通道与左右开关。
5. 确认四个摇杆轴回中、两个拨杆都居中，保持 500ms。所选模式所需的电机反馈正常时即可解锁。
6. 核对下文的电机 ID、达妙控制模式和机械行程。按需要选择底盘模式或整车模式。

## 三个应用文件夹

当前采用C与C++17混合工程：CubeMX基础外设与控制数学核心使用C，参考模板的CAN/电机对象实现使用C++。头文件在各自inc，实现文件在各自src。CLion仍直接打开根CMakeLists，不需要把CubeMX生成的.c改成.cpp。

```text
bsp/inc/can_device.h          收发设备类、回调注册、固定容量发送队列
bsp/src/can_device.cpp        报文复制、同ID合并、过期丢弃、世代取消
bsp/src/bsp_can.cpp           HAL过滤器/中断、CAN1/CAN2发送任务、异常停机
bsp/inc/bsp_io.h              面向C控制层的硬件接口
bsp/src/bsp_dbus.c            遥控器DMA/IDLE接收
bsp/src/bsp_board.c           LED、预留限位和急停
bsp/src/bsp_timebase.c        TIM7 HAL时基
user_driver/inc/motor_driver.h       motor_dev统一状态接口
user_driver/inc/DJI_motor_driver.h   DJI_motor与共享控制组
user_driver/src/DJI_motor_driver.cpp 大疆对象、编码器回调、分组槽位
user_driver/inc/DM_motor_driver.h    DM_motor的MIT/PV/使能/失能接口
user_driver/src/DM_motor_driver.cpp  达妙对象与状态回调
user_driver/inc/motor_system.h       C/C++桥接接口
user_driver/src/motor_system.cpp     十台电机和裁判接收对象的集中构造
user_driver/inc/src robot_config     ID、方向、传动比、PID与模式配置
user_driver/inc/src drv_motor       C协议编解码函数，供对象层复用
user_driver/inc/src drv_rc/drv_pid/drv_power 遥控器、PI与四轮功率模型
user_driver/inc/src motor_output    电机发帧、使能与500/200Hz时序
task/inc/src chassis_control  差速解算、四轮速度PI和后腿联动
task/inc/src gimbal_control   云台各轴控制和夹爪保持
task/inc/src robot_core       模式协调、关控和四轮速度PI后的功率限制
task/inc/src robot_app        控制/维护任务和调试状态
task/src/robot_selftest.c     原功能与功率主机测试，不编入固件
task/src/can_motor_selftest.cpp 对象/队列/控制器回放测试，不编入固件
```

### 参考模板的接入方式

接口组织参考 `D:/xx/dk/dx_assignment5_freertos-main` 的CAN_bsp、motor_driver、DJI_motor_driver。底层使用 `can_device_receive`、`can_device_transmit`、`motor_dev`、`DJI_motor` 和同风格的 `DM_motor`。所有电机实例集中在motor_system.cpp，便于阅读和更改接线映射。

接收流程为“标准数据帧 → 总线/ID/DLC匹配 → 已注册回调 → 电机对象反馈 → 临界区快照 → 控制器”。使用16位ID列表硬件过滤；CAN1有4个接收对象，CAN2有7个，包含DLC4的0x31裁判帧。回调带对象context，无需对带虚函数的继承类使用container_of。

发送流程为“控制器 → 大疆共享组/达妙独立对象 → 报文值副本队列 → can1_send/can2_send → HAL邮箱”。BSP_CAN_Send返回是否成功入队，实际发送完成数看 `g_robot_debug.can[0/1].completed`。大疆按组统一发送，达妙按各自ID发送；不会增加第二套速度/位置PI，四轮功率限制与抗积分饱和仍在robot_core中执行。

对象构造立即返回，第一帧反馈建立上电角度基准，离线与解锁由原控制器检查。构造阶段仅注册软件设备，HAL启动前一次性配置过滤器；后续添加或改ID需在motor_system.cpp更新对象并重新初始化，当前不支持运行中热插拔注册。

两条总线各有固定容量16的队列，队列复制完整payload，同ID待发命令合并为最新值。默认过期5ms的报文被丢弃并上报故障；关控先清队列、递增世代并中止硬件待发邮箱。发送任务在同一短临界区完成出队、世代检查及HAL入邮箱，避免关控后旧帧重新入邮箱。

邮箱完成与中止中断唤醒发送任务，发送前查询真实空邮箱，不依赖可能在中止或失败路径失配的3计数信号量。两路开启CAN自动重发，邮箱超过5ms仍未完成时中止并按原安全逻辑处理。队列容量、接收对象容量、最长待发时间集中在robot_config.h的ROBOT_CAN_*宏中。

驱动对象使用固定存储，不申请电机对象堆内存。底层设置函数只更新待发缓冲；大疆共享组和达妙对象的can_send_msg才将报文副本入队。正常整车控制统一由robot_core产生最终输出，再经MotorSystem_SendFrame提交，以保持遥控保护、限位和四轮功率约束。

GM6020明确使用电压接口motor_set_voltage；M3508/M2006使用motor_set_current。最终原始CAN指令用motor_set_raw_output，此接口不再重复反转方向。达妙MIT/PV位置使用减速输出轴rad，额外传动比与关节方向继续由上层配置映射。

生成的Core/Inc、Core/Src、Drivers和Middlewares仍属于CubeMX基础框架。应用C/C++都在上述三个文件夹，参考工程的任务与陀螺仪依赖未接入本工程。

## 运行模式

在 `user_driver/inc/robot_config.h` 设置 `ROBOT_DEFAULT_PROFILE`。调试时也可写 `g_robot_requested_profile`。

| 值 | 宏 | 运行范围 | 需要反馈的电机 |
|---|---|---|---|
| 1 | ROBOT_PROFILE_CHASSIS | 四轮与双后腿 | 四台3508、两台4310 |
| 3 | ROBOT_PROFILE_FULL | 底盘与云台 | 全部十台电机 |

默认是底盘模式。切换模式时撤掉当前动作输出，摇杆和拨杆回中 500ms 后解锁。底盘模式向云台发送零输出或失能指令。

## 遥控器映射

DBUS 通道0为右水平、1为右竖直、2为左水平、3为左竖直。协议左开关S1在字节5的bit6～7，右开关S2在bit4～5；上/下/中分别为1/2/3。

| 操作 | 行为 |
|---|---|
| 左摇杆上下 | 底盘前后，带速度斜坡 |
| 左摇杆左右 | 左右差速转向，支持原地转向 |
| 左拨杆上 / 中 / 下 | 后腿同步抬升 / 保持 / 下降 |
| 右拨杆上 | 模式一，右水平控制大yaw，右竖直控制大pitch |
| 右拨杆下 | 模式二，右竖直控制小pitch，右水平控制夹爪 |
| 右拨杆中 | 机械臂各轴保持已有目标位置或夹紧状态 |
| 对应摇杆轴回中 | 停止增加关节角目标，闭环保持已有目标 |

底盘和后腿不依赖右拨杆是否处于模式一或二。模式切换时，未被操作的关节持续保持目标。默认夹爪右推夹紧、左推松开；方向、各摇杆符号都在配置入口可改。

四台 M3508 使用 P19 减速箱，精确减速比为 3591/187。底盘按 `ROBOT_WHEEL_MAX_OUTPUT_RPM` 解算轮端目标，乘减速比后与 C620 转子转速反馈做速度 PI。当前轮端最高目标约 78.1 rpm，对应转子 1500 rpm。

## 总线与电机编号

两路均为标准 CAN 数据帧、1Mbps。APB1 为42MHz，CAN预分频3，SJW1、BS1=11、BS2=2，14TQ，采样点约85.7%。

| 总线 | 电机 | 电机本地ID | C板控制帧ID | 反馈帧ID |
|---|---|---:|---|---|
| CAN1 | 左前3508 + C620 | 1 | 0x200，槽1 | 0x201 |
| CAN1 | 右前3508 + C620 | 2 | 0x200，槽2 | 0x202 |
| CAN1 | 左后3508 + C620 | 3 | 0x200，槽3 | 0x203 |
| CAN1 | 右后3508 + C620 | 4 | 0x200，槽4 | 0x204 |
| CAN2 | GM6020大yaw | 1 | 0x1FF，槽1 | 0x205 |
| CAN2 | 小pitch M2006 + C610 | 1 | 0x200，槽1 | 0x201 |
| CAN2 | 夹爪 M2006 + C610 | 2 | 0x200，槽2 | 0x202 |
| CAN2 | 左后腿J4310-2EC（MIT） | 0x02 | 0x02 | 0x03 |
| CAN2 | 右后腿J4310-2EC（MIT） | 0x01 | 0x01 | 0x11 |
| CAN2 | 大pitch J6006-2EC（PV） | 3 | 0x103 | 0x303 |
| CAN2 | RoboSister裁判电源模块 | 不适用 | C板不发送查询 | 0x31（4字节、大端） |

CAN2 的两个 C610 只设置为 ID1、ID2，不要另设 C610 ID5～8 使用GM6020的控制槽。代码集中配置并校验了上述大疆固定槽位；若要重排大疆ID，需同时修改反馈映射和 `frames()` 分组，不能只改一个数字。达妙CAN_ID、Master ID可在配置表中调整，仍须与调试助手一致。

C板 CAN1 RX/TX 为 PD0/PD1；CAN2 RX/TX 为 PB5/PB6。DR16 使用 USART3 PC11 RX，PC10 TX不用于遥控器。串口100000 baud，HAL设置9位字长、偶校验、1停止位，得到8位有效数据加校验；DMA1_Stream1、Channel4，Normal模式，IDLE分帧。C板专用DBUS口已有反相电路。

CAN2接口的5V不是电机24V电源；电机按各自说明书供电，并按总线要求共地、连接H/L。每条总线两端布置终端电阻，避免每台电机都接入终端电阻。

### 达妙助手的必要设置

双后腿使用 **MIT 模式**，位置和增益按位打包，命令 ID 等于 `CAN_ID`。大 pitch 使用**位置速度 PV 模式**，位置和最高速度为 IEEE754 小端浮点数，命令 ID 为 `0x100+CAN_ID`。FC 使能和 FD 失能均发往各自的命令 ID。

- 按上表设置 CAN_ID 与 Master ID。调试助手栏位可能采用不同进制，请核对实际值。
- 将通信丢失保护设为约100ms。所附V1.3协议的 `CAN Timeout` 单位是50µs，100ms对应2000；若自己的助手显示单位不同，按实际单位换算，不能直接照搬数值。
- 分别读取每台电机实际的PMAX、VMAX、TMAX，写入 `g_dm_config`。当前12.5、45、18是协议文档示例值，代码不自动检测或写入这些EEPROM参数。
- 后腿 MIT 台架增益左侧为 `mit_kp=10.0`，右侧为 `mit_kp=16.0`，两侧 `mit_kd=0.5`。速度和前馈转矩给定为零。C 板限制 MIT 位置目标的变化率。大 pitch 的 PV 指令包含最高速度。驱动器电流保护按电机和机构实际负载设置。
- `external_reduction` 表示达妙协议角度增量与机构角度增量之比的绝对值。按左侧接减速前转子、右侧接减速输出法兰配置。以协议角度为减速输出端坐标，左侧取 0.10，右侧取 1.0。两侧升降目标和角差按机构角度计算。接法与角度比例需上板小行程确认。
- 失能状态下持续发FD以取得状态。解锁后先向失能电机发送当前位置目标，再发FC；收到正常使能反馈后才持续发目标。不同驱动固件对失能状态下位置目标的处理需空载核对，首次不得依赖未经验证的使能瞬间行为。
- 代码不发送FE保存零点、不自动清除故障，也不通过CAN修改电机参数。

## 频率与任务

| 项目 | 频率/周期 |
|---|---|
| FreeRTOS内核与robot_control任务 | 1000Hz，1ms |
| C板向各组大疆电机发命令 | 500Hz，2ms |
| 大疆反馈 | 按电调默认1000Hz；在调参助手确认 |
| C板向每台达妙发命令 | 200Hz，5ms |
| 达妙反馈 | 每条命令对应一条回复；不额外查询 |
| robot_service任务 | 50Hz，20ms |

`robot_control`是电机命令的唯一常规生产者，完成快照、保护判断、控制计算并提交本周期报文。`can1_send`、`can2_send`各自独占一条总线的HAL发送入口。CAN2大疆两组分别放在偶数/奇数时隙，三台达妙放在5ms周期的第0/1/2个时隙。

按每帧保守估计135bit、所有规定反馈均运行时，CAN1约60.8%、CAN2约70.2%总线占用；这是静态估算，调试时需用CAN分析仪确认实际帧间隔和丢帧。该估算未含新增裁判模块的功率反馈，按每个4字节帧100bit估算，100Hz约另加1%、1000Hz约另加10%的CAN2占用。

接收中断只解析与更新反馈，控制任务通过短临界区取得一致快照。服务任务仅在控制任务和两路CAN发送任务仍有心跳时喂独立看门狗，约250ms无喂狗会复位。Debug通过SWD停住CPU时冻结IWDG。

HAL毫秒时基由BSP的TIM7实现，SysTick留给FreeRTOS。本机CubeMX命令行生成模板出现时基IRQ遗漏及SysTick空函数，因此工程明确在CMake中排除 `Core/Src/stm32f4xx_hal_timebase_tim.c`，使用 `bsp_timebase.c`，并在生成文件的USER CODE区接入FreeRTOS SysTick。不要同时启用两套HAL时基。

## 零点、软限位与机械参数

集中修改 `robot_config.h/.c`，无需在底层驱动里搜索参数。

| 参数 | 当前设置 | 需要核对的内容 |
|---|---|---|
| yaw范围 | -180～+180° | 上电第一帧为0；重解锁不重置，外部传动比需实测 |
| 大pitch范围 | 0～90° | 上电机械姿态默认0°，可改boot_joint_deg |
| 小pitch范围 | -60～+45° | 上电机械姿态默认0°，可改boot_joint_deg |
| 后腿范围 | 最低位上电为 0°，全行程左 CAN +360°、右 CAN −3600° | 双侧联动使用两侧 PMAX 允许范围的交集 |
| 夹爪范围 | 暂填0～60° | 必须换成实际开合传动行程 |
| M2006总传动比 | 36:1 | 若还有外部减速，填总传动比 |
| 左/右轮方向 | 左正、右反 | 悬空低速逐个确认，轮子安装可能不同 |
| 后腿方向 | 左正、右反 | 根据镜像安装和升降传动实测 |
| PID、最大电流、角速度 | 低功率起调值 | 不保证能带动实机，也不保证无振荡 |

`boot_joint_deg`声明上电时关节所在的机械角度。除yaw按要求取当前为0之外，其余关节必须先手动置于声明姿态，或者装限位/绝对传感器后自行实现标定；软件不能从未提供的机械零点推断绝对关节角。没有上电自动走到零位的动作。

大疆电机通过转子编码器跨圈累计角度，M2006再除总减速比。DJI机械臂电机反馈丢失达到50ms后，本次上电的多圈位置被判为不可信，需检查姿态后重启；回中无法清除此类位置丢失标志。遥控器断开但电机反馈一直正常时，则可按回中流程重新解锁。

底盘和整车模式共用双后腿联动。两侧在最低位上电，左拨杆上拨抬升，下拨回到各自上电位置。全行程目标为左侧 CAN +360°、右侧 CAN −3600°。将右侧驱动器读回的 PMAX 填入 `ROBOT_RIGHT_LIFT_PMAX_RAD`。从 CAN 位置约 0 rad 出发，右侧全行程需要大于 62.83 rad 的 PMAX，实际运动使用双侧可编码行程的交集。联动速度在 `ROBOT_LIFT_SYNC_SPEED_DEG_S` 设置，当前对应左侧 CAN 50°/s、右侧 CAN 500°/s。两侧使能后才执行拨杆动作。换算后的角差达到 5° 时领先侧等候，缩到 3° 后恢复。拨杆居中时两侧各自保持目标。

夹爪在合拢命令下检测“反馈电流超过阈值、速度较低并持续150ms”，然后改为限流保持；摇杆回中和切换模式后保持此夹紧状态，反向操作松开。未检测接触时保持位置目标。阈值、保持电流和开合范围均需实测，不能用电机堵转代替永久机械限位。

硬件接口预留在 `bsp_board.c`：

```c
bool BSP_UserEmergencyStop(void);  // 默认false，接入急停后返回真实状态
uint32_t BSP_UserLimitMask(void);  // 默认0，任一非0位触发全局停机
```

这两个默认实现没有实际接线。它们是输入接口，不表示已经有硬件限位。硬限位触发后不自动后退、不自动回零；清除限位条件后才允许回中解锁。

## 关控

- 遥控信号无效或超时 60ms、运行所需电机反馈无效或超时 50ms、达妙报故障或意外失能、CAN 故障、急停或硬限位触发时撤去驱动。云台多圈位置丢失后需重新建立上电位置。
- 停止时清掉待发动作，大疆发送零电流或零电压，达妙发送 FD 失能。恢复后摇杆和拨杆回中 500ms 才重新解锁。达妙驱动器的通信超时用于总线断开时失能。
- 机构目标受配置行程和达妙 PMAX 约束。升降角差用于同步等候。四轮功率限制在速度 PI 后独立执行。

`g_robot_debug.controller.stop_reason` 记录当前停机原因。1 为遥控，2 为反馈，8 为达妙状态，16 为总线，32 为急停，64 为模式值无效，128 为多圈位置丢失，256 为硬限位，1024 为等待回中。

## 调试与验收

`g_robot_debug.can[0/1]`显示入队、同ID合并、发送完成、过期、拒收、取消、待发数量、HAL错误及邮箱超时计数。`g_robot_debug.input`显示遥控器与十台电机反馈；`controller`显示解锁、角度、目标、命令与保护原因；`can_send_failures`统计发送失败；`max_cycle_us`记录实机测得的控制周期最长用时。调试快照每10ms更新。

底盘首次调试时先使四轮悬空并可靠支撑后腿。四轮 `direction` 位于 `g_dji_config`，当前起调值按左前、右前、左后、右后为 +1、-1、+1、-1。每个阶段都检查方向、零位、行程、回中保持和关遥控器后的撤力；在最终机械载荷下调PID与夹紧电流。

主机逻辑测试执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\Test.ps1
```

它使用本机CLion自带的桌面MinGW运行真实的协议和控制核心，覆盖底盘与整车模式、频率、关控恢复、报文、限位、夹紧和多种故障。新增功率测试还覆盖原算法例题、10000组随机模型及整数命令回代、裁判反馈与缓冲保护，并验证升降和机械臂未参与功率缩放。它不能验证电机固件、接线、CAN物理层、机械行程、真实功率或PID稳定性。

## 重新用CubeMX生成

打开 `engineer_robot.ioc`，启用“Generate peripheral initialization as a pair of .c/.h files per peripheral”，当前 `ProjectManager.CoupleFile=true`；Core/Inc与Core/Src分开。保留USER CODE，工具链选CMake。

生成后保留本工程根CMake中的 `include(cmake/robot.cmake)`，保留BSP时基和FreeRTOS USER CODE接入点。CAN1_TX_IRQHandler/CAN2_TX_IRQHandler由bsp_can.cpp提供，TX IRQ也在BSP初始化中启用；若CubeMX生成同名向量，须保留一份有效实现。若新版CubeMX开始正确生成TIM7_IRQHandler，须避免它与BSP同名实现重复；保留一份实际调用HAL_TIM_IRQHandler的中断处理即可。不要删除 `bsp/user_driver/task`。

本机首次生成时曾遗漏syscalls/sysmem支持文件，现已补入ST模板，并把生成的CMake绝对路径改成相对路径。若升级CubeMX后这些文件或路径又被重写，需按新生成结果核对，再跑Debug、Release构建和Test.ps1。不要覆盖旧版本后直接烧录未经检查的工程。

## 资料来源

协议以你提供的遥控器控制协议V1.4、C板原理图/位号图、C620、C610、GM6020、M2006说明书、两款达妙2EC说明书及达妙调试助手V1.3为准；机械臂电机配置来自你提供的《机械臂电机.docx》及本次确认。

补充核对的官方资料：

- [RoboMaster C板产品页](https://www.robomaster.com/en-US/products/components/general/development-board-type-c)
- [C板官方用户手册](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%20%E5%BC%80%E5%8F%91%E6%9D%BF%20C%20%E5%9E%8B%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)
- [GM6020官方使用说明](https://rm-static.djicdn.com/tem/3724/RM%20GM6020%20%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6.pdf)

第三方代码来自本机STM32CubeF4 1.28.3包，保留ST、Arm CMSIS和FreeRTOS原有版权声明；自编代码位于上述三个应用目录。
