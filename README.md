# Flight_Data_Playback_Platform
Webots were used to replay and process the logs collected by aircraft, which are now available for the input of height, time stamp and quaternion.csv file.

更新日志：

---------------------Record_DataV1.0-------------------------
满足可仿真内容，用户可修改宏进行可视化仿真
1. 可修改宏：
CSV_PATH：(绝对路径)
2. 可利用键盘控制数据回放

---------------------Record_DataV2.0-------------------------
1. 新增高度比例尺（scale = 0.1）
2. 新增动态调整固定角度的跟随高度（ON）
3. 新增多角度视察功能（OFF）

---------------------Record_DataV3.0-------------------------
1. 新增位置重置功能
2. 将可修改的用户宏定义全部取消，转换成键控模式切换。
3. 新增读取csv行数的功能，代替宏定义csv数据的行数。
4. 新增在终端的LOG窗口界面。
（用户仅需将csv文件扔在文件夹内就可以运行）
---------------------Record_DataV4.0-------------------------
1. 新增仿真高度补偿
2. 新增输出t时间以及轴角度。

---------------------Record_DataV5.0-------------------------
1. 修复重置后飞机以第一帧数据的姿态显示的BUG
2. 新增时间和高度的屏显（分别保留小数点后3、2位）
