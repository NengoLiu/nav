# auto_construct 项目思维导图

```mermaid
mindmap
  root((auto_construct\n自动建图与覆盖导航系统))
    消息定义 msg/
      Pillar.msg\n柱子/障碍物
        cx, cy 中心坐标
        width, height 尺寸
        id 唯一标识
      CoverageRegion.msg\n覆盖区域
        Point32[] 外边界
        Pillar[] 内部障碍
    服务定义 srv/
      SetRegion.srv\n设置区域
        输入 CoverageRegion
        输出 success + message
      ConfirmRegion.srv\n确认规划
        输出 success + message + Path
      UpdateParams.srv\n更新参数
        robot_width 机器人宽度
        headland_width 边距宽度
        route_type 路线类型
        curve_type 曲线类型
        swath_angle 割幅角度
      GetMapList.srv\n获取地图列表
        输出 map_folders[]
      SetPathAndStart.srv\n设置路径并启动
        输入 map_dir 地图路径
        输出 success + message
    核心节点 src/
      MappingManager\n系统调度器
        状态机
          IDLE 空闲
          MAPPING 建图中
          NAVIGATION 导航中
          TRANSITIONING 切换中
        服务接口 sys/
          start_mapping 开始建图
          finish_mapping 结束建图
          start_navigation 开始导航
          stop_all 停止全部
          set_region 设置区域
          confirm_region 确认区域
          update_params 更新参数
          get_map_list 获取地图列表
          set_path_and_start 启动路径
        子进程管理
          start_launch_process 启动子进程
          stop_current_process 停止子进程
      SaveMap\n地图保存
        点云处理
          passThroughFilter Z轴过滤 0.2~1.5m
          radiusOutlierFilter 离群点过滤
          applyTransform 坐标变换
          setMapTopicMsg 体素化→OccupancyGrid
        地图存储
          异步调用 PGO 3D地图保存
          异步调用 Nav2 MapSaver 2D地图保存
          发布 OccupancyGrid
      CoverageManagerNode\n覆盖规划前端
        onSetRegion 接收边界+障碍
        onConfirmRegion 生成GML+触发规划
        onUpdateParams 更新算法参数
        writeGml 转换为GML格式
        triggerPlanningSync 同步调用规划动作
        publishPillarMarkers RViz可视化
      CoveragePath\n路径执行引擎
        loadPath 加载YAML路径文件
        执行控制
          svcStart 开始
          svcPause 暂停
          svcResume 恢复
          svcCancel 取消
        runExecution 主执行循环
        sendAndWait 发送至Nav2
        printProgress 终端进度条
    配置文件 config/
      save_map.yaml
        Z过滤 0.2~1.5m
        半径过滤 0.5m
        地图分辨率 0.05m
      coverage_params.yaml
        机器人宽度 robot_width
        DUBIN曲线
        BOUSTROPHEDON路线
        最小转弯半径
      nav2_params.yaml
        BT Navigator配置
        单点导航插件
        多点导航插件
        覆盖导航插件
    启动文件 launch/
      mapping_manager.launch.py\n建图总启动
        Livox驱动
        FastLIO2 激光里程计
        MappingManager节点
        SaveMap节点
      mapping_plugin.launch.py\n建图插件
        FastLIO2
        Octomap实时建图
      nav_plugin.launch.py\n导航插件
        Nav2导航栈
        机器人定位
      convert_map.launch.py\n覆盖规划启动
        CoverageManager节点
        覆盖动作服务器
      coverage_path_optimized.launch.py\n路径执行
    外部依赖
      FastLIO2\n激光里程计+建图
        发布 lio_odom
        发布点云数据
      PGO\n3D地图优化存储
        /pgo/save_maps 服务
      Nav2\n导航栈
        NavigateThroughPoses 动作
        地图保存服务
      opennav_coverage\n覆盖路径算法
        ComputeCoveragePath 动作
      Octomap\n3D占用地图
      Livox\n激光雷达驱动
    数据流向
      传感器数据
        LiDAR → FastLIO2 → PGO → SaveMap → OccupancyGrid
      覆盖规划
        前端App → SetRegion → ConfirmRegion → GML → opennav_coverage → YAML路径
      路径执行
        YAML路径 → CoveragePath → Nav2 NavigateThroughPoses → 机器人
```

---

## 系统架构总览

```
┌─────────────────────────────────────┐
│         移动端 / 前端应用             │
└──────────────┬──────────────────────┘
               │ ROS 2 服务调用
               ▼
      ┌─────────────────┐
      │  MappingManager │  ← 系统总调度（状态机）
      │  /sys/* 服务路由  │
      └──┬──────────┬───┘
         │          │
         ▼          ▼
  ┌──────────┐  ┌──────────────────┐
  │ SaveMap  │  │ CoverageManager  │
  │ 地图保存  │  │  覆盖路径规划     │
  └──────────┘  └────────┬─────────┘
                         │ ComputeCoveragePath 动作
                         ▼
                ┌─────────────────┐
                │  CoveragePath   │
                │  路径执行引擎    │
                └────────┬────────┘
                         │ NavigateThroughPoses 动作
                         ▼
                ┌─────────────────┐
                │  Nav2 导航栈    │
                └─────────────────┘
```

## 关键设计模式

| 模式 | 说明 |
|------|------|
| 分层服务代理 | `/sys/*` 公开API → 委托内部实现节点 |
| 异步 Future | 非阻塞调用 PGO/Nav2 地图保存服务 |
| 互斥状态机 | TRANSITIONING 状态防止并发切换竞争 |
| 无状态覆盖API | 每次 SetRegion+ConfirmRegion 独立执行 |
| 回调组并发 | Reentrant 组允许同一节点并发处理回调 |
