# Better Outer Wall Processing

用于 OrcaSlicer 的外墙 G-code 后处理工具，通过亚层螺旋、斜接缝和二次整形减弱外墙接缝痕迹。

## 功能

- `spiral`、`scarf`、`flat` 三种接缝处理模式
- 外墙二次整形与脚本熨烫
- 动态超分辨率、XY 层间插值与按实际表面角度降速
- 支持 M82/M83、G92 E 和常见 G2/G3 圆弧
- JSON 配置、命令行参数和图形设置界面
- 重复处理检测，避免同一 G-code 被再次修改

## 环境

- Python 3.12+
- OrcaSlicer
- Windows（仓库内提供批处理入口）

## 使用

直接处理 G-code：

```powershell
python .\better_outer_wall_processing.py .\example.gcode
```

打开图形设置界面：

```powershell
python .\bowp_settings_gui.py
```

接入 OrcaSlicer：

1. 打开 `Process Settings > Others > Post-Processing Scripts`。
2. 复制 `orca_postprocess_command.txt` 中的命令。
3. 将命令开头的 `<INSTALL_DIR>` 替换为本项目的实际安装目录。
4. 导出 G-code，OrcaSlicer 会自动执行后处理。

## 项目文件

- `better_outer_wall_processing.py`：核心处理器
- `better_outer_wall_processing.json`：默认配置
- `bowp_settings_gui.py`：图形设置界面
- `run_postprocess.bat`：Windows 后处理入口
- `orca_postprocess_command.txt`：OrcaSlicer 命令模板
- `selftest.py`：核心回归测试
- `audit_coverage.py`、`validate_gcode.py`：G-code 覆盖与路径安全验证
- `使用说明书.md`：完整中文参数说明

## 验证

```powershell
python -B .\selftest.py
```

验证自己的 G-code 时，工具只处理临时副本，不修改原文件：

```powershell
python -B .\validate_gcode.py "C:\path\to\model.gcode" --require-top-finish
```

## 注意

- 后处理结果不会反映在 OrcaSlicer 原始切片预览和时间估算中。
- 建议先用简单模型和小范围参数测试，再用于正式打印。
- G-code、处理副本、缓存和本地测试产物已通过 `.gitignore` 排除。

## License

MIT
