# Twilight Wilderness Robot / 微光荒野机器人

可编辑 Blender 4.5 场景：[output/Twilight_Wilderness_Robot.blend](output/Twilight_Wilderness_Robot.blend)。重建脚本：[build_scene.py](build_scene.py)。

## 场景整体结构（已重新打开 .blend 并核验）

| 集合 | 内容 |
| --- | --- |
| Architecture | 顶层组织集合；子集合 `Architecture / CC0 source assets` 存放 7 个隐藏源网格（包含发光球源） |
| Environment | 30 m 可编辑地形；6 套 Geometry Nodes 散射器：约 34 松树、15 枯树、6 阔叶树、142 岩石、1,318 草丛及 108 灌木；另有 3 个手工放置的前景岩石 |
| Robot | 31 个独立可编辑机械零件；约 2.82 m 高，有肩、髋、肘、膝、颈部机构和错步落地脚掌 |
| Fireflies | 物理模拟器与力场（隐藏）及 92 个静态发光实例；静态实例不依赖粒子缓存 |
| Lights | 3 个面积光源、1 个正交相机；EEVEE **640 × 480** 预览设置 |

萤火虫由 Blender Newton 粒子系统、布朗运动与湍流力场模拟，顺序求值第 1–75 帧；在第 75 帧筛选 92 个存活位置，写入静态顶点网格，再由 Geometry Nodes 实例化发光球。帧 48–75 的共有粒子累计位移约 0.61 m；静态萤火虫高度范围约 0.52–2.82 m。

## JEV（Hardness: High）

各阶段的**完整结构证据、场景摘要与判定**：[`output/JEV_reviews.json`](output/JEV_reviews.json)。独立重开 `.blend` 的非渲染复核结果：[`output/scene_audit.json`](output/scene_audit.json)（8 项结构检查均通过；不代替视觉验收）。

1. **基础环境：结构 PASS。** 三种 CC0 树木和 Kenney 草、岩石、灌木源资产均仅导入一次，再通过六个 Geometry Nodes 散射器生成重复元素。复查时发现树木 FBX 指向不存在的贴图，已换成文件内的树皮、针叶材质；去除了遗留无用户贴图引用。下一步：机器人。
2. **机器人：结构 PASS。** 31 个可编辑部件，有关节、传感器、足底；双脚高度根据地形设置。下一步：萤火虫物理模拟。
3. **萤火虫：物理／静态数据 PASS。** 粒子模拟及运动量已确认；92 个静态实例、高度变化和烘焙顶点均经验证。下一步：灯光和材质。
4. **灯光／材质：PENDING_VISUAL。** 已设置冷色月光、冷填光、弱暖光和合理基础材质；仅对 640 × 480 EEVEE 参数完成机器验证。**尚需图形环境下的低分辨率预览**来检查构图、对比度、轮廓及材质。下一步：视觉审查。
5. **清理／交付：PENDING_VISUAL。** 集合、材质外部依赖、文件保存、重新打开与依赖图散射数量已核验；但前一步的视觉审查尚未通过，不能声称最终通过。下一步：制作并复核低分辨率预览。已准备受限渲染脚本 [`render_lowres_preview.py`](render_lowres_preview.py)，仅接受 EEVEE 640 × 480 并在写盘后验证 PNG 分辨率；不会自动给予视觉 JEV 通过。

**低分辨率预览图：尚未生成。** 当前无可用 EGL/GLX 上下文，EEVEE 渲染会崩溃；未使用其他渲染器或高分辨率渲染来规避此限制。请在有图形环境的 Blender 4.5 中打开 `.blend`，运行 `blender -b output/Twilight_Wilderness_Robot.blend --python render_lowres_preview.py`，脚本将严格校验并输出 640 × 480 EEVEE 预览，再人工目视确认。**目前不能声明「结构已确认，可交由人类进行高清渲染」。**

## 开源资产与许可

- [SkywolfGameStudios/CC0Tree](https://github.com/SkywolfGameStudios/CC0Tree)：3 个树木 FBX，CC0-1.0；许可证副本：[`assets/CC0Tree_LICENSE`](assets/CC0Tree_LICENSE)。
- [Open-Golf 的 Kenney Nature Kit 模型](https://github.com/mgerdes/Open-Golf/tree/master/data/models/nature_kit)：草、石、灌木 OBJ（文件声明 *Created by Kenney*）；该仓库 MIT 许可证副本：[`assets/Open-Golf_LICENSE`](assets/Open-Golf_LICENSE)。

资产作为网格已嵌入 `.blend`；仓库 `assets/` 中保留源文件供脚本重建。打开最终场景不需要外部贴图或联网。`build_scene.py` 可在装有 Blender `bpy` 4.5 的环境中重新构建，不尝试高清渲染。
