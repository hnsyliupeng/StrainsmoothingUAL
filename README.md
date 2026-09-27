# Twilight Wilderness Robot — corrective candidate (not accepted)

**状态：未通过最终 JEV；不能作为已完成场景交付。** 用户的两张 Blender 截图表明旧版场景存在明显问题：机器人被树挡住，树冠线框/树枝残留，草、石头和森林地面几乎无有效材质，花朵也不可辨。旧版的“结构通过”结论失效。

## 最新候选版（尚无真实画面验收）

`output/Twilight_Wilderness_Robot.blend` 是可编辑的 Blender 4.5 场景，**EEVEE 640 × 480**。切勿将上一版截图或自动实例计数当作新版预览。

- **Environment**：7 套 Geometry Nodes 散射器；导入的 MIT 授权树木/灌木/草/岩石资产与 LGPL-2.1 花瓣资产组合。当前依赖图约 20 松树、4 枯树、4 阔叶树、83 石头、975 草、63 灌木、23 株花。草和花有叶/花体网格；花朵是在导入的灌木叶片上合并导入的红花，非单独悬浮花瓣。
- **材质纠错**：上一版误把 *灰度遮罩图* 当作彩色照片贴图，尤其石头贴图平均像素接近纯白。这次将灰度明暗经 ColorRamp 映射到深绿、暖灰及树皮褐色；针叶/草的透明度采用 alpha 阈值，减少黑色透明卡片边缘；地面增加暗土色噪声变化。纹理已打包进 `.blend`。
- **Robot**：99 个可编辑机械/壳体部件，仍需按实际画面确认是否可信。GN 大树设有相机中央通道避让，数据检查确认通道内无实例树干，但不等于证明枝叶绝不挡画面。
- **Fireflies**：保留 Blender Newton 粒子、布朗运动、湍流力场及第 75 帧的 92 个静态发光实例。
- **场景组织**：Architecture / Environment / Robot / Fireflies / Lights 和源资产集合。

## JEV / 验收状态

[`output/JEV_reviews.json`](output/JEV_reviews.json) 对五步均标记 **PENDING_VISUAL**，不是 PASS。[`output/scene_audit.json`](output/scene_audit.json) 是重开 `.blend` 的非渲染数据复核，记录接地、中央通道、材质节点、实例与图像打包等检查，**不构成外观验收**。

**低分辨率预览图仍未生成。** 当前沙箱无可用 EGL/GLX 图形上下文，不能在这里完成要求的 EEVEE 视图审查；不会拿其他渲染器或伪图充数。具备图形支持的 Blender 4.5 可运行：

```bash
blender -b output/Twilight_Wilderness_Robot.blend --python render_lowres_preview.py
```

脚本严格限制 EEVEE 640 × 480、核验 PNG 文件头，生成后仍须人工检查可见的机器人、植物轮廓、落地、色彩与光感。**目前不能声明「结构已确认，可交由人类进行高清渲染」。**

## 资产许可

- [HungryProton/scatter demo assets](https://github.com/HungryProton/scatter/tree/main/addons/proton_scatter/demos/assets)，MIT；见 `assets/proton_scatter/LICENSE.md`。
- [PlantSimulationLab/Helios RedbudFlower](https://github.com/PlantSimulationLab/Helios)，LGPL-2.1；见 `assets/proton_scatter/RedbudFlower_LICENSE`。
- 历史源资产 CC0Tree FBX、Open-Golf Kenney OBJ 保留在 `assets/` 及各自许可文件，但新版不实例化它们。

`build_scene.py` 重建候选场景；`audit_scene.py` 只做独立的数据复核。材质纹理已打包入 `.blend`，打开文件不需要联网。
