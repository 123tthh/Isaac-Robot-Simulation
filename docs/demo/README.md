# Simulation previews / 仿真预览

The homepage GIFs are cropped, accelerated excerpts of the original videos.
主页 GIF 是原始演示的裁切、加速片段；完整视频内容未修改。

| Preview | Source | Source interval | Playback |
| --- | --- | --- | --- |
| `simulation_part1.gif` | `test_part1.mp4` | 05:20–06:40 | 4×, ~20 s loop |
| `simulation_part2.gif` | `test_part2.mp4` | 07:20–08:40 | 4×, ~20 s loop |

Generate from the repository root with FFmpeg:

```bash
for part in 1 2; do
  if [ "$part" = 1 ]; then start=320; else start=440; fi
  ffmpeg -v error -ss "$start" -t 80 -i "docs/demo/test_part${part}.mp4" \
    -filter_complex '[0:v]crop=1190:715:40:85,setpts=(PTS-STARTPTS)/4,fps=8,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4' \
    -loop 0 -y "docs/demo/simulation_part${part}.gif"
done
```

These historical demonstrations are not new capture or cross-simulator validation runs.
这些历史演示不代表新增采集或跨仿真器测试结果。
